"""The arbiter (M-b, plan docs/plans/2026-10-16-m-b-arbiter.md; design docs/design/2026-10-13-operator-console.md
sections 4-5; contract docs/interfaces/2026-10-13-operator-console-contract-draft.md sections 1-6).

Pure: every entry point takes the vehicle's monotonic time ``now_mono`` and the tokens and lease ids come from an
injected ``token_source`` -- the module never reads a clock or a random source itself (contract section 9, 1).

What it owns: the control lease, the four generations, the order a reliable request is handled in, the mode and
the stop kind, arming and the dead-man switch, which source drives (mission or operator) and the source-independent
movement ban. What it does not: the mission's progress (``forklift_core.mission``) and the actuator's own expiry.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

from forklift_core.localization.slam_pose import StopDetector
from forklift_core.mission.commands import FINAL, Cancel, Resume, Submit

from .generations import PENDING, RequestBook
from .ledger import FrameLedger, TickLedger
from .teleop_policy import Policy, teleop_policy

MODES = ("PAUSED", "AUTO", "TELEOP", "STOPPED")
LEASED = ("rearm", "mode_request", "arm", "disarm", "mission_submit", "mission_pause", "mission_resume",
          "mission_cancel", "recovery")
RELIABLE = LEASED + ("stop", "lease_request", "lease_release", "targets_request")
AUTO_STATUSES = ("accepted", "running", "resuming", "cancelling")  # the mission states AUTO stays in (B5)


@dataclass(frozen=True)
class ArbiterConfig:
    """Every number is provisional until the H3 bench measurements (design 5.4); none is a constant here."""

    command_max_age_s: float = 0.30
    lease_timeout_s: float = 1.0
    grace_s: float = 10.0
    video_max_age_s: float = 0.5
    camera_latency_bound_s: float = 0.15
    request_capacity: int = 4096
    frame_window_s: float = 5.0
    feedback_max_age_s: float = 0.1
    not_stopped_s: float = 10.0  # the mission's E3 limit: not standing by then is a fault (B5)
    tick_s: float = 1 / 120  # the control cycle the stop detector counts in

    def __post_init__(self):
        if self.lease_timeout_s < self.command_max_age_s:
            raise ValueError("lease_timeout_s must be at least command_max_age_s (design 5.4)")


@dataclass(frozen=True)
class Feedback:
    """What the vehicle measured this cycle, with when (vehicle monotonic time). None where it cannot measure."""

    stamp_mono: float
    rear_axle_speed_mps: float | None
    yaw_rate_radps: float = 0.0


@dataclass
class Output:
    mode: str
    stop_kind: str
    source: str  # auto / teleop / core_stop / zero
    speed_mps: float
    curvature_inv_m: float
    policy: Policy
    control_gen: int
    armed: bool
    reason: str | None = None


@dataclass
class Lease:
    client_id: str
    lease_id: str
    renewed_mono: float  # the send time of the tick its last valid reply named (the grant time before any)


class Arbiter:
    def __init__(self, session_epoch: int, config: ArbiterConfig, token_source: Callable[[], str],
                 mission=None) -> None:
        self.session_epoch = int(session_epoch)
        self.config = config
        self._token = token_source
        self.ticks = TickLedger(max(config.command_max_age_s, config.lease_timeout_s), token_source)
        self.frames = FrameLedger(config.frame_window_s)
        self.requests = RequestBook(config.request_capacity)
        self.clients: dict[str, bool] = {}  # connected client -> its connection may control (contract 10)
        self.lease: Lease | None = None
        self.reserved: tuple[str, float] | None = None  # (client_id, until) during the reconnection grace
        self.mode = "PAUSED"
        self.control_gen = 0
        self.arm_gen = 0
        self.armed = False
        self.preempt_latch: str | None = None  # why a safety preempt holds until the truck is seen standing
        self.events: list[dict] = []  # every boundary with its time and reason, for the record
        self.mission = mission  # the mission lifecycle (MissionCommands): its hold and the requests it judges
        self.detector = StopDetector(tick_s=config.tick_s)
        self.feedback_ok = False
        self.stop_confirmed = False
        self.stopping_since: float | None = None
        self.last_applied = 0.0  # the speed applied last cycle (the stop detector's command input)
        self.last_safety: dict | None = None  # the mission's safety state from the last step
        self.pending: list[dict] = []  # mission requests waiting for this cycle's lifecycle verdict (B3)
        self.provisional_auto = False
        self.outbox: list[dict] = []  # replies settled outside handle (mission requests), sent by resolve

    # --- connections and the lease ------------------------------------------------------------------------------
    def connect(self, client_id: str, now_mono: float, *, control: bool = True) -> dict:
        self._expire(now_mono)
        self.clients[client_id] = bool(control)
        return {"t": "welcome", "api": "console/1", "session_epoch": self.session_epoch}

    def disconnect(self, client_id: str, now_mono: float) -> None:
        self._expire(now_mono)
        self.clients.pop(client_id, None)
        if self.lease is not None and self.lease.client_id == client_id:
            self._lose_lease(now_mono, "disconnected", reserve=True)

    def _boundary(self, now_mono: float, reason: str) -> None:
        self.control_gen += 1
        self.events.append({"t": now_mono, "event": "control_gen", "control_gen": self.control_gen,
                            "reason": reason})

    def _disarm(self, now_mono: float, reason: str) -> None:
        if self.armed:
            self.armed = False
            self.events.append({"t": now_mono, "event": "disarm", "reason": reason})

    def _lose_lease(self, now_mono: float, reason: str, *, reserve: bool) -> None:
        """Expiry, a lost connection or a release: the lease goes and the control generation moves. The mode
        decides the rest -- AUTO keeps working (user decision, design 2), TELEOP stops on a safety preempt."""
        holder = self.lease.client_id
        self.lease = None
        self.reserved = (holder, now_mono + self.config.grace_s) if reserve else None
        if self.mode == "TELEOP":
            self.preempt_latch = reason
            self._set_mode("PAUSED", now_mono, reason)
        else:
            self._boundary(now_mono, reason)

    def _expire(self, now_mono: float) -> None:
        if self.lease is not None and now_mono > self.lease.renewed_mono + self.config.lease_timeout_s:
            self._lose_lease(now_mono, "lease_expired", reserve=True)
        if self.reserved is not None and now_mono > self.reserved[1]:
            self.reserved = None
        self.ticks.prune(now_mono)

    def _set_mode(self, mode: str, now_mono: float, reason: str) -> None:
        """Every mode change is a control boundary (B2) and disarms (design 5.1); leaving TELEOP hands the mission
        back (held -> paused, B5)."""
        if mode not in MODES:
            raise ValueError(mode)
        if mode == self.mode:
            return
        self.events.append({"t": now_mono, "event": "mode", "from": self.mode, "to": mode, "reason": reason})
        if self.mode == "TELEOP" and self.mission is not None:
            self.mission.hold_off(now_mono)
        self.mode = mode
        self._disarm(now_mono, reason)
        self._boundary(now_mono, reason)

    # --- reliable requests (contract section 1, the handling order) ---------------------------------------------
    def handle(self, client_id: str, msg: dict, now_mono: float) -> dict | None:
        """One message from a client. Reliable requests are answered (ack or reject); latest-value messages
        (teleop, echo, input_state) are not. A mission request may be answered later (``resolve``)."""
        self._expire(now_mono)
        kind = msg.get("t")
        if client_id not in self.clients:
            return None  # not connected: nothing it says counts
        if kind not in RELIABLE:
            return self._latest_value(client_id, kind, msg, now_mono)
        request_id = msg.get("request_id")
        if not isinstance(request_id, str) or not request_id:
            return self._reply(kind, request_id, "invalid")
        if kind == "stop":
            # 0. STOP before the store and every generation check, every time (never stored).
            if not self.clients[client_id]:
                return self._reply(kind, request_id, "unsupported")
            self.stop(now_mono, f"stop:{msg.get('source', 'console')}")
            return self._reply(kind, request_id)
        # 1. Handled before: its original answer, without running it again or checking generations.
        stored = self.requests.get(client_id, request_id)
        if stored is not None:
            return None if stored == PENDING else stored
        # 2. New: refused busy (not stored) when the store is full, else checked, run and stored.
        if self.requests.full():
            return self._reply(kind, request_id, "busy")
        result = self._run(client_id, kind, msg, now_mono)
        self.requests.put(client_id, request_id, result if result is not None else PENDING)
        return result

    def _reply(self, kind: str, request_id, reason: str | None = None, **fields) -> dict:
        reply = {"t": "ack" if reason is None else "reject", "of": kind, "request_id": request_id}
        if reason is not None:
            reply["reason"] = reason
        reply.update(fields)
        return reply

    def _generation_problem(self, client_id: str, msg: dict, *, lease: bool) -> str | None:
        if msg.get("session_epoch") != self.session_epoch:
            return "stale_epoch"
        if not lease:
            return None
        if self.lease is None or self.lease.client_id != client_id:
            return "no_lease"
        if msg.get("lease_id") != self.lease.lease_id:
            return "stale_lease"
        if msg.get("control_gen") != self.control_gen:
            return "stale_control"
        return None

    def _run(self, client_id: str, kind: str, msg: dict, now_mono: float) -> dict | None:
        request_id = msg["request_id"]
        if not self.clients[client_id]:
            return self._reply(kind, request_id, "unsupported")  # an observe-only connection (contract 10)
        problem = self._generation_problem(client_id, msg, lease=kind in LEASED)
        if problem is not None:
            return self._reply(kind, request_id, problem)
        handler = getattr(self, f"_on_{kind}")
        return handler(client_id, request_id, msg, now_mono)

    def _on_lease_request(self, client_id, request_id, msg, now_mono):
        if self.lease is not None:
            if self.lease.client_id == client_id:
                return self._reply("lease_request", request_id, "invalid")  # it holds one already
            return self._reply("lease_request", request_id, "no_lease")
        if self.reserved is not None and self.reserved[0] != client_id:
            return self._reply("lease_request", request_id, "lease_reserved")
        self.reserved = None
        self.lease = Lease(client_id, self._token(), now_mono)
        self.events.append({"t": now_mono, "event": "lease", "client_id": client_id})
        return self._reply("lease_request", request_id, lease_id=self.lease.lease_id,
                           lease_timeout_s=self.config.lease_timeout_s, control_gen=self.control_gen)

    def _on_lease_release(self, client_id, request_id, msg, now_mono):
        if self.lease is None or self.lease.client_id != client_id or msg.get("lease_id") != self.lease.lease_id:
            return self._reply("lease_release", request_id, "no_lease")
        self._lose_lease(now_mono, "lease_released", reserve=False)
        return self._reply("lease_release", request_id)

    def _on_targets_request(self, client_id, request_id, msg, now_mono):
        return self._reply("targets_request", request_id)  # the runner sends the list itself

    def _on_recovery(self, client_id, request_id, msg, now_mono):
        return self._reply("recovery", request_id, "unsupported")  # plan section 1: not this slice

    # --- stops ---------------------------------------------------------------------------------------------------
    def stop(self, now_mono: float, reason: str) -> None:
        """A safety preempt stop from any mode: STOPPED until a rearm, disarmed, a control boundary. A repeated
        STOP while stopped is a boundary too -- a rearm made after the first STOP must not run after the second
        (Codex M-b plan review P1)."""
        if self.mode == "STOPPED":
            self._disarm(now_mono, reason)
            self._boundary(now_mono, reason)
        else:
            self._set_mode("STOPPED", now_mono, reason)

    # --- latest-value messages -------------------------------------------------------------------------------------
    def _latest_value(self, client_id: str, kind: str, msg: dict, now_mono: float) -> None:
        if kind == "echo" and self.clients.get(client_id):
            self._renew(client_id, msg)
        return None

    def _renew(self, client_id: str, msg: dict) -> bool:
        """A valid reply from the lease holder renews the lease from the time its tick was sent."""
        lease = self.lease
        if lease is None or lease.client_id != client_id or msg.get("session_epoch") != self.session_epoch:
            return False
        if msg.get("lease_id") != lease.lease_id:
            return False
        echo = msg.get("echo") or {}
        sent = self.ticks.sent_mono(echo.get("state_seq"), echo.get("token"))
        if sent is None:
            return False
        lease.renewed_mono = max(lease.renewed_mono, sent)
        return True

    def tick(self, now_mono: float) -> dict:
        """A ``tick`` message: a fresh token remembered as sent now (contract section 4)."""
        self._expire(now_mono)
        state_seq, token = self.ticks.issue(now_mono)
        return {"t": "tick", "session_epoch": self.session_epoch, "state_seq": state_seq, "token": token}

    # --- the cycle (B3): observe, handle, mission_context, the mission's tick, resolve, step ----------------------
    def observe(self, now_mono: float, feedback: Feedback | None) -> None:
        """First in the cycle: this cycle's feedback, the stop detector (once a cycle), the expiries, the latch and
        the stop failure -- so every transition this cycle judges uses this cycle's stop confirmation."""
        self._expire(now_mono)
        fresh = (feedback is not None and feedback.rear_axle_speed_mps is not None
                 and now_mono - feedback.stamp_mono <= self.config.feedback_max_age_s)
        self.feedback_ok = fresh
        if fresh:
            self.stop_confirmed = self.detector.update(commanded_speed=self.last_applied,
                                                       speed=float(feedback.rear_axle_speed_mps),
                                                       yaw_rate=float(feedback.yaw_rate_radps))
        else:
            self.stop_confirmed = False
            if self.mode == "TELEOP" and self.preempt_latch is None:
                self._disarm(now_mono, "no_feedback")
                self.preempt_latch = "no_feedback"
        if self.preempt_latch is not None and self.stop_confirmed:
            self.events.append({"t": now_mono, "event": "latch_released", "reason": self.preempt_latch})
            self.preempt_latch = None
        must_stand = self.mode in ("PAUSED", "STOPPED") or self.preempt_latch is not None
        if must_stand and not self.stop_confirmed:
            if self.stopping_since is None:
                self.stopping_since = now_mono
            elif now_mono - self.stopping_since >= self.config.not_stopped_s and self.mode != "STOPPED":
                self.stop(now_mono, "not_stopped")  # a fault: the stop did not happen (B5)
        else:
            self.stopping_since = None

    def mission_stop_kind(self) -> str:
        """The stop the mission is given (B5 table, third column): it runs only in AUTO."""
        if self.mode == "STOPPED" or self.preempt_latch is not None:
            return "preempt"
        if self.mode == "AUTO":
            return "none"
        return "controlled"

    def mission_context(self, now_mono: float) -> str:
        """After the messages: queue the mission requests still valid (a boundary since they arrived refuses them
        stale_control) and return the stop kind to give the mission. ``none`` replaces ``controlled`` only when a
        queued submit or resume can be taken by the lifecycle as it stands (B5)."""
        status = self.mission.status if self.mission is not None else "idle"
        self.provisional_auto = False
        for p in self.pending:
            if p["queued"]:
                continue
            if p["control_gen"] != self.control_gen:
                self._settle(p, self._reply(p["kind"], p["request_id"], "stale_control"))
                continue
            command = p["command"]
            if self.mode == "PAUSED" and self.preempt_latch is None and (
                    (command.kind == "submit" and (status == "idle" or status in FINAL))
                    or (command.kind == "resume" and status == "paused")):
                self.provisional_auto = True
            self.mission.queue(command)
            p["queued"] = True
        self.pending = [p for p in self.pending if not p.get("settled")]
        stop_kind = self.mission_stop_kind()
        return "none" if self.provisional_auto and stop_kind == "controlled" else stop_kind

    def resolve(self, now_mono: float, records: list[dict]) -> list[dict]:
        """After the mission's tick: answer the queued requests from the lifecycle's records, then keep AUTO only
        while the mission is in a state AUTO stays in (its state after the tick -- a resume accepted at the tick's
        start can be refused by the same tick's checks). Returns the replies made."""
        by_id = {r.get("request_id"): r for r in records if r.get("request_id") is not None}
        took_auto = False
        for p in self.pending:
            rec = by_id.get(p["request_id"])
            if rec is None:
                reply = self._reply(p["kind"], p["request_id"], "invalid")
            elif rec["accepted"]:
                reply = self._reply(p["kind"], p["request_id"], mission_id=rec.get("mission_id"))
                took_auto |= p["kind"] in ("mission_submit", "mission_resume")
            else:
                reply = self._reply(p["kind"], p["request_id"], rec["reason"])
            self._settle(p, reply)
        self.pending = []
        status = self.mission.status if self.mission is not None else "idle"
        if took_auto and self.provisional_auto and status in AUTO_STATUSES:
            self._set_mode("AUTO", now_mono, "mission")
        elif self.mode == "AUTO" and status not in AUTO_STATUSES:
            last = self.mission.log[-1] if self.mission is not None and self.mission.log else {}
            self._set_mode("PAUSED", now_mono, f"mission:{last.get('reason') or status}")
        self.provisional_auto = False
        replies, self.outbox = self.outbox, []
        return replies

    def _settle(self, p: dict, reply: dict) -> None:
        p["settled"] = True
        self.requests.put(p["client_id"], p["request_id"], reply)
        self.outbox.append(dict(reply, client_id=p["client_id"]))

    def step(self, now_mono: float, safety: dict | None, candidate: tuple[float, float],
             teleop: tuple[float, float] = (0.0, 0.0)) -> Output:
        """The output (B11): the policy, the source and the speed. ``candidate`` is the core's (speed, curvature);
        ``teleop`` the operator's command after its chain (M-b4)."""
        self.last_safety = safety
        policy = teleop_policy(safety, feedback_ok=self.feedback_ok, standing=self.stop_confirmed)
        speed, curvature = 0.0, float(candidate[1])
        if self.mode == "STOPPED" or self.preempt_latch is not None:
            source, stop_kind = "zero", "preempt"
        elif self.mode == "AUTO":
            source, stop_kind = "auto", "none"
            speed = float(candidate[0])
        elif self.mode == "PAUSED":
            # The core's paused tick brakes on its slew; never faster than what was applied (B11).
            source, stop_kind = "core_stop", "controlled"
            speed = max(-abs(self.last_applied), min(abs(self.last_applied), float(candidate[0])))
        else:  # TELEOP
            source, stop_kind = "teleop", "none"
            speed, curvature = (float(teleop[0]), float(teleop[1])) if policy.drive_allowed else (0.0, float(teleop[1]))
        self.last_applied = speed
        return Output(self.mode, stop_kind, source, speed, curvature, policy, self.control_gen, self.armed,
                      self.preempt_latch)

    # --- mode and mission requests (B5) --------------------------------------------------------------------------
    def _needs_stand(self, kind: str, request_id) -> dict | None:
        if not self.feedback_ok:
            return self._reply(kind, request_id, "no_feedback:rear_axle_speed")
        if not self.stop_confirmed:
            return self._reply(kind, request_id, "not_stopped")
        return None

    def _near_active(self) -> bool:
        return bool(((self.last_safety or {}).get("near") or {}).get("active"))

    def _on_rearm(self, client_id, request_id, msg, now_mono):
        if self.mode != "STOPPED":
            return self._reply("rearm", request_id, "invalid")
        refused = self._needs_stand("rearm", request_id)
        if refused is not None:
            return refused
        self._set_mode("PAUSED", now_mono, "rearm")
        return self._reply("rearm", request_id, control_gen=self.control_gen)

    def _to_paused(self, kind, request_id, now_mono):
        if self.mode == "STOPPED":
            return self._reply(kind, request_id, "stopped_locked")
        if self.mode == "AUTO" and self._near_active():
            return self._reply(kind, request_id, "near_tracking_active")
        self._set_mode("PAUSED", now_mono, kind)
        return self._reply(kind, request_id, control_gen=self.control_gen)

    def _on_mode_request(self, client_id, request_id, msg, now_mono):
        to = msg.get("to")
        if to == "PAUSED":
            return self._to_paused("mode_request", request_id, now_mono)
        if to != "TELEOP":
            return self._reply("mode_request", request_id, "invalid")
        if self.mode == "STOPPED":
            return self._reply("mode_request", request_id, "stopped_locked")
        if self.mode != "PAUSED":
            return self._reply("mode_request", request_id, "invalid")  # no AUTO -> TELEOP: PAUSED first
        refused = self._needs_stand("mode_request", request_id)
        if refused is not None:
            return refused
        policy = teleop_policy(self.last_safety, feedback_ok=self.feedback_ok, standing=self.stop_confirmed)
        if not policy.teleop_allowed:
            return self._reply("mode_request", request_id, "state_forbids_teleop", detail=policy.reason)
        if self.mission is not None:
            self.mission.hold_on(now_mono)
        self._set_mode("TELEOP", now_mono, "mode_request")
        return self._reply("mode_request", request_id, control_gen=self.control_gen)

    def _mission_request(self, client_id, request_id, kind, command):
        if self.mission is None:
            return self._reply(kind, request_id, "unsupported")
        self.pending.append({"client_id": client_id, "request_id": request_id, "kind": kind, "command": command,
                             "control_gen": self.control_gen, "queued": False})
        return None  # answered by resolve, this cycle

    def _on_mission_submit(self, client_id, request_id, msg, now_mono):
        if self.mode != "PAUSED":
            return self._reply("mission_submit", request_id, "stopped_locked" if self.mode == "STOPPED" else "invalid")
        refused = self._needs_stand("mission_submit", request_id)
        if refused is not None:
            return refused
        return self._mission_request(client_id, request_id, "mission_submit", Submit(
            msg.get("pallet_id"), msg.get("destination_id"), msg.get("targets_revision"), request_id=request_id))

    def _on_mission_resume(self, client_id, request_id, msg, now_mono):
        if self.mode != "PAUSED":
            return self._reply("mission_resume", request_id, "stopped_locked" if self.mode == "STOPPED" else "invalid")
        if self.mission is not None and msg.get("mission_id") != self.mission.mission_id:
            return self._reply("mission_resume", request_id, "unknown_mission")
        refused = self._needs_stand("mission_resume", request_id)
        if refused is not None:
            return refused
        return self._mission_request(client_id, request_id, "mission_resume", Resume(request_id=request_id))

    def _on_mission_pause(self, client_id, request_id, msg, now_mono):
        if self.mode != "AUTO":
            return self._reply("mission_pause", request_id, "stopped_locked" if self.mode == "STOPPED" else "invalid")
        if self.mission is not None and msg.get("mission_id") != self.mission.mission_id:
            return self._reply("mission_pause", request_id, "unknown_mission")
        return self._to_paused("mission_pause", request_id, now_mono)

    def _on_mission_cancel(self, client_id, request_id, msg, now_mono):
        if self.mode not in ("PAUSED", "AUTO"):
            return self._reply("mission_cancel", request_id, "stopped_locked" if self.mode == "STOPPED" else "invalid")
        return self._mission_request(client_id, request_id, "mission_cancel",
                                     Cancel(msg.get("mission_id"), request_id=request_id))
