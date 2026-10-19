"""State machine of the fake vehicle that speaks the console/1 draft.

Mock only: it imitates the arbiter rules the console relies on so the web app can be
developed; it is not forklift_core/safety (M-b) and its tests are not evidence for it.
Times come from the injected monotonic clock; the browser's clock is never used.
Section numbers in comments refer to docs/interfaces/2026-10-13-operator-console-contract-draft.md.
"""

from __future__ import annotations

import math
import secrets
import time
from collections import OrderedDict
from collections.abc import Callable
from dataclasses import asdict, dataclass

from console.mock_vehicle import paths
from forklift_core.mission.results import ResultsBook
from forklift_core.safety.generations import RequestLedger
from console.mock_vehicle.world import (
    CAMERA_MODEL,
    CHASSIS,
    Pallet,
    Pose,
    World,
    default_world,
    loaded_nose_m,
    permission,
)

API = "console/1"
PHASES = (
    "observe",
    "approach",
    "insert",
    "lift",
    "extract",
    "transport",
    "lower",
    "withdraw",
    "return",
)
TELEOP_FORBIDDEN = {"insert", "lift", "lower", "withdraw", "recover"}
NO_MANUAL_LIFT = {"insert", "withdraw", "recover"}
LEASE_REQUIRED = {
    "rearm",
    "mode_request",
    "arm",
    "disarm",
    "mission_submit",
    "mission_pause",
    "mission_resume",
    "mission_cancel",
    "recovery",
    "lease_release",
    "session_end",  # persistent console plan D7/D13: as the live runner
    "sim_reset",
}
RELIABLE = LEASE_REQUIRED | {"stop", "lease_request", "targets_request"}
LATEST_ONLY = {"echo", "teleop", "input_state"}
STOPPED_ALLOWED = {
    "stop",
    "rearm",
    "disarm",
    "lease_request",
    "lease_release",
    "mission_cancel",
    "mission_pause",
    "targets_request",
    "session_end",  # the arbiter takes them in STOPPED too (stand, feedback, no mission)
    "sim_reset",
}
INT_FIELDS = ("session_epoch", "control_gen", "arm_gen", "seq")
LIFT_TOP_M = 0.28
LIFT_CARRY_M = 0.12
LIFT_RATE_MPS = 0.06


@dataclass(frozen=True)
class Config:
    command_max_age_s: float = 0.30
    tick_hz: float = 20.0
    lease_timeout_s: float = 1.0
    grace_s: float = 5.0
    video_max_age_s: float = 0.50
    camera_latency_bound_s: float = 0.05
    forward_mps: float = 0.50
    reverse_mps: float = 0.25
    loaded_scale: float = 0.6
    accel_mps2: float = 0.8
    decel_mps2: float = 1.5
    stop_threshold_mps: float = 0.02
    stop_hold_s: float = 0.5
    result_cache_max: int = 4096
    # persistent console plan D7/D13: the live runner's request ledger (tokens); the server turns it on, the older
    # unit tests keep the request-id store
    request_tokens: bool = False


def _is_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _is_ratio(v) -> bool:
    return (
        isinstance(v, (int, float))
        and not isinstance(v, bool)
        and math.isfinite(v)
        and -1.0 <= v <= 1.0
    )


def _valid(msg) -> str | None:
    """Reason a message is malformed, or None (contract 1; plan Task 2 rule 0)."""
    if not isinstance(msg, dict) or not isinstance(msg.get("t"), str):
        return "invalid"
    t = msg["t"]
    if t in RELIABLE:
        rid = msg.get("request_id")
        if not isinstance(rid, str) or not rid:
            return "invalid"
    for key in INT_FIELDS:
        if key in msg and not _is_int(msg[key]):
            return "invalid"
    if t == "teleop":
        if not all(_is_int(msg.get(k)) for k in INT_FIELDS):
            return "invalid"
        if not isinstance(msg.get("lease_id"), str):
            return "invalid"
        if not _is_ratio(msg.get("speed_ratio")) or not _is_ratio(
            msg.get("curvature_ratio")
        ):
            return "invalid"
        if msg.get("lift_dir") not in (-1, 0, 1) or isinstance(
            msg.get("lift_dir"), bool
        ):
            return "invalid"
        if msg.get("mode") not in ("direct", "video") or not isinstance(
            msg.get("deadman"), bool
        ):
            return "invalid"
        frame = msg.get("frame_echo")
        if frame is not None and (
            not isinstance(frame, dict) or not _is_int(frame.get("frame_id"))
        ):
            return "invalid"
    if t in ("teleop", "echo"):
        echo = msg.get("echo")
        if not isinstance(echo, dict) or not _is_int(echo.get("state_seq")):
            return "invalid"
        if not isinstance(echo.get("token"), str):
            return "invalid"
    return None


class MockVehicle:
    def __init__(
        self,
        clock: Callable[[], float],
        token: Callable[[], str],
        config: Config | None = None,
        world: World | None = None,
        session_epoch: int | None = None,
    ):
        self.clock, self.token, self.cfg = clock, token, config or Config()
        self.world = world or default_world()
        # random 48-bit (JSON-safe) per instance: a restarted fake vehicle never
        # reuses an epoch (contract 1)
        self.session_epoch = (
            session_epoch if session_epoch is not None else secrets.randbits(48) | 1
        )
        self.lease = None  # {"client_id", "conn_id", "lease_id", "last_sent"}
        self.reserved = None  # {"client_id", "until"}
        self.control_gen = 1
        self.arm_gen = 0
        self.armed = False
        self.mode = "PAUSED"
        self.stop_kind = "controlled_decel"
        self.state_seq = 0
        self._sent: OrderedDict[int, tuple[str, float]] = OrderedDict()
        self._frames: OrderedDict[int, float] = OrderedDict()
        self._results: dict = {}
        self._teleop = None
        self._last_seq = 0
        self.last_teleop_reject = None
        self.pose = self.world.home
        self.applied_speed_mps = 0.0
        self.applied_curvature_inv_m = 0.0
        self.command = {"speed_mps": 0.0, "curvature_inv_m": 0.0}
        self.lift_m = 0.0
        self.lift_moving = False
        self.mission = None
        self.missions_created = 0
        self.targets_revision = 1
        self.plan_revision = 0
        self.forks_in_pocket = False
        self.permit = (0.5, "ok")
        self.drive_allowed = None
        self.lift_allowed = None
        self.stopped_confirmed = False
        self._still_since = None
        self._last_nonzero_sign = 1
        self.last_permission_direction = 1
        self.clients: dict[str, set] = {}
        self.events: list[dict] = []
        # persistent console plan D13 item 8: as the live runner -- the shared request ledger, the results book, the
        # scene latch, the session requests, and the states the screen's D13 cases need (``inject``)
        self.ledger = RequestLedger()
        self.results = ResultsBook()
        self.scene = {"damaged": None}
        self.session_request = None  # "end" | "reset", taken by the server
        self.session_ending = False  # accepted: every later leased request is refused session_ending
        self._policy_override = None  # injected recovery policy fields (lift_dirs, drive_mode, ...)
        self.recovery = None
        self._permit_override = None
        self._booked = None
        self.at: dict[str, str] = {}  # plan D3: where each pallet is verified to stand (set at a delivery here)

    # ------------------------------------------------------------------ helpers
    def _now(self) -> float:
        return self.clock()

    def _event(self, kind: str, **fields) -> None:
        self.events.append({"t": "event", "kind": kind, **fields})
        del self.events[:-100]

    def _phase(self) -> str | None:
        m = self.mission
        if m is None or m["status"] in ("done", "cancelled", "failed"):
            return None
        return m["phase"]

    def _load(self) -> Pallet | None:
        for p in self.world.pallets:
            if p.state == "carried":
                return p
        return None

    def _boundary(self) -> None:
        """Stop/permission boundary (contract 1, control_gen; design 5.1)."""
        self.armed = False
        self._teleop = None
        self.control_gen += 1

    def _set_mode(self, new: str) -> None:
        if new != self.mode:
            self.mode = new
            self._boundary()
        self.stop_kind = {
            "PAUSED": "controlled_decel",
            "STOPPED": "safety_preempt",
        }.get(new, "none")

    def _hold_mission(self, status: str, reason: str) -> None:
        m = self.mission
        if m is not None and m["status"] in ("running", "resuming"):
            m["status"], m["reason"] = status, reason

    def _stop(self, source: str) -> None:
        if self.mode == "STOPPED":
            self._boundary()  # a new stop is a boundary even in the same mode
        else:
            self._set_mode("STOPPED")
        self._hold_mission("held", "stop")
        self._event("stop", source=source)

    def _safety_stop(self, reason: str) -> None:
        """Expiry or lost lease while in TELEOP: cut output, disarm, keep the mode."""
        self.stop_kind = "safety_preempt"
        self._boundary()
        self._event("safety_stop", reason=reason)

    def _lease_lost(self) -> None:
        if self.mode == "TELEOP":
            self._safety_stop("lease_lost")
        else:
            self.control_gen += (
                1  # AUTO keeps working (user decision); authority boundary
            )
            self._teleop = None

    def _same_conn(self, client_id: str, conn_id) -> bool:
        return (
            self.lease is not None
            and self.lease["client_id"] == client_id
            and self.lease["conn_id"] == conn_id
        )

    def _echo_sent(self, echo) -> float | None:
        sent = self._sent.get(echo.get("state_seq"))
        if sent is None or sent[0] != echo.get("token"):
            return None
        return sent[1]

    def _teleop_reason(self) -> str | None:
        phase = self._phase()
        if self.mode == "STOPPED":
            return "stopped_locked"
        if phase in TELEOP_FORBIDDEN or self.forks_in_pocket:
            return "state_forbids_teleop"
        return None

    # --------------------------------------------------------------- connection
    def connect(self, client_id: str, conn_id: str | None = None) -> dict:
        self.clients.setdefault(client_id, set()).add(conn_id)
        token = self.ledger.issue() if self.cfg.request_tokens else None
        registered = token is not None and self.ledger.register(token, client_id, conn_id, self._now())
        caps = {
            "mission": True,
            "teleop_direct": True,
            "teleop_video": True,
            "lift": True,
            "recovery_actions": ["withdraw_straight"],
            "feedback": {
                "rear_axle_speed": True,
                "steering": True,
                "lift_height": True,
                "load_state": True,
            },
            "media": {
                "camera_rgb": {"max_hz": 15, "max_width": CAMERA_MODEL["width"]},
                "scan": {"max_hz": 10},
                "grid": {"max_hz": 1},
            },
            "camera_model": CAMERA_MODEL,
            "chassis": {
                "rear_overhang_m": CHASSIS.rear_overhang_m,
                "nose_from_rear_m": CHASSIS.nose_from_rear_m,
                "half_width_m": CHASSIS.half_width_m,
                "wheelbase_m": CHASSIS.wheelbase_m,
                "max_curvature_inv_m": CHASSIS.max_curvature_inv_m,
                "model_turning_radius_m": 1.0 / CHASSIS.max_curvature_inv_m,
            },
            "provisional_limits": True,
            "sim_reset": True,  # a scene reset starts a new session (D13; the live runner's when supervised)
        }
        welcome = {
            "t": "welcome",
            "api": API,
            "vehicle_id": "mock-1",
            "session_epoch": self.session_epoch,
            "backend": "mock",
            "capabilities": caps,
        }
        if registered:
            welcome["request_token"] = token
        return welcome

    def disconnect(self, client_id: str, conn_id: str | None = None) -> None:
        token = self.ledger.token_of(client_id, conn_id)
        if token is not None:
            self.ledger.retire(token)
        conns = self.clients.get(client_id)
        if conns is not None:
            conns.discard(conn_id)
            if not conns:
                del self.clients[client_id]
        if self._same_conn(client_id, conn_id):
            self._release_lease(reserve=True)

    def _release_lease(self, reserve: bool) -> None:
        holder = self.lease["client_id"]
        self.lease = None
        self.reserved = (
            {"client_id": holder, "until": self._now() + self.cfg.grace_s}
            if reserve
            else None
        )
        self._lease_lost()

    # ------------------------------------------------------------------ outbound
    def make_tick(self) -> dict:
        now = self._now()
        self.state_seq += 1
        tok = self.token()
        self._sent[self.state_seq] = (tok, now)
        keep = max(self.cfg.command_max_age_s, self.cfg.lease_timeout_s) + 0.5
        while self._sent and now - next(iter(self._sent.values()))[1] > keep:
            self._sent.popitem(last=False)
        return {
            "t": "tick",
            "session_epoch": self.session_epoch,
            "state_seq": self.state_seq,
            "token": tok,
        }

    def record_frame(self, frame_id: int) -> None:
        now = self._now()
        self._frames[frame_id] = now
        while len(self._frames) > 64:
            self._frames.popitem(last=False)

    def make_targets(self) -> dict:
        return {
            "t": "targets",
            "targets_revision": self.targets_revision,
            "map_frame": "map",
            "correction_version": 1,
            "pallets": [
                {
                    "pallet_id": p.pallet_id,
                    "kind": p.kind,
                    "pose": None if p.state == "carried" else asdict(p.pose),
                    "depth_m": p.depth_m,
                    "width_m": p.width_m,
                    "state": p.state,
                }
                for p in self.world.pallets
            ],
            "destinations": [
                {"destination_id": k, "pose": asdict(v)}
                for k, v in self.world.destinations.items()
            ],
        }

    def make_state(self) -> dict:
        self.state_seq += 1
        load = self._load()
        m = self.mission
        scale = self.cfg.loaded_scale if load else 1.0
        teleop_reason = self._teleop_reason()
        mission = None
        if m is not None:
            mission = {
                "mission_id": m["mission_id"],
                "status": m["status"],
                "phase": m["phase"],
                "reason": m.get("reason"),
                "pallet_id": m["pallet_id"],
                "destination_id": m["destination_id"],
                "phase_elapsed_s": round(m["phase_t"], 2),
                "resume_steps": m.get("resume_steps"),
            }
        return {
            "t": "state",
            "state_seq": self.state_seq,
            "session_epoch": self.session_epoch,
            "mode": self.mode,
            "stop_kind": self.stop_kind,
            "control_gen": self.control_gen,
            "armed": self.armed,
            "arm_gen": self.arm_gen,
            "lease": {
                "holder": self.lease["client_id"] if self.lease else None,
                "observers": max(
                    0,
                    sum(len(c) for c in self.clients.values())
                    - (1 if self.lease else 0),
                ),
                "reserved_until_s": (
                    round(self.reserved["until"] - self._now(), 2)
                    if self.reserved
                    else None
                ),
            },
            "pose": {
                "frame": "map",
                "x_m": self.pose.x_m,
                "y_m": self.pose.y_m,
                "yaw_rad": self.pose.yaw_rad,
                "correction_version": 1,
                "source": "mock_truth",
            },
            "rear_axle_speed_mps": self.applied_speed_mps,
            "steering_rad": [
                math.atan(CHASSIS.wheelbase_m * self.applied_curvature_inv_m)
            ]
            * 2,
            "lift_m": self.lift_m,
            "load": {
                "state": "loaded" if load else "empty",
                "pallet_id": load.pallet_id if load else None,
            },
            "command": dict(self.command),
            "applied": {
                "speed_mps": self.applied_speed_mps,
                "curvature_inv_m": self.applied_curvature_inv_m,
                "stop_kind": self.stop_kind,
            },
            "permit": (
                {"limit_mps": self._permit_override[0], "reason": self._permit_override[1]}
                if self._permit_override
                else {"limit_mps": self.permit[0], "reason": self.permit[1]}
            ),
            "limits": {
                "forward_mps": self.cfg.forward_mps * scale,
                "reverse_mps": self.cfg.reverse_mps * scale,
                "curvature_inv_m": CHASSIS.teleop_curvature_inv_m,
                "provisional": True,
            },
            "drive_allowed": self.drive_allowed if self.mode == "TELEOP" else None,
            "lift_allowed": self.lift_allowed if self.mode == "TELEOP" else None,
            "mission": mission,
            "freshness": {
                "grid_free_age_s": None,
                "slam_correction_age_s": None,
                "near_obs_age_s": None,
            },
            "pacing": None,
            "stopped_confirmed": self.stopped_confirmed,
            "feedback_ok": True,  # the fake vehicle always measures its speed
            "last_result": self.results.last,
            "scene": {"damaged": dict(self.scene["damaged"]) if self.scene["damaged"] else None},
            "recovery": dict(self.recovery) if self.recovery else None,
            "teleop_allowed": teleop_reason is None,
            "teleop_forbidden_reason": teleop_reason,
            "lift_dirs": [-1, 1] if self.lift_allowed else [],
            "drive_mode": "normal",
            "stand_waiver": None,
            **(self._policy_override or {}),
            "plan": self._plan(),
        }

    def _plan(self) -> dict | None:
        m = self.mission
        if m is None or not m["legs"]:
            return None
        poses = []
        for leg in m["legs"][m["leg_index"] :]:
            start = leg.index if leg is m["legs"][m["leg_index"]] else 0
            for x, y, _yaw, _s, _k in leg.points[start::10]:
                poses.append([round(x, 3), round(y, 3), leg.direction])
        return {"revision": self.plan_revision, "poses": poses}

    # ------------------------------------------------------------------- inbound
    def handle(
        self, client_id: str, msg: dict, conn_id: str | None = None
    ) -> dict | None:
        reason = _valid(msg)
        t = msg.get("t") if isinstance(msg, dict) else None
        if reason is not None:
            if t in RELIABLE and isinstance(msg.get("request_id"), str):
                return self._reply(msg, reason=reason)
            if t in RELIABLE:
                return {"t": "reject", "request_id": None, "reason": reason}
            self.last_teleop_reject = reason
            return None
        if t == "stop":  # contract 1, rule 0: always executed, never cached
            if client_id not in self.clients:
                return self._reply(msg, reason="invalid")
            self._stop(msg.get("source", "console"))
            return self._reply(msg)
        if t in LATEST_ONLY:
            self._latest(client_id, msg, conn_id)
            return None
        if t not in RELIABLE:
            return {
                "t": "reject",
                "request_id": msg.get("request_id"),
                "reason": "unsupported",
            }
        if self.cfg.request_tokens or "request_token" in msg:
            return self._handle_ledger(client_id, msg, conn_id)
        key = (client_id, msg.get("session_epoch"), msg["request_id"])
        if key in self._results:  # contract 1, rule 1
            return self._results[key]
        if len(self._results) >= self.cfg.result_cache_max:
            return self._reply(msg, reason="busy")  # not stored
        result = self._reliable(client_id, msg, conn_id)
        self._results[key] = result
        return result

    def _handle_ledger(self, client_id: str, msg: dict, conn_id) -> dict:
        """Persistent console plan D7 (contract 1, the request ledger), as the live arbiter: admitted by the
        connection's token and seq before it runs; a forgotten request is stale, never run again."""
        token, seq = msg.get("request_token"), msg.get("request_seq")
        if not (_is_int(token) and _is_int(seq)):
            return self._reply(msg, reason="invalid")
        if msg.get("session_epoch") != self.session_epoch:
            return self._reply(msg, reason="stale_epoch")  # before the ledger, unstored
        verdict, stored = self.ledger.admit(token, client_id, conn_id, seq)
        if verdict == "stored":
            return stored
        if verdict in ("stale", "busy"):
            return self._reply(msg, reason=verdict)
        result = self._reliable(client_id, msg, conn_id)
        self.ledger.put(token, seq, result, self._now())
        return result

    @staticmethod
    def _reply(msg: dict, reason: str | None = None, **extra) -> dict:
        if reason is None:
            return {"t": "ack", "request_id": msg.get("request_id"), **extra}
        return {
            "t": "reject",
            "request_id": msg.get("request_id"),
            "reason": reason,
            **extra,
        }

    def _reliable(self, client_id: str, msg: dict, conn_id) -> dict:
        t = msg["t"]
        if msg.get("session_epoch") != self.session_epoch:
            return self._reply(msg, reason="stale_epoch")
        if t == "lease_request":
            return self._lease_request(client_id, msg, conn_id)
        if t == "targets_request":
            return self._reply(msg, targets=self.make_targets())
        # LEASE_REQUIRED (contract 1, rule 2)
        if self.lease is None or self.lease["client_id"] != client_id:
            return self._reply(msg, reason="no_lease")
        if self.lease["conn_id"] != conn_id:
            return self._reply(msg, reason="no_lease")
        if msg.get("lease_id") != self.lease["lease_id"]:
            return self._reply(msg, reason="stale_lease")
        if msg.get("control_gen") != self.control_gen:
            return self._reply(msg, reason="stale_control")
        if self.session_ending:
            return self._reply(msg, reason="session_ending")  # the session closes (as the live arbiter, D7)
        if self.mode == "STOPPED" and t not in STOPPED_ALLOWED:
            return self._reply(msg, reason="stopped_locked")
        handler = getattr(self, "_req_" + t)
        return handler(msg)

    def _lease_request(self, client_id: str, msg: dict, conn_id) -> dict:
        if self.lease is not None:
            return self._reply(msg, reason="busy")
        if self.reserved is not None and self.reserved["client_id"] != client_id:
            return self._reply(msg, reason="lease_reserved")
        self.reserved = None
        self.lease = {
            "client_id": client_id,
            "conn_id": conn_id,
            "lease_id": self.token(),
            "last_sent": self._now(),
        }
        self._event("lease", holder=client_id)
        return self._reply(
            msg,
            lease_id=self.lease["lease_id"],
            lease_timeout_s=self.cfg.lease_timeout_s,
        )

    def _req_lease_release(self, msg: dict) -> dict:
        self._release_lease(reserve=False)
        return self._reply(msg)

    def _req_mode_request(self, msg: dict) -> dict:
        to = msg.get("to")
        if to == "PAUSED":
            if self.mode == "PAUSED":
                self._boundary()
            else:
                self._hold_mission("paused", "operator")
                self._set_mode("PAUSED")
            return self._reply(msg)
        if to == "TELEOP":
            if self.mode == "TELEOP":
                return self._reply(msg)  # same mode: nothing changes, seq order kept
            reason = self._teleop_reason()
            if reason is not None:
                return self._reply(msg, reason=reason)
            self._hold_mission("held", "teleop")
            self._set_mode("TELEOP")
            self._last_seq = 0
            return self._reply(msg)
        return self._reply(msg, reason="invalid")

    def _req_arm(self, msg: dict) -> dict:
        if self.mode != "TELEOP":
            return self._reply(msg, reason="invalid")
        self.arm_gen += 1
        self.armed = True
        self.stop_kind = "none"
        self._last_seq = 0
        self._teleop = None
        return self._reply(msg, arm_gen=self.arm_gen)

    def _req_disarm(self, msg: dict) -> dict:
        self.armed = False
        self._teleop = None
        return self._reply(msg)

    def _req_rearm(self, msg: dict) -> dict:
        if self.mode != "STOPPED":
            return self._reply(msg, reason="invalid")
        if not self.stopped_confirmed:
            return self._reply(msg, reason="not_stopped")
        self._set_mode("PAUSED")
        return self._reply(msg)

    # --------------------------------------------------------------- mission API
    def _req_mission_submit(self, msg: dict) -> dict:
        self._book_end(self._now())  # the ended mission is booked before a new one replaces it
        if self.scene["damaged"] is not None:
            return self._reply(msg, reason="scene_damaged")  # automatic work waits for a scene reset (D6)
        if msg.get("targets_revision") != self.targets_revision:
            return self._reply(msg, reason="stale_targets")
        pallet = next(
            (p for p in self.world.pallets if p.pallet_id == msg.get("pallet_id")), None
        )
        dest = self.world.destinations.get(msg.get("destination_id"))
        if pallet is None or dest is None or pallet.state not in ("available", "delivered"):
            return self._reply(msg, reason="invalid")  # delivered: available again where it lies (plan D3)
        if self.at.get(pallet.pallet_id) == msg.get("destination_id"):
            return self._reply(msg, reason="already_there")
        if self._occupied(pallet, dest):
            return self._reply(msg, reason="destination_occupied")
        if self.forks_in_pocket:  # a new job starts with an empty observe approach
            return self._reply(msg, reason="fork_in_pocket")
        if self._load() is not None:
            return self._reply(msg, reason="load_present")
        if self.mission is not None and self.mission["status"] in (
            "running",
            "paused",
            "held",
            "resuming",
        ):
            return self._reply(msg, reason="busy")
        self.missions_created += 1
        self.mission = {
            "mission_id": f"M-{self.missions_created:04d}",
            "pallet_id": pallet.pallet_id,
            "destination_id": msg["destination_id"],
            "status": "running",
            "reason": None,
            "phase_index": -1,
            "phase": None,
            "phase_t": 0.0,
            "legs": [],
            "leg_index": 0,
        }
        self._next_phase()
        self._set_mode("AUTO")
        self._event("mission", mission_id=self.mission["mission_id"], status="accepted")
        return self._reply(msg, mission_id=self.mission["mission_id"])

    def _occupied(self, pallet, dest) -> bool:
        """Plan D3, as the targets book: another pallet on the floor (its outline grown 0.21 m) overlapping the
        submitted one placed at the destination."""
        from forklift_core.safety.pallet_permit import obb_overlap  # noqa: PLC0415

        for other in self.world.pallets:
            if other is pallet or other.state == "carried":
                continue
            grow = 0.21
            if obb_overlap([[other.pose.x_m, other.pose.y_m]],
                           (other.depth_m / 2 + grow, other.width_m / 2 + grow), [other.pose.yaw_rad],
                           (dest.x_m, dest.y_m), (pallet.depth_m / 2, pallet.width_m / 2), dest.yaw_rad)[0]:
                return True
        return False

    def _mission_matches(self, msg: dict) -> bool:
        return (
            self.mission is not None
            and msg.get("mission_id") == self.mission["mission_id"]
        )

    def _req_mission_pause(self, msg: dict) -> dict:
        if not self._mission_matches(msg):
            return self._reply(msg, reason="invalid")
        self._hold_mission("paused", "operator")
        if self.mode in ("PAUSED", "STOPPED"):
            self._boundary()
        else:
            self._set_mode("PAUSED")
        return self._reply(msg)

    def _req_mission_cancel(self, msg: dict) -> dict:
        if not self._mission_matches(msg):
            return self._reply(msg, reason="invalid")
        # load, lift and fork state are physical: cancelling the job does not change them
        self.mission["status"] = "cancelled"
        self.mission["legs"] = []
        if self.mode != "STOPPED":
            self._set_mode("PAUSED")
        return self._reply(msg)

    def _req_mission_resume(self, msg: dict) -> dict:
        if self.scene["damaged"] is not None:
            return self._reply(msg, reason="scene_damaged")
        m = self.mission
        if not self._mission_matches(msg) or m["status"] not in ("paused", "held"):
            return self._reply(msg, reason="invalid")
        steps = []
        ok = True
        if not self.stopped_confirmed:
            steps.append(
                {
                    "name": "stop_confirmed",
                    "result": "failed",
                    "reason": "not_stopped_timeout",
                }
            )
            ok = False
        else:
            steps.append({"name": "stop_confirmed", "result": "ok"})
        if ok and self.forks_in_pocket:  # only recovery actions with forks in a pocket
            steps.append(
                {
                    "name": "state_verified",
                    "result": "failed",
                    "reason": "fork_in_pocket",
                }
            )
            ok = False
        elif ok:
            steps.append({"name": "state_verified", "result": "ok"})
        if ok:
            steps.append({"name": "frame_reexpressed", "result": "ok"})
            self._replan()
            blocked = self._first_leg_blocked()
            if blocked is None:
                steps.append({"name": "path_rechecked", "result": "ok"})
            else:
                steps.append(
                    {
                        "name": "path_rechecked",
                        "result": "failed",
                        "reason": f"path_blocked:{blocked}",
                    }
                )
                ok = False
        m["resume_steps"] = steps
        if not ok:
            return self._reply(msg, reason="resume_failed", resume_steps=steps)
        m["status"], m["reason"] = "running", None
        self._set_mode("AUTO")
        return self._reply(msg, resume_steps=steps)

    def _first_leg_blocked(self) -> str | None:
        """path_rechecked: every remaining leg, sampled along its length (plan rule 14)."""
        m = self.mission
        skip = tuple(p.pallet_id for p in self.world.pallets if self._in_pocket_with(p))
        load = self._load()
        for k, leg in enumerate(m["legs"][m["leg_index"] :]):
            start = leg.index if k == 0 else 0
            pts = leg.points[start::5] + [leg.points[-1]]
            for x, y, yaw, _s, kappa in pts:
                limit, reason = permission(
                    self.world,
                    Pose(x, y, yaw),
                    leg.speed_mps * leg.direction,
                    kappa,
                    load,
                    skip,
                )
                if limit == 0.0:
                    return reason
        return None

    # ---------------------------------------------- persistent console plan D13 (as the live runner)
    FINAL = ("done", "failed", "cancelled", "recovery_required")
    ACTIVE = ("accepted", "preparing", "running", "paused", "held", "resuming", "recovering")

    def make_missions(self) -> dict:
        return self.results.message()

    def _book(self, *, mission_id, outcome, reason, started_s, pallet_id, destination_id, advice=None) -> dict:
        entry = self.results.add(mission_id=mission_id, outcome=outcome, reason=reason, started_s=started_s,
                                 ended_s=self._now(), pallet_id=pallet_id, destination_id=destination_id,
                                 advice=advice)
        self.events.append(self.make_missions())
        return entry

    def _book_end(self, now: float) -> None:
        """A mission that reached a final status is booked once and published (contract 5.2)."""
        m = self.mission
        if m is None or m["status"] not in self.FINAL or self._booked == m["mission_id"]:
            return
        self._booked = m["mission_id"]
        reason = m.get("reason") or ("operator" if m["status"] == "cancelled" else None)
        self._book(mission_id=m["mission_id"], outcome=m["status"], reason=reason, started_s=m.get("started_t"),
                   pallet_id=m["pallet_id"], destination_id=m["destination_id"], advice=m.get("advice"))

    def _session_request(self, msg: dict, kind: str) -> dict:
        # the arbiter's order after the lease: the feedback (always measured here), the stand, no mission on
        if not self.stopped_confirmed:
            return self._reply(msg, reason="not_stopped")
        if self.mission is not None and self.mission["status"] in self.ACTIVE:
            return self._reply(msg, reason="mission_active")
        self.session_request = kind
        self.session_ending = True
        if self.mode in ("AUTO", "TELEOP"):
            self._set_mode("PAUSED")
        else:
            self._boundary()
        return self._reply(msg)

    def _req_session_end(self, msg: dict) -> dict:
        return self._session_request(msg, "end")

    def _req_sim_reset(self, msg: dict) -> dict:
        return self._session_request(msg, "reset")

    def reset_session(self) -> None:
        """The scene back to its start under a new session epoch (the live runner's supervisor restarts it)."""
        self.__init__(self.clock, self.token, self.cfg)

    def inject(self, kind: str) -> None:
        """The screen's D13 cases without driving into them (fake vehicle only, for the page's tests)."""
        now = self._now()
        if kind == "scene_damage":
            if self.scene["damaged"] is None:
                self.scene["damaged"] = {"reason": "Forbidden fork/pallet contact", "t": now, "unix": time.time()}
            return
        self.missions_created += 1
        mission_id = f"M-{self.missions_created:04d}"
        if kind == "recovery_required":
            self.recovery = {"stage": "in_pocket", "grounded": False, "evidence": True, "aligned": True,
                             "pallet_id": "P-01"}
            # as the live state sends a lifted, pressed pocket: lower only, the stand waived (plan D14)
            self._policy_override = {"lift_dirs": [-1], "drive_mode": "normal", "stand_waiver": "lower_only",
                                     "teleop_forbidden_reason": "recovery:lower_first", "teleop_allowed": True}
            self._book(mission_id=mission_id, outcome="recovery_required", reason="recovery:blocked:test",
                       started_s=now, pallet_id="P-01", destination_id="D-A")
        elif kind == "steering_wait":
            self.recovery = {"stage": "in_pocket", "grounded": True, "evidence": True, "aligned": True,
                             "pallet_id": "P-01"}
            self._permit_override = (0.0, "recovery:steering")
            self._policy_override = {"lift_dirs": [], "drive_mode": "reverse_straight", "stand_waiver": None,
                                     "teleop_forbidden_reason": None, "teleop_allowed": True}
            self.missions_created -= 1
        elif kind == "invalid_start":
            self._book(mission_id=mission_id, outcome="failed",
                       reason="prepare:all_candidates_failed:invalid_start;invalid_start", started_s=now,
                       pallet_id="P-01", destination_id="D-A",
                       advice={"backoff_m": 0.15, "backoff_max_m": 0.6,
                               "from_rear": [self.pose.x_m, self.pose.y_m, self.pose.yaw_rad]})
        else:
            self.missions_created -= 1
            raise ValueError(f"unknown injection {kind!r}")

    def _req_recovery(self, msg: dict) -> dict:
        if msg.get("action") != "withdraw_straight" or not self.forks_in_pocket:
            return self._reply(msg, reason="invalid")
        back = CHASSIS.fork_length_m + 0.1
        p0 = self.pose
        p1 = Pose(
            p0.x_m - back * math.cos(p0.yaw_rad),
            p0.y_m - back * math.sin(p0.yaw_rad),
            p0.yaw_rad,
        )
        m = self.mission
        m["before_recovery"] = m["status"]
        m["phase"], m["phase_t"] = "recover", 0.0
        m["legs"], m["leg_index"] = [paths.hermite_leg(p0, p1, -1, 0.1)], 0
        m["status"], m["reason"] = "running", "recovery"
        self.plan_revision += 1
        self._set_mode("AUTO")
        return self._reply(msg)

    # ------------------------------------------------------------- latest-only
    def _latest(self, client_id: str, msg: dict, conn_id) -> None:
        t = msg["t"]
        holder = self._same_conn(client_id, conn_id)
        if t == "input_state":
            if holder and (
                msg.get("visible") is False
                or msg.get("focused") is False
                or msg.get("gamepad_lost") is True
            ):
                self.armed = False
                self._teleop = None
                self._event("disarm", reason="input_state")
            return
        if msg.get("session_epoch") != self.session_epoch:
            self.last_teleop_reject = "stale_epoch"
            return
        if not holder:
            self.last_teleop_reject = "no_lease"
            return
        if msg.get("lease_id") != self.lease["lease_id"]:
            self.last_teleop_reject = "stale_lease"
            return
        sent = self._echo_sent(msg["echo"])
        if t == "echo":
            if sent is not None:
                self.lease["last_sent"] = max(self.lease["last_sent"], sent)
            return
        self._teleop_command(msg, sent)

    def _teleop_command(self, msg: dict, sent: float | None) -> None:
        now = self._now()
        if msg["control_gen"] != self.control_gen:
            self.last_teleop_reject = "stale_control"
            return
        if msg["arm_gen"] != self.arm_gen:
            self.last_teleop_reject = "stale_arm"
            return
        if msg["seq"] <= self._last_seq:
            self.last_teleop_reject = "stale_seq"
            return
        if not self.armed or self.mode != "TELEOP":
            self.last_teleop_reject = "not_armed"
            return
        if sent is None:
            self.last_teleop_reject = "bad_echo"
            return
        deadline = sent + self.cfg.command_max_age_s  # contract 4.1
        if msg["mode"] == "video":  # contract 4.2
            frame = msg.get("frame_echo") or {}
            acquired = self._frames.get(frame.get("frame_id"))
            if acquired is None:
                self.last_teleop_reject = "stale_video"
                return
            deadline = min(
                deadline,
                acquired + self.cfg.video_max_age_s - self.cfg.camera_latency_bound_s,
            )
        if now >= deadline:
            self.last_teleop_reject = "expired"
            return
        self.last_teleop_reject = None
        self._last_seq = msg["seq"]
        self.lease["last_sent"] = max(self.lease["last_sent"], sent)
        self._teleop = {
            "msg": msg,
            "deadline": deadline,
            "gens": (
                self.session_epoch,
                self.lease["lease_id"],
                self.control_gen,
                self.arm_gen,
            ),
        }

    # ------------------------------------------------------------------ control
    def step(self, dt_s: float) -> None:
        now = self._now()
        self.ledger.purge(now)  # D7: once a cycle, as the live arbiter
        self._book_end(now)
        if (
            self.lease is not None
            and now - self.lease["last_sent"] > self.cfg.lease_timeout_s
        ):
            self._event("lease_expired", holder=self.lease["client_id"])
            self._release_lease(reserve=True)
        if self.reserved is not None and now > self.reserved["until"]:
            self.reserved = None
        tp = self._teleop
        if tp is not None:
            current = (
                self.session_epoch,
                self.lease["lease_id"] if self.lease else None,
                self.control_gen,
                self.arm_gen,
            )
            if tp["gens"] != current or not self.armed:
                self._teleop = None
            elif now >= tp["deadline"]:
                self._teleop = None
                self._safety_stop("expired")
        if self.mode == "AUTO":
            self._step_mission(dt_s)
        else:
            self._step_manual(dt_s)
        self._update_still(dt_s)

    def _update_still(self, dt_s: float) -> None:
        now = self._now()
        if abs(self.applied_speed_mps) < self.cfg.stop_threshold_mps:
            if self._still_since is None:
                self._still_since = now
            if now - self._still_since >= self.cfg.stop_hold_s - 1e-9:
                self.stopped_confirmed = True
        else:
            self._still_since = None
            self.stopped_confirmed = False

    def _caps(self, load: Pallet | None) -> tuple[float, float]:
        scale = self.cfg.loaded_scale if load else 1.0
        return self.cfg.forward_mps * scale, self.cfg.reverse_mps * scale

    def _step_manual(self, dt_s: float) -> None:
        load = self._load()
        phase = self._phase()
        msg = self._teleop["msg"] if self._teleop else None
        lift_dir = msg["lift_dir"] if msg else 0
        target, curvature = 0.0, 0.0
        if self.mode == "TELEOP":
            lift_active = lift_dir != 0 or self.lift_moving
            forbidden = phase in TELEOP_FORBIDDEN or self.forks_in_pocket
            self.drive_allowed = not forbidden and not lift_active
            self.lift_allowed = phase not in NO_MANUAL_LIFT and self.stopped_confirmed
            if msg is not None and msg["deadman"] and self.armed:
                fwd, rev = self._caps(load)
                ratio = msg["speed_ratio"]
                target = ratio * (fwd if ratio >= 0 else rev)
                curvature = msg["curvature_ratio"] * CHASSIS.teleop_curvature_inv_m
        else:
            self.drive_allowed = self.lift_allowed = None
        # command = the requested physical values; applied shows what the gates let through
        self.command = {"speed_mps": target, "curvature_inv_m": curvature}
        if self.mode == "TELEOP" and not self.drive_allowed:
            target = 0.0
        self._output(target, curvature, dt_s)
        self._integrate(dt_s)
        moved = False
        if (
            self.mode == "TELEOP"
            and msg is not None
            and msg["deadman"]
            and self.armed
            and self.lift_allowed
            and lift_dir != 0
        ):
            new = min(
                LIFT_TOP_M, max(0.0, self.lift_m + lift_dir * LIFT_RATE_MPS * dt_s)
            )
            moved = new != self.lift_m
            self.lift_m = new
        self.lift_moving = moved
        self._carry()

    def debug_output(
        self,
        target_mps: float,
        curvature_inv_m: float,
        permission_cap_mps: float,
        dt_s: float,
    ) -> float:
        """Test entry: the output order with an explicit permission cap."""
        return self._output(target_mps, curvature_inv_m, dt_s, cap=permission_cap_mps)

    def _output(
        self, target: float, curvature: float, dt_s: float, cap: float | None = None
    ) -> float:
        """ratio -> caps -> curvature limit -> slew -> final permission cap -> preempt (design 4.4)."""
        lim = CHASSIS.teleop_curvature_inv_m
        curvature = max(-lim, min(lim, curvature))
        prev = self.applied_speed_mps
        goal = target
        if prev * target < 0 or (
            not self.stopped_confirmed and target * self._last_nonzero_sign < 0
        ):
            goal = (
                0.0  # reversing: stop and confirm the stop before moving the other way
            )
        rate = (
            self.cfg.decel_mps2 if abs(goal) < abs(prev) else self.cfg.accel_mps2
        ) * dt_s
        out = prev + max(-rate, min(rate, goal - prev))
        if prev != 0.0:
            self._last_nonzero_sign = 1 if prev > 0 else -1
        if cap is None:
            # actual motion first: any residual speed decides, then the last motion until
            # the stop is confirmed, and only a confirmed stop lets the request choose
            if prev != 0.0:
                direction = 1 if prev > 0 else -1
            elif not self.stopped_confirmed:
                direction = self._last_nonzero_sign
            else:
                direction = 1 if target >= 0 else -1
            self.last_permission_direction = direction
            probe = max(abs(out), abs(prev)) * direction
            if probe == 0.0:
                probe = 1e-6 * direction
            skip = tuple(
                p.pallet_id for p in self.world.pallets if self._in_pocket_with(p)
            )
            self.permit = permission(
                self.world, self.pose, probe, curvature, self._load(), skip
            )
            cap = self.permit[0]
        if abs(out) > cap:
            out = math.copysign(cap, out)
        if (
            self.mode == "TELEOP" and not self.drive_allowed
        ) or self.stop_kind == "safety_preempt":
            out = 0.0
        self.applied_speed_mps = out
        self.applied_curvature_inv_m = curvature
        if out != 0.0:
            self._last_nonzero_sign = 1 if out > 0 else -1
        return out

    def _in_pocket_with(self, pallet: Pallet) -> bool:
        m = self.mission
        return (
            m is not None
            and self.mode != "TELEOP"  # manual driving always sees the pallet
            and m["status"] in ("running", "paused", "held", "resuming")
            and m["pallet_id"] == pallet.pallet_id
            and m["phase"]
            in ("approach", "insert", "extract", "recover", "withdraw", "return")
        )

    def _integrate(self, dt_s: float) -> None:
        v, k = self.applied_speed_mps, self.applied_curvature_inv_m
        p = self.pose
        w, h = self.world.size_m
        x = min(w - 0.5, max(0.5, p.x_m + v * math.cos(p.yaw_rad) * dt_s))
        y = min(h - 0.5, max(0.5, p.y_m + v * math.sin(p.yaw_rad) * dt_s))
        yaw = math.atan2(
            math.sin(p.yaw_rad + v * k * dt_s), math.cos(p.yaw_rad + v * k * dt_s)
        )
        self.pose = Pose(x, y, yaw)

    def _carry(self) -> None:
        load = self._load()
        if load is not None:
            load.pose = self._carried_pose(load)

    def _carried_pose(self, p: Pallet) -> Pose:
        off = loaded_nose_m(p) - p.depth_m / 2
        q = self.pose
        return Pose(
            q.x_m + off * math.cos(q.yaw_rad),
            q.y_m + off * math.sin(q.yaw_rad),
            q.yaw_rad,
        )

    # ----------------------------------------------------------- mock mission
    def _pallet(self) -> Pallet:
        return next(
            p for p in self.world.pallets if p.pallet_id == self.mission["pallet_id"]
        )

    @staticmethod
    def _pre_pose(p: Pallet) -> Pose:
        o = p.depth_m / 2 + CHASSIS.nose_from_rear_m + 0.6
        h = p.pose.yaw_rad
        return Pose(p.pose.x_m - o * math.cos(h), p.pose.y_m - o * math.sin(h), h)

    @staticmethod
    def _insert_pose(p: Pallet) -> Pose:
        o = loaded_nose_m(p) - p.depth_m / 2
        h = p.pose.yaw_rad
        return Pose(p.pose.x_m - o * math.cos(h), p.pose.y_m - o * math.sin(h), h)

    def _drop_poses(self, p: Pallet) -> tuple[Pose, Pose, Pose]:
        d = self.world.destinations[self.mission["destination_id"]]
        off = loaded_nose_m(p) - p.depth_m / 2
        drop = Pose(d.x_m + off, d.y_m, math.pi)
        return (
            Pose(drop.x_m + 1.5, d.y_m, math.pi),
            drop,
            Pose(drop.x_m + 0.9, d.y_m, math.pi),
        )

    def _via(self, p: Pallet) -> tuple[list[Pose], Pose]:
        if p.pallet_id == "P-02":
            return [Pose(7.0, 4.6, -0.6), Pose(8.8, 1.9, -0.15)], Pose(
                8.6, 1.15, math.pi / 2
            )
        return [Pose(7.0, 5.5, 0.0)], Pose(9.2, 6.6, -math.pi / 2)

    @staticmethod
    def _chain(poses: list[Pose], direction: int, speed: float) -> list[paths.Leg]:
        return [
            paths.hermite_leg(a, b, direction, speed)
            for a, b in zip(poses, poses[1:], strict=False)
        ]

    def _legs_for(self, phase: str) -> list[paths.Leg]:
        p = self._pallet()
        here = self.pose
        via, turn = self._via(p)
        pre_drop, drop, back = self._drop_poses(p)
        if phase == "approach":
            return self._chain([here, *via, self._pre_pose(p)], 1, 0.35)
        if phase == "insert":
            return [paths.hermite_leg(here, self._insert_pose(p), 1, 0.1)]
        if phase == "extract":
            return [paths.hermite_leg(here, self._pre_pose(p), -1, 0.15)]
        if phase == "transport":
            return [
                paths.hermite_leg(here, turn, -1, 0.2),
                *self._chain([turn, pre_drop, drop], 1, 0.3),
            ]
        if phase == "withdraw":
            return [paths.hermite_leg(here, back, -1, 0.15)]
        if phase == "return":
            # back away from the delivered load first, then turn home forward
            back = 1.1
            away = Pose(
                here.x_m - back * math.cos(here.yaw_rad),
                here.y_m - back * math.sin(here.yaw_rad),
                here.yaw_rad,
            )
            return [
                paths.hermite_leg(here, away, -1, 0.2),
                paths.hermite_leg(away, self.world.home, 1, 0.35),
            ]
        return []

    def _replan(self) -> None:
        m = self.mission
        if m["phase"] in (
            "approach",
            "transport",
            "return",
            "insert",
            "extract",
            "withdraw",
        ):
            if m["phase"] == "transport" and m["leg_index"] > 0:
                p = self._pallet()
                pre_drop, drop, _ = self._drop_poses(p)
                m["legs"] = self._chain([self.pose, pre_drop, drop], 1, 0.3)
            elif m["phase"] == "approach":
                m["legs"] = [
                    paths.hermite_leg(
                        self.pose, self._pre_pose(self._pallet()), 1, 0.35
                    )
                ]
            else:
                m["legs"] = self._legs_for(m["phase"])
            m["leg_index"] = 0
            self.plan_revision += 1

    def _next_phase(self) -> None:
        m = self.mission
        m["phase_index"] += 1
        if m["phase_index"] >= len(PHASES):
            m["status"], m["phase"] = "done", "complete"
            self.at[m["pallet_id"]] = m["destination_id"]  # plan D3: verified there only at done (as live)
            m["legs"] = []
            self._event("mission", mission_id=m["mission_id"], status="done")
            self._set_mode("PAUSED")
            return
        m["phase"] = PHASES[m["phase_index"]]
        m["phase_t"] = 0.0
        m["legs"] = self._legs_for(m["phase"])
        m["leg_index"] = 0
        self.plan_revision += 1
        self.forks_in_pocket = m["phase"] in (
            "insert",
            "lift",
            "extract",
            "lower",
            "withdraw",
        )
        p = self._pallet()
        if m["phase"] == "lift":
            p.state, self.world.carried = "carried", p.pallet_id
            self.at.pop(p.pallet_id, None)
            self.targets_revision += 1  # contract 5.1: a target's state changed
        self._event("phase", phase=m["phase"])

    def _step_mission(self, dt_s: float) -> None:
        self.drive_allowed = self.lift_allowed = None
        m = self.mission
        if m is None or m["status"] != "running":
            self.command = {"speed_mps": 0.0, "curvature_inv_m": 0.0}
            self._output(0.0, 0.0, dt_s)
            return
        m["phase_t"] += dt_s
        phase = m["phase"]
        if phase == "observe":
            self._output(0.0, 0.0, dt_s)
            if m["phase_t"] >= 1.5:
                self._next_phase()
            return
        if phase in ("lift", "lower"):
            self._output(0.0, 0.0, dt_s)
            goal = LIFT_CARRY_M if phase == "lift" else 0.0
            step = LIFT_RATE_MPS * dt_s
            self.lift_m += max(-step, min(step, goal - self.lift_m))
            if abs(self.lift_m - goal) < 1e-6:
                if phase == "lower":
                    p = self._pallet()
                    p.state, self.world.carried = "delivered", None
                    p.pose = self._carried_pose(p)
                    self.targets_revision += 1
                self._next_phase()
            return
        legs = m["legs"]
        if m["leg_index"] >= len(legs):
            self._finish_motion_phase()
            return
        leg = legs[m["leg_index"]]
        if leg.s_m == 0.0:
            x0, y0, yaw0 = leg.points[0][:3]
            if math.hypot(self.pose.x_m - x0, self.pose.y_m - y0) > 1e-6:
                # residual motion moved the truck: start this leg from where it really is
                x1, y1, yaw1 = leg.points[-1][:3]
                leg = paths.hermite_leg(
                    self.pose, Pose(x1, y1, yaw1), leg.direction, leg.speed_mps
                )
                legs[m["leg_index"]] = leg
        ramp = min(
            1.0,
            max(
                0.15, min(leg.s_m / 0.25 + 0.25, (leg.length_m - leg.s_m) / 0.25 + 0.15)
            ),
        )
        target = leg.speed_mps * ramp * leg.direction
        curvature = paths.curvature_at(leg)
        self.command = {"speed_mps": target, "curvature_inv_m": curvature}
        out = self._output(target, curvature, dt_s)
        if out * leg.direction > 0:
            self.pose = paths.advance(leg, abs(out) * dt_s)
        elif out != 0.0:
            # residual motion from the previous leg's direction: move the way the wheels turn,
            # and start this leg only once that motion has stopped
            self._integrate(dt_s)
        self._carry()
        if leg.s_m >= leg.length_m - 1e-9:
            m["leg_index"] += 1
            if m["leg_index"] >= len(legs):
                self._finish_motion_phase()

    def _finish_motion_phase(self) -> None:
        m = self.mission
        if m["phase"] == "recover":
            # the job cannot simply resume from a withdrawn fork: it ends, a new one is submitted
            self.forks_in_pocket = False
            m["status"], m["reason"] = "cancelled", "recovered_by_withdraw"
            m["legs"] = []
            self._set_mode("PAUSED")
            return
        self._next_phase()

    # ------------------------------------------------------------------ testing
    def debug_set_phase(
        self, phase: str, forks_in_pocket: bool, carried: str | None = None
    ) -> None:
        """Test helper: put a (fake) mission in a phase with the given fork and load state."""
        if self.mission is None:
            self.missions_created += 1
            self.mission = {
                "mission_id": f"M-{self.missions_created:04d}",
                "pallet_id": carried or "P-01",
                "destination_id": "D-A",
                "status": "running",
                "reason": None,
                "phase_index": PHASES.index(phase) if phase in PHASES else 0,
                "phase": phase,
                "phase_t": 0.0,
                "legs": [],
                "leg_index": 0,
            }
            self._set_mode("AUTO")
        else:
            self.mission["phase"] = phase
            if phase in PHASES:
                self.mission["phase_index"] = PHASES.index(phase)
        self.forks_in_pocket = forks_in_pocket
        for p in self.world.pallets:
            if p.state == "carried":
                p.state = "available"
        self.world.carried = None
        if carried is not None:
            p = next(q for q in self.world.pallets if q.pallet_id == carried)
            p.state, self.world.carried = "carried", carried
