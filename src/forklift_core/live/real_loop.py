"""The real truck's 50 Hz control loop (docs/design/2026-11-19-real-loop.md; contract
docs/design/2026-11-18-real-control-tick.md). No device here: the transport is injected (4c: a fake MCU only).

``RealSession`` is the minimal mission lifecycle (the Isaac ``LiveRuntime``'s: the console cycle, a mission's end
collected once, the results). ``RealLoop.cycle(now_ns)`` is one tick, in the contract's order: the head (STATEs,
the host, the AUTO permit, the output record, the watches, the console), SLAM and the preparation, the adoption and
the arming, the control (S10-A's branches only -- anything else latches before it acts), the output assembly, one
output branch (emergency DISARM -> S10 control stop -> an output wait -> the normal AUTO DRIVE) and the send.
"""

from __future__ import annotations

import math
import secrets
from collections import deque
from dataclasses import dataclass, field

import numpy as np

from forklift_core.live import real_autonomy as ra
from forklift_core.live.output_book import Frame, OutputBook
from forklift_core.live.real_assembly import freeze_submit, known_request
from forklift_core.live.real_channels import send_boundary
from forklift_core.live.real_teleop import counts_still, encoder_speed, real_auto_refusal
from forklift_core.mcu_link.build_policy import Client
from forklift_core.mcu_link.messages import FaultFlag, Mode
from forklift_core.mission.commands import FINAL
from forklift_core.plant.bringup import Sample
from forklift_core.plant.mcu_host import McuLinkHost
from forklift_core.safety.arbiter import Feedback
from forklift_core.safety.truth_access import SafetySpeed

TICK_S = 1 / 50
DRIVE_LIFE_S = 0.10  # a decision's absolute life from its tick (the MCU's cap is 150 ms)
SLOW_TICK_S = 0.030  # the tick's own elapsed time before its first write: over this, a wait (design 3)
LATE_TICK_S = 0.060  # over this (or the start interval), the emergency branch
SLOW_STARTS = 3  # this many start intervals over SLOW_TICK_S in a row: the emergency branch
MAX_STATES_PER_TICK = 64
ENCODER_CUT = FaultFlag.ENCODER_GAP | FaultFlag.ENCODER_JUMP_L | FaultFlag.ENCODER_JUMP_R  # contract 3.3
SAMPLES_KEPT = 128


def real_host(cfg, *, initial_seq: int | None = None, **kw) -> McuLinkHost:
    """The executor's host: the real client, its first seq random unless given (4c design: a new executor's frames are
    never mistaken for an earlier one's by seq; the teleop bridge does the same)."""
    seq = secrets.randbelow(0x10000) if initial_seq is None else int(initial_seq)
    return McuLinkHost(cfg, client=Client.REAL, initial_seq=seq, **kw)


def motion_allowed(transport) -> bool:
    """The device block (4c design 5): ARM and every DRIVE (the normal one, the host's zero after ARM, the wait frame,
    S10's stop frame) go out only on a transport that declares itself the internal fake MCU (``fake_mcu is True``);
    any other transport -- an operating serial link included -- passes the DISARM only. The real MCU's AUTO opens
    with PR 5 and S10's preconditions."""
    return getattr(transport, "fake_mcu", False) is True


@dataclass
class TickRecord:
    t: float
    start_interval_s: float | None
    branch: str = ""  # emergency | s10 | wait | auto | none
    reason: str | None = None
    seqs: list = field(default_factory=list)
    phase: str | None = None
    permit: str | None = None


class RealSession:
    """The console cycle and a mission's end (the Isaac ``LiveRuntime``'s minimum)."""

    def __init__(self, core, link) -> None:
        self.core, self.link = core, link
        self.results: list = []
        self._ended_id = None

    def tick_start(self, t: float, now_mono: float, feedback, context_kwargs: dict) -> tuple:
        from forklift_core.mission.commands import Context

        stop_kind = self.link.begin(now_mono, feedback)
        records = self.core.commands.apply(t, Context(stop_kind=stop_kind, **context_kwargs))
        return stop_kind, records

    def tick_end(self, now_mono: float, records: list, safety: dict, candidate: tuple, drive: dict | None):
        return self.link.end(now_mono, records, safety, candidate, drive)

    def ended_unclaimed(self) -> bool:
        cmds = self.core.commands
        return cmds.status in FINAL and cmds.mission_id is not None and cmds.mission_id != self._ended_id

    def end_mission(self, t: float, *, lift_m: float | None, release_allowed) -> dict | None:
        """Collect the mission the commands ended, once: the lift held at its measured place, the jobs dropped, the
        open stops closed, the clock taken, the end reset (the loop's release decision passed), the result kept."""
        core, cmds = self.core, self.core.commands
        mission_id, mission = cmds.mission_id, cmds.mission
        if mission_id is None or mission_id == self._ended_id:
            return None
        self._ended_id = mission_id
        if lift_m is not None:
            core.ms.lift_command = float(lift_m)
        core.abandon_jobs(t)
        cmds.close_open(t)
        if cmds.end_logged_for != mission_id:
            cmds.note_end(t, cmds.end_reason)
        clock = cmds.take_clock()
        core.end_reset(t, release_allowed=release_allowed)
        entry = {"mission_id": mission_id, "outcome": cmds.status, "reason": cmds.end_reason, "ended_s": float(t),
                 "pallet_id": None if mission is None else mission.pallet_id,
                 "destination_id": None if mission is None else mission.destination_id, "clock": clock}
        self.results.append(entry)
        return entry


class RealLoop:
    def __init__(self, *, core, session: RealSession, permit, port, host, link, channels, transport, params,
                 odometry, kinematics, max_speed_mps: float, start_ns: int, slam_source, evidence: ra.Evidence,
                 base_scenario, wait_pose, delay_s: float = 0.0, steering_ok=None, record=None,
                 depth_source=None) -> None:
        self.core, self.session, self.permit, self.port, self.host = core, session, permit, port, host
        self.link, self.channels, self.transport, self.params = link, channels, transport, params
        self.odometry, self.kin, self.max_speed_mps = odometry, kinematics, float(max_speed_mps)
        self.start_ns, self.slam_source, self.evidence = int(start_ns), slam_source, evidence
        self.base_scenario, self.wait_pose = base_scenario, wait_pose
        self.delay_s = delay_s
        self.steering_ok = steering_ok or (lambda q: kinematics.cfg.steering.curvature(q) is not None)
        self.record = record or (lambda rec: None)
        self.book = OutputBook()
        self.samples: deque = deque(maxlen=SAMPLES_KEPT)
        self.odom_pose = (0.0, 0.0, 0.0)
        self.odom_continuous = False
        self.speed = SafetySpeed(0.0, 0.0, (1, -1), valid=False)
        self.still_since_ns: int | None = None
        self.frozen = None
        self.adopted_for = None  # the mission id the ARM was asked for
        self.waiting = None  # the output wait's own target (steering, lift) while one lasts
        self._s10_stopping = False
        self.last_start_ns: int | None = None
        self.slow_starts = 0
        self.serial = 0
        self.steering_command = None
        self.records: list = []
        self.depth_source = depth_source  # frames(now_ns) -> [frame(stamp_s, depth, mission_gen, phase, correction)]
        self.pose_history: deque = deque(maxlen=64)  # (t, rear, continuous) per tick, for a frame's own pose
        self._wait_started_s = None  # the current D5 still-frame wait's start (core seconds)
        host._on_state_cb = self._on_state  # noqa: SLF001 -- every STATE the host accepts, once
        self._console_disarm = False
        self._odom_broken = False  # a continuity break among this tick's STATEs
        self._rebooted = False  # a boot change seen (not the first boot)
        link.a.on_disarm = self._on_console_disarm  # AUTO has no arbiter `armed` to drop: the loop's ① (design 2.1 4)

    def _on_console_disarm(self, now_mono: float) -> None:
        self._console_disarm = True  # taken inside the console cycle; the emergency branch runs right after it

    # --- the head -------------------------------------------------------------------------------------------------
    def _on_state(self, state, rx_ns: int) -> None:
        if self.samples and (state.boot_id, state.mcu_time_us) <= (self.samples[-1].state.boot_id,
                                                                    self.samples[-1].state.mcu_time_us) \
                and state.boot_id == self.samples[-1].state.boot_id:
            return  # a duplicate (or older) sample of this boot: no second count
        if self.samples and state.boot_id != self.samples[-1].state.boot_id:
            self.samples.clear()  # a new boot: the old boot's samples are no evidence of anything now
            self._rebooted = True
        self.samples.append(Sample(int(rx_ns), state))
        step = self.odometry.update(int(state.mcu_time_us), int(state.boot_id), int(state.wheel_count_left),
                                    int(state.wheel_count_right), float(state.steering_rad),
                                    encoder_cut=bool(int(state.fault_flags) & int(ENCODER_CUT)))
        self.odom_pose = tuple(float(v) for v in step.pose)
        if step.continuity_broken:
            self._odom_broken = True  # sticky for the tick: a later good sample does not mend this tick's break
            self.still_since_ns = None
        self.book.on_state(state, int(rx_ns))

    def _measured_lift(self):
        s = self.host.state
        return None if s is None or not math.isfinite(s.lift_m) else float(s.lift_m)

    def _measured_steering(self):
        s = self.host.state
        return None if s is None else float(s.steering_rad)

    def _still(self, now_ns: int) -> bool:
        """The still span (design 2.2, 8 축별 정지): the earlier commands settled, then a measured stand on samples
        measured after that -- taken as received at least the link's latency bound (``state_max_age_s``) after the
        settlement --, with the link fresh, the speed reading valid, the lift steady over them, the odometry
        continuous and SLAM fresh."""
        kind, since = self.port.outstanding(now_ns)
        standing = False
        if kind == "settled" and since is not None and self.host.health(now_ns).fresh and self.speed.valid:
            bound = since + int(self.host.cfg.state_max_age_s * 1e9)
            after = [x for x in self.samples if x.host_ns >= bound]
            lifts = [float(x.state.lift_m) for x in after]
            steady = bool(lifts) and all(math.isfinite(v) for v in lifts) \
                and max(lifts) - min(lifts) <= self.port.lift_still_m
            standing = steady and counts_still(after, self.params)
        sl = self.evidence.slam(now_ns)
        fresh_slam = sl.pose_reply_ns is not None and now_ns - sl.pose_reply_ns <= ra.SLAM_MAX_AGE_S * 1e9
        still = standing and self.odom_continuous and fresh_slam
        if not still:
            self.still_since_ns = None
        elif self.still_since_ns is None:
            self.still_since_ns = int(now_ns)
        return still

    def _t(self, now_ns: int) -> float:
        return (int(now_ns) - self.start_ns) / 1e9

    # --- one tick -------------------------------------------------------------------------------------------------
    def cycle(self, now_ns: int) -> TickRecord:
        core, permit, host = self.core, self.permit, self.host
        t = self._t(now_ns)
        interval = None if self.last_start_ns is None else (now_ns - self.last_start_ns) / 1e9
        self.last_start_ns = int(now_ns)
        rec = TickRecord(t, interval)
        try:
            self._head(now_ns, t, interval, rec)
            if rec.branch != "emergency":
                self._control_and_output(now_ns, t, rec)
        except Exception as exc:  # noqa: BLE001 -- any failure before the output is the emergency branch
            rec.branch, rec.reason = "emergency", f"exception:{type(exc).__name__}:{exc}"
            self._emergency(now_ns, rec.reason)
        try:
            self._send(now_ns, rec)  # S10's control stop goes out before the collection (design 4)
        except Exception as exc:  # noqa: BLE001 -- a failing final submit is ① too: DISARM queued and written
            rec.branch, rec.reason = "emergency", f"exception:send:{type(exc).__name__}:{exc}"
            self._emergency(now_ns, rec.reason)
            self._transmit(now_ns)
        self._collect(now_ns, t, rec)
        rec.phase, rec.permit = core.ms.phase, permit.state
        self.records.append(rec)
        self.record(rec)
        return rec

    def _head(self, now_ns: int, t: float, interval, rec: TickRecord) -> None:
        core, permit, host = self.core, self.permit, self.host
        self._odom_broken = False
        self.transport.receive(host, now_ns, limit=MAX_STATES_PER_TICK)
        self.odom_continuous = bool(self.samples) and not self._odom_broken
        if self._rebooted:
            # an MCU reboot (design 8 재시작): the position evidence goes too -- relocalize before any submit
            self._rebooted = False
            core.require_relocalization(t, "mcu_reboot")
        if self._odom_broken and permit.state in (ra.ARMING, ra.READY, ra.DRIVING, ra.STOP_PENDING):
            rec.branch, rec.reason = "emergency", "odometry_break"  # S10-A does not drive across a broken odometry
            self._emergency(now_ns, "odometry_break")
            return
        host.tick(now_ns)
        self._bind_permit(now_ns)
        permit.tick(now_ns)
        if self._s10_stopping and permit.state == ra.LATCHED:
            # the control stop confirmed and DISARMED seen (or the DISARM after a failure): the arbiter STOPPED with
            # the permit, so only a rearm in STOPPED goes on (design 4 5)
            self._s10_stopping = False
            if self.link.a.mode != "STOPPED":
                self.link.a.stop(now_ns / 1e9, permit.reason or "s10_stopped")
        self.book.settle(now_ns)
        q = self._measured_steering()
        self.book.publish(core.state, core.slam, now_ns, (0.0 if q is None else q,))
        reading = encoder_speed(list(self.samples), now_ns, self.params, self.steering_ok, self.delay_s)
        self.speed = reading.speed
        still = self._still(now_ns)
        if core.slam is not None:
            core.slam["stop_now"] = still
            core.slam["still_since"] = None if self.still_since_ns is None else self._t(self.still_since_ns)
        # the watches (design 3): the start interval, the absolute wheel cap
        if interval is not None:
            self.slow_starts = self.slow_starts + 1 if interval > SLOW_TICK_S + TICK_S else 0
            if interval > LATE_TICK_S + TICK_S or self.slow_starts >= SLOW_STARTS:
                rec.branch, rec.reason = "emergency", "loop_slow"
                self._emergency(now_ns, "loop_slow")
                return
        if reading.wheel_mps is not None and permit.armed:
            cap = self.max_speed_mps * 1.5 * self.params.turn_factor
            if max(abs(v) + e for v, e in zip(reading.wheel_mps, reading.wheel_err)) > cap:
                rec.branch, rec.reason = "emergency", "overspeed"
                self._emergency(now_ns, "overspeed")
                return
        s = host.state
        feedback = None if s is None or not self.samples else Feedback(
            stamp_mono=self.samples[-1].host_ns / 1e9, rear_axle_speed_mps=float(self.speed.signed_mps),
            lift_m=self._measured_lift(), steering_rad=abs(float(s.steering_rad)), odom_xy=self.odom_pose[:2])
        refusal = core.cargo.submit_refusal() or self._submit_refusal(now_ns)
        self.serial = self.link.emergency(now_ns / 1e9)  # the emergency batch first (its serial: the send's reference)
        self._stop_kind, self._cmd_records = self.session.tick_start(
            t, now_ns / 1e9, feedback, {"submit_refusal": refusal, "clear_for_submit": True,
                                        "localization_ok": True})
        a = self.link.a
        if self._console_disarm:
            self._console_disarm = False
            rec.branch, rec.reason = "emergency", "console_disarm"
            self._emergency(now_ns, "console_disarm")
            return
        # while an authority lasts (arming to the S10 stop; a latched permit retries its own DISARM, design 4 7):
        # STOPPED or the lease gone in any arbiter mode -- PAUSED after S10's end included -- is ①
        if permit.state in (ra.ARMING, ra.READY, ra.DRIVING, ra.STOP_PENDING, ra.DISARMING):
            if a.mode == "STOPPED":
                rec.branch, rec.reason = "emergency", "stop"
                self._emergency(now_ns, "stop")
            elif a.lease is None:
                rec.branch, rec.reason = "emergency", "lease_lost"
                self._emergency(now_ns, "lease_lost")

    def _submit_refusal(self, now_ns: int) -> str | None:
        why = ra.auto_ready_refusal(now_ns, "submit", self.evidence)
        return None if why is None else f"not_ready:{why}"

    def _bind_permit(self, now_ns: int) -> None:
        """The permit's callbacks at this instant (a latch's or an unanswered DISARM's deadline)."""
        a = self.link.a

        def arbiter_stop(r):
            if a.mode != "STOPPED":
                a.stop(now_ns / 1e9, r)

        self.permit.bind(transmit=lambda: self._transmit(now_ns), arbiter_stop=arbiter_stop,
                         collect=lambda r: self._end_now(now_ns, r))

    def _emergency(self, now_ns: int, reason: str) -> None:
        self.waiting = None
        # the DISARM is tried before the arbiter's stop and the collection (4b design 7)
        self._bind_permit(now_ns)
        try:
            self.permit.latch(now_ns, reason)
        except Exception:  # noqa: BLE001 -- the DISARM is queued and the arbiter stopped before a collection fails
            pass

    def _end_now(self, now_ns: int, reason: str) -> None:
        cmds = self.core.commands
        if cmds.status in ("accepted", "preparing", "running", "paused", "resuming", "held"):
            outcome = "recovery_required" if (self.core.cargo.pocket_entered
                                              or self.core.cargo.relation != "free") else "cancelled"
            cmds.terminate(outcome, self._t(now_ns), f"real:{reason}")

    # --- control and output ---------------------------------------------------------------------------------------
    def _control_and_output(self, now_ns: int, t: float, rec: TickRecord) -> None:
        core, permit, cmds, ms = self.core, self.permit, self.core.commands, self.core.ms
        replies = self.slam_source.replies(now_ns)
        core.slam_replies(t, replies, ms.phase in ("lift", "extract", "transport", "lower"))
        rear = np.asarray(core.hooks.slam_rear(t), dtype=float)
        core.hooks.set_rear(rear)
        self.pose_history.append((t, tuple(float(v) for v in rear), self.odom_continuous))
        self._feed_depth(now_ns, t)
        # the preparation (design 2.2 5)
        if cmds.status == "accepted" and self.frozen is None:
            self.frozen = freeze_submit(core)
        if cmds.status in ("accepted", "preparing") and not core.job_pending():
            if self.frozen is not None:
                known_request(core, t, rear, self.frozen, base_scenario=self.base_scenario, wait_pose=self.wait_pose)
        if core.end_release_due is not None:
            sl = self.evidence.slam(now_ns)
            if sl.mode == "holding" and sl.hold_travel_m > ra.HOLD_MAX_M:
                # a deferred release re-checks the hold's travel each tick, not only at the collection (Codex 4c impl
                # P1-10): too long a hold is not released -- a relocalization instead
                core.require_relocalization(t, "hold_too_long")
        gate_ = core.release_gate(t, rear)
        rear = gate_.rear
        # as the Isaac loop: paused, resuming or cancelling, or no mission yet (or over) -- a preparation is not
        paused_ = cmds.stopped or cmds.status in ("idle", *FINAL)
        plan_wait_ = core.jobs is not None and core.job_hold(t, rear, self.speed, paused_).hold
        if cmds.status in ("accepted", "preparing"):
            plan_wait_ = True
        if cmds.status in FINAL:
            self.frozen = None
        # the adoption and the arming (design 2.3)
        if (cmds.status == "running" and cmds.adopted_id == cmds.mission_id and ms.phase == "approach"
                and self.adopted_for != cmds.mission_id and permit.state == ra.OFF):
            self.adopted_for = cmds.mission_id
            why = self._pre_arm_refusal(now_ns)
            cmds.begin_wait(t)
            if not permit.adopted(now_ns, pre_arm_refusal=why):
                rec.branch, rec.reason = "emergency", f"arm:{permit.reason}"
                self._emergency(now_ns, permit.reason or "arm")
                return
        if permit.state == ra.ARMING:
            rec.branch = "none"  # no loop DRIVE while arming (the host's own zero only)
            self._assemble_and_arbitrate(now_ns, t, rear, 0.0, 0.0, True, rec, sink=True)
            return
        if permit.state in (ra.READY, ra.DRIVING) and cmds.waits and cmds.waits[-1][1] is None:
            cmds.end_wait(t)
        # S10-A's reach (design 2.4): anything else latches before it acts
        bad = self._unsupported()
        if bad is not None:
            rec.branch, rec.reason = "emergency", f"unsupported:{bad}"
            self._emergency(now_ns, f"unsupported:{bad}")
            return
        requested_speed, curvature, dwell_hold, cusp_hold, safety_hold = 0.0, 0.0, False, False, False
        tracking = None
        arrival_wait = None
        if (ms.phase in core.trackers and not gate_.releasing and not gate_.warming and not paused_
                and not plan_wait_ and permit.state in (ra.READY, ra.DRIVING)):
            follow_ = core.follow(t, rear, self.speed.signed_mps, TICK_S)
            if getattr(follow_, "scene_raised", False):
                rec.branch, rec.reason = "emergency", "unsupported:scene_raised"
                self._emergency(now_ns, "unsupported:scene_raised")
                return
            tracking, cusp_hold = follow_.tracking, follow_.cusp_hold
            stall_ = core.docking_and_stall(t, rear, tracking, self.speed.signed_mps, TICK_S, False)
            tracking, rear = stall_.tracking, stall_.rear
            recover_ = core.cusp_and_observe_recovery(t, rear, tracking, self.speed.signed_mps, TICK_S,
                                                      stall_.d7_docking_, stall_.d7_owned, 0)
            tracking, rear = recover_.tracking, recover_.rear
            command_ = core.command(t, rear, None, tracking, self.speed.signed_mps, dwell_hold, False, safety_hold)
            requested_speed, curvature, tracking, rear = (command_.requested_speed, command_.curvature,
                                                          command_.tracking, command_.rear)
            dwell_hold, safety_hold = command_.dwell_hold, command_.safety_hold
            if tracking.status == "arrived":
                arrival_ = core.arrive(t, tracking)
                if arrival_.rear is not None:
                    rear = arrival_.rear
                arrival_wait = getattr(arrival_, "wait", None)
        if core.jobs is not None and not plan_wait_ and core.job_pending():
            plan_wait_ = True
        if not paused_ and (not plan_wait_ or core.lift_ramps_in_plan_wait()) and core.lift_and_timers(t, TICK_S):
            pass  # S10-A never delivers: a delivery verdict is not built here
        if core.take_s10_stop() is not None:
            rec.branch = "s10"
            self._assemble_and_arbitrate(now_ns, t, rear, 0.0, 0.0, True, rec, sink=True)
            return
        kappa = float(self.kin.held_curvature((self._measured_steering() or 0.0,)) or 0.0)
        obstacle_hold = False
        if (not paused_ and core.obstacle is not None and ms.phase in core.trackers and not plan_wait_
                and (requested_speed != 0.0 or abs(self.speed.signed_mps) > 0.05)):
            perm_ = core.permission(t, rear, kappa, requested_speed, False, None)
            requested_speed, obstacle_hold = perm_.requested_speed, perm_.obstacle_hold
        elif (plan_wait_ or paused_) and core.obstacle is not None and not paused_:
            watch_ = core.permission_watch(t, rear, kappa, False)
            obstacle_hold = watch_.obstacle_hold
        if paused_:
            pt_ = core.paused_tick(t, rear, kappa, False)
            rear, obstacle_hold = pt_.rear, pt_.obstacle_hold
            requested_speed, curvature, dwell_hold = 0.0, 0.0, True
        elif plan_wait_:
            requested_speed, curvature, dwell_hold = 0.0, 0.0, True
        elif core.live_leg(t, rear, False):
            requested_speed = 0.0
        self._wait_reason = None
        if permit.state in (ra.READY, ra.DRIVING):
            if arrival_wait is not None:
                requested_speed, curvature, dwell_hold = 0.0, 0.0, True
                self._wait_reason = arrival_wait  # the wait frame: wheels 0 at once, the old queue dropped
            elif plan_wait_:
                self._wait_reason = "plan_wait"
        self._assemble_and_arbitrate(now_ns, t, rear, requested_speed, curvature, dwell_hold, rec,
                                     obstacle_hold=obstacle_hold, cusp_hold=cusp_hold, safety_hold=safety_hold)

    def _feed_depth(self, now_ns: int, t: float) -> None:
        """The D5 depth feed (design 2.2 4): each new frame of this mission, phase and correction, younger than the
        check's own age limit and covered by a continuous pose interval, goes to ``add_frame`` at its interpolated
        pose; the still-frame wait is released only by a frame the check took as new, acquired after the wait began
        and inside the still span."""
        core, pocket = self.core, self.core.pocket
        check = pocket.get("check")
        if self.depth_source is None or check is None:
            return
        if pocket.get("await_still_frame"):
            if self._wait_started_s is None:
                self._wait_started_s = t
        else:
            self._wait_started_s = None
        applied = tuple(float(v) for v in (core._applied_now() or (0.0, 0.0, 0.0)))  # noqa: SLF001
        max_age = float(getattr(getattr(check, "config", None), "frame_max_age_s", 0.2))
        lag = float(getattr(getattr(check, "config", None), "frame_lag_s", 0.0))
        for frame in self.depth_source.frames(now_ns):
            stamp = float(frame.stamp_s)
            if (frame.mission_gen != core.commands.mission_gen or frame.phase != core.ms.phase
                    or tuple(float(v) for v in frame.correction) != applied):
                continue
            if stamp > t + 1e-6 or stamp <= float(getattr(check, "last_new_s", -math.inf)) \
                    or t - (stamp - lag) > max_age:
                continue
            rear_at = self._pose_at(stamp)
            if rear_at is None:
                continue  # no continuous pose interval covers it
            lift = self._measured_lift()
            out = check.add_frame(stamp, frame.depth, rear_at, speed_mps=abs(self.speed.signed_mps),
                                  yaw_rate_rps=0.0, lift_m=0.0 if lift is None else lift)
            pocket["frames_read"] = pocket.get("frames_read", 0) + 1
            still_from = None if self.still_since_ns is None else self._t(self.still_since_ns)
            if (pocket.get("await_still_frame") and (out or {}).get("new") and self._wait_started_s is not None
                    and stamp >= self._wait_started_s and still_from is not None and stamp >= still_from
                    and t - (stamp - lag) <= max_age):
                pocket["await_still_frame"] = False
                pocket["still_frame_s"] = stamp

    def _pose_at(self, stamp: float):
        h = [e for e in self.pose_history]
        for (t0, p0, c0), (t1, p1, c1) in zip(h, h[1:]):
            if t0 <= stamp <= t1 and c0 and c1:
                a = 0.0 if t1 <= t0 else (stamp - t0) / (t1 - t0)
                return tuple(p0[i] + a * (p1[i] - p0[i]) for i in range(3))
        if h and abs(h[-1][0] - stamp) < 1e-9 and h[-1][2]:
            return h[-1][1]
        return None

    def _unsupported(self) -> str | None:
        core = self.core
        if core.commands.status in ("paused", "resuming", "held", "recovering"):
            return f"mission_{core.commands.status}"  # S10-A has no pause and resume: DISARM and collect
        if getattr(core, "capture_live", None) is not None or getattr(core, "observation_live", None) is not None:
            return "observation_capture"
        if core.scene_replan.active():
            return "scene_replan"
        if core.loaded_start_wait(self._t(self.last_start_ns)):
            return "loaded_start"
        if core.observe_backoff is not None:
            return "observe_backoff"
        if core.near is not None or core.bottom_deck_on():
            return "near_or_bottom_deck"
        return None

    def _pre_arm_refusal(self, now_ns: int) -> str | None:
        why = ra.auto_ready_refusal(now_ns, "drive", self.evidence)
        if why is not None and ra.latches(why):
            return why
        s = self.host.state
        if s is None or s.mode.name != "DISARMED":
            return "mcu_not_disarmed"
        if self.port.outstanding(now_ns)[0] != "settled":
            return "outstanding"
        if not self.core.slam.get("stop_now"):
            return "not_still"
        return None

    def _assemble_and_arbitrate(self, now_ns, t, rear, requested_speed, curvature, dwell_hold, rec, *, sink=False,
                                obstacle_hold=False, cusp_hold=False, safety_hold=False) -> None:
        core = self.core
        q = self._measured_steering() or 0.0
        if self.steering_command is None:
            self.steering_command = np.array([q], dtype=float)
        steer = self.steering_command.copy()  # the candidate's own copy (design 2.5 1)
        kappa = float(self.kin.held_curvature((q,)) or 0.0)
        try:
            command_ = core.assemble_command(t, TICK_S, requested_speed, curvature, kappa, steer, False,
                                             obstacle_hold, dwell_hold, cusp_hold, safety_hold)
        finally:
            self.book.publish(core.state, core.slam, now_ns, (q,))  # the core reads applied values only
        core.cargo.observe(core.ms.phase, rear)
        core.observe_motion(np.asarray(self.odom_pose, dtype=float))
        safety = core.safety_state(t)
        out = self.session.tick_end(now_ns / 1e9, self._cmd_records, safety,
                                    (float(command_.drive.speed_mps), float(command_.drive.curvature_inv_m)), None)
        self._candidate = (command_, command_.steering_command, out, sink)

    # --- collection, then the one output branch and the send ---------------------------------------------------------
    def _collect(self, now_ns: int, t: float, rec: TickRecord) -> None:
        if self.session.ended_unclaimed():
            release = self._release_allowed(now_ns)
            self.session.end_mission(t, lift_m=self._measured_lift(), release_allowed=release)
            # S10's end leaves the authority through its own control stop (STOP_PENDING); any other end latches
            if self.permit.state in (ra.ARMING, ra.READY, ra.DRIVING):
                self.permit.mission_ended(now_ns, self.core.commands.end_reason or "end")
            self.frozen, self.adopted_for, self.waiting = None, None, None

    def _release_allowed(self, now_ns: int):
        sl = self.evidence.slam(now_ns)
        if sl.mode == "holding" and sl.hold_travel_m > ra.HOLD_MAX_M:
            return "relocalize"
        fresh = sl.pose_reply_ns is not None and now_ns - sl.pose_reply_ns <= ra.SLAM_MAX_AGE_S * 1e9
        return bool(fresh and self.odom_continuous and self.core.slam.get("stop_now"))

    def _send(self, now_ns: int, rec: TickRecord) -> None:
        permit, port = self.permit, self.port
        elapsed = 0.0  # 4c: the fake clock gives every tick its own instant (the real loop measures it, design 3)
        if rec.branch == "emergency":
            pass  # the latch queued the DISARM
        elif rec.branch == "s10" and permit.state in (ra.READY, ra.DRIVING):
            lift, q = self._measured_lift(), self._measured_steering()
            why = ra.auto_ready_refusal(now_ns, "drive", self.evidence)
            if why is not None and ra.latches(why):
                rec.branch, rec.reason = "emergency", f"s10:{why}"
                self._emergency(now_ns, rec.reason)
            elif lift is None or q is None:
                self._emergency(now_ns, "s10_measurement")
            else:
                r = port.control_stop(steering_rad=q, lift_m=lift, now_ns=now_ns,
                                      expires_ns=now_ns + int(DRIVE_LIFE_S * 1e9))
                if r.kind != "queued":
                    self._emergency(now_ns, f"s10_stop:{r.reason}")
                else:
                    self._book(now_ns, r.seq, 0.0, 0.0, (q,), lift, False, "stop")
                    permit.control_stop(now_ns, r.seq)
                    self._s10_stopping = True
                    rec.seqs.append(r.seq)
        elif permit.state in (ra.READY, ra.DRIVING) and getattr(self, "_candidate", None) is not None:
            command_, steer, out, sink = self._candidate
            wait = self._wait_reason_now(now_ns, out)
            if elapsed > LATE_TICK_S:
                self._emergency(now_ns, "tick_late")
            elif wait is not None or elapsed > SLOW_TICK_S:
                rec.branch, rec.reason = "wait", wait or "tick_slow"
                self._wait_frame(now_ns, rec)
            elif out.source == "auto" and permit.drive_allowed():
                rec.branch = "auto"
                self._auto_drive(now_ns, command_, steer, rec)
            else:
                rec.branch, rec.reason = "wait", f"output:{out.source}"
                self._wait_frame(now_ns, rec)
        else:
            rec.branch = rec.branch or "none"
        self._candidate = None
        self._transmit(now_ns)

    def _wait_reason_now(self, now_ns, out) -> str | None:
        why = ra.auto_ready_refusal(now_ns, "drive", self.evidence)
        if why is not None and ra.latches(why):
            self._emergency(now_ns, why)
            return "latched"
        if why is not None:
            return why
        return getattr(self, "_wait_reason", None)

    def _wait_frame(self, now_ns: int, rec: TickRecord) -> None:
        port = self.port
        if self.waiting is None:
            lift, q = self._measured_lift(), self._measured_steering()
            if lift is None or q is None:
                self._emergency(now_ns, "wait_measurement")
                return
            self.waiting = (q, lift)
            r = port.control_stop(steering_rad=q, lift_m=lift, now_ns=now_ns,
                                  expires_ns=now_ns + int(DRIVE_LIFE_S * 1e9))
        else:
            q, lift = self.waiting
            r = port.hold_frame(steering_rad=q, lift_m=lift, now_ns=now_ns,
                                expires_ns=now_ns + int(DRIVE_LIFE_S * 1e9))
        if r.kind != "queued":
            self._emergency(now_ns, f"wait:{r.reason}")
            return
        measured = self._measured_lift()
        self._book(now_ns, r.seq, 0.0, 0.0, (self.waiting[0],), self.waiting[1],
                   measured is None or abs(self.waiting[1] - measured) > port.lift_still_m, "hold")
        rec.seqs.append(r.seq)

    def _auto_drive(self, now_ns: int, command_, steer, rec: TickRecord) -> None:
        port, permit = self.port, self.permit
        if self.waiting is not None:
            self.host.drop_drives("wait_end")  # the wait's unsent hold never follows the normal DRIVE out
            self.waiting = None
        drive = command_.drive
        speed = float(drive.speed_mps)
        if abs(speed) > self.max_speed_mps:  # the rear-axle command cap (design 2.5 4)
            drive = self.kin.command(math.copysign(self.max_speed_mps, speed), float(drive.curvature_inv_m))
            speed = float(drive.speed_mps)
        may = self.book.may_act(now_ns)
        if speed != 0.0 and may.get(-1 if speed > 0 else 1, 0.0) > 0.0:
            drive = self.kin.command(0.0, float(drive.curvature_inv_m))  # the turn waits for the other way to settle
            speed = 0.0
        lift = float(self.core.ms.lift_command)
        cmd = port.make_command(now_ns, now_ns + int(DRIVE_LIFE_S * 1e9), drive, steer, lift, "target")
        r = port.command(cmd, now_ns)
        if r.kind != "queued":
            self._emergency(now_ns, f"port:{r.kind}:{r.reason}")
            return
        self.steering_command = np.asarray(steer, dtype=float).copy()
        movable = speed != 0.0 or (self._measured_lift() is None
                                   or abs(lift - self._measured_lift()) > port.lift_still_m)
        self._book(now_ns, r.seq, speed, float(drive.curvature_inv_m),
                   tuple(float(v) for v in np.asarray(drive.steering_rad)), lift, movable, "drive")
        permit.drove(now_ns)
        rec.seqs.append(r.seq)

    def _book(self, now_ns, seq, speed, kappa, steering, lift, movable, kind) -> None:
        s = self.host.state
        self.book.queued(Frame(int(seq) & 0xFFFF, None if s is None else s.boot_id, None if s is None else s.arm_gen,
                               now_ns + int(DRIVE_LIFE_S * 1e9), float(speed), float(kappa), tuple(steering),
                               float(lift), bool(movable), kind, queued_ns=int(now_ns)))

    def _transmit(self, now_ns: int) -> None:
        """Every queued frame through the send boundary: the transport calls ``gate(frame, write)`` for each and the
        write happens inside it -- a motion frame under the channels' lock only when no emergency came in since this
        tick's batch, a DISARM always; written frames reported to the permit and the record."""
        motion = motion_allowed(self.transport)
        a = self.link.a
        refused = []

        def gate(frame, write) -> bool:
            if frame.kind == "DISARM":
                write()
                return True
            # the device block (nothing but a DISARM on an operating transport); the authority at the first byte: an
            # arbiter STOPPED or a lease already lost (a disconnect taken before this tick's batch passes the serial)
            ok = motion and a.mode != "STOPPED" and a.lease is not None \
                and send_boundary(self.channels, self.serial, write)
            if not ok:
                refused.append((frame.kind, frame.seq))
            return ok

        for kind, seq, written_ns in self.transport.transmit(self.host, now_ns, gate):
            self.permit.written(kind, seq, written_ns)
            if kind == "DRIVE":
                self.book.written(seq, written_ns)
        for kind, seq in refused:
            if kind == "DRIVE":
                self.book.discarded(seq)
                self.permit.stop_discarded(now_ns, seq)
                self.steering_command = None  # an unwritten candidate is never the next slew's start


__all__ = ["RealLoop", "RealSession", "TickRecord", "motion_allowed", "real_host"]
