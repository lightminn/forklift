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
import time
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
EVIDENCE_READ_MAX_S = 0.005  # the first byte's evidence read must take no longer (its ages are judged at its start)
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
    elapsed_s: float | None = None  # the tick's own time before its first write


class RealSession:
    """The console cycle and a mission's end (the Isaac ``LiveRuntime``'s minimum)."""

    def __init__(self, core, link) -> None:
        self.core, self.link = core, link
        self.results: list = []
        self._ended_id = None

    def tick_start(self, t: float, now_mono: float, feedback, context_kwargs: dict, collect=None) -> tuple:
        """``link.begin`` -> collect -> ``commands.apply`` -> collect (design 2.1 5): a mission the head ended (a latch
        in ``permit.tick``) is collected before the apply could clear its final status."""
        from forklift_core.mission.commands import Context

        stop_kind = self.link.begin(now_mono, feedback)
        if collect is not None:
            collect()
        records = self.core.commands.apply(t, Context(stop_kind=stop_kind, **context_kwargs))
        if collect is not None:
            collect()
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
                 depth_source=None, watch=None, clock=None, operator_start=None, real_layer=None,
                 delay_bound=None, state_delay_s: float = 0.03, v_scan_mps: float | None = None,
                 kappa_max_inv_m: float = 0.0, pallet_size=None) -> None:
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
        # (t, rear, continuous, speed bound, lift) per tick: a depth frame's own pose and motion at its acquisition
        self.pose_history: deque = deque(maxlen=64)
        self._wait_started_s = None  # the current D5 still-frame wait's start (core seconds)
        host._on_state_cb = self._on_state  # noqa: SLF001 -- every STATE the host accepts, once
        self.operator_start = operator_start  # the operator's physical check, one submit only (4b design 2, 13)
        # 5a (docs/design/2026-11-19-real-safety-wiring.md 1-3): the real obstacle layer, the delay bound for the
        # STATEs' measurement intervals, the raw odometry history the layer places scans on
        self.real_layer = real_layer
        self.delay_bound = delay_bound
        self.state_delay_s = float(state_delay_s)
        self.v_scan_mps = (1.5 * float(max_speed_mps)) if v_scan_mps is None else float(v_scan_mps)
        self.kappa_max = float(kappa_max_inv_m)
        self.pallet_size = pallet_size  # (depth, width) of the S10-A pallet (s10a.yaml): the planning grid's split
        self.odom_history: deque = deque(maxlen=256)  # (lo_s, hi_s, pose, continuous, included, speed_ok, yaw_ok)
        self._layer_asked = None  # the last (mission gen, path id, loaded) a refresh was asked for
        self._planning_gen = None  # the correction generation the planning request used (design 1.2, round 3 P2-3)
        self.watch = watch  # the real safety watch (the teleop leg's SafetyWatch); None: off (design 3)
        self._watch_rx: int | None = None
        # the host monotonic clock the tick's own elapsed time is read on before the first write (design 3); the
        # tests' fake clock passes the tick's instant
        self.clock = clock or time.monotonic_ns
        self._console_disarm = False
        self._odom_broken = False  # a continuity break among this tick's STATEs
        self._rebooted = False  # a boot change seen (not the first boot)
        self._disconnects = host.disconnects  # the host's device losses seen so far
        self._last_seen = None  # the newest STATE's (boot, MCU time)
        self._lift_before = None  # the lift target before this tick's ramp
        self._written_now: list = []
        self._in_transmit = False
        self._motion_mcu_until: int | None = None  # the latest MCU time a movable frame may act to (its boot's clock)
        self._watch_trip: str | None = None  # the safety watch's trip on a STATE of this tick
        self._binding: dict = {}  # seq -> (lease id, arbiter control generation) a loop frame was decided under
        self._steer_of: dict = {}  # seq -> the steering command a loop frame carries (the next slew's start once written)
        link.a.on_disarm = self._on_console_disarm  # AUTO has no arbiter `armed` to drop: the loop's ① (design 2.1 4)

    def _on_console_disarm(self, now_mono: float) -> None:
        self._console_disarm = True  # taken inside the console cycle; the emergency branch runs right after it

    # --- the head -------------------------------------------------------------------------------------------------
    def _on_state(self, state, rx_ns: int) -> None:
        last = self._last_seen  # (boot, MCU time) of the newest STATE taken -- kept across sample clears
        if last is not None and state.boot_id == last[0] and state.mcu_time_us <= last[1]:
            return  # a duplicate (or older) sample of this boot: no second count
        if last is not None and state.boot_id != last[0]:
            # a new boot (also after a device loss and empty ticks, round 3 P1-3): the old boot's samples are no
            # evidence of anything now
            self.samples.clear()
            self._rebooted = True
            self._motion_mcu_until = None  # another clock; a rebooted MCU acts on no earlier frame
        self._last_seen = (state.boot_id, state.mcu_time_us)
        self.samples.append(Sample(int(rx_ns), state))
        step = self.odometry.update(int(state.mcu_time_us), int(state.boot_id), int(state.wheel_count_left),
                                    int(state.wheel_count_right), float(state.steering_rad),
                                    encoder_cut=bool(int(state.fault_flags) & int(ENCODER_CUT)))
        self.odom_pose = tuple(float(v) for v in step.pose)
        if step.continuity_broken:
            self._odom_broken = True  # sticky for the tick: a later good sample does not mend this tick's break
            self.still_since_ns = None
        self.book.on_state(state, int(rx_ns))
        self.permit.on_state(state, int(rx_ns))
        if self.real_layer is not None:
            self._odom_entry(state, int(rx_ns), step)
        if self.watch is not None and self._watch_trip is None:
            self._watch_trip = self._watch_state(state)

    def _odom_entry(self, state, rx_ns: int, step) -> None:
        """5a (design 1.1): the STATE's measurement interval ``[receipt - delay bound, receipt]`` (core seconds) with
        its raw odometry pose; excluded (``included`` False) without a bound or with one over ``state_delay_s``; the
        motion bounds the scan placement checks against ``v_scan`` (speed from consecutive poses over the interval's
        worst case, yaw rate likewise)."""
        sample = Sample(rx_ns, state)
        b = None
        if self.delay_bound is not None:
            self.delay_bound.add(sample)
            b = self.delay_bound.delay_s(sample)
        hi = self._t(rx_ns)
        included = b is not None and b <= self.state_delay_s
        lo = hi - (self.state_delay_s if b is None else b)
        pose = tuple(float(v) for v in step.pose)
        speed_ok = yaw_ok = True
        mcu_s = int(state.mcu_time_us) / 1e6
        prev_mcu = getattr(self, "_odom_prev_mcu", None)
        self._odom_prev_mcu = (int(state.boot_id), mcu_s)
        if self.odom_history and prev_mcu is not None and prev_mcu[0] == int(state.boot_id):
            ppose = self.odom_history[-1][2]
            dt = max(1e-4, mcu_s - prev_mcu[1])  # the MCU's own interval between the two measurements
            if math.hypot(pose[0] - ppose[0], pose[1] - ppose[1]) / dt > self.v_scan_mps + 1e-9:
                speed_ok = False
            if abs(math.atan2(math.sin(pose[2] - ppose[2]), math.cos(pose[2] - ppose[2]))) / dt \
                    > self.v_scan_mps * max(self.kappa_max, 1e-9) + 1e-9:
                yaw_ok = False
        self.odom_history.append((lo, hi, pose, not step.continuity_broken, included, speed_ok, yaw_ok))

    def _layer_context(self):
        core = self.core
        tracker = core.trackers.get(core.ms.phase) if getattr(core, "trackers", None) else None
        leg = getattr(tracker, "leg_index", 0) if tracker is not None else 0
        leg = leg() if callable(leg) else leg
        path_id = (core.ms.phase,) if tracker is None else (
            core.ms.phase, (getattr(tracker, "_book", None) or {}).get("serial", id(tracker)), int(leg or 0))
        return core.commands.mission_gen, path_id, bool(core.cargo.relation == "carrying")

    def _layer_tick(self, t: float, rear) -> None:
        """5a (design 1.2 (c)): the key's parts this tick; when they changed, a refresh with the current path; the
        frozen pallet the planning grid suppresses."""
        core, lay = self.core, self.real_layer
        mission_gen, path_id, loaded = self._layer_context()
        lay.context = (mission_gen, path_id)
        tracker = core.trackers.get(core.ms.phase) if getattr(core, "trackers", None) else None
        path = None
        if tracker is not None and hasattr(tracker, "leg_ahead"):
            path = np.asarray(tracker.leg_ahead()[0], dtype=float)
        if (mission_gen, path_id, loaded) != self._layer_asked:
            self._layer_asked = (mission_gen, path_id, loaded)
            lay.refresh(t, core.obstacle["applied"], int(core.obstacle["version"]),
                        current_pose=tuple(float(v) for v in rear), path_ahead=path, loaded=loaded)
        else:
            lay.track(tuple(float(v) for v in rear), path)  # the next scan's check from here, along what is left
        if self.frozen is not None:
            depth, width = self.pallet_size
            x, y, yaw = self.frozen.pose
            bound = float(lay.grid_config.sensor_bound_m)
            rect = (x, y, yaw, depth / 2 + bound, width / 2 + bound)
            lay.planning_pallet = (rect, (self.frozen.pallet_id, int(core.obstacle["version"])))
        else:
            lay.planning_pallet = None

    def planning_occupancy(self):
        """The assembly's planning-grid hook (design 1.2): the installed planning grid of the frozen pallet and the
        applied correction generation, or ``planning_grid_not_ready`` (no grid-less plan)."""
        from forklift_core.live.real_assembly import RealRequireFailed

        grid = None if self.real_layer is None else self.real_layer.planning_grid(self._planning_key())
        if grid is None:
            raise RealRequireFailed("planning_grid_not_ready")
        return grid

    def _planning_ready(self) -> bool:
        if self.real_layer is None:
            return True
        return self.real_layer.planning_grid(self._planning_key()) is not None

    def _planning_key(self):
        return None if self.frozen is None else (self.frozen.pallet_id, int(self.core.obstacle["version"]))

    def _reset_layer(self, now_ns: int) -> None:
        """A new grid, no installed or computing result (the worker's late answer is stale by its key) -- the odometry
        the stored scans were placed on is gone."""
        lay = self.real_layer
        lay.grid = type(lay.grid)(lay.grid_config)
        lay._installed = None  # noqa: SLF001 -- the layer's own reset
        lay.exempt_serial += 1
        lay.last_scan_s.clear()
        self.odom_history.clear()

    def place_scan(self, first_s: float, last_s: float):
        """The layer's placer (design 1.1, the teleop scan bridge's way): a continuous, included history whose
        entries reach before the first beam and after the last, every entry in between within the motion bounds;
        the pose interpolated at the first beam (entry interval midpoints), or None."""
        h = list(self.odom_history)
        before = [k for k, e in enumerate(h) if e[1] <= first_s]
        after = [k for k, e in enumerate(h) if e[0] >= last_s]
        if not before or not after:
            return None
        k0, k1 = before[-1], after[0]
        span = h[k0:k1 + 1]
        if not all(e[3] and e[4] for e in span) or not all(e[5] and e[6] for e in span[1:]):
            return None  # a break, an excluded STATE or a motion over the bound inside the scan's span
        mids = [((e[0] + e[1]) / 2, e[2]) for e in span]
        for (ta, pa), (tb, pb) in zip(mids, mids[1:]):
            if ta <= first_s <= tb:
                a = 0.0 if tb <= ta else (first_s - ta) / (tb - ta)
                dyaw = math.atan2(math.sin(pb[2] - pa[2]), math.cos(pb[2] - pa[2]))
                return (pa[0] + a * (pb[0] - pa[0]), pa[1] + a * (pb[1] - pa[1]), pa[2] + a * dyaw)
        return mids[0][1] if first_s < mids[0][0] else mids[-1][1]

    def _watch_state(self, st):
        """The safety watch on one STATE (design 3; Codex 4c impl round 2 P2-4: every STATE of a batch, on its own MCU
        time) against the wheels of the confirmed target (zero once none is confirmed); returns its trip or None."""
        c = self.book.confirmed
        if c is None or c.speed_mps == 0.0:
            target = (0.0, 0.0)
        else:
            d = self.kin.command(float(c.speed_mps), float(c.curvature_inv_m))
            target = tuple(float(w) for w in d.wheel_rates_rad_s)
        res = self.watch.step(st.mcu_time_us / 1e6, armed=self.permit.armed, fresh=True, new_state=True,
                              currents_a=st.current_a, wheel_targets_rad_s=target,
                              wheel_rates_rad_s=(st.wheel_rate_left_rad_s, st.wheel_rate_right_rad_s))
        return res.trip

    def _measured_lift(self):
        s = self.host.state
        return None if s is None or not math.isfinite(s.lift_m) else float(s.lift_m)

    def _measured_steering(self):
        s = self.host.state
        return None if s is None else float(s.steering_rad)

    def _still(self, now_ns: int) -> bool:
        """The still span (design 2.2, 8 축별 정지): the earlier commands settled, then a measured stand on samples
        measured after them on the MCU's own clock -- later than every movable frame's MCU deadline (its echo time plus
        the host's validity cap: the MCU stops acting on it there whatever the link's delay) --, with the link fresh,
        the speed reading valid, the lift steady over them, the odometry continuous and SLAM fresh."""
        kind, since = self.port.outstanding(now_ns)
        standing = False
        if kind == "settled" and since is not None and self.host.health(now_ns).fresh and self.speed.valid:
            until = self._motion_mcu_until
            after = [x for x in self.samples if until is None or x.state.mcu_time_us > until]
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
        self._rec = rec
        self._lift_before = None
        self._written_now = []  # the seqs written whole in this tick, in order
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
            try:
                self._transmit(now_ns)
            except Exception as exc2:  # noqa: BLE001 -- the DISARM stays queued; the permit retries it next tick
                rec.reason = f"{rec.reason};transmit:{type(exc2).__name__}"
        # the record: the frames written whole in this tick -- queued now or finished from an earlier tick (round 8 P3)
        rec.seqs = list(self._written_now)
        self._collect(now_ns, t, rec)
        rec.phase, rec.permit = core.ms.phase, permit.state
        self.records.append(rec)
        self.record(rec)
        return rec

    def _head(self, now_ns: int, t: float, interval, rec: TickRecord) -> None:
        core, permit, host = self.core, self.permit, self.host
        if core.slam is not None:
            core.slam["stop_now"] = False  # nothing this tick stands before the stand is judged again (receive may fail)
        self._odom_broken = False
        core.state.pop("motion_hold", None)  # a hold is this tick's only (round 3 P2-2)
        try:
            self.transport.receive(host, now_ns, limit=MAX_STATES_PER_TICK)
        except Exception as exc:  # noqa: BLE001 -- RX failing: the permit still retries its DISARM and judges its deadline
            host.tick(now_ns)
            self._bind_permit(now_ns)
            permit.tick(now_ns)
            rec.branch, rec.reason = "emergency", f"receive:{type(exc).__name__}:{exc}"
            self._emergency(now_ns, rec.reason)
            return
        if host.disconnects != self._disconnects:
            # a device loss (a same-boot reconnect included): the queue, the samples and the stand go with it
            self._disconnects = host.disconnects
            self.samples.clear()
            self._odom_broken = True
            self.still_since_ns = None
        stuck = self._cut_stuck_write(now_ns)
        if stuck:
            # the host's and the permit's ticks still run (its DISARM asked again on the new link, its deadline kept)
            host.tick(now_ns)
            self._bind_permit(now_ns)
            permit.tick(now_ns)
            permit.link_reset(now_ns)
            rec.branch, rec.reason = "emergency", stuck
            self._emergency(now_ns, stuck)
            return
        self.odom_continuous = bool(self.samples) and not self._odom_broken
        if self.real_layer is not None:
            if self._odom_broken:
                self._reset_layer(now_ns)  # a same-boot break: the layer starts again (design 1.1, round 6 P1-3)
            self.real_layer.install_ready(self._t(now_ns), self.odom_pose)
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
            # the start interval (design 3): three in a row over 30 ms, or one over 60 ms (Codex 4c impl P1-11)
            self.slow_starts = self.slow_starts + 1 if interval > SLOW_TICK_S else 0
            if interval > LATE_TICK_S or self.slow_starts >= SLOW_STARTS:
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
        if self._watch_trip is not None:
            trip, self._watch_trip = self._watch_trip, None
            rec.branch, rec.reason = "emergency", f"watch:{trip}"
            self._emergency(now_ns, rec.reason)
            return
        feedback = None if s is None or not self.samples else Feedback(
            stamp_mono=self.samples[-1].host_ns / 1e9, rear_axle_speed_mps=float(self.speed.signed_mps),
            lift_m=self._measured_lift(), steering_rad=abs(float(s.steering_rad)), odom_xy=self.odom_pose[:2])
        refusal = core.cargo.submit_refusal() or self._submit_refusal(now_ns)
        self.serial = self.link.emergency(now_ns / 1e9)  # the emergency batch first (its serial: the send's reference)
        self._stop_kind, self._cmd_records = self.session.tick_start(
            t, now_ns / 1e9, feedback, {"submit_refusal": refusal, "clear_for_submit": True,
                                        "localization_ok": True}, collect=lambda: self._collect(now_ns, t, rec))
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

        self.permit.bind(transmit=lambda: self._transmit(now_ns, escalate=False), arbiter_stop=arbiter_stop,
                         collect=lambda r: self._end_now(now_ns, r))

    def _emergency(self, now_ns: int, reason: str) -> None:
        self.waiting = None
        if self.permit.state == ra.LATCHED:
            return  # latched already: its DISARM is retried every 100 ms, not renewed each tick (round 2 P2-2)
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
        lift_ = self._measured_lift()
        applied_ = tuple(float(v) for v in (core._applied_now() or (0.0, 0.0, 0.0)))  # noqa: SLF001
        self.pose_history.append((t, tuple(float(v) for v in rear), self.odom_continuous,
                                  max(abs(self.speed.signed_mps), self.speed.bound_mps), lift_, applied_))
        self._feed_depth(now_ns, t)
        if self.real_layer is not None:
            self._layer_tick(t, rear)
        # the preparation (design 2.2 5)
        if cmds.status == "accepted" and self.frozen is None:
            if self.operator_start is not None:
                lease = self.link.a.lease
                if not self.operator_start.take(None if lease is None else lease.lease_id):
                    cmds.terminate("cancelled", t, "operator_start_unconfirmed")  # its check was used or voided
                    return
            self.frozen = freeze_submit(core)
        if cmds.status in ("accepted", "preparing") and not core.job_pending():
            if self.frozen is not None and self._planning_ready():
                self._planning_gen = int(core.obstacle["version"]) if self.real_layer is not None else None
                known_request(core, t, rear, self.frozen, base_scenario=self.base_scenario, wait_pose=self.wait_pose)
        if core.end_release_due is not None:
            sl = self.evidence.slam(now_ns)
            if sl.mode == "holding" and sl.hold_travel_m > ra.HOLD_MAX_M:
                # a deferred release re-checks the hold's travel each tick, not only at the collection (Codex 4c impl
                # P1-10): too long a hold is not released -- a relocalization instead
                core.require_relocalization(t, "hold_too_long")
        gate_ = core.release_gate(t, rear)
        rear = gate_.rear
        # the layer latency watchdog on every tick, whatever the phase (as the Isaac loop with its budget on; Codex 4c
        # impl P1-12): past its end it is ①, a watchdog stop holds the wheels and the lift (the wait frame)
        layer_ = None if core.obstacle is None else core.obstacle.get("layer")
        layer_stop = layer_.watchdog(t) if layer_ is not None and hasattr(layer_, "watchdog") else None
        if layer_stop in ("layer_latency_exceeded", "layer_worker_dead"):
            rec.branch, rec.reason = "emergency", layer_stop
            self._emergency(now_ns, layer_stop)
            if cmds.status in ("accepted", "preparing"):
                self._end_now(now_ns, layer_stop)  # a preparation the latch does not collect (design 3)
            return
        if self.real_layer is not None and (not getattr(core.options, "obstacle_act", False)
                                            or layer_ is not self.real_layer):
            rec.branch, rec.reason = "emergency", "obstacle_layer_off"  # never released while running (design 3)
            self._emergency(now_ns, "obstacle_layer_off")
            return
        if self.real_layer is not None and cmds.status in ("accepted", "preparing") and core.job_pending():
            # before the adoption (design 1.2, round 3 P2-3/P2-4): a planning answer from another correction
            # generation is dropped and asked again; a fault latches and ends the preparation; a wait defers it
            if self._planning_gen is not None and int(core.obstacle["version"]) != self._planning_gen:
                core.abandon_jobs(t)
            why_ = ra.auto_ready_refusal(now_ns, "drive", self.evidence)
            if why_ is not None and ra.latches(why_):
                rec.branch, rec.reason = "emergency", f"prepare:{why_}"
                self._emergency(now_ns, rec.reason)
                self._end_now(now_ns, rec.reason)
                return
        # as the Isaac loop: paused, resuming or cancelling, or no mission yet (or over) -- a preparation is not
        paused_ = cmds.stopped or cmds.status in ("idle", *FINAL)
        defer_ = self.real_layer is not None and cmds.status in ("accepted", "preparing") and (
            layer_stop is not None or (ra.auto_ready_refusal(now_ns, "drive", self.evidence) or "").startswith("wait:"))
        plan_wait_ = core.jobs is not None and (defer_ or core.job_hold(t, rear, self.speed, paused_).hold)
        if cmds.status in ("accepted", "preparing"):
            plan_wait_ = True
        if cmds.status in FINAL:
            self.frozen = None
        # the adoption and the arming (design 2.3)
        ready_wait = ra.auto_ready_refusal(now_ns, "drive", self.evidence)
        if (cmds.status == "running" and cmds.adopted_id == cmds.mission_id and ms.phase == "approach"
                and self.adopted_for != cmds.mission_id and permit.state == ra.OFF
                and not (ready_wait or "").startswith("wait:")):  # no ARM while a wait holds (Codex 4c impl P2-3)
            self.adopted_for = cmds.mission_id
            why = self._pre_arm_refusal(now_ns)
            cmds.begin_wait(t)
            if not permit.adopted(now_ns, pre_arm_refusal=why):
                rec.branch, rec.reason = "emergency", f"arm:{permit.reason}"
                self._emergency(now_ns, permit.reason or "arm")
                return
        bad = self._unsupported()
        if bad is not None:
            rec.branch, rec.reason = "emergency", f"unsupported:{bad}"
            self._emergency(now_ns, f"unsupported:{bad}")
            return
        if permit.state == ra.ARMING:
            rec.branch = "none"  # no loop DRIVE while arming (the host's own zero only)
            self._assemble_and_arbitrate(now_ns, t, rear, 0.0, 0.0, True, rec, sink=True)
            return
        if permit.state in (ra.READY, ra.DRIVING) and cmds.waits and cmds.waits[-1][1] is None:
            cmds.end_wait(t)
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
        self._lift_before = float(ms.lift_command)  # undone if the output turns into a wait at the first byte
        ramp_hold = layer_stop is not None or (
            permit.state in (ra.READY, ra.DRIVING) and (ready_wait or "").startswith("wait:"))
        if not ramp_hold and not paused_ and (not plan_wait_ or core.lift_ramps_in_plan_wait()) \
                and core.lift_and_timers(t, TICK_S):
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
            if layer_stop is not None:
                requested_speed, curvature, dwell_hold = 0.0, 0.0, True
                self._wait_reason = f"layer:{layer_stop}"
            elif arrival_wait is not None:
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
            at_ = self._pose_at(stamp, applied)
            if at_ is None:
                continue  # no continuous pose interval covers it (or no valid lift then)
            rear_at, speed_at, yaw_rate_at, lift_at = at_
            # the frame's own motion and lift at its acquisition, not this tick's (Codex 4c impl P1-15)
            out = check.add_frame(stamp, frame.depth, rear_at, speed_mps=speed_at, yaw_rate_rps=yaw_rate_at,
                                  lift_m=lift_at)
            pocket["frames_read"] = pocket.get("frames_read", 0) + 1
            still_from = None if self.still_since_ns is None else self._t(self.still_since_ns)
            if (pocket.get("await_still_frame") and (out or {}).get("new") and self._wait_started_s is not None
                    and stamp >= self._wait_started_s and still_from is not None and stamp >= still_from
                    and t - (stamp - lag) <= max_age):
                pocket["await_still_frame"] = False
                pocket["still_frame_s"] = stamp

    def _pose_at(self, stamp: float, correction=None):
        """The pose interpolated at ``stamp`` over a continuous interval, with the motion bounding it: the larger
        speed bound of its two ends, the interval's yaw rate (at least the bound of either neighbouring interval) and
        the lift nearer in time -- None when no continuous interval covers it or the lift was not valid then."""
        # only adjacent poses both under the frame's own SLAM correction (round 2 P1-8): a pose of another correction
        # between two of its own breaks the interval (round 3 P1-4) -- such a pose is kept, marked not continuous
        h = [e[:5] if correction is None or e[5] == tuple(correction) else (e[0], e[1], False, e[3], e[4])
             for e in self.pose_history]

        def rate(i):
            if i < 0 or i + 1 >= len(h) or not (h[i][2] and h[i + 1][2]) or h[i + 1][0] <= h[i][0]:
                return 0.0
            d = h[i + 1][1][2] - h[i][1][2]
            return abs(math.atan2(math.sin(d), math.cos(d))) / (h[i + 1][0] - h[i][0])

        for i, ((t0, p0, c0, v0, l0), (t1, p1, c1, v1, l1)) in enumerate(zip(h, h[1:])):
            if t0 <= stamp <= t1 and c0 and c1:
                a = 0.0 if t1 <= t0 else (stamp - t0) / (t1 - t0)
                lift = l0 if a < 0.5 else l1
                if lift is None:
                    return None
                return (tuple(p0[k] + a * (p1[k] - p0[k]) for k in range(3)), max(v0, v1),
                        max(rate(i - 1), rate(i), rate(i + 1)), float(lift))
        if h and abs(h[-1][0] - stamp) < 1e-9 and h[-1][2] and h[-1][4] is not None:
            return h[-1][1], h[-1][3], rate(len(h) - 2), float(h[-1][4])
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
        send_ns = int(self.clock())
        elapsed = max(0.0, (send_ns - now_ns) / 1e9)  # the tick's own time before its first write (design 3)
        rec.elapsed_s = elapsed
        late = None
        if rec.branch != "emergency" and permit.state in (ra.READY, ra.DRIVING) \
                and (rec.branch == "s10" or getattr(self, "_candidate", None) is not None):
            late = self._send_refusal(now_ns, send_ns, elapsed)
        if rec.branch == "emergency":
            pass  # the latch queued the DISARM
        elif late is not None:
            # the authority and the evidence at the send, not at the tick's start (round 2 P1-1): S10's stop included
            rec.branch, rec.reason = "emergency", late
            self._emergency(now_ns, late)
        elif rec.branch == "s10" and permit.state in (ra.READY, ra.DRIVING):
            lift, q = self._measured_lift(), self._measured_steering()
            if lift is None or q is None:
                self._emergency(now_ns, "s10_measurement")
            else:
                r = port.control_stop(steering_rad=q, lift_m=lift, now_ns=now_ns,
                                      expires_ns=now_ns + int(DRIVE_LIFE_S * 1e9))
                if r.kind != "queued":
                    self._emergency(now_ns, f"s10_stop:{r.reason}")
                else:
                    self._book(now_ns, r.seq, 0.0, 0.0, (q,), lift, False, "stop", steer=(q,))
                    permit.control_stop(now_ns, r.seq)
                    self._s10_stopping = True
                    rec.seqs.append(r.seq)
        elif permit.state in (ra.READY, ra.DRIVING) and getattr(self, "_candidate", None) is not None:
            command_, steer, out, sink = self._candidate
            wait = self._wait_reason_now(send_ns, out)
            if wait is not None or elapsed > SLOW_TICK_S:
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

    def _cut_stuck_write(self, now_ns: int):
        """A frame started and not finished by its own expiry (design 8), read on the clock (round 3 P1-2): the link
        is lost -- closed and reopened (the half frame abandoned, never finished), the stop left to the MCU's command
        expiry meanwhile, the first frame after it the DISARM. True when it cut."""
        partial = getattr(self.transport, "partial_frame", None)
        stuck = getattr(self.transport, "partial_since_ns", None)
        deadline = None
        if partial is not None and getattr(partial, "expires_ns", None) is not None:
            deadline = int(partial.expires_ns)  # the frame's own expiry, not its first byte's (round 2 P2-1)
        elif stuck is not None:
            deadline = int(stuck) + int(DRIVE_LIFE_S * 1e9)
        if deadline is None or int(self.clock()) < deadline:
            return None
        try:
            self.transport.resync(self.host, now_ns)
        except Exception as exc:  # noqa: BLE001 -- still stuck: judged again next tick, the permit still ticking
            return f"link_stuck:resync_failed:{type(exc).__name__}"
        return "link_stuck"

    def _lease_valid(self, ns: int) -> bool:
        """The operator's lease still held at ``ns`` by the arbiter's own expiry rule (a lease the next ``begin`` would
        drop is no authority now)."""
        a = self.link.a
        lease = a.lease
        return lease is not None and ns / 1e9 <= lease.renewed_mono + a.config.lease_timeout_s

    def _send_refusal(self, now_ns: int, send_ns: int, elapsed: float) -> str | None:
        """Read on the clock just before the first write (design 3): the tick's own time, the measurement, the
        readiness evidence (a scan or a SLAM answer that aged during the tick) and the lease -- any is ①."""
        if elapsed > LATE_TICK_S:
            return "tick_late"
        if not self.host.health(send_ns).fresh:
            return "measurement_stale"
        why = ra.auto_ready_refusal(send_ns, "drive", self.evidence)
        if why is not None and ra.latches(why):
            return f"send:{why}"
        if not self._lease_valid(send_ns):
            return "lease_expired"
        return None

    def _wait_reason_now(self, now_ns, out) -> str | None:
        hold = self.core.state.pop("motion_hold", None)  # this tick's, consumed whichever reason is returned
        why = ra.auto_ready_refusal(now_ns, "drive", self.evidence)
        if why is not None and ra.latches(why):
            self._emergency(now_ns, why)
            return "latched"
        if why is not None:
            return why
        if hold is not None:
            return f"hold:{hold}"  # an unconfirmed output beyond what is allowed: wheels 0 now, its queue dropped
        return getattr(self, "_wait_reason", None)

    def _wait_frame(self, now_ns: int, rec: TickRecord | None) -> None:
        port = self.port
        if self._lift_before is not None:
            # the output holds: this tick's lift ramp step is undone, whichever stage chose the wait (round 7 P2-2)
            self.core.ms.lift_command = self._lift_before
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
                   measured is None or abs(self.waiting[1] - measured) > port.lift_still_m, "hold",
                   steer=(self.waiting[0],))
        if rec is not None:
            rec.seqs.append(r.seq)

    def _auto_drive(self, now_ns: int, command_, steer, rec: TickRecord) -> None:
        port, permit = self.port, self.permit
        # the newest decision only: every unsent older DRIVE (a held moving frame, the wait's hold) is dropped before
        # this one is queued -- a tick that decided to stop never lets an older moving frame out first (Codex PR 5
        # design round 6 P1-1). The wait itself (its target) ends only when this DRIVE is written -- refused at its
        # first byte, the wait goes on at the same target (4c round 5 P1-4).
        self.host.drop_drives("superseded")
        partial = getattr(self.transport, "partial_frame", None)
        partial_seq = None if partial is None else int(partial.seq) & 0xFFFF
        for f in [f for f in self.book.frames if f.written_ns is None and f.seq != partial_seq]:
            self.book.discarded(f.seq)  # never written, now never will be: it cannot act (a half-written one may)
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
        movable = speed != 0.0 or (self._measured_lift() is None
                                   or abs(lift - self._measured_lift()) > port.lift_still_m)
        self._book(now_ns, r.seq, speed, float(drive.curvature_inv_m),
                   tuple(float(v) for v in np.asarray(drive.steering_rad)), lift, movable, "drive", steer=steer)
        permit.drove(now_ns)
        rec.seqs.append(r.seq)

    def _book(self, now_ns, seq, speed, kappa, steering, lift, movable, kind, *, steer=None) -> None:
        s = self.host.state
        seq = int(seq) & 0xFFFF
        a = self.link.a
        # the authority it was decided under: a later lease or control generation never writes it (round 2 P1-2)
        self._binding[seq] = (None if a.lease is None else a.lease.lease_id, a.control_gen)
        if steer is not None:
            self._steer_of[seq] = np.asarray(steer, dtype=float).copy()
        if movable and s is not None:
            # its MCU deadline is at most the echo (this STATE's MCU time) plus the host's validity cap
            until = int(s.mcu_time_us) + int(self.host.cfg.cap_ms) * 1000
            self._motion_mcu_until = until if self._motion_mcu_until is None else max(self._motion_mcu_until, until)
        for old in [k for k in self._binding if k not in {f.seq for f in self.book.frames} and k != seq]:
            self._binding.pop(old, None)
            self._steer_of.pop(old, None)
        self.book.queued(Frame(int(seq) & 0xFFFF, None if s is None else s.boot_id, None if s is None else s.arm_gen,
                               now_ns + int(DRIVE_LIFE_S * 1e9), float(speed), float(kappa), tuple(steering),
                               float(lift), bool(movable), kind, queued_ns=int(now_ns)))

    def _transmit(self, now_ns: int, *, escalate: bool = True) -> None:
        """Every queued frame through the send boundary (design 2.5 5, 5, 8). The transport calls ``gate(frame, write)``
        for each frame's first byte and again before it continues a frame it started; the write happens inside it.
        A DISARM always passes. Any other frame (ARM, the host's zero, a loop DRIVE, S10's stop) is judged under the
        channels' lock just before its write (Codex 4c impl round 4 P1-3): no emergency event since this tick's batch,
        the arbiter not STOPPED, the tick's time, its own expiry, the measurement, readiness, the lease by its expiry
        rule and the authority it was decided under. Once any of those fails, no other frame of this call moves
        (round 4 P1-1); a failure of the authority or the evidence is ① in this very tick, its DISARM written first
        (the latch drops the queue); a new wait reason, or 30 ms passed, turns a normal moving DRIVE into ③. The
        permit's own transmits (``escalate=False``) never come back to ①."""
        motion = motion_allowed(self.transport)
        a, permit = self.link.a, self.permit
        refused, alarm, waits = [], [], []
        authority = (ra.ARMING, ra.READY, ra.DRIVING, ra.STOP_PENDING)
        can_escalate = escalate and not self._in_transmit

        def escalate_now(reason: str) -> None:
            if can_escalate and permit.state in authority:
                rec_ = getattr(self, "_rec", None)
                if rec_ is not None:
                    rec_.branch, rec_.reason = "emergency", f"first_byte:{reason}"
                self._in_transmit = True
                try:
                    self._emergency(now_ns, f"first_byte:{reason}")
                finally:
                    self._in_transmit = False

        stuck = self._cut_stuck_write(now_ns)
        if stuck:
            alarm.append(stuck)
            permit.link_reset(now_ns)
            escalate_now(stuck)  # the DISARM queued (and the queue dropped) before anything else is sent

        def judge(frame) -> tuple:
            """(reason, class) for a non-DISARM frame now; class: alarm | wait | drop. The readiness evidence is read
            first (its ages at ``ns0``); everything timed is then read on the clock after it, and a read that took
            more than EVIDENCE_READ_MAX_S refuses (round 5 P1-3)."""
            if permit.state not in authority:
                return f"permit_{permit.state.lower()}", "drop"  # a latch or a DISARM under way: no motion (P1-1)
            if a.mode == "STOPPED":
                return "stopped", "alarm"
            # every timed judgement at ``ns``, the latest instant the write may happen: the evidence's ages too (a
            # scan judged young at the read's start may be over its limit at the write, round 6 P1-1)
            ns0 = int(self.clock())
            ns = ns0 + int(EVIDENCE_READ_MAX_S * 1e9)
            why = ra.auto_ready_refusal(ns, "drive", self.evidence)
            if int(self.clock()) > ns:
                return "evidence_slow", "alarm"
            if (ns - now_ns) / 1e9 > LATE_TICK_S:
                return "tick_late", "alarm"  # a late tick is ① whatever its frame (design 3)
            if not self.host.health(ns).fresh:
                return "measurement_stale", "alarm"
            if frame.kind == "ARM" and why is not None and not ra.latches(why):
                return f"arm_wait:{why}", "alarm"  # no new ARM while a wait holds: latch, a rearm (design 3)
            if why is not None and ra.latches(why):
                return f"send:{why}", "alarm"
            if not self._lease_valid(ns):
                return "lease_expired", "alarm"
            seq = int(frame.seq) & 0xFFFF
            bound = self._binding.get(seq)
            if bound is not None and bound != (None if a.lease is None else a.lease.lease_id, a.control_gen):
                return "authority_changed", "alarm"
            # every alarm first: an old frame's expiry never hides one (round 7 P2-1)
            expires = getattr(frame, "expires_ns", None)
            if expires is not None and ns >= int(expires):
                return "expired", "drop"  # an old frame left in the queue: dropped, no alarm by itself
            booked = next((f for f in self.book.frames if f.seq == seq), None)
            if booked is not None and booked.kind == "drive":
                # a normal DRIVE (moving or a small lift step alike, round 6 P1-3): a wait reason that came up since the
                # decision, or 30 ms spent (round 4 P1-4)
                if why is not None:
                    return why, "wait"
                if (ns - now_ns) / 1e9 > SLOW_TICK_S:
                    return "tick_slow", "wait"
            return None, None

        def gate(frame, write) -> bool:
            if frame.kind == "DISARM":
                write()
                return True
            continuing = frame is getattr(self.transport, "partial_frame", None)
            if not motion or alarm:
                verdict = ("device_block" if not motion else "after_alarm", "drop")
                ok = False
            else:
                verdict = None

                def check() -> bool:
                    nonlocal verdict
                    verdict = judge(frame)
                    return verdict[0] is None

                ok = send_boundary(self.channels, self.serial, write, check=check)
                if not ok and (verdict is None or verdict[0] is None or self.channels.serial() != self.serial):
                    # a STOP, DISARM or (re)connect since the batch (round 4 P2-3), also one taken during the check --
                    # whatever else the check found (round 7 P2-1)
                    verdict = ("emergency_event", "alarm")
            if not ok:
                refused.append((frame.kind, frame.seq))
                why, cls = verdict
                if continuing:
                    alarm.append("link_stuck")  # a half frame not finished: the transport abandons it (link reset)
                elif cls == "alarm":
                    alarm.append(why)
                elif cls == "wait":
                    waits.append(why)
            return ok

        if self.real_layer is not None and permit.anchor_disarm(now_ns):
            pass  # an anchor DISARM queued (design 1.1): written with this send
        for kind, seq, written_ns in self.transport.transmit(self.host, now_ns, gate):
            if kind == "DRIVE":
                self._written_now.append(int(seq) & 0xFFFF)  # the record's DRIVEs (a DISARM is not one)
            permit.written(kind, seq, written_ns)
            if self.delay_bound is not None:
                self.delay_bound.written(seq, written_ns)
                pair = permit.session_pair()  # after the permit took the write: the second DISARM's seq is known
                if pair is not None:
                    self.delay_bound.expect_step(*pair)
            if kind == "DRIVE":
                self.book.written(seq, written_ns)
                booked = next((f for f in self.book.frames if f.seq == (int(seq) & 0xFFFF)), None)
                if booked is not None and booked.kind == "drive":
                    self.waiting = None  # a normal DRIVE out: the wait is over
                steer = self._steer_of.get(int(seq) & 0xFFFF)
                if steer is not None:
                    self.steering_command = steer.copy()  # the next slew starts from what was written (P2-5)
        rec_ = getattr(self, "_rec", None)
        if rec_ is not None and refused:
            gone_ = {int(q) & 0xFFFF for _, q in refused}
            rec_.seqs = [q for q in rec_.seqs if (int(q) & 0xFFFF) not in gone_]  # the record keeps written seqs only
        for kind, seq in refused:
            if kind == "DRIVE":
                self.book.discarded(seq)
                permit.stop_discarded(now_ns, seq)
                self.steering_command = None  # an unwritten candidate is never the next slew's start
        if "link_stuck" in alarm and not stuck:
            permit.link_reset(now_ns)
        if alarm and not stuck:
            escalate_now(alarm[0])
        elif waits and can_escalate and permit.state in (ra.READY, ra.DRIVING):
            # ③ in place of the normal candidate: the wait frame (a new generation drops the old queue), once
            rec = getattr(self, "_rec", None)
            if rec is not None:
                rec.branch, rec.reason = "wait", waits[0]
                gone = {int(q) & 0xFFFF for k_, q in refused}
                rec.seqs = [q for q in rec.seqs if (int(q) & 0xFFFF) not in gone]
            self._in_transmit = True
            try:
                self._wait_frame(now_ns, rec)
            finally:
                self._in_transmit = False
            self._transmit(now_ns)  # a hold frame never turns into a wait again; its own failure is still ①


__all__ = ["RealLoop", "RealSession", "TickRecord", "motion_allowed", "real_host"]
