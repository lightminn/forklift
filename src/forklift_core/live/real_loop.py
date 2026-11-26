"""The real truck's 50 Hz control loop (docs/design/2026-11-19-real-loop.md; contract
docs/design/2026-11-18-real-control-tick.md). No device here: the transport is injected (4c: a fake MCU only).

``RealSession`` is the minimal mission lifecycle (the Isaac ``LiveRuntime``'s: the console cycle, a mission's end
collected once, the results). ``RealLoop.cycle(now_ns)`` is one tick, in the contract's order: the head (STATEs,
the host, the AUTO permit, the output record, the watches, the console), SLAM and the preparation, the adoption and
the arming, the control (S10-A's branches only -- anything else latches before it acts), the output assembly, one
output branch (emergency DISARM -> S10 control stop -> an output wait -> the normal AUTO DRIVE) and the send.
"""

from __future__ import annotations

import dataclasses
import math
import numbers
import secrets
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np

from forklift_core.live import real_autonomy as ra
from forklift_core.live.odom_history import ENCODER_CUT, odom_entry, odometry_step
from forklift_core.live.output_book import Frame, OutputBook
from forklift_core.live.real_assembly import freeze_submit, known_request
from forklift_core.live.real_channels import send_boundary
from forklift_core.live.real_teleop import counts_still, encoder_speed, real_auto_refusal
from forklift_core.localization.slam_pose import SlamPoseTracker, compose, invert
from forklift_core.localization.start_reloc import base_from_rear, rear_from_base
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
SAMPLES_KEPT = 128
WRITE_MARGIN_S = SLOW_TICK_S + EVIDENCE_READ_MAX_S  # 5d (7.1 6): the latest write of a tick's DRIVE (core.WRITE_MARGIN_S)
# 6a (docs/design/2026-11-20-real-hil-6a.md 2): the last clock read is this much before the bound it is held to (the
# read-to-write latency, measured on the board day); a position-holding frame's sample at most this old, at the frame's
# making and again at its write; the axis watch's alarm thresholds over a holding span (provisional, S4)
WRITE_ALLOWANCE_S = 0.001
HOLD_SAMPLE_MAX_AGE_S = 0.070  # one 20 ms read period + link delay headroom 10 ms + W 35 ms + the 5 ms lookahead
WAIT_LIFT_RISE_M = 0.005
WAIT_STEER_MOVE_RAD = 0.02
HOLDING_KINDS = ("hold", "stop")  # the book's position-holding frames (a wait's control stop and holds, S10's stop)
# 5c-2 (docs/design/2026-11-19-real-safety-wiring.md 6): relocalization
RELOCALIZE_WAIT_S = 3.0  # an attempt's limit from its acceptance (provisional)
# PR 7 design 6.1, 2: a declaration on a source with seed end states waits for its seed's end state (the executor's 20 s
# seed limit) and commits within 0.5 s after it -- one limit from the acceptance (provisional)
SEED_WAIT_S = 20.0
DECLARE_COMMIT_S = 0.5
START_TOL_M = 0.10  # the registered start's tolerance (provisional)
START_TOL_RAD = math.radians(5.0)
DECLARED_SPEED_MPS = 0.06  # the slam-bypass / declared-start cap (provisional, S10 speed approval)


def real_host(cfg, *, initial_seq: int | None = None, client: Client = Client.REAL, **kw) -> McuLinkHost:
    """The executor's host: the real client unless the run says HIL (6a-2b design 10.1: the HIL board's ARM is refused
    to Client.REAL), its first seq random unless given (4c design: a new executor's frames are never mistaken for an
    earlier one's by seq; the teleop bridge does the same)."""
    seq = secrets.randbelow(0x10000) if initial_seq is None else int(initial_seq)
    return McuLinkHost(cfg, client=Client(client), initial_seq=seq, **kw)


def motion_allowed(transport) -> bool:
    """The device block (4c design 5): ARM and every DRIVE (the normal one, the host's zero after ARM, the wait frame,
    S10's stop frame) go out only on a transport that declares itself the internal fake MCU (``fake_mcu is True``);
    any other transport -- an operating serial link included -- passes the DISARM only. The real MCU's AUTO opens
    with PR 5 and S10's preconditions. 6a (design 1): or a transport that verified an output-locked HIL board
    (``hil_plant is True``, the loop TTY transport's own check of every STATE of the current boot). The operating
    unlock (docs/plans/2026-11-26-real-operating-assembly.md 2.1): or the real link's own permission
    (``real_operating is True``: granted after the operating preconditions held, revoked on a loss, a boot or build
    change) -- every other send check stays."""
    return (getattr(transport, "fake_mcu", False) is True or getattr(transport, "hil_plant", False) is True
            or getattr(transport, "real_operating", False) is True)


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
    notes: list = field(default_factory=list)  # 6a: what the tick skipped (wait_frame_skipped:<why>)
    stop_decided_ns: int | None = None  # 6a-2b (design 10.7): when the tick body first decided a stop or a wait


class RealSession:
    """The console cycle and a mission's end (the Isaac ``LiveRuntime``'s minimum)."""

    def __init__(self, core, link) -> None:
        self.core, self.link = core, link
        self.results: list = []
        self._ended_id = None
        # 6a (design 5): this tick's apply records once made (None: not yet), and whether link.begin was entered
        self.last_records: list | None = None
        self.began = False
        # a mission collection that raised (6a impl rounds 2-5): not retried part-way (its steps are not idempotent);
        # the run refuses new missions until the executor restarts (the stopping side)
        self.collect_failed: str | None = None

    def tick_start(self, t: float, now_mono: float, feedback, context_kwargs: dict, collect=None) -> tuple:
        """``link.begin`` -> collect -> ``commands.apply`` -> collect (design 2.1 5): a mission the head ended (a latch
        in ``permit.tick``) is collected before the apply could clear its final status."""
        from forklift_core.mission.commands import Context

        self.began = True  # before the call: a begin that raises part-way has taken requests in (6a design 5)
        stop_kind = self.link.begin(now_mono, feedback)
        if collect is not None:
            collect()
        records = self.core.commands.apply(t, Context(stop_kind=stop_kind, **context_kwargs))
        self.last_records = records  # kept before the second collect (a failing collect leaves them settled)
        if collect is not None:
            collect()
        return stop_kind, records

    def tick_end(self, now_mono: float, records: list, safety: dict, candidate: tuple, drive: dict | None):
        return self.link.end(now_mono, records, safety, candidate, drive)

    def ended_unclaimed(self) -> bool:
        cmds = self.core.commands
        return cmds.status in FINAL and cmds.mission_id is not None and cmds.mission_id != self._ended_id

    def end_mission(self, t: float, *, lift_m: float | None, release_allowed, bypassed=None) -> dict | None:
        """Collect the mission the commands ended, once: the lift held at its measured place, the jobs dropped, the
        open stops closed, the clock taken, the end reset (the loop's release decision passed), the result kept."""
        core, cmds = self.core, self.core.commands
        mission_id, mission = cmds.mission_id, cmds.mission
        if mission_id is None or mission_id == self._ended_id:
            return None
        self._ended_id = mission_id
        clock = None
        try:
            if lift_m is not None:
                core.ms.lift_command = float(lift_m)
            core.abandon_jobs(t)
            cmds.close_open(t)
            if cmds.end_logged_for != mission_id:
                cmds.note_end(t, cmds.end_reason)
            clock = cmds.take_clock()
            core.end_reset(t, release_allowed=release_allowed)
        except Exception as exc:
            # 6a impl rounds 2-5: kept as collected with the failure; the caller's emergency latches, and no new
            # mission runs on a half-reset core until the executor restarts
            self.collect_failed = f"{type(exc).__name__}:{exc}"
            self.results.append({"mission_id": mission_id, "outcome": cmds.status, "reason": cmds.end_reason,
                                 "ended_s": float(t), "pallet_id": None if mission is None else mission.pallet_id,
                                 "destination_id": None if mission is None else mission.destination_id,
                                 "clock": clock, "collect_failed": self.collect_failed,
                                 **({} if bypassed is None else {"bypassed": list(bypassed)}),
                                 **_excluded(bypassed)})
            raise
        entry = {"mission_id": mission_id, "outcome": cmds.status, "reason": cmds.end_reason, "ended_s": float(t),
                 "pallet_id": None if mission is None else mission.pallet_id,
                 "destination_id": None if mission is None else mission.destination_id, "clock": clock,
                 **({} if bypassed is None else {"bypassed": list(bypassed)}), **_excluded(bypassed)}
        self.results.append(entry)
        return entry


def _excluded(bypassed) -> dict:
    """A mission run with the F1 start block bypassed counts in neither the F1 nor the validation figures (lead #623
    decision 2, 7c-2b): marked on its result entry, and the runner's counts leave it out."""
    return {"excluded_counts": ["f1", "validation"]} if bypassed is not None and "f1" in bypassed else {}


def mission_counts(results) -> dict:
    """The session's missions for the F1 and validation figures: all, those counted, those left out (f1 bypassed)."""
    rows = list(results)
    out = [r for r in rows if "f1" in (r.get("excluded_counts") or [])]
    return {"all": len(rows), "counted": len(rows) - len(out), "excluded_f1": len(out)}


# real safety wiring design 5 (5c-1): the checks the operator may bypass for the session, and the names refused
BYPASSABLE = ra.READY_BYPASSABLE + ("imu", "watch_current", "watch_wheel", "start_pose",  # start_pose: 5c-2 (6, 7)
                                    "d5_pocket",  # 5d (7.2)
                                    "f1")  # PR 7 design 6.1, 7 (7c-2b, lead #623 decision 2): the F1 start block
NOT_BYPASSABLE = ("stop", "disarm", "lease", "rearm", "link", "measurement_stale", "link_stuck", "layer_worker",
                  "obstacle_layer", "overspeed", "loop_slow", "worker", "baseline", "core",
                  "relocalize",  # relocalize: the declaration's session mark is no toggle (5c-2, 7b)
                  "wait_axis_moved")  # 6a: the axis watch over a holding span (design 2, lead decision A')

# the operating REAL run's checks that are not bypassable there although they are elsewhere (unlock U3a, plan 2.5:
# with no deadline judged a late layer drives on old evidence -- a stopping-side protection)
OPERATING_NOT_BYPASSABLE = ("layer_timing",)

# the layer watchdog's ends: the latch at once (a wait is "layer_overdue"); scan_generation_changed: the operating
# run's scan evidence boundary (U3a)
LAYER_ENDS = ("layer_latency_exceeded", "layer_worker_dead", "scan_source_lost", "scan_generation_changed")


def _evidence(reply) -> tuple:
    """PR 7 design 6.1, 0: a reply's first-beam evidence interval [Flo, Fhi] (core seconds); a source without one (the
    synthetic and Isaac ones) has Flo = Fhi = its order stamp."""
    stamp = float(reply.stamp_s)
    lo, hi = getattr(reply, "evidence_lo_s", None), getattr(reply, "evidence_hi_s", None)
    return (stamp if lo is None else float(lo)), (stamp if hi is None else float(hi))


def _imu_int(v):
    return None if isinstance(v, bool) or not isinstance(v, numbers.Integral) else int(v)


def _imu_item(item):
    """``(gen, lo, hi, accel)`` from a source's sample -- ``(stamp, accel)`` is the point ``lo == hi`` with no
    generation --, or ``hi`` None when its times are not integers or it has neither shape (PR 7 design 5). A source
    that has generations sends its point samples as ``(gen, t, t, accel)`` too: a 2-tuple among them reads as a
    generation change (a lost watch while armed)."""
    try:
        if len(item) == 2:
            stamp, accel = item
            t = _imu_int(stamp)
            return None, t, t, accel
        gen, lo, hi, accel = item
    except (TypeError, ValueError):
        return None, None, None, None
    lo, hi = _imu_int(lo), _imu_int(hi)
    if lo is None or hi is None or (gen is not None and _imu_int(gen) is None):
        return None, None, None, None
    return gen, lo, hi, accel


class RealLoop:
    def __init__(self, *, core, session: RealSession, permit, port, host, link, channels, transport, params,
                 odometry, kinematics, max_speed_mps: float, start_ns: int, slam_source, evidence: ra.Evidence,
                 base_scenario, wait_pose, delay_s: float = 0.0, steering_ok=None, record=None,
                 depth_source=None, watches=None, clock=None, operator_start=None, real_layer=None,
                 delay_bound=None, state_delay_s: float = 0.03, v_scan_mps: float | None = None,
                 kappa_max_inv_m: float = 0.0, pallet_size=None, imu_source=None, start_rear=None,
                 rear_offset_m: float | None = None, declared_speed_mps: float = DECLARED_SPEED_MPS,
                 scan_feed=None, events=None, operating: bool = False, scan_events=None,
                 imu_unqualified: bool = False) -> None:
        self.core, self.session, self.permit, self.port, self.host = core, session, permit, port, host
        # the operating REAL run (unlock U3a, named by the assembly -- not inferred from the client, which the 4c rigs
        # share): its non-bypassable checks, its scan source's events judged at the tick head before any install, and
        # an IMU that did not qualify (C7: no watch, no source; readiness refuses imu:unqualified until bypassed)
        self.operating, self.scan_events = bool(operating), scan_events
        self.imu_unqualified = bool(imu_unqualified) and self.operating
        self.scan_boundaries = 0
        self.scan_refusal = None  # () -> the scan source's newest refusal reason (the run sets it; plan 2.5 console)
        # 6a-2b (design 10): the run's scan feed after the tick's STATE (None: the rig feeds before cycle), a source's
        # fatal error (set by the run's source guard or here; the run ends on it), the closing state (DISARM only,
        # no source called) with the DISARMs written in it, an event sink for the run's record (never blocks)
        self.scan_feed, self.events = scan_feed, events
        self.fatal_source: str | None = None
        self.closing = False
        self.closing_started_ns: int | None = None
        self.closing_disarms: list = []  # (boot, seq, written_ns) of each DISARM written whole while closing
        self.closing_confirmed_ns: int | None = None
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
        self.odom_travel = 0.0  # the raw odometry's path length (both directions), for the layer's verified distance
        self._travel_ticked = 0.0  # the path length the layer was last told about
        self._break_open = False  # a same-boot odometry break has asked its relocalization (once per break)
        self._ever_continuous = False  # the start's own first reference is not a break of anything
        self._odom_good = False  # the newest STATE did not break the odometry
        self._resession = False  # a reboot seen: the session check runs again once the permit is off or latched
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
        # PR 7 7r (plan 2-3): the replay run's anchor -- called once with the receipt of the first STATE whose delay
        # bound is finite -- and the HIL check's round (raised at each loss and boot change the HIL transport shows;
        # an attempt completes only on the round it began in, with the check held)
        self.on_first_bounded_state = None
        self.anchor_rx_ns: int | None = None
        self._hil_client = getattr(host, "client", None) is Client.HIL
        self.hil_check_round = 0
        self._hil_seen = None
        self.operator_start = operator_start  # the operator's physical check, one submit only (4b design 2, 13)
        # 5a (docs/design/2026-11-19-real-safety-wiring.md 1-3): the real obstacle layer, the delay bound for the
        # STATEs' measurement intervals, the raw odometry history the layer places scans on
        self.real_layer = real_layer
        if real_layer is not None:
            real_layer.defer_submit = True  # placements at the tick head, the computation after this tick's track()
        self.delay_bound = delay_bound
        self.state_delay_s = float(state_delay_s)
        self.v_scan_mps = (1.5 * float(max_speed_mps)) if v_scan_mps is None else float(v_scan_mps)
        self.kappa_max = float(kappa_max_inv_m)
        self.pallet_size = pallet_size  # (depth, width) of the S10-A pallet (s10a.yaml): the planning grid's split
        self.odom_history: deque = deque(maxlen=256)  # (lo_s, hi_s, pose, continuous, included, speed_ok, yaw_ok)
        self._layer_asked = None  # the last (mission gen, path id, loaded) a refresh was asked for
        self._planning_gen = None  # the correction generation the planning request used (design 1.2, round 3 P2-3)
        # the watches as build_watches assembled them, whole (5b, safety wiring design 4: the policy travels with
        # the watches -- no SafetyWatch alone without its IMU policy); None: off (the 4c and 5a rigs)
        safety, imu_watch, imu_policy = (None, None, None) if watches is None else (
            watches.safety, watches.imu, watches.imu_policy)
        if watches is not None and imu_policy not in ("required", "none"):
            raise ValueError(f"imu_policy {imu_policy!r}")
        if self.imu_unqualified:
            imu_watch, imu_source = None, None  # nothing judged from a source that did not qualify (plan 2.7)
        elif imu_policy == "required" and (imu_watch is None or imu_source is None):
            raise ValueError("imu_required")
        if (imu_watch is None) != (imu_source is None):
            raise ValueError("imu_watch and imu_source go together")
        self.watch = safety  # the real safety watch (currents and wheels, design 3)
        self.imu_watch, self.imu_source, self.imu_policy = imu_watch, imu_source, imu_policy
        self.imu_backlog = False  # the last IMU read reached its per-call cap (6a-2b design 10.7)
        self.imu_gen_unknown = False  # the last read could not ask the source's generation (PR 7 design 5)
        self._imu_wait_until: dict = {}  # mission id -> the baseline's wait deadline (host ns), set once
        self._watch_rx: int | None = None
        # the host monotonic clock the tick's own elapsed time is read on before the first write (design 3); the
        # tests' fake clock passes the tick's instant
        self.clock = clock or time.monotonic_ns
        self._console_disarm = False
        self._odom_broken = False  # a continuity break among this tick's STATEs
        # PR 7 design 6.1, 2 (7c-1b-2a): raised at every odometry break, reboot and device loss, never lowered -- the
        # real SLAM source compares it with a seed's own, since a break clears the history it would otherwise read
        self.odom_gen = 0
        # (odometry history entry) -> None, called with every entry as it is made: the real SLAM source judges a
        # seed's stillness, continuity and inclusion on every one, not only those a poll still finds in the history
        # (Codex 7c-1b-2a rounds 3-4)
        self.odom_listeners: list = []
        # 5c-1 (design 5): the session's bypass set (applied at a tick head; this tick's copy judges), the operator's
        # answers by serial (the first only), the last refusal told on real_status
        self.bypass: set = set()
        self._bypass_epoch = link.a.session_epoch
        self._bypass_pending: list = []
        self._tick_bypass: frozenset = frozenset()
        self.operator_answers: dict = {}
        self.last_refusal = None
        self.bypass_events: list = []
        if getattr(link.a.config, "real_requests", False):
            link.a.real_request = self._real_request
            link.a.real_rejected = self._real_rejected  # the arbiter's own refusals of them too (r5 P3-2)
        self._rebooted = False  # a boot change seen (not the first boot)
        self._disconnects = host.disconnects  # the host's device losses seen so far
        self._last_seen = None  # the newest STATE's (boot, MCU time)
        self._lift_before = None  # the lift target before this tick's ramp
        self._written_now: list = []
        self._in_transmit = False
        self._motion_mcu_until: int | None = None  # the latest MCU time a movable frame may act to (its boot's clock)
        self._watch_trip: str | None = None  # the safety watch's trip on a STATE of this tick
        self._binding: dict = {}  # seq -> (lease id, arbiter control generation) a loop frame was decided under
        # 6a (design 2): the clock value the last judgement compared (passed to the write), each position-holding
        # frame's sample acquisition (host ns), the wait frames made this tick, the axis watch and its trip, the
        # per-STATE delay bounds by (boot, MCU time), the watch's judging instants (the HIL bound's evidence)
        self._judged_ns: int | None = None
        self._arm_binding: tuple | None = None  # (lease id, control generation) the ARM was asked under (6a)
        self._kept: set = set()  # seqs of ARMs / arming zeros sent back to the queue once (re-judged on echo windows)
        self._cancel_arm: str | None = None  # an arming exchange to give up after this send (6a)
        self._lost_seen = False  # the transport's device loss handled once (6a-2a)
        self._hold_acq: dict = {}
        self._wait_frames_tick = 0
        self._axis: dict | None = None
        self._axis_trip: str | None = None
        self._sample_delay: dict = {}
        self.axis_watch_times: deque = deque(maxlen=1024)
        # (clock ns, reason) of each stop the permit decides -- a latch or a DISARM, the loop's or its own -- taken at
        # the transition, before anything is queued or sent (6a impl rounds 1-4: the axis watch's last gap ends here)
        self.disarm_decisions: deque = deque(maxlen=256)
        permit.on_stop = self._on_stop_decided
        self._cause = None  # 6a-2b (impl round 5): this tick's latest latching evidence (instant, reason)
        self._tick_ns = None
        self._steer_of: dict = {}  # seq -> the steering command a loop frame carries (the next slew's start once written)
        link.a.on_disarm = self._on_console_disarm  # AUTO has no arbiter `armed` to drop: the loop's ① (design 2.1 4)
        # 5c-2 (design 6): relocalization. The registered start (rear axle, map) and the rear-axle offset (base_link ->
        # rear) come from the S10-A config and the chassis model; None only in rigs built without them
        self.start_rear = None if start_rear is None else tuple(float(v) for v in start_rear)
        self.rear_offset_m = None if rear_offset_m is None else float(rear_offset_m)
        self.declared_speed_mps = float(declared_speed_mps)
        tracker_ = (core.slam or {}).get("tracker") if core.slam is not None else None
        if isinstance(tracker_, SlamPoseTracker) and abs(float(tracker_.max_age_s) - ra.SLAM_MAX_AGE_S) > 1e-12:
            raise ValueError("tracker_max_age")  # the freshness evidence and the tracker's age limit are one value (2b)
        # a source with the epoch contract (relocalize / seed / request_processed): the loop filters its replies and
        # builds the SLAM freshness evidence itself (design 6, 4); the older rigs' sources keep their behaviour
        self._epoch_mode = callable(getattr(slam_source, "relocalize", None))
        self.epoch, self.t_epoch = 0, -math.inf
        self._last_stamp = None  # the newest reply stamp passed to the core (the tracker's time order)
        self._last_scan = None  # the newest scan id passed in this epoch
        self.provenance = None  # (epoch, attempt, scan id, stamp) of the reply the applied correction came from
        self._fresh_ns = None  # the newest real processed reply of this epoch (first beam, host ns)
        self.reloc_dropped: Counter = Counter()
        self.reloc_gen = None  # the relocalization generation the loop opened
        self._reloc_reset_due = False  # the tracker's and the core's reset, after the mission is over
        self.attempt: dict | None = None
        self._attempt_serial = 0
        self.relocalize_status = {"generation": None, "attempt": None, "kind": None, "state": "idle", "reason": None,
                                  "start_pose_m": None, "start_pose_deg": None}
        self.declared_epoch = None  # the session epoch carrying a declaration's mark (7b)
        self.stand_since_ns: int | None = None  # the MCU sample stand, no SLAM term (design 6, 3)
        self._moved_for = None  # the mission whose first moving DRIVE has been written whole (design 6, 6)
        self._rear_now = None  # this tick's control rear pose (the transmit gate's start comparison)
        self.reloc_events: list = []
        self._disarms_taken = 0
        # PR 7 design 6.1, 2: the session's manual-only latch (a declaration completed on a `not_localized` seed) --
        # kept to the session's end; the three start boundaries read it with the slam bypass each judgement
        self.manual_only: dict | None = None
        # PR 7 design 6.1, 7 (7c-2b): every seed of the session (loop-made, never the console's or a reply's) and the
        # F1 start block judged when a generation closes -- one-way to the session's end
        self.seed_log: list = []
        self.f1_block: dict | None = None
        self._f1_epoch = link.a.session_epoch
        self._mission_bypass: set = set()
        if hasattr(link.a, "start_gate"):
            link.a.start_gate = self._start_gate
        if self._epoch_mode:
            src_ = evidence.slam
            self.evidence = dataclasses.replace(evidence, slam=lambda n: dataclasses.replace(
                src_(n), pose_reply_ns=self._fresh_ns))

    # --- 5c-1: the real console requests, the bypass, real_status (design 5) --------------------------------------
    def _ready(self, now_ns: int, stage: str):
        why = ra.auto_ready_refusal(now_ns, stage, self.evidence, bypass=self._tick_bypass)
        if why is not None and ra.latches(why):
            # 6a-2b (impl round 4): a latching reason with the instant its evidence was judged at
            self._emit({"t": "cause", "ns": int(now_ns), "why": why, "where": "ready"})
            self._cause = (int(now_ns), why)
        return why

    def _shown_bypass(self, bypass=None) -> set:
        """The bypass set as told and recorded: the session's (this tick's by default) with the declaration's mark."""
        return set(self._tick_bypass if bypass is None else bypass) | ({"relocalize"} if self._declared() else set())

    def _bypassed(self, check: str) -> bool:
        return check in self._tick_bypass

    def _apply_bypass(self, now_ns: int) -> None:
        """The tick head (before the submit is judged): the session's set (a new session or an accepted session end
        empties it), then the toggles accepted since -- only while the permit is off or latched -- and this tick's
        copy every judgement uses."""
        a = self.link.a
        if a.session_epoch != self._bypass_epoch or a.session_request is not None:
            if self.bypass or self._bypass_pending:
                self.bypass_events.append({"ns": int(now_ns), "event": "bypass_cleared:session"})
            self._bypass_epoch = a.session_epoch
            self.bypass.clear()
            self._bypass_pending.clear()
            self.declared_epoch = None  # 5c-2 (7b): the declaration's mark is the session's
        if a.session_epoch != self._f1_epoch:
            # 6.1, 7: a new session is a new bridge with no seed -- its records and block start empty. Only a new
            # session: an accepted session end keeps them for the end record (Codex 7c-2b round 1 P2)
            self._f1_epoch = a.session_epoch
            self.seed_log = []
            self.f1_block = None
        # the check and the change under one channel lock: a connection event queued between them cannot slip past
        # (r7 P2-1; the channels' lock is re-entrant)
        with self.channels.lock:
            self._apply_pending(int(self.clock()))  # the lease judged at the instant it applies (r8 P2)
        self._bypass_pending.clear()
        self._tick_bypass = frozenset(self.bypass)
        if self.core.commands.status in self.MISSION_ACTIVE:
            self._mission_bypass |= self._shown_bypass()  # 6.1, 2: every bypass applied while the mission lasted
        if self.real_layer is not None:
            self.real_layer.timing_bypassed = "layer_timing" in self._tick_bypass
            self.real_layer.correction_bypassed = "snapshot" in self._tick_bypass
        self.core.d5_bypassed = "d5_pocket" in self._tick_bypass  # 5d (7.2): read by the core's first-frame stand

    def _apply_pending(self, now_ns: int) -> None:
        for check, on, lease_id in self._bypass_pending:
            why = self._bypass_refusal(lease_id, now_ns)
            if why is not None:
                # still off, no mission, the same lease -- else not applied (r4 P1, P2-1); real_status shows the set
                self.bypass_events.append({"ns": int(now_ns), "event": f"bypass_dropped:{why}", "check": check,
                                           "on": bool(on), "lease_id": lease_id})
                continue
            if on:
                self.bypass.add(check)
            else:
                self.bypass.discard(check)
            if check == "operator_start" and self.operator_start is not None:
                self.operator_start.void("bypass_changed")  # a check given before is not carried across (r2 P2-2)
            self.bypass_events.append({"ns": int(now_ns), "event": "bypass_on" if on else "bypass_off", "check": check,
                                       "lease_id": lease_id})

    MISSION_ACTIVE = ("accepted", "preparing", "running", "paused", "resuming", "held", "cancelling", "recovering")

    def _bypass_refusal(self, lease_id, now_ns: int | None = None) -> str | None:
        """A bypass toggle needs the truck disarmed, no mission under way and the lease that asked -- not expired by
        the arbiter's own rule and its holder's connection not being closed (5c-1 r1 P1, r4, r5 P2)."""
        if self.permit.state not in (ra.OFF, ra.LATCHED):
            return "armed"
        if self.core.commands.status in self.MISSION_ACTIVE:
            return "mission_active"
        lease = self.link.a.lease
        if lease is None or lease.lease_id != lease_id:
            return "lease_changed"
        if now_ns is not None and not self._lease_valid(int(now_ns)):
            return "lease_changed"
        with self.channels.lock:
            # a disconnect, or a new connection of the same client (it takes the lease back), waiting (r5 P2, r6 P2-1)
            current = self.link.a.conns.get(lease.client_id)  # only the holder's own connection closing counts (r7 P3)
            closing = any(ev[2] == lease.client_id and (ev[0] == "connect" or ev[1] == current)
                          for ev in list(getattr(self.channels, "_conns", ())))
        if closing:
            return "lease_changed"
        return None

    def _real_rejected(self, kind: str, reason: str) -> None:
        self.last_refusal = {"kind": kind, "reason": reason}

    def _real_request(self, kind: str, msg: dict, now_mono: float):
        """The arbiter's real requests after its checks (design 5): (refusal reason or None, reply fields)."""
        reason, fields = self._real_request_(kind, msg)
        if reason is not None:
            self.last_refusal = {"kind": kind, "reason": reason}
        return reason, fields

    def _real_request_(self, kind: str, msg: dict):
        core, permit = self.core, self.permit
        now_ns = int(self.clock())  # the instant it is judged at, not the tick's start (r3 P2-2)
        if not self._lease_valid(now_ns):
            return "no_lease", {}  # the lease ran out during the tick: nothing changes (r7 P2-2, r9 P2)
        disarmed = permit.state in (ra.OFF, ra.LATCHED)
        if kind == "pallet_register":
            from forklift_core.live.real_assembly import register_pallet

            if core.commands.status in ("accepted", "preparing", "running", "paused", "resuming", "held",
                                        "cancelling", "recovering"):
                return "mission_active", {}
            if msg["pallet_id"] not in core.targets.pallets:
                return "unknown_pallet", {}
            return None, {"targets_revision": register_pallet(core, msg["pallet_id"], msg["pose"])}
        if kind == "operator_start":
            if core.relocalize_generation() is not None:
                return "relocalize_required", {}  # 5c-2 (6, 2): no start check while the place is not known
            if not disarmed:
                return "armed", {}
            if self.still_since_ns is None:
                # standing on the MCU samples but no fresh SLAM: say so (5c-2, 6, 3)
                return ("slam_stale" if self.stand_since_ns is not None else "not_still"), {}
            lease = self.link.a.lease
            if self.operator_start is None or lease is None:
                return "unsupported", {}
            if not self.operator_start.confirm(int(msg["session_epoch"]), lease.lease_id):
                return "stale_epoch", {}
            return None, {}
        if kind == "operator_answer":
            open_ = core.operator_open()
            if open_ is None:
                return "no_operator_request", {}
            if open_["kind"] != msg["kind"] or open_["serial"] != msg["serial"]:
                return "stale_operator_request", {}
            if msg["serial"] in self.operator_answers:
                return "already_answered", {}
            self.operator_answers[msg["serial"]] = msg["verdict"]
            return None, {"received": True}
        if kind == "check_bypass":
            check = msg["check"]
            if check in NOT_BYPASSABLE or (self.operating and check in OPERATING_NOT_BYPASSABLE):
                return "not_bypassable", {}
            if check not in BYPASSABLE:
                return "unknown_check", {}
            lease = self.link.a.lease
            lease_id = None if lease is None else lease.lease_id
            refusal = self._bypass_refusal(lease_id, now_ns)  # at the instant it is taken (r6 P3-1)
            if refusal is not None:
                return refusal, {}
            self._bypass_pending.append((check, bool(msg["on"]), lease_id))  # applied at the next tick head (r3)
            return None, {"applies": "next_tick"}
        if kind in ("relocalize_confirm", "relocalize_declare"):
            return self._attempt_request(kind, msg, now_ns)
        if kind == "recovery":
            done = core.field_recovery_done(self._t(now_ns), int(msg["recovery_id"]),
                                            disarmed=self._disarm_confirmed(now_ns),
                                            outstanding=self.port.outstanding(now_ns)[0] != "settled")
            if not done[0]:
                return done[1], {}  # the core's refusal reason (accepted, reason)
            if self.operator_start is not None:
                self.operator_start.void("field_recovery")
            return None, {}
        return "unsupported", {}

    def _disarm_confirmed(self, now_ns: int) -> bool:
        """Design 5: no DISARM episode awaited, no console DISARM accepted and not yet run, no DISARM queued or half
        written (the anchor's included), and the newest DISARM written answered by a fresh DISARMED of its seq."""
        permit = self.permit
        if getattr(permit, "_disarm_first_ns", None) is not None or self._console_disarm:
            return False
        with self.channels.lock:  # a console DISARM in the channel, not yet through the arbiter (r6 P2-2)
            if list(getattr(self.channels, "_disarms", ())):
                return False
        if any(getattr(o, "kind", None) == "DISARM" for o in getattr(self.host, "_outbox", ())):
            return False
        partial = getattr(self.transport, "partial_frame", None)
        if partial is not None and getattr(partial, "kind", None) == "DISARM":
            return False
        st = self.host.state
        if st is None or getattr(permit, "disarm_boot", None) != int(st.boot_id):
            return False  # the answer must come from the boot the DISARM was written to (r4 P2-2)
        return permit._disarmed_after_write(int(now_ns))  # noqa: SLF001 -- the permit's own answer check

    def operator_check(self, request):
        """The core's operator hook (RealHooks.operator): the operator's first answer for this request's serial."""
        verdict = self.operator_answers.get(request.get("serial"))
        return None if verdict is None else (request["serial"], verdict)

    def real_status(self, now_ns: int) -> dict:
        """real_status (design 5): what the console needs beyond the arbiter's tick."""
        a, core, permit = self.link.a, self.core, self.permit
        recovery = getattr(core, "field_recovery", None)
        start = self.operator_start
        return {
            "t": "real_status", "session_epoch": a.session_epoch, "control_gen": a.control_gen,
            "lease": {"holder": None if a.lease is None else a.lease.client_id},
            "permit": {"state": permit.state, "reason": permit.reason},
            "mission": {"status": core.commands.status, "mission_id": core.commands.mission_id},
            "operator_request": core.operator_open(),
            "field_recovery": None if recovery is None else recovery.get("id"),
            "relocalize_generation": core.relocalize_generation(),
            "operator_start": None if start is None else start.ok(None if a.lease is None else a.lease.lease_id),
            "targets_revision": core.targets.revision,
            "last_refusal": self.last_refusal,
            "restart_required": self.session.collect_failed,  # 6a: a mission collection raised (restart the executor)
            "bypassed": sorted(self._shown_bypass(self.bypass)),
            "mode": a.mode,
            "relocalize": dict(self.relocalize_status),  # 5c-2 (6, 8): the last attempt's, kept after the close
            # PR 7 design 6.1, 2: the latch itself and whether it blocks now (the slam bypass off) -- the console's
            # notice reads these, not the SLAM state, so a later ready seed does not clear it
            "manual_only": None if self.manual_only is None else dict(self.manual_only),
            "manual_only_blocking": self._latch_blocks(),
            # 6.1, 7 (7c-2b): the F1 block itself and whether it blocks now (the f1 bypass off)
            "f1_block": None if self.f1_block is None else dict(self.f1_block),
            "f1_blocking": self._f1_blocks(),
            # the MCU sample stand (6, 3) with its link on the clock now: not_still's own value (impl round 6 P2-1)
            "standing": self.stand_since_ns is not None and bool(self.host.health(int(self.clock())).fresh),  # the arbiter's (STOPPED: a rearm is needed -- the console's real panel, 5c-1b)
            # the operating run (U3a): whether it is one, its IMU's qualification (C7) and the scan source's newest
            # refusal and evidence boundaries (plan 2.5)
            "operating": self.operating,
            "imu_unqualified": self.imu_unqualified,
            "scan": {"last_refusal": self._scan_refusal_now(), "boundaries": self.scan_boundaries},
        }

    def _scan_refusal_now(self):
        try:
            return None if self.scan_refusal is None else self.scan_refusal()
        except Exception:  # noqa: BLE001 -- a status line, never a reason to stop
            return None

    # --- 5c-2: relocalization (design 6) -------------------------------------------------------------------------
    @property
    def outputs_closed(self) -> bool:
        """The permit off or latched and no mission under way (design 6, 2b): the real pose hooks read without the
        age comparison then (their contract, PR 7; the rigs' fake hooks honour it)."""
        return self.permit.state in (ra.OFF, ra.LATCHED) and self.core.commands.status not in self.MISSION_ACTIVE

    def physical_stop(self, now_s: float) -> tuple:
        """The arbiter's stop evidence (design 6, 2b): the link fresh and the MCU sample stand -- no SLAM term."""
        return bool(self.host.health(int(now_s * 1e9)).fresh), self.stand_since_ns is not None

    def rearm_stand(self, now_ns: int):
        """The rearm's stand (design 6, 2b): (since, boot) of the MCU sample stand, or None."""
        s = self.host.state
        return None if self.stand_since_ns is None or s is None else (self.stand_since_ns, s.boot_id)

    def _declared(self) -> bool:
        return self.declared_epoch is not None and self.declared_epoch == self.link.a.session_epoch

    def _speed_cap(self) -> float:
        """The rear-axle command cap: 0.06 m/s while slam, d5_pocket or f1 is bypassed or the session carries a
        declaration (6, 7b; f1: 7c-2b, lead #623 decision 2)."""
        if ("slam" in self._tick_bypass or "d5_pocket" in self._tick_bypass or "f1" in self._tick_bypass
                or self._declared()):
            return min(self.max_speed_mps, self.declared_speed_mps)  # 5d: the D5 bypass keeps the insertion's low speed
        return self.max_speed_mps

    def _tracker(self):
        return None if self.core.slam is None else self.core.slam.get("tracker")

    def _new_epoch(self, t: float) -> None:
        self.epoch += 1
        self.t_epoch = float(t)
        self._last_scan = None
        self._fresh_ns = None  # the freshness evidence is the new epoch's own (6, 4; round 6 P1)

    def _sync_relocalization(self, now_ns: int, t: float) -> None:
        """Design 6, 1: open the core's newest generation once; the tracker's and the core's reset once no mission is
        under way."""
        gen = self.core.relocalize_generation()
        if gen is not None and gen != self.reloc_gen:
            self._open_relocalization(now_ns, t, gen)
        if self._reloc_reset_due and self.core.commands.status not in self.MISSION_ACTIVE:
            self._reloc_reset_due = False
            tr = self._tracker()
            if tr is not None and hasattr(tr, "reset_for_relocalization"):
                tr.reset_for_relocalization()
            self.core.relocalization_reset(t)
            self.reloc_events.append({"t": float(t), "event": "reset", "generation": self.reloc_gen})

    def _open_relocalization(self, now_ns: int, t: float, gen: int) -> None:
        self.reloc_gen = gen
        if self.attempt is not None:
            self._attempt_end(t, "refused", "relocalize_superseded")
        self.still_since_ns = None
        self.stand_since_ns = None
        if self.operator_start is not None:
            self.operator_start.void("relocalize")
        self.provenance = None
        if self.real_layer is not None:
            self._reset_layer(now_ns)
        if self._epoch_mode:
            self._new_epoch(t)
            self.slam_source.relocalize(self.epoch)
        self._reloc_reset_due = True
        self.reloc_events.append({"t": float(t), "event": "opened", "generation": gen, "epoch": self.epoch})

    def _attempt_request(self, kind: str, msg: dict, now_ns: int):
        """relocalize_confirm / relocalize_declare (design 6, 5): answered at once; the verdict on real_status."""
        t = self._t(now_ns)
        self._sync_relocalization(now_ns, t)
        gen = self.core.relocalize_generation()
        if gen is None:
            return "no_relocalization", {}
        if int(msg["generation"]) != gen:
            return "stale_relocalization", {}
        if self._reloc_reset_due or self.reloc_gen != gen:
            return "mission_active", {}
        if self.permit.state not in (ra.OFF, ra.LATCHED):
            return "armed", {}
        if self._console_disarm or self._emergency_pending():
            return "emergency_pending", {}  # impl round 4 P1: a DISARM taken in this batch, or a STOP/DISARM queued
        if self.stand_since_ns is None or not self.host.health(now_ns).fresh:
            return "not_still", {}  # the stand and its link on the clock now (impl round 5 P1)
        if self.start_rear is None or self.rear_offset_m is None:
            return "no_start_pose", {}
        # PR 7 design 6.1, 2: the source's own acceptance, asked before anything changes (no attempt ended, no layer
        # cut): a confirmation needs a `ready` source, a declaration no seed in progress
        hil = self._hil_watch()
        if hil is not None and not hil[0]:
            return "hil_check_lost", {}  # 7r plan 3: nothing completes while it is lost
        if not self._source_accepts(kind):
            return ("slam_not_ready" if kind == "relocalize_confirm" else "seed_in_progress"), {}
        if self.attempt is not None:
            self._attempt_end(t, "refused", "relocalize_superseded")
        self._attempt_serial += 1
        lease = self.link.a.lease
        self.attempt = {"kind": "confirm" if kind == "relocalize_confirm" else "declare", "generation": gen,
                        "attempt": self._attempt_serial, "lease_id": None if lease is None else lease.lease_id,
                        "control_gen": self.link.a.control_gen, "bypass": frozenset(self.bypass),
                        "stand": self.stand_since_ns, "t": t, "reference": False, "seeded": False,
                        # a DISARM taken after this voids it (impl round 3: the taken ones counted, not the queue)
                        "disarms": self._disarms_taken,
                        # 6.1, 2: the declaration completes on its seed's end state (a source that reports them)
                        "seed_states": self._seed_states(), "seed_epoch": None, "seed_id": None, "seed_end": None,
                        "hil_round": None if hil is None else hil[1]}
        if self.real_layer is not None:
            self._reset_layer(now_ns)  # the scans from before the truck was carried are cut (6, 5; round 2 P1-1)
        if self.attempt["kind"] == "confirm" and self._epoch_mode:
            self.slam_source.request_processed(self.epoch, self._attempt_serial, not_before=t)
        self.relocalize_status = {"generation": gen, "attempt": self._attempt_serial, "kind": self.attempt["kind"],
                                  "state": "waiting", "reason": None, "start_pose_m": None, "start_pose_deg": None}
        return None, {"generation": gen, "attempt": self._attempt_serial}

    def _source_accepts(self, kind: str) -> bool:
        """The source's side-effect-free acceptance (``can_confirm()`` -> bool: ready; ``can_seed()`` -> bool: no seed
        in progress), when it has one; the refusal codes are the loop's own (the console's texts)."""
        if not self._epoch_mode:
            return True
        ask = getattr(self.slam_source, "can_confirm" if kind == "relocalize_confirm" else "can_seed", None)
        return True if ask is None else ask() is True

    def _seed_states(self) -> bool:
        return self._epoch_mode and callable(getattr(self.slam_source, "seed_state", None))

    def _attempt_deadline(self, a: dict) -> float:
        """6.1, 2: an attempt's last instant (core s). A confirmation, and a declaration on a source without seed end
        states, has 3 s from its acceptance; a declaration on one with them has the 20 s seed limit plus 0.5 s from
        its acceptance, and once its end state is seen the commit within 0.5 s of it (Codex 7c-1b-1 round 1 P3)."""
        if a["kind"] == "declare" and a.get("seed_states"):
            limit = a["t"] + SEED_WAIT_S + DECLARE_COMMIT_S
            end = a.get("seed_end")
            return limit if end is None else min(limit, end[2] + DECLARE_COMMIT_S)
        return a["t"] + RELOCALIZE_WAIT_S

    def _latch_blocks(self) -> bool:
        return self.manual_only is not None and "slam" not in self._tick_bypass

    def _f1_blocks(self) -> bool:
        return self.f1_block is not None and "f1" not in self._tick_bypass

    def _start_gate(self):
        """The arbiter's start gate (6.1, 2 and 7), read at each judgement by the request check, the AUTO entry and
        (through ``_start_blocked``) the Context: ``(F1 block ∧ f1 off) ∨ (manual_only ∧ slam off)``. On the real path
        F1 lives here, not in the arbiter's one-way ``start_block``: the gate's refusals leave the field recovery's
        report open, and an f1 bypass toggled in the session is read at once (7c-2b)."""
        if self._f1_blocks():
            return {"reason": "operator_placed_start", "source": "f1", **self.f1_block}
        if not self._latch_blocks():
            return None
        return {"reason": "operator_placed_start", "source": "manual_only", **self.manual_only}

    def _start_blocked(self) -> bool:
        """6.1, 2 / 7: the one expression the three boundaries read -- the arbiter's own one-way block (never set on the
        real path), F1 with the f1 bypass off, or the latch with the slam bypass off."""
        return (getattr(self.link.a, "start_block", None) is not None or self._f1_blocks()
                or self._latch_blocks())

    def _attempt_end(self, t: float, state: str, reason) -> None:
        a = self.attempt
        self.attempt = None
        if a is None:
            return
        self.relocalize_status.update(state=state, reason=reason)
        self.reloc_events.append({"t": float(t), "event": state, "reason": reason, "attempt": a["attempt"],
                                  "kind": a["kind"]})

    def _hil_watch(self):
        """PR 7 7r plan 3: ``(held, round)`` of the HIL check now -- the round raised at every loss and every boot
        change the HIL transport shows -- or None off HIL (another client). Read at each tick's receive, when an
        attempt starts and in its every void check, the last just before it completes."""
        if not self._hil_client:
            return None
        tr = self.transport
        held = getattr(tr, "hil_plant", False) is True
        # the transport's loss count too: a loss and a new boot within one read leave only the new boot showing
        seen = (held, getattr(tr, "hil_boot", None) if held else None, getattr(tr, "hil_losses", 0))
        if self._hil_seen is not None and seen != self._hil_seen:
            self.hil_check_round += 1
            self._emit({"t": "hil_check", "held": held, "boot": seen[1], "round": self.hil_check_round,
                        "ns": int(self.clock())})
        self._hil_seen = seen
        return held, self.hil_check_round

    def _attempt_void(self, now_ns: int, t: float):
        a, arb = self.attempt, self.link.a
        if self.real_layer is not None and self.real_layer.watchdog(t) in LAYER_ENDS:
            return "relocalize_voided:layer"  # the fault before any unlock (round 3 P2-2)
        if a.get("hil_round") is not None:
            # 7r plan 3 (Codex plan round 4 P1): a completion while the HIL check was lost, or across a boot change, is
            # no evidence -- whatever the source (the synthetic HIL too)
            hil = self._hil_watch()
            if hil is None or not hil[0] or hil[1] != a["hil_round"]:
                return "hil_check_lost"
        if arb.lease is None or arb.lease.lease_id != a["lease_id"] or not self._lease_valid(now_ns):
            return "relocalize_voided:lease"
        if arb.control_gen != a["control_gen"]:
            return "relocalize_voided:control"
        if self.permit.state not in (ra.OFF, ra.LATCHED):
            return "relocalize_voided:armed"
        if frozenset(self.bypass) != a["bypass"]:
            return "relocalize_voided:bypass"
        if self.stand_since_ns != a["stand"]:
            return "relocalize_moved"
        if self._disarms_taken != a["disarms"]:
            # impl rounds 2-4: a DISARM taken since (a STOP taken moves the control generation); one still waiting in
            # the channel holds the commit until the arbiter has judged it (_emergency_pending); a viewer's connection
            # does not count
            return "relocalize_voided:emergency"
        return None

    def _emergency_pending(self) -> bool:
        with self.channels.lock:
            return bool(getattr(self.channels, "_stops", None)) or bool(getattr(self.channels, "_disarms", None))

    def _commit_void(self, t: float, closing: bool = True):
        """Just before a commit (the source's seed, the generation's close): the attempt's own voids and then the
        lease judged on the clock now with the channel's pending connection changes (impl round 1 P1-1: a lease
        expired or a disconnect queued since the tick began)."""
        a = self.attempt
        now_ns = int(self.clock())
        t_now = self._t(now_ns)
        if self._emergency_pending():
            return "wait"  # a STOP or DISARM waiting: no commit until it is judged (impl round 4 P2)
        why = self._attempt_void(now_ns, t_now)  # the layer's watchdog at the commit's own instant (impl round 3 P1-2)
        if why is None and not self.host.health(now_ns).fresh:
            why = "relocalize_voided:link"  # the stand's own evidence on the clock now (impl round 5 P1)
        if why is None and self._bypass_refusal(a["lease_id"], now_ns) is not None:
            why = "relocalize_voided:lease"
        # impl round 2 P1-2/P1-3: the limit, the freshness and the layer on the clock now, not the tick's start
        lay = self.real_layer
        if why is None and closing and lay is not None and (not lay.worker.alive or not lay.ok(t_now)):
            why = "relocalize_voided:layer"
        if why is None and t_now > self._attempt_deadline(a):
            why = "relocalize_timeout"
        prov = self.provenance
        if why is None and closing and a["kind"] == "confirm" and not self._bypassed("slam") and (
                prov is None or t_now - prov[3] > ra.SLAM_MAX_AGE_S):
            why = "relocalize_stale"
        return why

    def _declare_apply(self, now_ns: int, t: float, loaded: bool) -> None:
        """The declaration's pose (design 6, 5 / 7b), on its first judgement, before the source's replies: the source
        seeded at the start's base_link pose, a synthetic processed reply alone through the core."""
        a = self.attempt
        target = base_from_rear(self.start_rear, self.rear_offset_m)
        odom = tuple(float(v) for v in self.core.hooks.slam_odom_base())
        c = tuple(float(v) for v in compose(target, invert(odom)))
        with self.channels.lock:  # the check and the seed under one lock: no connection change slips between
            why = self._commit_void(t, closing=False)  # the layer was just cut: its result is judged at the close
            if why == "wait":
                return  # seeded on a later tick, once the waiting STOP/DISARM is judged
            if why is not None:
                self._attempt_end(t, "refused", why)
                return
            self._new_epoch(t)
            seed_id = self.slam_source.seed(self.epoch, target, t)
            # issued: never a second seed for this attempt (Codex 7c-1b-1 round 1 P2-3)
            a["seed_epoch"], a["seed_id"] = self.epoch, seed_id
            # 6.1, 7: the seed recorded as the source took it (a seed cancelled before its frame went out is still
            # the session's last seed)
            self.seed_log.append({"session_epoch": self.link.a.session_epoch, "generation": a["generation"],
                                  "attempt": a["attempt"], "epoch": self.epoch, "seed_id": seed_id, "declared": False,
                                  "t": float(t)})
            # the mark, the cap and the record from the seed on, before anything else can fail (impl rounds 1-2)
            self.declared_epoch = self.link.a.session_epoch
            self.reloc_events.append({"t": float(t), "event": "relocalize_seeded", "generation": a["generation"],
                                      "attempt": a["attempt"], "odom_from_base": list(odom),
                                      "start_rear": list(self.start_rear), "base_target": list(target)})
        reply = SimpleNamespace(scan_id=-1, stamp_s=float(t), status="processed", map_from_odom=c, slam_wall_s=None,
                                epoch=self.epoch, attempt=None)
        tr = self._tracker()
        before = None if tr is None else tr.applied
        try:
            self.core.slam_replies(t, [(reply, {"source": "operator_declared"}, None, None)], loaded)
        except Exception:
            self._attempt_end(t, "refused", "relocalize_declare_failed")  # seeded once, never retried (round 1 P2-3)
            raise
        self._last_stamp = float(t) if self._last_stamp is None else max(self._last_stamp, float(t))
        if tr is not None and tr.applied is not before:
            self.provenance = (self.epoch, None, "operator_declared", float(t))
        a["seeded"] = True

    def _filter_replies(self, t: float, raw) -> list:
        """Design 6, 4: the replies the core may take (the epoch mode only)."""
        tr = self._tracker()
        out = []
        has_received = tr is not None and tr.received is not None
        for item in raw:
            reply = item[0]
            ep = getattr(reply, "epoch", None)
            ep = 0 if ep is None else int(ep)
            stamp, sid = float(reply.stamp_s), int(reply.scan_id)
            lo, hi = _evidence(reply)
            why = None
            if ep != self.epoch:
                why = "epoch"
            elif lo < self.t_epoch:
                why = "before_epoch"
            elif hi > t + 1e-9:
                why = "future"
            elif self._last_stamp is not None and stamp < self._last_stamp:
                why = "out_of_order"
            elif self._last_scan is not None and sid <= self._last_scan:
                why = "repeated"
            elif reply.status == "skipped" and not has_received:
                why = "skipped_first"  # a processed earlier in this batch counts (impl round 1 P2-2)
            if why is not None:
                self.reloc_dropped[why] += 1
                continue
            self._last_stamp, self._last_scan = stamp, sid
            out.append(item)
            if reply.status == "processed":
                has_received = True
                # the age from this reply's first beam's earliest instant, this reply's own (Codex 7c-1b-1 round 1 P1:
                # a later reply's Flo may be earlier than an older one's; the evidence is the newest reply, as the
                # provenance is, never a max over replies)
                self._fresh_ns = self.start_ns + int(round(lo * 1e9))
                a = self.attempt
                if (a is not None and a["kind"] == "confirm" and getattr(reply, "attempt", None) == a["attempt"]
                        and lo >= a["t"]):
                    a["reference"] = True
        return out

    def _slam_step(self, now_ns: int, t: float, loaded: bool) -> None:
        """This tick's SLAM replies into the core (the epoch mode: the declaration first, then the filtered replies,
        the applied correction's provenance kept)."""
        raw = self.slam_source.replies(now_ns)
        if not self._epoch_mode:
            self.core.slam_replies(t, raw, loaded)
            return
        if self.attempt is not None and self.attempt["kind"] == "declare" and self.attempt["seed_epoch"] is None:
            self._declare_apply(now_ns, t, loaded)
        passed = self._filter_replies(t, raw)
        tr = self._tracker()
        self.core.slam_replies(t, passed, loaded)
        # the applied correction's source (impl round 1 P2-2): outside a hold every processed becomes applied, so the
        # batch's last processed is it -- a later skipped only refreshes the stamp; under a hold nothing was applied
        last = next((it[0] for it in reversed(passed) if it[0].status == "processed"), None)
        if last is not None and tr is not None and getattr(tr, "mode", None) != "holding":
            self.provenance = (self.epoch, getattr(last, "attempt", None), int(last.scan_id), _evidence(last)[0])

    def _start_gap(self, rear):
        """(distance m, |yaw| rad) of a rear-axle pose from the registered start."""
        sx, sy, syaw = self.start_rear
        return (math.hypot(float(rear[0]) - sx, float(rear[1]) - sy),
                abs(math.atan2(math.sin(float(rear[2]) - syaw), math.cos(float(rear[2]) - syaw))))

    def _off_start(self, rear) -> bool:
        if self.start_rear is None or self._bypassed("start_pose") or rear is None:
            return False
        d, yaw = self._start_gap(rear)
        return d > START_TOL_M or yaw > START_TOL_RAD

    def _attempt_judge(self, now_ns: int, t: float) -> None:
        """Design 6, 5: an attempt's verdict after this tick's replies and installs."""
        a = self.attempt
        if a is None:
            return
        why = self._attempt_void(now_ns, t)
        if why is not None:
            self._attempt_end(t, "refused", why)
            return
        if self.core.relocalize_generation() != a["generation"]:
            self._attempt_end(t, "refused", "relocalize_superseded")
            return
        failed = None
        prov = self.provenance
        if a["kind"] == "confirm":
            if not a["reference"]:
                failed = "relocalize_no_reply"
            elif prov is None or prov[0] != self.epoch or (
                    not self._bypassed("slam") and t - prov[3] > ra.SLAM_MAX_AGE_S):
                failed = "relocalize_stale"
        elif prov is None or prov[0] != self.epoch:
            failed = "relocalize_no_reply"
        elif a.get("seed_states") and a["seed_epoch"] is None:
            failed = "relocalize_seeding"  # not issued yet (a STOP/DISARM waiting): nothing to ask (round 1 P2-2)
        elif a.get("seed_states"):
            # 6.1, 2: the seed's end state (ready / not_localized) completes it; void refuses it at once
            end = self.slam_source.seed_state(a["seed_epoch"])
            # the instant it was seen, on the clock now -- not the tick's start (Codex 7c-1b-1 round 2 P2): the 0.5 s
            # to the commit and the void's limit judged from it
            seen = self._t(int(self.clock()))
            kind, reason = (None, None) if end is None else end
            if kind == "void":
                # the limit first (impl round 1 P2-1): a void seen past it is the declaration's timeout
                self._attempt_end(t, "refused", "relocalize_timeout" if seen > self._attempt_deadline(a)
                                  else f"relocalize_voided:seed:{reason}")
                return
            if kind in ("ready", "not_localized"):
                if a["seed_end"] is None:
                    a["seed_end"] = (kind, reason, float(seen))
                    self.reloc_events.append({"t": float(t), "event": "seed_end", "kind": kind, "reason": reason,
                                              "attempt": a["attempt"], "epoch": a["seed_epoch"],
                                              "seed_id": a.get("seed_id")})
            else:
                failed = "relocalize_seeding"
        tr = self._tracker()
        rear = None
        if failed is None:
            if tr is None or tr.applied is None:
                failed = "not_at_start"
            else:
                odom = tuple(float(v) for v in self.core.hooks.slam_odom_base())
                rear = rear_from_base(compose(tuple(tr.applied[0]), odom), self.rear_offset_m)
                d, yaw = self._start_gap(rear)
                self.relocalize_status.update(start_pose_m=float(d), start_pose_deg=float(math.degrees(yaw)))
                if self._off_start(rear):
                    failed = "not_at_start"
        if failed is None:
            lay = self.real_layer  # no layer: no generation closes (impl round 1 P2-3)
            newest = None if lay is None else lay.installed_newest_s()
            if lay is None or not lay.ok(t) or lay.installing(ignore_correction=False) or newest is None \
                    or newest < a["t"]:
                failed = "relocalize_layer"
        if t > self._attempt_deadline(a):
            # the limit first (impl round 1 P2-1); a seed still running is the declaration's own timeout
            self._attempt_end(t, "refused", "relocalize_timeout" if failed in (None, "relocalize_seeding") else failed)
            return
        if failed is None:
            with self.channels.lock:
                why = self._commit_void(t)
                if why == "wait":
                    return  # judged again next tick
                if why is not None:
                    self._attempt_end(t, "refused", why)
                    return
                self._close_generation(t, a)
            return

    def _close_generation(self, t: float, a: dict) -> None:
        tr = self._tracker()
        self.core.relocalized(t, a["generation"])
        self._f1_judge(t, a)  # at the close, before the next batch's link.begin() and Context (6.1, 7)
        if tr is not None and getattr(tr, "_relocalizing", False):
            tr.relocalization_done()
        if self.operator_start is not None:
            self.operator_start.void("relocalized")
        end = a.get("seed_end")
        if end is not None and end[0] == "not_localized" and self.manual_only is None:
            # 6.1, 2: the session is manual-only from here to its end (lifted for a start only by the slam bypass)
            self.manual_only = {"reason": "not_localized", "session_epoch": self.link.a.session_epoch,
                                "generation": a["generation"], "attempt": a["attempt"], "epoch": a["seed_epoch"],
                                "seed_id": a.get("seed_id"), "since": float(t)}
            self.reloc_events.append({"t": float(t), "event": "manual_only", **self.manual_only})
        self._attempt_end(t, "confirmed" if a["kind"] == "confirm" else "declared", None)

    def _f1_judge(self, t: float, a: dict) -> None:
        """6.1, 7: once a generation closes. A declaration closing marks its own seed row (the whole key must match --
        a declaration without one is unsourced); then the session's last seed must be a completed declaration, else
        the F1 start block holds to the session's end. Never judged right after a seed (it would block a declaration
        still finishing)."""
        session = self.link.a.session_epoch
        if a["kind"] == "declare":
            key = (session, a["generation"], a["attempt"], a.get("seed_epoch"))
            rows = [r for r in self.seed_log
                    if (r["session_epoch"], r["generation"], r["attempt"], r["epoch"]) == key]
            if len(rows) == 1:
                rows[0]["declared"] = True
            elif self.f1_block is None:
                self._f1_set(t, "declare_unsourced", a, None)
                return
        seeds = [r for r in self.seed_log if r["session_epoch"] == session]
        if seeds and not seeds[-1]["declared"] and self.f1_block is None:
            self._f1_set(t, "seed_not_declared", a, seeds[-1])

    def _f1_set(self, t: float, source: str, a: dict, seed) -> None:
        self.f1_block = {"source_kind": source, "session_epoch": self.link.a.session_epoch,
                         "closed_generation": a["generation"], "closed_kind": a["kind"],
                         "seed": None if seed is None else {k: seed[k] for k in ("generation", "attempt", "epoch",
                                                                                 "seed_id")},
                         "since": float(t)}
        self.reloc_events.append({"t": float(t), "event": "f1_block", **self.f1_block})

    def _on_console_disarm(self, now_mono: float) -> None:
        self._disarms_taken += 1  # 5c-2: a waiting relocalization attempt is void after any DISARM taken
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
            self.odom_gen += 1
            self._motion_mcu_until = None  # another clock; a rebooted MCU acts on no earlier frame
        self._last_seen = (state.boot_id, state.mcu_time_us)
        sample_ = Sample(int(rx_ns), state)
        self.samples.append(sample_)
        if self.delay_bound is not None:
            # every STATE's delay bound, kept by (boot, MCU time) -- a read's STATEs share one receipt time (6a design 2)
            self.delay_bound.add(sample_)
            self._sample_delay[(int(state.boot_id), int(state.mcu_time_us))] = (int(rx_ns),
                                                                                self.delay_bound.delay_s(sample_))
            if self.on_first_bounded_state is not None and self.anchor_rx_ns is None:
                d_ = self._sample_delay[(int(state.boot_id), int(state.mcu_time_us))][1]
                if d_ is not None and math.isfinite(d_):
                    self.anchor_rx_ns = int(rx_ns)
                    try:
                        self.on_first_bounded_state(int(rx_ns))
                    except Exception as exc:  # noqa: BLE001 -- the run ends on it
                        self.fatal_source = self.fatal_source or f"anchor:{type(exc).__name__}:{exc}"
            if len(self._sample_delay) > 4 * SAMPLES_KEPT:
                for key in list(self._sample_delay)[: len(self._sample_delay) - 2 * SAMPLES_KEPT]:
                    self._sample_delay.pop(key, None)
        if self.events is not None:
            # 6a-2b (design 10.3): what the gate judged this STATE by -- its receipt, its delay bound and the rule
            sd_ = self._sample_delay.get((int(state.boot_id), int(state.mcu_time_us)))
            self._emit({"t": "state", "boot": int(state.boot_id), "mcu_us": int(state.mcu_time_us), "rx_ns": int(rx_ns),
                        "last_rx_seq": int(state.last_rx_seq), "mode": int(state.mode),
                        "faults": int(state.fault_flags), "reject": int(state.reject_reason),
                        "delay_bound_s": None if sd_ is None else sd_[1], "rule": self.host.health(int(rx_ns)).gap_rule,
                        "max_age_s": float(self.host.cfg.state_max_age_s)})
        step = odometry_step(self.odometry, state)
        before = self.odom_pose
        self.odom_pose = tuple(float(v) for v in step.pose)
        if step.integrated:
            self.odom_travel += math.hypot(self.odom_pose[0] - before[0], self.odom_pose[1] - before[1])
        if step.continuity_broken:
            self._odom_broken = True  # sticky for the tick: a later good sample does not mend this tick's break
            self.odom_gen += 1
            self._odom_good = False
            self.still_since_ns = None
            self.stand_since_ns = None
        else:
            self._odom_good = True
        self._axis_judge(state)  # 6a: the displacement first, on every STATE, before the book takes it
        if self.closing and self.closing_confirmed_ns is None:
            # 6a-2b (design 10.5): judged on each STATE -- a later one of the same read must not hide it
            fresh = int(self.clock()) - int(rx_ns) <= int(self.host.cfg.state_max_age_s * 1e9)
            seq_ = int(state.last_rx_seq) & 0xFFFF
            if fresh and state.mode == Mode.DISARMED and any(
                    b == state.boot_id and q == seq_ and int(rx_ns) > int(w) for b, q, w in self.closing_disarms):
                self.closing_confirmed_ns = int(self.clock())
        self.book.on_state(state, int(rx_ns))
        self._axis_end(state, int(rx_ns))
        self.permit.on_state(state, int(rx_ns))
        if self.real_layer is not None:
            self._odom_entry(state, int(rx_ns), step)
        if self.watch is not None and self._watch_trip is None:
            self._watch_trip = self._watch_state(state)

    def _odom_entry(self, state, rx_ns: int, step) -> None:
        """5a (design 1.1): this STATE's history entry (``odom_history.odom_entry``, the observe run's too -- PR 7
        design 6.1, 8): its measurement interval with the delay bound the head recorded for it, its raw odometry pose,
        ``included`` and the motion bounds against ``v_scan``."""
        b = None
        if self.delay_bound is not None:
            b = (self._sample_delay.get((int(state.boot_id), int(state.mcu_time_us))) or (None, None))[1]  # _on_state
        entry, self._odom_prev = odom_entry(
            state, step, hi_s=self._t(rx_ns), delay_b=b, state_delay_s=self.state_delay_s,
            prev=getattr(self, "_odom_prev", None), has_history=bool(self.odom_history), params=self.params,
            v_scan_mps=self.v_scan_mps, kappa_max=self.kappa_max)
        self.odom_history.append(entry)
        for listener in self.odom_listeners:
            listener(entry)

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
        path, direction = None, 0
        if tracker is not None and hasattr(tracker, "leg_ahead"):
            poses, direction = tracker.leg_ahead()  # the gear leg's direction too (Codex 5a impl round 2 P2-5)
            path, direction = np.asarray(poses, dtype=float), int(direction)
        if (mission_gen, path_id, loaded) != self._layer_asked:
            self._layer_asked = (mission_gen, path_id, loaded)
            lay.refresh(t, core.obstacle["applied"], int(core.obstacle["version"]),
                        current_pose=tuple(float(v) for v in rear), path_ahead=path, loaded=loaded,
                        direction=direction)
        else:
            # the next scan's check from here, along what is left
            lay.track(tuple(float(v) for v in rear), path, direction)
        if self.frozen is not None:
            depth, width = self.pallet_size
            x, y, yaw = self.frozen.pose
            # the registered pallet's half sizes: the worker grows them by the range bound of each stored scan's
            # own sensor position (Codex 5a impl round 6 P2-1)
            rect = (x, y, yaw, depth / 2, width / 2)
            lay.planning_pallet = (rect, self._planning_key())
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
        """The frozen pallet -- its id and its registered pose (a re-registration of the same id elsewhere is another
        grid, Codex 5a impl P2-4) -- and the applied correction generation."""
        if self.frozen is None:
            return None
        return (self.frozen.pallet_id, tuple(float(v) for v in self.frozen.pose), int(self.core.obstacle["version"]))

    def scan_boundary(self, now_ns: int, sensor) -> None:
        """The operating run's scan evidence boundary (U3a, plan 2.5; Codex U3 split rounds 1-3): a producer change or a
        lost qualification. The layer's end latches (``scan_generation_changed``: stop, DISARM, no resume of its
        own); its grid, installed result, planning grid and placed scans go and the computing ones are stale by their
        key; this epoch's processed freshness goes (a reply already taken never vouches for the new generation); a
        relocalization attempt in progress is refused -- even while already latched, where the emergency returns
        early. A rearm then needs the new generation's own install and its own processed."""
        t = self._t(now_ns)
        self.scan_boundaries += 1
        if self.real_layer is not None:
            lay = self.real_layer
            lay.grid = type(lay.grid)(lay.grid_config)
            lay._installed = None  # noqa: SLF001 -- the layer's own reset (as _reset_layer)
            lay._unplaced.clear()  # noqa: SLF001
            lay.exempt_serial += 1  # a computing job's key is stale: its late answer is never installed
            lay._placed_id = 0  # noqa: SLF001
            lay.last_scan_s.clear()
            lay.last_beam_s.clear()
            lay.evidence_boundary()
        self._fresh_ns = None
        if self.attempt is not None:
            self._attempt_end(t, "refused", "relocalize_voided:scan_boundary")
        self._emit({"t": "scan_boundary", "ns": int(now_ns), "sensor": str(sensor), "count": self.scan_boundaries})

    def _reset_layer(self, now_ns: int) -> None:
        """A new grid, no installed or computing result (the worker's late answer is stale by its key) -- the odometry
        the stored scans were placed on is gone."""
        lay = self.real_layer
        lay.grid = type(lay.grid)(lay.grid_config)
        lay._installed = None  # noqa: SLF001 -- the layer's own reset
        lay._unplaced.clear()  # noqa: SLF001 -- placed on the old odometry or not at all
        lay.exempt_serial += 1
        lay._placed_id = 0  # noqa: SLF001 -- 5c-2 (6, 2): an empty job after the reset carries no old scan id
        lay.last_scan_s.clear()
        lay.last_beam_s.clear()
        self.odom_history.clear()

    def place_scan(self, first_s: float, last_s: float):
        """The layer's placer (design 1.1, the teleop scan bridge's way): a continuous, included history whose
        entries reach before the first beam and after the last, every entry in between within the motion bounds;
        the bounds checked from the first beam less the timing uncertainty ``dt = 2 period + state_delay_s`` (the
        age table's, Codex 5a impl P1-4); the pose interpolated at the first beam (entry interval midpoints), or
        None."""
        h = list(self.odom_history)
        dt = 2.0 * float(self.real_layer.scan_period_s) + self.state_delay_s
        before = [k for k, e in enumerate(h) if e[1] <= first_s]
        early = [k for k, e in enumerate(h) if e[1] <= first_s - dt]
        after = [k for k, e in enumerate(h) if e[0] >= last_s]
        if not before or not early or not after:
            return None
        k0, k1, ke = before[-1], after[0], early[-1]
        span = h[k0:k1 + 1]
        if not all(e[3] and e[4] for e in h[ke:k1 + 1]) or not all(e[5] and e[6] for e in h[ke + 1:k1 + 1]):
            return None  # a break, an excluded STATE or a motion over the bound inside the scan's window
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
                              wheel_rates_rad_s=(st.wheel_rate_left_rad_s, st.wheel_rate_right_rad_s),
                              skip=self._tick_bypass & {"watch_current", "watch_wheel"})
        return res.trip

    def _measured_lift(self):
        s = self.host.state
        return None if s is None or not math.isfinite(s.lift_m) else float(s.lift_m)

    def _measured_steering(self):
        s = self.host.state
        return None if s is None else float(s.steering_rad)

    # --- 6a (docs/design/2026-11-20-real-hil-6a.md 2): position-holding frames and the axis watch -------------------
    def _hold_sample(self):
        """``((steering, lift, acquisition_ns), None)`` from this tick's newest STATE when its acquisition age -- the
        clock now minus its receipt plus its own delay bound -- is at most HOLD_SAMPLE_MAX_AGE_S; else ``(None, why)``:
        ``no_state`` (none received this tick), ``no_delay_bound``, ``state_old`` or ``no_measurement``. A rig without
        the delay evidence (``delay_bound`` None) counts the receipt age alone; the HIL and real assemblies always pass
        one (6a-2)."""
        if not self.samples or self.last_start_ns is None or self.samples[-1].host_ns < self.last_start_ns:
            return None, "no_state"
        smp = self.samples[-1]
        st = smp.state
        if self.delay_bound is None:
            b = 0.0
        else:
            b = (self._sample_delay.get((int(st.boot_id), int(st.mcu_time_us))) or (None, None))[1]
            if b is None:
                return None, "no_delay_bound"
        acq = int(smp.host_ns) - int(round(float(b) * 1e9))
        if int(self.clock()) - acq > int(HOLD_SAMPLE_MAX_AGE_S * 1e9):
            return None, "state_old"
        if not math.isfinite(st.lift_m) or not math.isfinite(st.steering_rad):
            return None, "no_measurement"
        return (float(st.steering_rad), float(st.lift_m), acq), None

    def _skip_hold(self, now_ns: int, rec, why: str) -> None:
        """No position-holding frame this tick (design 2): the unsent DRIVEs go too (an older hold sent back to the
        queue head must not go out now); a frame accepted earlier is stopped by its MCU expiry."""
        self.host.drop_drives("wait_skipped")
        partial = getattr(self.transport, "partial_frame", None)
        partial_seq = None if partial is None else int(partial.seq) & 0xFFFF
        for f in [f for f in self.book.frames if f.written_ns is None and f.seq != partial_seq]:
            self.book.discarded(f.seq)
        if rec is not None:
            rec.notes.append(f"wait_frame_skipped:{why}")

    def _axis_start(self, frame) -> None:
        """A position-holding frame written: the watch starts at its sample's lift and steering (design 2); a holding
        frame written while the end waits cancels the end."""
        if self._axis is None:
            self._axis = {"h0": float(frame.lift_m), "q0": float(frame.steering[0]), "end_pending": False,
                          "seq": int(frame.seq) & 0xFFFF}
            # 6a-2b (design 2 (가)): the span the HIL judge reads, from the frame's own sample
            self._emit({"t": "axis_watch", "event": "start", "ns": int(self.clock()), "seq": int(frame.seq) & 0xFFFF,
                        "h0": self._axis["h0"], "q0": self._axis["q0"]})
        else:
            self._axis["end_pending"] = False

    def _axis_judge(self, state) -> None:
        a = self._axis
        if a is None or self._axis_trip is not None:
            return
        # (clock, valid): valid when the sample's acquisition is known to be within HOLD_SAMPLE_MAX_AGE_S (impl round 3
        # P2-2) -- the HIL bound counts the gaps between valid judgements only
        now_c = int(self.clock())
        rx_b = self._sample_delay.get((int(state.boot_id), int(state.mcu_time_us)))
        smp = self.samples[-1] if self.samples else None
        if self.delay_bound is None:
            b = 0.0
        else:
            b = None if rx_b is None else rx_b[1]
        valid = smp is not None and b is not None and \
            now_c - (int(smp.host_ns) - int(round(float(b) * 1e9))) <= int(HOLD_SAMPLE_MAX_AGE_S * 1e9) \
            and math.isfinite(state.lift_m) and math.isfinite(state.steering_rad)  # impl round 4 P2-2
        self.axis_watch_times.append((now_c, bool(valid)))
        self._emit({"t": "axis_judge", "ns": now_c, "valid": bool(valid), "lift": float(state.lift_m),
                    "steer": float(state.steering_rad), "mcu_us": int(state.mcu_time_us), "span": a.get("seq")})
        if math.isfinite(state.lift_m) and float(state.lift_m) - a["h0"] > WAIT_LIFT_RISE_M:
            self._axis_trip = "lift"
        elif math.isfinite(state.steering_rad) and abs(float(state.steering_rad) - a["q0"]) > WAIT_STEER_MOVE_RAD:
            self._axis_trip = "steer"

    def _axis_end(self, state, rx_ns: int) -> None:
        """The watch ends once the MCU confirmed the exact seq of a normal DRIVE of this boot and arm generation (the
        book's confirmation) with no later position-holding frame written and still on the book, judged on that STATE
        and the next one; or on DISARMED (design 2, rounds 17-19)."""
        a = self._axis
        if a is None:
            return
        if state.mode == Mode.DISARMED:
            self._axis = None
            self._emit({"t": "axis_watch", "event": "end", "ns": int(rx_ns), "why": "disarmed", "span": a.get("seq")})
            return
        c = self.book.confirmed
        later_hold = any(f.kind in HOLDING_KINDS and f.written_ns is not None for f in self.book.frames)
        ok = c is not None and c.kind == "drive" and c.confirmed_ns is not None and not later_hold
        if not ok:
            a["end_pending"] = False
        elif a["end_pending"]:
            self._axis = None
            self._emit({"t": "axis_watch", "event": "end", "ns": int(rx_ns), "why": "drive_confirmed",
                        "span": a.get("seq")})
        else:
            a["end_pending"] = True

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
        stand = standing and self.odom_continuous  # 5c-2 (6, 3): the MCU sample stand, no SLAM term
        if not stand:
            self.stand_since_ns = None
        elif self.stand_since_ns is None:
            self.stand_since_ns = int(now_ns)
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
        if self.closing:
            return self._closing_tick(now_ns)
        core, permit, host = self.core, self.permit, self.host
        t = self._t(now_ns)
        interval = None if self.last_start_ns is None else (now_ns - self.last_start_ns) / 1e9
        self.last_start_ns = int(now_ns)
        rec = TickRecord(t, interval)
        self._rec = rec
        self._lift_before = None
        self._written_now = []  # the seqs written whole in this tick, in order
        self._stop_decided = False  # 5d (7.1 6): this tick decided a stop or a wait (no moving continuation then)
        self._wait_frames_tick = 0  # 6a: at most one wait frame per tick (design 2, round 10 P2-1)
        self.session.last_records, self.session.began = None, False  # 6a (design 5)
        self._tick_end_tried = False
        self._cause, self._tick_ns = None, int(now_ns)
        try:
            self._head(now_ns, t, interval, rec)
            if rec.branch != "emergency":
                self._control_and_output(now_ns, t, rec)
        except Exception as exc:  # noqa: BLE001 -- any failure before the output is the emergency branch
            rec.branch, rec.reason = "emergency", f"exception:{type(exc).__name__}:{exc}"
            self._emergency(now_ns, rec.reason)
            self._settle_exception(now_ns, t, rec.reason)
        try:
            self._send(now_ns, rec)  # S10's control stop goes out before the collection (design 4)
        except Exception as exc:  # noqa: BLE001 -- a failing final submit is ① too: DISARM queued and written
            rec.branch, rec.reason = "emergency", f"exception:send:{type(exc).__name__}:{exc}"
            self._emergency(now_ns, rec.reason)
            self._settle_exception(now_ns, t, rec.reason)
            try:
                self._transmit(now_ns)
            except Exception as exc2:  # noqa: BLE001 -- the DISARM stays queued; the permit retries it next tick
                rec.reason = f"{rec.reason};transmit:{type(exc2).__name__}"
        # the record: the frames written whole in this tick -- queued now or finished from an earlier tick (round 8 P3)
        rec.seqs = list(self._written_now)
        partial_ = getattr(self.transport, "partial_frame", None)
        if partial_ is not None:
            rec.notes.append(f"partial:{int(partial_.seq) & 0xFFFF}:{getattr(self.transport, 'partial_since_ns', None)}")
        try:
            self._collect(now_ns, t, rec)
        except Exception as exc:  # noqa: BLE001 -- 6a impl round 1 P1-2: a failing collection is ① too, DISARM written
            rec.branch, rec.reason = "emergency", f"exception:collect:{type(exc).__name__}:{exc}"
            self._emergency(now_ns, rec.reason)
            try:
                self._transmit(now_ns)
            except Exception as exc2:  # noqa: BLE001 -- the DISARM stays queued; the permit retries it next tick
                rec.reason = f"{rec.reason};transmit:{type(exc2).__name__}"
            self._settle_exception(now_ns, t, rec.reason)  # impl round 2 P2-1: the tick's requests answered once
        rec.phase, rec.permit = core.ms.phase, permit.state
        self.records.append(rec)
        self.record(rec)
        if getattr(self.link.a.config, "real_requests", False) and self.link.state_due(now_ns / 1e9):
            try:
                self.channels.put(None, self.real_status(now_ns))  # 5c-1: the console's real fields (design 5)
            except Exception:  # noqa: BLE001 -- a status the console misses is not a reason to stop
                pass
        return rec

    def _head(self, now_ns: int, t: float, interval, rec: TickRecord) -> None:
        core, permit, host = self.core, self.permit, self.host
        if core.slam is not None:
            core.slam["stop_now"] = False  # nothing this tick stands before the stand is judged again (receive may fail)
        self._odom_broken = False
        core.state.pop("motion_hold", None)  # a hold is this tick's only (round 3 P2-2)
        self._apply_bypass(now_ns)  # 5c-1: the session's bypass for every judgement of this tick
        try:
            self.transport.receive(host, now_ns, limit=MAX_STATES_PER_TICK)
        except Exception as exc:  # noqa: BLE001 -- RX failing: the permit still retries its DISARM and judges its deadline
            self._odometry_head(now_ns, t)  # the STATEs taken before the failure count (a break among them locks)
            host.tick(now_ns)
            self._bind_permit(now_ns)
            permit.tick(now_ns)
            rec.branch, rec.reason = "emergency", f"receive:{type(exc).__name__}:{exc}"
            self._emergency(now_ns, rec.reason)
            return
        self._hil_watch()  # 7r plan 3: the round follows every loss and boot change as the receive shows it
        if host.disconnects != self._disconnects:
            # a device loss (a same-boot reconnect included): the queue, the samples and the stand go with it
            self._disconnects = host.disconnects
            if self.operator_start is not None:
                self.operator_start.void("link_lost")  # 5c-1: a check given before a device loss does not count
            self.samples.clear()
            self._odom_broken = True
            self.odom_gen += 1
            self._odom_good = False
            self.still_since_ns = None
            self.stand_since_ns = None
        if self.operating and self.scan_events is not None and self.fatal_source is None:
            # U3a (Codex U3 split round 1 P1-2): the scan source's events before any install this tick -- a boundary
            # throws the layer's evidence away before an older result could be installed
            try:
                self.scan_events(now_ns)
            except Exception as exc:  # noqa: BLE001 -- a source's error ends the run (the run reads fatal_source)
                self.fatal_source = self.fatal_source or f"scan:{type(exc).__name__}:{exc}"
        self._odometry_head(now_ns, t)  # before any early return: a break's lock is never skipped (round 2 P1-3)
        self._sync_relocalization(now_ns, t)  # 5c-2 (6, 1): a reboot's or a break's generation
        if self.scan_feed is not None and self.fatal_source is None:
            try:
                self.scan_feed(now_ns)  # 6a-2b (design 10.1): after the STATE, the reboot and the odometry
            except Exception as exc:  # noqa: BLE001 -- a source's error ends the run (the run reads fatal_source)
                self.fatal_source = self.fatal_source or f"scan:{type(exc).__name__}:{exc}"
        if self.fatal_source is not None:
            rec.branch, rec.reason = "emergency", f"source_fatal:{self.fatal_source}"
            self._emergency(now_ns, rec.reason)
            return
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
        # 6a (design 2): a STATE read after the tick's start counts as received at it for this evaluation
        samples_ = [smp if smp.host_ns <= now_ns else Sample(int(now_ns), smp.state) for smp in self.samples]
        reading = encoder_speed(samples_, now_ns, self.params, self.steering_ok, self._speed_delay_s())
        self.speed = reading.speed
        still = self._still(now_ns)
        if not still and self.operator_start is not None:
            self.operator_start.void("moved")  # 5c-1: a check given before the truck moved does not count
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
        if self.imu_watch is not None and not self._bypassed("imu"):
            self._imu_read_ns = self._pull_imu()  # up to the clock read now, not the tick's start (round 3 P2-3)
            if self.fatal_source is not None:
                rec.branch, rec.reason = "emergency", f"source_fatal:{self.fatal_source}"  # 6a-2b: SourceFatal
                self._emergency(now_ns, rec.reason)
                return
            if permit.state in (ra.OFF, ra.LATCHED):
                self.imu_watch.disarm()
            elif permit.state in (ra.ARMING, ra.READY, ra.DRIVING, ra.STOP_PENDING):
                # judged first: samples taken while the ARM was unanswered still meet the ARMING rule on the tick
                # the permit turned READY (5b round 7 P2-1); then the tilt hold rules
                trip_ = self.imu_watch.judge(now_ns)
                if trip_ is not None:
                    rec.branch, rec.reason = "emergency", f"watch:{trip_}"  # design 4: a lost watch or a tilt is ①
                    self._emergency(now_ns, rec.reason)
                    return
                if permit.state != ra.ARMING:
                    # the ARM answered: samples taken up to this tick's read meet the ARMING rule (by their own time,
                    # at the head or the first byte), later ones the tilt hold -- conservative: READY came no later
                    self.imu_watch.settled(self._imu_read_ns)
        if self._watch_trip is not None:
            trip, self._watch_trip = self._watch_trip, None
            rec.branch, rec.reason = "emergency", f"watch:{trip}"
            self._emergency(now_ns, rec.reason)
            return
        if self._axis_trip is not None:
            trip, self._axis_trip = self._axis_trip, None
            rec.branch, rec.reason = "emergency", f"wait_axis_moved:{trip}"  # 6a (design 2, A'): not bypassable
            self._emergency(now_ns, rec.reason)
            return
        # a STATE read after the tick's start is taken as received at it for the tick's own evaluations (6a design 2)
        feedback = None if s is None or not self.samples else Feedback(
            stamp_mono=min(self.samples[-1].host_ns, now_ns) / 1e9, rear_axle_speed_mps=float(self.speed.signed_mps),
            lift_m=self._measured_lift(), steering_rad=abs(float(s.steering_rad)), odom_xy=self.odom_pose[:2])
        # 6a impl round 6: a failed collection's restart need comes first (the cargo state after it is half-reset)
        refusal = ("not_ready:collect_failed" if self.session.collect_failed is not None else None) \
            or core.cargo.submit_refusal() or self._submit_refusal(now_ns)
        self.serial = self.link.emergency(now_ns / 1e9)  # the emergency batch first (its serial: the send's reference)
        self._stop_kind, self._cmd_records = self.session.tick_start(
            t, now_ns / 1e9, feedback, {"submit_refusal": refusal, "clear_for_submit": True,
                                        "localization_ok": True, "start_blocked": self._start_blocked()}, collect=lambda: self._collect(now_ns, t, rec))
        self._sync_relocalization(now_ns, t)  # 5c-2 (6, 1): after the batch's requests and the end's collection
        if self.fatal_source is not None:
            # 6a-2b (design 10.5): a source call made by the console's requests (relocalize, request_processed); the
            # tick's requests are answered once (design 10.6)
            rec.branch, rec.reason = "emergency", f"source_fatal:{self.fatal_source}"
            self._emergency(now_ns, rec.reason)
            self._settle_exception(now_ns, t, rec.reason)
            return
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

    IMU_BATCH = 64  # the most samples taken from the source per read (its contract: non-blocking, ascending)

    def _pull_imu(self, upto_ns: int | None = None) -> int:
        """The source's samples taken up to ``upto_ns`` (the clock read just before, by default) into the IMU watch,
        in acquisition order (design 4: later ones stay in the source for the next read); a failing source gives
        none this time (the watch's stale and gap judgements take it). Returns the instant used.

        A sample is ``(stamp, accel)`` (a point, no generation: the 5b contract) or ``(gen, lo, hi, accel)`` (PR 7
        design 5: an acquisition interval and its producer generation). A source with ``generation()`` is asked for
        its current generation after a read that did not stop at its cap, so a restart reaches the watch with no new
        sample (design 5, Codex round 5 P2); after a capped read the older samples still waiting come first. The
        generation is asked even when the samples could not be read; when it cannot be asked, ``imu_gen_unknown``
        holds the ARM and every moving DRIVE until a read can (Codex 7b round 1 P1-1). A source that stops a read at
        its own cost cap (``last_read_capped``, raw items counted) is a capped read too (round 1 P2)."""
        upto = int(self.clock()) if upto_ns is None else int(upto_ns)
        self.imu_backlog = False
        self.imu_gen_unknown = False
        try:
            batch = list(self.imu_source.samples(upto, self.IMU_BATCH))[: self.IMU_BATCH]
        except Exception:  # noqa: BLE001
            batch = []  # no samples this time (the watch's stale and gap judgements take it)
        else:
            # 6a-2b (design 10.7-10.8): a read that reached the per-call cap may have left samples up to ``upto``
            self.imu_backlog = (len(batch) >= self.IMU_BATCH
                                or getattr(self.imu_source, "last_read_capped", False) is True)
        good = []
        for item in batch:
            gen, lo, hi, accel = _imu_item(item)
            if hi is None:
                # host ns are integers: anything else is that sample's fault only, kept at full precision otherwise
                # (design 4; 5b rounds 4-5 P2)
                self.imu_watch.counts["not_finite"] += 1
                continue
            good.append((gen, lo, hi, accel))
        for gen, lo, hi, accel in sorted(good, key=lambda b: b[2]):
            self.imu_watch.add_interval(gen, lo, hi, accel, upto)
        current = getattr(self.imu_source, "generation", None)
        if current is not None and not self.imu_backlog:
            try:
                gen = current()
            except Exception:  # noqa: BLE001 -- a SourceFatal was marked by the guard
                self.imu_gen_unknown = True
            else:
                self.imu_watch.note_generation(gen)
        return upto

    def _imu_arm_gate(self, now_ns: int, mission_id, *, not_still: bool = False):
        """Before the ARM (design 4), after the existing refusals: ("ok", baseline), ("wait", reason) within the
        mission's one deadline for the stand and the baseline to gather, or ("refuse", reason)."""
        if self.imu_backlog or self.imu_gen_unknown:
            # samples still waiting or a generation not known: no baseline is frozen on them (Codex 7b round 1 P2)
            base, why = None, "imu_backlog" if self.imu_backlog else "imu_generation"
        else:
            base, why = (None, "not_still") if not_still else self.imu_watch.baseline(self.still_since_ns)
        if why is None:
            until = self._imu_wait_until.get(mission_id)
            # judged on the clock read now, just before the baseline is frozen -- not the tick's start (round 3 P1-2)
            if until is not None and max(int(now_ns), int(self.clock())) >= until:
                return "refuse", "imu_wait_expired"  # ready only after the mission's one deadline (round 2 P2)
            return "ok", base
        if why in ("not_still", "imu_no_baseline", "imu_not_still", "imu_backlog", "imu_generation"):
            until = self._imu_wait_until.setdefault(
                mission_id, int(now_ns) + int(self.imu_watch.cfg.baseline_wait_s * 1e9))
            if int(now_ns) < until:
                return "wait", why
        return "refuse", why

    def _before_arm(self) -> bool:
        """A mission the permit does not collect yet: accepted or preparing, or adopted and running with the permit
        still off before its ARM (``AutoPermit`` collects from ``adopted()`` on)."""
        cmds = self.core.commands
        if cmds.status in ("accepted", "preparing"):
            return True
        return (cmds.status == "running" and self.permit.state == ra.OFF and self.adopted_for != cmds.mission_id)

    def _odometry_head(self, now_ns: int, t: float) -> None:
        """The odometry's part of the head, whatever follows: the continuity, the layer's reset on a break, the path
        driven and the installs, and the relocalization lock of a reboot or a same-boot break."""
        core = self.core
        # continuous: no break this tick and a good STATE since the last one (an empty tick does not mend a break,
        # Codex 5a impl round 3 P2-2)
        self.odom_continuous = bool(self.samples) and not self._odom_broken and self._odom_good
        if self.real_layer is not None:
            if self._odom_broken:
                self._reset_layer(now_ns)  # a same-boot break: the layer starts again (design 1.1, round 6 P1-3)
            # the path driven since the last tick off the verified distance, then the installs (whose own travel
            # since their request replaces it) -- Codex 5a impl P2-3
            self.real_layer.advance(self.odom_travel - self._travel_ticked)
            self._travel_ticked = self.odom_travel
            self.real_layer.install_ready(self._t(now_ns), self.odom_travel)
        if self._rebooted:
            # an MCU reboot (design 8 재시작): the position evidence goes too -- relocalize before any submit
            self._rebooted = False
            self._break_open = True
            core.require_relocalization(t, "mcu_reboot")
            if self.delay_bound is not None:
                self._resession = True  # the new boot's session check and delay anchors (round 2 P2-4)
        elif self._odom_broken and self._ever_continuous and not self._break_open:
            # a same-boot break (a gap, an encoder cut, an invalid steering, a device loss): the motion meanwhile
            # was not integrated, so the map pose is not trusted either (Codex 5a impl P1-3)
            self._break_open = True
            core.require_relocalization(t, "odometry_break")
        if self.odom_continuous:
            self._break_open = False
            self._ever_continuous = True
        if self._resession and self.permit.state in (ra.OFF, ra.LATCHED) \
                and getattr(self.permit, "_disarm_first_ns", None) is None:
            self._resession = False
            self.delay_bound.restart_check()
            self.permit.restart_session(now_ns)

    def _submit_refusal(self, now_ns: int) -> str | None:
        if self.session.collect_failed is not None:
            return "not_ready:collect_failed"  # 6a: a mission collection raised -- restart the executor
        why = self._ready(now_ns, "submit")
        return None if why is None else f"not_ready:{why}"

    def _bind_permit(self, now_ns: int) -> None:
        """The permit's callbacks at this instant (a latch's or an unanswered DISARM's deadline)."""
        a = self.link.a

        def arbiter_stop(r):
            if a.mode != "STOPPED":
                a.stop(now_ns / 1e9, r)

        self.permit.bind(transmit=lambda: self._transmit(now_ns, escalate=False), arbiter_stop=arbiter_stop,
                         collect=lambda r: self._end_now(now_ns, r))

    def _on_stop_decided(self, reason) -> None:
        """The permit decided a stop (a latch or a DISARM): kept with the evidence instant and reason that led to it in
        this tick -- the readiness's, the send's or the gate's own instant; else this tick's start (6a-2b round 5)."""
        now_c = int(self.clock())
        self.disarm_decisions.append((now_c, str(reason)))
        cause = self._cause
        if cause is not None and str(cause[1]) not in str(reason):
            cause = None  # an evidence read this tick that is not what this stop names (round 6 P2-1)
        self._emit({"t": "stop_decision", "ns": now_c, "reason": str(reason), "tick_ns": self._tick_ns,
                    "cause_ns": self._tick_ns if cause is None else cause[0],
                    "cause_why": None if cause is None else str(cause[1])})

    def _emergency(self, now_ns: int, reason: str) -> None:
        self._emit({"t": "cause", "ns": int(now_ns), "why": str(reason), "where": "emergency"})
        if self._cause is None:
            self._cause = (int(now_ns), str(reason))
        self.waiting = None
        self._mark_stop()  # 5d (7.1 6)
        if self.permit.state == ra.LATCHED:
            return  # latched already: its DISARM is retried every 100 ms, not renewed each tick (round 2 P2-2)
        # a mission the permit does not collect yet (before its ARM) ends with any latch, whatever the cause (5a,
        # Codex impl round 6 P2-2): read before the latch moves the permit off OFF
        before_arm = self.real_layer is not None and self._before_arm()
        # the DISARM is tried before the arbiter's stop and the collection (4b design 7)
        self._bind_permit(now_ns)
        try:
            self.permit.latch(now_ns, reason)
        except Exception:  # noqa: BLE001 -- the DISARM is queued and the arbiter stopped before a collection fails
            pass
        if before_arm:
            self._end_now(now_ns, reason)

    def _end_now(self, now_ns: int, reason: str) -> None:
        cmds = self.core.commands
        if cmds.status in ("accepted", "preparing", "running", "paused", "resuming", "held"):
            outcome = "recovery_required" if (self.core.cargo.pocket_entered
                                              or self.core.cargo.relation != "free") else "cancelled"
            cmds.terminate(outcome, self._t(now_ns), f"real:{reason}")

    # --- control and output ---------------------------------------------------------------------------------------
    def _control_and_output(self, now_ns: int, t: float, rec: TickRecord) -> None:
        core, permit, cmds, ms = self.core, self.permit, self.core.commands, self.core.ms
        self._slam_step(now_ns, t, ms.phase in ("lift", "extract", "transport", "lower"))
        rear = np.asarray(core.hooks.slam_rear(t), dtype=float)
        core.hooks.set_rear(rear)
        self._rear_now = tuple(float(v) for v in rear)
        if self.outputs_closed and core.slam is not None:
            # 5c-2 (6, 2b; round 9 P2-2, round 10 P2-4): the hold's travel over its limit while the outputs are
            # closed opens a generation, whether or not a release is deferred (the read raised nothing)
            sl_ = self.evidence.slam(now_ns)
            if sl_.mode == "holding" and sl_.hold_travel_m > ra.HOLD_MAX_M and core.relocalize_generation() is None:
                core.require_relocalization(t, "hold_too_long")
            self._sync_relocalization(now_ns, t)
        lift_ = self._measured_lift()
        applied_ = tuple(float(v) for v in (core._applied_now() or (0.0, 0.0, 0.0)))  # noqa: SLF001
        self.pose_history.append((t, tuple(float(v) for v in rear), self.odom_continuous,
                                  max(abs(self.speed.signed_mps), self.speed.bound_mps), lift_, applied_))
        self._feed_depth(now_ns, t)
        if self.real_layer is not None:
            self._layer_tick(t, rear)
            if self._before_arm():
                # a fault before the ARM latches and ends the mission whether or not a plan is out yet (design 1.2,
                # round 3 P2-4; Codex 5a impl P2-2: waiting for the planning grid included; round 3 P1: the adopted
                # mission waiting for its new grid before the ARM too)
                why_ = self._ready(now_ns, "drive")
                lw_ = self.real_layer.watchdog(t)
                if lw_ in LAYER_ENDS:
                    why_ = f"layer:{lw_}"  # the layer's own end names the cause, not "missing" (round 5 P3)
                if why_ is not None and ra.latches(why_):
                    rec.branch, rec.reason = "emergency", f"prepare:{why_}"
                    self._emergency(now_ns, rec.reason)
                    self._end_now(now_ns, rec.reason)
                    return
        self._attempt_judge(now_ns, t)  # 5c-2 (6, 5): after this tick's replies and installs
        if self._before_arm() and self._off_start(rear):
            cmds.terminate("cancelled", t, "not_at_start")  # 5c-2 (6, 6): the start held until the first move
            return
        # the preparation (design 2.2 5)
        if cmds.status == "accepted" and self.frozen is None:
            if self.operator_start is not None and not self._bypassed("operator_start"):
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
                self._sync_relocalization(now_ns, t)  # 5c-2 (6, 1)
        gate_ = core.release_gate(t, rear)
        rear = gate_.rear
        # the layer latency watchdog on every tick, whatever the phase (as the Isaac loop with its budget on; Codex 4c
        # impl P1-12): past its end it is ①, a watchdog stop holds the wheels and the lift (the wait frame)
        layer_ = None if core.obstacle is None else core.obstacle.get("layer")
        layer_stop = layer_.watchdog(t) if layer_ is not None and hasattr(layer_, "watchdog") else None
        if layer_stop in LAYER_ENDS:
            rec.branch, rec.reason = "emergency", layer_stop
            before_arm = self._before_arm()  # read before the latch turns the permit off OFF (round 4 P2-1)
            self._emergency(now_ns, layer_stop)
            if before_arm:
                self._end_now(now_ns, layer_stop)  # a mission the latch does not collect yet (design 3, round 3 P2-1)
            if layer_stop in ("layer_latency_exceeded", "scan_source_lost", "scan_generation_changed") \
                    and hasattr(layer_, "end_taken") \
                    and self.permit.state in (ra.LATCHED, ra.DISARMING):
                layer_.end_taken()  # latched on it: the next result's deadlines start afresh (a rearm follows)
            return
        if self.real_layer is not None and (not getattr(core.options, "obstacle_act", False)
                                            or layer_ is not self.real_layer):
            rec.branch, rec.reason = "emergency", "obstacle_layer_off"  # never released while running (design 3)
            before_arm = self._before_arm()
            self._emergency(now_ns, "obstacle_layer_off")
            if before_arm:
                self._end_now(now_ns, "obstacle_layer_off")  # before the ARM too (Codex 5a impl round 2 P2-3)
            return
        if self.real_layer is not None and cmds.status in ("accepted", "preparing") and core.job_pending():
            # before the adoption (design 1.2, round 3 P2-3/P2-4): a planning answer from another correction
            # generation is dropped and asked again; a fault latches and ends the preparation; a wait defers it
            if self._planning_gen is not None and int(core.obstacle["version"]) != self._planning_gen:
                core.abandon_jobs(t)
        # as the Isaac loop: paused, resuming or cancelling, or no mission yet (or over) -- a preparation is not
        paused_ = cmds.stopped or cmds.status in ("idle", *FINAL)
        defer_ = self.real_layer is not None and cmds.status in ("accepted", "preparing") and (
            layer_stop is not None or (self._ready(now_ns, "drive") or "").startswith("wait:"))
        plan_wait_ = core.jobs is not None and (defer_ or core.job_hold(t, rear, self.speed, paused_).hold)
        if cmds.status in ("accepted", "preparing"):
            plan_wait_ = True
        if cmds.status in FINAL:
            self.frozen = None
        if self.real_layer is not None:
            # the adoption may have changed the mission, the tracker or the leg this tick: the key and the request
            # follow before readiness is read (Codex 5a impl P2-1); then the computation the placements asked for
            self._layer_tick(t, rear)
            self.real_layer.flush(t)
        # the adoption and the arming (design 2.3)
        ready_wait = self._ready(now_ns, "drive")
        layer_wait = (self.real_layer is not None and (self.real_layer.installing() or layer_stop is not None)) \
            or bool(self._bypass_pending)  # 5c-1: no ARM while a bypass toggle waits for the next tick head
        if (cmds.status == "running" and cmds.adopted_id == cmds.mission_id and ms.phase == "approach"
                and self.adopted_for != cmds.mission_id and permit.state == ra.OFF and not layer_wait
                and not (ready_wait or "").startswith("wait:")):  # no ARM while a wait holds (Codex 4c impl P2-3)
            why = self._pre_arm_refusal(now_ns)  # the existing refusals first (round 3 P2-2)
            gate_imu = ("ok", None)
            if self.imu_watch is not None and not self._bypassed("imu") and why in (None, "not_still"):
                gate_imu = self._imu_arm_gate(now_ns, cmds.mission_id, not_still=why == "not_still")
                if gate_imu[0] == "refuse":
                    why = gate_imu[1]
            if gate_imu[0] != "wait":  # a wait leaves adopted_for unset: the ARM is asked again next tick
                self.adopted_for = cmds.mission_id
                if self.imu_watch is not None and why is None and gate_imu[1] is not None:
                    self.imu_watch.arm(gate_imu[1])  # frozen before the ARM is asked (design 4)
                cmds.begin_wait(t)
                a_ = self.link.a
                # 6a (premise review round 2 P1-3): the authority the ARM and its zero are written under
                self._arm_binding = (None if a_.lease is None else a_.lease.lease_id, a_.control_gen)
                self._kept = set()
                if not permit.adopted(now_ns, pre_arm_refusal=why):
                    if self.imu_watch is not None:
                        self.imu_watch.disarm()
                    rec.branch, rec.reason = "emergency", f"arm:{permit.reason}"
                    self._emergency(now_ns, permit.reason or "arm")
                    return
        bad = self._unsupported()
        if bad is not None:
            rec.branch, rec.reason = "emergency", f"unsupported:{bad}"
            self._emergency(now_ns, f"unsupported:{bad}")
            return
        if permit.state == ra.ARMING and self.real_layer is not None and ra.latches(ready_wait):
            # a fault while the ARM is answered: no frame of this tick may carry it to the first byte (round 4 P1)
            rec.branch, rec.reason = "emergency", f"arming:{ready_wait}"
            self._emergency(now_ns, rec.reason)
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
                and self._permission_needed(requested_speed)):
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
        if self.depth_source is None or check is None or getattr(check, "bypass", False):
            return  # 5d (7.2): under the D5 bypass the depth source is not read at all
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

    def _permission_needed(self, requested_speed: float) -> bool:
        """Whether this tick asks the permission (4c: a request, or a measured speed over 0.05 m/s); 5d (design 7.1 5):
        under a pocket check (the bypass check too) also a remaining output or a possible motion with no request."""
        if requested_speed != 0.0 or abs(self.speed.signed_mps) > 0.05:
            return True
        core = self.core
        return core.pocket.get("check") is not None and (
            bool(core.state.get("outstanding_motion")) or self.speed.bound_mps > 0.0)

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
        why = self._ready(now_ns, "drive")
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
        self._tick_end_tried = True  # 6a (design 5): set before the call -- a tick_end that raises is not repeated
        out = self.session.tick_end(now_ns / 1e9, self._cmd_records, safety,
                                    (float(command_.drive.speed_mps), float(command_.drive.curvature_inv_m)), None)
        self._candidate = (command_, command_.steering_command, out, sink)

    def _speed_delay_s(self):
        """The newest STATE's measurement-to-receipt delay the safety speed adds to its age (6a-2b impl round 1 P1-1):
        its round-trip bound when the loop keeps them (None, unknown: no safety speed), else the fixed ``delay_s``."""
        if self.delay_bound is None:
            return self.delay_s
        if not self.samples:
            return None
        st = self.samples[-1].state
        return (self._sample_delay.get((int(st.boot_id), int(st.mcu_time_us))) or (None, None))[1]

    def _mark_stop(self) -> None:
        """This tick decided a stop or a wait (5d 7.1 6): no older moving frame is finished after it; the first such
        decision's clock is the tick record's (6a-2b design 10.7)."""
        self._stop_decided = True
        rec = getattr(self, "_rec", None)
        if rec is not None and rec.stop_decided_ns is None:
            rec.stop_decided_ns = int(self.clock())

    # --- 6a-2b (design 10.3): the session's start and the closing ---------------------------------------------------
    def start_session(self, now_ns: int) -> None:
        """The permit's callbacks bound at this instant and its session check started (the two DISARM exchanges)."""
        self._bind_permit(now_ns)
        self.permit.start_session(now_ns)

    def begin_closing(self, now_ns: int, reason: str = "run_end") -> None:
        """The run ends: no new command (the arbiter STOPPED, a rearm refused by the run), the gate passes the DISARM
        only, no source is called; the permit's latch asks a DISARM (a latched permit retries it)."""
        if self.closing:
            return
        self.closing, self.closing_started_ns = True, int(self.clock())
        self._cause, self._tick_ns = None, int(now_ns)  # the closing's own decision (round 6 P2-1)
        cancel = getattr(self.permit, "cancel_session", None)
        if cancel is not None:
            cancel()  # design 10.6: no second session DISARM while closing
        a = self.link.a
        try:
            if a.mode != "STOPPED":
                a.stop(now_ns / 1e9, reason)
        except Exception:  # noqa: BLE001 -- the DISARM below is what matters
            pass
        self._bind_permit(now_ns)
        try:
            self.permit.latch(now_ns, reason)
        except Exception:  # noqa: BLE001 -- its DISARM is queued before anything that can fail
            pass
        try:
            self._transmit(now_ns, escalate=False)
        except Exception:  # noqa: BLE001 -- retried by the closing ticks
            pass

    def _closing_tick(self, now_ns: int) -> TickRecord:
        """A closing tick: the STATEs, the permit (its DISARM retries) and the DISARM out -- nothing else."""
        t = self._t(now_ns)
        rec = TickRecord(t, None if self.last_start_ns is None else (now_ns - self.last_start_ns) / 1e9, "closing")
        self.last_start_ns = int(now_ns)
        self._rec = rec
        self._written_now = []
        try:
            self.transport.receive(self.host, now_ns, limit=MAX_STATES_PER_TICK)
        except Exception as exc:  # noqa: BLE001 -- a lost device is the transport's own record
            rec.reason = f"receive:{type(exc).__name__}"
        try:
            self.host.tick(now_ns)
            self._bind_permit(now_ns)
            self.permit.tick(now_ns)
            self._transmit(now_ns, escalate=False)
        except Exception as exc:  # noqa: BLE001
            rec.reason = f"{rec.reason or ''};closing:{type(exc).__name__}"
        rec.seqs = list(self._written_now)
        rec.permit = self.permit.state
        self.records.append(rec)
        self.record(rec)
        return rec

    def _emit(self, event: dict) -> None:
        if self.events is not None:
            try:
                self.events(event)
            except Exception:  # noqa: BLE001 -- the record never holds the tick (design 10.3)
                pass

    def _settle_exception(self, now_ns: int, t: float, reason: str) -> None:
        """6a (design 5): after the emergency (DISARM first, and its own mission end), the tick's console requests are
        answered now rather than ``invalid`` next tick: this tick's apply records, or -- none made -- what an apply under
        way recorded plus ``tick_failed`` for every command not recorded or still queued (the queue emptied either
        way); ``tick_end`` once, only when this tick's ``link.begin`` was entered and no ``tick_end`` was tried."""
        session, cmds = self.session, self.core.commands
        try:
            records = session.last_records if session.last_records is not None else cmds.abandon(t)
        except Exception:  # noqa: BLE001 -- the queue is emptied by force: nothing old runs in a later generation
            cmds._queue, cmds.applying = [], None  # noqa: SLF001
            records = []
        if not session.began or self._tick_end_tried:
            return
        try:
            safety = self.core.safety_state(t)
        except Exception:  # noqa: BLE001 -- the least state: the exception itself
            safety = {"real_exception": reason}
        self._tick_end_tried = True
        try:
            session.tick_end(now_ns / 1e9, records, safety, (0.0, 0.0), None)
        except Exception:  # noqa: BLE001 -- the rest is answered invalid next tick (the present behaviour)
            pass

    # --- collection, then the one output branch and the send ---------------------------------------------------------
    def _collect(self, now_ns: int, t: float, rec: TickRecord) -> None:
        if self.session.ended_unclaimed():
            release = self._release_allowed(now_ns)
            # PR 7 design 6.1, 2: the bypasses applied during this mission, in its result whether the collection
            # holds or fails (Codex 7c-1b-1 round 1 P2-4) -- an aggregate of F1 / SLAM evidence leaves out the runs
            # that had any (a session's end state cannot tell one turned off mid-session)
            used, self._mission_bypass = sorted(self._mission_bypass | self._shown_bypass()), set()
            self.session.end_mission(t, lift_m=self._measured_lift(), release_allowed=release, bypassed=used)
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
            self._mark_stop()  # 6a: S10's stop is this tick's stop too
            sample, why = self._hold_sample()  # 6a (design 2): the same fresh-sample rule as a wait's frame
            if self._measured_lift() is None or self._measured_steering() is None or why == "no_measurement":
                self._emergency(now_ns, "s10_measurement")
            elif sample is None:
                self._emergency(now_ns, f"s10_no_sample:{why}")  # no position-holding stop: the DISARM stops
            else:
                q, lift, acq = sample
                r = port.control_stop(steering_rad=q, lift_m=lift, now_ns=now_ns,
                                      expires_ns=now_ns + int(DRIVE_LIFE_S * 1e9))
                if r.kind != "queued":
                    self._emergency(now_ns, f"s10_stop:{r.reason}")
                else:
                    self._book(now_ns, r.seq, 0.0, 0.0, (q,), lift, False, "stop", steer=(q,))
                    self._hold_acq[int(r.seq) & 0xFFFF] = acq
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
            self._emit({"t": "cause", "ns": int(send_ns), "why": "measurement_stale", "where": "send"})
            self._cause = (int(send_ns), "measurement_stale")
            return "measurement_stale"
        why = self._ready(send_ns, "drive")
        if why is not None and ra.latches(why):
            return f"send:{why}"
        if not self._lease_valid(send_ns):
            return "lease_expired"
        return None

    def _wait_reason_now(self, now_ns, out) -> str | None:
        hold = self.core.state.pop("motion_hold", None)  # this tick's, consumed whichever reason is returned
        why = self._ready(now_ns, "drive")
        if why is not None and ra.latches(why):
            self._emergency(now_ns, why)
            return "latched"
        if why is not None:
            return why
        if hold is not None:
            return f"hold:{hold}"  # an unconfirmed output beyond what is allowed: wheels 0 now, its queue dropped
        return getattr(self, "_wait_reason", None)

    def _wait_frame(self, now_ns: int, rec: TickRecord | None) -> None:
        self._mark_stop()  # 5d (7.1 6)
        port = self.port
        if self._lift_before is not None:
            # the output holds: this tick's lift ramp step is undone, whichever stage chose the wait (round 7 P2-2)
            self.core.ms.lift_command = self._lift_before
        if self._wait_frames_tick >= 1:
            if rec is not None:
                rec.notes.append("wait_frame_skipped:once_per_tick")  # 6a (round 10 P2-1)
            return
        if self._measured_lift() is None or self._measured_steering() is None:
            self._emergency(now_ns, "wait_measurement")
            return
        # 6a (design 2, rounds 10-15): this tick's measurement, never the wait's first target -- the frame holds the
        # axes where they are now; without a fresh sample no frame this tick
        sample, why = self._hold_sample()
        if sample is None:
            if why == "no_measurement":
                self._emergency(now_ns, "wait_measurement")
            else:
                self._skip_hold(now_ns, rec, why)
            return
        q, lift, acq = sample
        self._wait_frames_tick += 1
        first = self.waiting is None
        if first:
            self.waiting = (q, lift)  # the wait's mark (its first measurement), not a target any more
            r = port.control_stop(steering_rad=q, lift_m=lift, now_ns=now_ns,
                                  expires_ns=now_ns + int(DRIVE_LIFE_S * 1e9))
        else:
            r = port.hold_frame(steering_rad=q, lift_m=lift, now_ns=now_ns,
                                expires_ns=now_ns + int(DRIVE_LIFE_S * 1e9))
        if r.kind != "queued":
            self._emergency(now_ns, f"wait:{r.reason}")
            return
        self._book(now_ns, r.seq, 0.0, 0.0, (q,), lift, False, "hold", steer=(q,))
        self._hold_acq[int(r.seq) & 0xFFFF] = acq
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
        cap_ = self._speed_cap()  # 5c-2 (6, 7b): 0.06 m/s while slam is bypassed or a start was declared
        if abs(speed) > cap_:  # the rear-axle command cap (design 2.5 4)
            drive = self.kin.command(math.copysign(cap_, speed), float(drive.curvature_inv_m))
            speed = float(drive.speed_mps)
        may = self.book.may_act(now_ns)
        if speed != 0.0 and may.get(-1 if speed > 0 else 1, 0.0) > 0.0:
            drive = self.kin.command(0.0, float(drive.curvature_inv_m))  # the turn waits for the other way to settle
            speed = 0.0
        if speed == 0.0:
            self._mark_stop()  # 5d (7.1 6): a zero DRIVE is this tick's stop
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
            self._hold_acq.pop(old, None)
        self._emit({"t": "frame_queued", "seq": seq, "purpose": kind, "speed_mps": float(speed), "lift_m": float(lift),
                    "decided_ns": int(now_ns), "expires_ns": int(now_ns + int(DRIVE_LIFE_S * 1e9)),
                    "boot": None if s is None else s.boot_id, "arm_gen": None if s is None else s.arm_gen,
                    "echo_mcu_time_us": None if s is None else int(s.mcu_time_us)})
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
        motion = motion_allowed(self.transport) and not self.closing  # 6a: after a resync; closing: DISARM only

        def judge(frame) -> tuple:
            """(reason, class) for a non-DISARM frame now; class: alarm | wait | drop. The readiness evidence is read
            first (its ages at ``ns0``); everything timed is then read on the clock after it, and a read that took
            more than EVIDENCE_READ_MAX_S refuses (round 5 P1-3)."""
            self._judged_ns = None
            self._gate_at = (None, None)
            if self.fatal_source is not None:
                return f"source_fatal:{self.fatal_source}", "alarm"  # 6a-2b (design 10.5): at every send
            if permit.state not in authority:
                return f"permit_{permit.state.lower()}", "drop"  # a latch or a DISARM under way: no motion (P1-1)
            if a.mode == "STOPPED":
                return "stopped", "alarm"
            # every timed judgement at ``ns``, the latest instant the write may happen: the evidence's ages too (a
            # scan judged young at the read's start may be over its limit at the write, round 6 P1-1)
            ns0 = int(self.clock())
            ns = ns0 + int(EVIDENCE_READ_MAX_S * 1e9)
            self._gate_at = (ns0, ns)  # 6a-2b (design 10.7): the gate event's instants
            why = self._ready(ns, "drive")
            imu_trip = None
            if self.imu_watch is not None and not self._bypassed("imu"):
                # the IMU read and judged inside the same budget, at the write's latest instant (design 4)
                self._pull_imu()  # the samples taken so far; their age judged at the write's latest instant
                if self.fatal_source is not None:
                    return f"source_fatal:{self.fatal_source}", "alarm"  # 6a-2b: SourceFatal at the gate's read
                imu_trip = self.imu_watch.judge(ns)
            if int(self.clock()) > ns:
                return "evidence_slow", "alarm"
            if imu_trip is not None:
                return f"send:watch:{imu_trip}", "alarm"
            if (self.imu_backlog or self.imu_gen_unknown) and self.imu_watch is not None \
                    and not self._bypassed("imu") and why is None:
                # 6a-2b (design 10.8): only after SourceFatal, the watch's trips and every alarm before it -- the read
                # stopped at its cap, so the newest samples were not judged, or the source's generation could not be
                # asked (PR 7 design 5): this frame waits
                why = "wait:imu_backlog" if self.imu_backlog else "wait:imu_generation"
            if (ns - now_ns) / 1e9 > LATE_TICK_S:
                return "tick_late", "alarm"  # a late tick is ① whatever its frame (design 3)
            if not self.host.health(ns).fresh:
                return "measurement_stale", "alarm"
            if self.real_layer is not None:
                # the layer's watchdog at the write too (Codex 5a impl P1-1): a deadline passed since the decision
                lw = self.real_layer.watchdog(self._t(ns))
                if lw in LAYER_ENDS:
                    return f"send:{lw}", "alarm"
                if lw is not None and why is None:
                    why = f"wait:{lw}"
                elif self.real_layer.installing() and why is None:
                    why = "wait:snapshot_installing"
            if self.real_layer is not None and (not getattr(self.core.options, "obstacle_act", False)
                                                or self.core.obstacle.get("layer") is not self.real_layer):
                return "send:obstacle_layer_off", "alarm"  # design 3 at the write too (Codex 5a impl round 5 P1)
            if frame.kind == "ARM" and why is not None and not ra.latches(why):
                return f"arm_wait:{why}", "alarm"  # no new ARM while a wait holds: latch, a rearm (design 3)
            if frame.kind == "ARM":
                # lead #623 decision 1 (7c-2b, Codex design check P1): a start needs this round's real processed
                # fresh at the ARM's write itself, whatever is bypassed (slam, f1, operator_start)
                sl_ = self.evidence.slam(ns)
                if sl_.pose_reply_ns is None or ns - sl_.pose_reply_ns > ra.SLAM_MAX_AGE_S * 1e9:
                    return "arm_slam_stale", "alarm"
            if why is not None and ra.latches(why):
                return f"send:{why}", "alarm"
            if not self._lease_valid(ns):
                return "lease_expired", "alarm"
            seq = int(frame.seq) & 0xFFFF
            bound = self._binding.get(seq)
            echo_ = getattr(frame, "echo", None)
            arming_ = frame.kind == "ARM" or (frame.kind == "DRIVE" and echo_ is not None and bound is None)
            if arming_:
                bound = self._arm_binding  # 6a: the ARM's own binding, kept through the exchange
            if bound is not None and bound != (None if a.lease is None else a.lease.lease_id, a.control_gen):
                return "authority_changed", "alarm"
            if arming_ and self.delay_bound is not None:
                # 6a (design 2, rounds 12-13, premise review; impl round 1 P2-1): an ARM or zero carries its original
                # echo -- written only within the MCU's echo window from that STATE's acquisition, whether it came
                # back by a late judgement or a 0-byte EAGAIN; a retry without the echo's delay bound is given up
                rx_b = self._sample_delay.get((int(echo_[0]), int(echo_[1]))) if echo_ is not None else None
                retry_ = seq in self._kept
                self._kept.add(seq)
                if rx_b is None or rx_b[1] is None:
                    if retry_:
                        return "arm_echo_unknown", "cancel"
                    rx_b = None
                window_ms = int(self.host.cfg.cap_ms) if frame.kind == "ARM" else min(
                    int(self.host.cfg.zero_valid_ms), int(self.host.cfg.cap_ms))
                if rx_b is not None and ns > int(rx_b[0]) - int(round(rx_b[1] * 1e9)) + window_ms * 1_000_000 \
                        - int(WRITE_ALLOWANCE_S * 1e9):
                    return "arm_echo_expired", "cancel"
            # every alarm first: an old frame's expiry never hides one (round 7 P2-1)
            expires = getattr(frame, "expires_ns", None)
            if expires is not None and ns >= int(expires):
                if arming_ and frame.kind == "DRIVE":
                    return "zero_expired", "cancel"  # 6a impl round 2 P2-3: the arming zero ran out at the gate
                return "expired", "drop"  # an old frame left in the queue: dropped, no alarm by itself
            booked = next((f for f in self.book.frames if f.seq == seq), None)
            continuing_ = frame is getattr(self.transport, "partial_frame", None)
            if arming_ and frame.kind == "DRIVE" and continuing_ and expires is not None \
                    and int(expires) - int(self.host.cfg.zero_valid_ms) * 1_000_000 < int(now_ns):
                return "zero_carried", "drop"  # 6a impl round 1 P1-1: a half arming zero is not finished next tick
            if booked is not None and booked.kind == "drive" and self._stop_decided and continuing_:
                # 5d (7.1 6), 6a premise review P1-1: this tick stops -- no older normal DRIVE (a lift- or steering-only
                # one too) is finished first
                return "stop_decided", "wait"
            if booked is not None and booked.kind in HOLDING_KINDS:
                # 6a (design 2, rounds 11-14 and premise review): a position-holding frame goes out only in the tick
                # that made it, its sample still within the age bound at the write
                if int(booked.queued_ns) != int(now_ns):
                    return "hold_carried", "drop"
                acq = self._hold_acq.get(seq)
                if acq is None or ns - int(acq) > int(HOLD_SAMPLE_MAX_AGE_S * 1e9):
                    return "hold_sample_old", "drop"
            if (booked is not None and booked.kind == "drive" and float(booked.speed_mps) != 0.0
                    and self._moved_for != self.core.commands.mission_id and self._off_start(self._rear_now)):
                return "not_at_start", "alarm"  # 5c-2 (6, 6): at the first byte and a continuation alike
            if booked is not None and booked.kind == "drive":
                # a normal DRIVE (moving or a small lift step alike, round 6 P1-3): a wait reason that came up since the
                # decision, or 30 ms spent (round 4 P1-4)
                if why is not None:
                    return why, "wait"
                if (ns - now_ns) / 1e9 > SLOW_TICK_S:
                    return "tick_slow", "wait"
            # 5d (7.1 6), 6a (design 2, rounds 9-16): the write bound read last, on every frame but DISARM -- the
            # instant the evidence was judged at (ns) for every frame, and the decision's own evidence instant
            # (decision + W) for every DRIVE, each held A ahead (the read-to-write latency); a continuation refused here
            # is link_stuck by the gate's rule. The value compared is the one passed to the write (judged_ns).
            limit = ns - int(WRITE_ALLOWANCE_S * 1e9)
            decided = None
            if booked is not None:
                decided = int(booked.queued_ns)
            elif frame.kind == "DRIVE" and expires is not None:
                decided = int(expires) - int(self.host.cfg.zero_valid_ms) * 1_000_000  # the host's zero (its queueing)
            if decided is not None:
                limit = min(limit, decided + int((WRITE_MARGIN_S - WRITE_ALLOWANCE_S) * 1e9))
            judged = int(self.clock())
            self._judged_ns = judged
            if judged > limit:
                if frame.kind == "DRIVE" and booked is None and decided is not None \
                        and judged > decided + int((WRITE_MARGIN_S - WRITE_ALLOWANCE_S) * 1e9):
                    return "zero_late", "cancel"  # 6a impl round 1 P2-2: past its own bound, the exchange is given up
                if frame.kind == "ARM" or (frame.kind == "DRIVE" and booked is None):
                    self._kept.add(seq)
                    return "write_late", "keep"  # 0 bytes out: back to the queue head, judged again (6a design 2)
                if booked is not None and booked.kind in HOLDING_KINDS:
                    return "write_late", "drop"  # no new wait frame for it this tick; S10's stop -> its DISARM
                return "write_late", "wait"
            return None, None

        def gate(frame, write) -> bool:
            gate.keep = False
            if frame.kind == "DISARM":
                write()
                return True
            continuing = frame is getattr(self.transport, "partial_frame", None)
            self._gate_at, self._judged_ns = (None, None), None
            if not motion or alarm:
                verdict = ("device_block" if not motion else "after_alarm", "drop")
                ok = False
            else:
                verdict = None

                def check() -> bool:
                    nonlocal verdict
                    verdict = judge(frame)
                    return verdict[0] is None

                # 6a (design 6 ②): the write carries the clock value the judgement compared (keyword only)
                ok = send_boundary(self.channels, self.serial, lambda: write(judged_ns=self._judged_ns), check=check)
                if not ok and (verdict is None or verdict[0] is None or self.channels.serial() != self.serial):
                    # a STOP, DISARM or (re)connect since the batch (round 4 P2-3), also one taken during the check --
                    # whatever else the check found (round 7 P2-1)
                    verdict = ("emergency_event", "alarm")
            ns0_, ns_ = getattr(self, "_gate_at", (None, None))
            if not ok and verdict is not None and verdict[1] == "alarm":
                self._cause = (int(ns_ if ns_ is not None else self.clock()), str(verdict[0]))  # round 5
            self._emit({"t": "gate", "seq": int(frame.seq) & 0xFFFF, "kind": frame.kind, "continuing": continuing,
                        "ns0": ns0_, "ns": ns_, "judged_ns": self._judged_ns,
                        "verdict": "pass" if ok else f"{verdict[0]}:{verdict[1]}"})  # 6a-2b (design 10.7)
            if not ok:
                refused.append((frame.kind, frame.seq))
                why, cls = verdict
                if continuing:
                    alarm.append("link_stuck")  # a half frame not finished: the transport abandons it (link reset)
                    rec_c = getattr(self, "_rec", None)
                    if rec_c is not None:
                        rec_c.notes.append(f"continuation_refused:{why}")  # 6a-2b (design 10.3)
                elif cls == "keep":
                    refused.pop()  # 6a: an ARM or the host's zero stays queued (0 bytes out), judged again
                    gate.keep = True  # the transport leaves the queue as it is (no drop of the zero)
                elif cls == "cancel":
                    refused.pop()
                    self._cancel_arm = why  # given up after the transport sent it back (6a design 2)
                elif cls == "alarm":
                    alarm.append(why)
                elif cls == "wait":
                    waits.append(why)
            return ok

        if self.real_layer is not None and not self.closing and permit.anchor_disarm(now_ns):
            pass  # an anchor DISARM queued (design 1.1): written with this send
        self._cancel_arm = None
        for kind, seq, written_ns in self.transport.transmit(self.host, now_ns, gate):
            if kind == "DRIVE":
                self._written_now.append(int(seq) & 0xFFFF)  # the record's DRIVEs (a DISARM is not one)
            permit.written(kind, seq, written_ns)
            if self.closing and kind == "DISARM" and self.host.state is not None:
                self.closing_disarms.append((self.host.state.boot_id, int(seq) & 0xFFFF, int(written_ns)))
            self._emit({"t": "frame_written", "kind": kind, "seq": int(seq) & 0xFFFF, "written_ns": int(written_ns),
                        # a DISARM: the permit's ask it answers (round 6 P2-2)
                        "asked_ns": getattr(permit, "_disarm_asked_ns", None) if kind == "DISARM" else None,
                        "judged_ns": getattr(self.transport, "judged", {}).get(int(seq) & 0xFFFF),
                        "first_byte_ns": getattr(self.transport, "first_byte_ns", {}).get(int(seq) & 0xFFFF),
                        "last_write_ns": getattr(self.transport, "last_write_ns", {}).get(int(seq) & 0xFFFF),
                        "data": (getattr(self.transport, "written_data", {}).get(int(seq) & 0xFFFF) or b"").hex()})
            if self.delay_bound is not None:
                # the round trip's write is the first byte's (6a-2a impl round 1 P1-3): a write that returned late does
                # not shrink the bound of a STATE measured after the bytes went out
                first_ = getattr(self.transport, "first_byte_ns", {}).get(int(seq) & 0xFFFF, written_ns)
                self.delay_bound.written(seq, first_)
                pair = permit.session_pair()  # after the permit took the write: the second DISARM's seq is known
                if pair is not None:
                    self.delay_bound.expect_step(*pair)
            if kind == "DRIVE":
                self.book.written(seq, written_ns)
                fb_ = next((f for f in self.book.frames if f.seq == (int(seq) & 0xFFFF)), None)
                if fb_ is not None and fb_.kind == "drive" and float(fb_.speed_mps) != 0.0:
                    self._moved_for = self.core.commands.mission_id  # 5c-2 (6, 6): the start check ends
                booked = next((f for f in self.book.frames if f.seq == (int(seq) & 0xFFFF)), None)
                if booked is not None and booked.kind == "drive":
                    self.waiting = None  # a normal DRIVE out: the wait is over
                elif booked is not None and booked.kind in HOLDING_KINDS:
                    self._axis_start(booked)  # 6a: the axis watch over the holding span (design 2)
                steer = self._steer_of.get(int(seq) & 0xFFFF)
                if steer is not None:
                    self.steering_command = steer.copy()  # the next slew starts from what was written (P2-5)
        if self._cancel_arm is not None:
            self.host.cancel_arm(now_ns, self._cancel_arm)  # the permit latches arm_failed / revoked on its tick
            self._cancel_arm = None
        lost_ = getattr(self.transport, "lost", None)
        if lost_ is not None and not self._lost_seen:
            # 6a-2a impl round 2 P2-1: the frames finished before the loss are booked above; the loss itself is ①
            # in this tick (the runner ends the run on it)
            self._lost_seen = True
            rec_ = getattr(self, "_rec", None)
            if rec_ is not None:
                rec_.branch, rec_.reason = "emergency", str(lost_.get("reason", "device_lost"))
            self._in_transmit = True
            try:
                self._emergency(now_ns, str(lost_.get("reason", "device_lost")))
            finally:
                self._in_transmit = False
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
