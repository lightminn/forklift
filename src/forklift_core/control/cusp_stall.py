"""The cusp position stall: a leg standing at its cusp end outside the cusp tolerance.

Plan docs/plans/2026-10-04-lidar-obstacle-map.md, "cusp 위치 정체 복구 설계 7판". The tracker neither
accepts such a cusp (the position is outside the cusp tolerance) nor fails it, and its 1.5 x remaining
speed limit holds the truck there; nothing in the runner recovered it in transport or on the return.
This module holds the detection (K2), the standing decision (K3) and the hand-off's state. The checks
that need the runner's world -- whether the obstacle side owns the stop, the prefix's sensor checks,
the leg's planner obstacles, the existing rules' budgets -- are passed in, so it holds only the state
machine and the decision.
"""

from __future__ import annotations

import math
import weakref
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, field, replace

import numpy as np

from forklift_core.control.path_tracking import (
    RearAxlePathTracker,
    TrackerConfig,
    TrackingCommand,
)
from forklift_core.control.rollout import PrefixResult, leg_prefix_rollout

PHASES = ("observe", "transport", "return_home")


@dataclass(frozen=True)
class StallConfig:
    # Progress gained over stall_ticks, on one leg of one tracker, under stall_progress_m is a stall.
    stall_ticks: int = 120
    stall_progress_m: float = 1e-3
    prefix_m: float = 3.0  # the next leg's first part a release must converge on
    # The prefix's end against the reference run, at equal progress.
    match_m: float = 0.010
    match_rad: float = 0.010
    # For the stop to be confirmed, and then for a scan that qualifies.
    wait_s: float = 5.0
    far_m: float = 0.03  # past the acceptance bounds by this much: decided at once
    far_rad: float = 0.03
    # An occupied prefix this many scans running goes to the obstacle side.
    occupied_scans: int = 3
    step_m: float = 0.02  # the prefix trajectory's sample spacing for the checks


DEFAULT_STALL = StallConfig()


def cusp_tolerances(config: TrackerConfig) -> tuple[float, float]:
    """The position and heading tolerances the tracker applies at a cusp."""
    position = (
        config.position_tolerance_m
        if config.cusp_position_tolerance_m is None
        else config.cusp_position_tolerance_m
    )
    heading = (
        config.yaw_tolerance_rad
        if config.cusp_yaw_tolerance_rad is None
        else config.cusp_yaw_tolerance_rad
    )
    return float(position), float(heading)


def pose_bounds(
    config: TrackerConfig,
    position_error_m: float,
    yaw_error_rad: float,
    stall: StallConfig = DEFAULT_STALL,
    bounds: tuple[float, float] | None = None,
) -> tuple[bool, bool]:
    """(within, far): within 2 x the cusp position tolerance and the cusp heading tolerance (the
    existing cusp heading limit, not loosened); far when past them by far_m / far_rad. bounds =
    (position_m, heading_rad) replaces both limits for the cusp being judged only (the cusp heading
    hold of the EPAL 6 fork spacing plan, fix A: 30 mm and 0.08 rad)."""
    if bounds is None:
        position, heading = cusp_tolerances(config)
        position = 2.0 * position
    else:
        position, heading = (float(v) for v in bounds)
    within = position_error_m <= position and abs(yaw_error_rad) <= heading
    far = (
        position_error_m > position + stall.far_m
        or abs(yaw_error_rad) > heading + stall.far_rad
    )
    return bool(within), bool(far)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def prefix_converged(
    run: PrefixResult,
    reference: PrefixResult,
    end_recovered: Callable[[bool], bool],
    stall: StallConfig = DEFAULT_STALL,
) -> bool:
    """Whether the next leg's prefix from the measured pose converges: it went prefix_m and matches the
    reference run (from the stalled leg end's path pose) within match_m / match_rad at equal progress;
    or the leg is shorter and the tracker itself accepted its end; or that end is an endpoint_heading
    failure, or a stall at the final goal (cusp design amendment), an existing rule recovers within its
    budget (end_recovered(the end is the goal))."""
    if run.status == "prefix":
        return (
            reference.status == "prefix"
            and abs(run.lateral_m - reference.lateral_m) <= stall.match_m
            and abs(_wrap(run.heading_rad - reference.heading_rad)) <= stall.match_rad
        )
    if run.status in ("leg_end", "arrived"):
        return True
    if run.status == "failed" and run.failure == "endpoint_heading":
        return bool(end_recovered(run.leg_end_is_goal))
    if run.status == "goal_stall":
        return bool(end_recovered(True))
    return False


def thin(poses, step_m: float) -> np.ndarray:
    """Poses at least step_m of path apart, the first and the last kept."""
    poses = np.asarray(poses, dtype=float)
    if len(poses) < 3:
        return poses
    travelled = np.concatenate(
        ([0.0], np.cumsum(np.hypot(*np.diff(poses[:, :2], axis=0).T)))
    )
    keep, next_m = [0], step_m
    for i in range(1, len(poses) - 1):
        if travelled[i] >= next_m:
            keep.append(i)
            next_m = travelled[i] + step_m
    keep.append(len(poses) - 1)
    return poses[keep]


@dataclass
class Judgement:
    """One standing scan's decision. pose_ok: the bounds, the prefix convergence and the planner sweep
    (all bound to the pose); sensor_ok: the prefix's sensor checks; far: past the bounds by far."""

    pose_ok: bool
    sensor_ok: bool
    far: bool
    prefix_occupied: bool = False
    next_direction: int = 1
    trajectory: np.ndarray | None = None
    record: dict = field(default_factory=dict)


def judge(
    tracker: RearAxlePathTracker,
    rear,
    raw: TrackingCommand,
    cache: dict,
    *,
    end_recovered: Callable[[bool], bool],
    sweep_free: Callable[[np.ndarray], bool],
    sensors: Callable[[np.ndarray, int], tuple[bool, bool, dict | None]] | None,
    stall: StallConfig = DEFAULT_STALL,
    bounds: tuple[float, float] | None = None,
) -> Judgement:
    """K3 for the control pose rear now. The next leg's prefix is rolled from rear (the tracker on the
    whole path, the stalled leg passed by its own switch) and the reference from the stalled leg end's
    path pose (kept in cache). sweep_free(trajectory) checks the leg's planner obstacles; sensors
    (trajectory, direction) -> (sensor_ok, prefix_occupied, record), None without an obstacle layer.
    bounds overrides the current cusp's pose limits only (pose_bounds); the rollouts keep the
    tracker's config."""
    within, far = pose_bounds(
        tracker.config, raw.position_error_m, raw.yaw_error_rad, stall, bounds
    )
    record = {
        "rear": [float(v) for v in rear],
        "position_error_m": float(raw.position_error_m),
        "yaw_error_rad": float(raw.yaw_error_rad),
        "bounds_ok": within,
        "far": far,
    }
    if not within:
        return Judgement(False, False, far, record=record)
    poses, directions, curvatures = tracker.path_arrays()
    next_leg = tracker.leg_index + 1
    next_direction = (
        int(directions[min(tracker.leg_end_index + 1, len(directions) - 1)]) or 1
    )
    if cache.get("reference") is None:
        cache["reference"] = leg_prefix_rollout(
            poses,
            directions,
            curvatures,
            tracker.config,
            poses[tracker.leg_end_index],
            start_leg=next_leg,
            prefix_m=stall.prefix_m,
        )
    reference = cache["reference"]
    run = leg_prefix_rollout(
        poses,
        directions,
        curvatures,
        tracker.config,
        np.asarray(rear, dtype=float),
        start_leg=next_leg,
        prefix_m=stall.prefix_m,
    )
    converged = prefix_converged(run, reference, end_recovered, stall)
    trajectory = thin(run.trajectory, stall.step_m)
    swept = bool(sweep_free(trajectory))
    record.update(
        prefix={
            "status": run.status,
            "progress_m": run.progress_m,
            "lateral_m": run.lateral_m,
            "heading_rad": run.heading_rad,
            "failure": run.failure,
            "leg_end_is_goal": run.leg_end_is_goal,
            "d_lateral_m": run.lateral_m - reference.lateral_m,
            "d_heading_rad": _wrap(run.heading_rad - reference.heading_rad),
        },
        reference={
            "status": reference.status,
            "lateral_m": reference.lateral_m,
            "heading_rad": reference.heading_rad,
        },
        converged=converged,
        sweep_ok=swept,
    )
    sensor_ok, occupied = True, False
    if sensors is not None and len(trajectory) >= 2:
        sensor_ok, occupied, sensor_record = sensors(trajectory, next_direction)
        record["sensor"] = sensor_record
    return Judgement(
        converged and swept,
        bool(sensor_ok),
        far,
        bool(occupied),
        next_direction,
        trajectory,
        record,
    )


@dataclass
class TickResult:
    tracking: TrackingCommand
    hold: bool = False  # zero command, steering held
    end: str | None = None  # the run ends with this reason
    # A hand-off forcing the phase's obstacle replan (once per stall key).
    force_replan: bool = False


@dataclass
class Latch:
    key: tuple  # (phase, tracker serial, leg)
    tracker: RearAxlePathTracker
    record: dict
    since_s: float  # the brake's start (latched, stop lost, ownership returned)
    # brake -> judge -> released / observe_rule / handed (-> brake) / ended
    mode: str = "brake"
    scan: object = None  # the scan last judged (None: judge on this tick)
    occupied_scans: int = 0
    counted_scan: object = None  # the scan the occupied count last counted
    stopped_s: float | None = None  # the first confirmed stop: the wait runs from it
    last: Judgement | None = None  # the last scan's decision
    hand_reason: str | None = None
    hand_direction: int = 1
    prefix_trajectory: np.ndarray | None = None
    prefix_occupied_now: bool = False
    cache: dict = field(default_factory=dict)


class CuspStallMonitor:
    """K2/K3 across ticks. records receives one dict per stall (K4)."""

    def __init__(self, records: list, stall: StallConfig = DEFAULT_STALL):
        self.records = records
        self.stall = stall
        self.latch: Latch | None = None
        self._watch: tuple | None = None  # (key, tracker, progress per tick)
        # One serial per tracker object for this monitor's life (an id() may be reused), and the
        # stall keys whose hand-off has already forced a replan.
        self._serials: weakref.WeakKeyDictionary = weakref.WeakKeyDictionary()
        self._forced: set = set()
        self._count = 0


    def shift_time(self, length_s: float) -> None:
        """A mission pause of ``length_s`` ended (M-a3 E4): the latch's brake start and first stop move by it, so its
        waits -- measured as time since them -- do not count the pause."""
        latch = self.latch
        if latch is None or length_s == 0.0:
            return
        latch.since_s += length_s
        if latch.stopped_s is not None:
            latch.stopped_s += length_s

    def serial(self, tracker: RearAxlePathTracker) -> int:
        """The tracker object's serial for this monitor (1 for the first it saw)."""
        found = self._serials.get(tracker)
        if found is None:
            self._count += 1
            found = self._serials[tracker] = self._count
        return found

    def handed(self, phase: str) -> Latch | None:
        """The stall handed to the obstacle side in this phase, if any."""
        latch = self.latch
        if latch is not None and latch.mode == "handed" and latch.key[0] == phase:
            return latch
        return None

    def holds_observe(self) -> bool:
        """Whether an observe stall is held here (its brake, sensor wait or hand-off): the observe stop
        rule waits until this hands the stall to it."""
        latch = self.latch
        return (
            latch is not None
            and latch.key[0] == "observe"
            and latch.mode in ("brake", "judge", "handed")
        )

    def _close(self, now_s: float, why: str) -> None:
        if self.latch is not None:
            self.latch.record["events"].append(
                {"time_s": now_s, "event": "closed", "why": why}
            )
            self.latch.record.setdefault("closed", why)
            self.latch = None

    def _hand(
        self, now_s: float, latch: Latch, reason: str, direction: int, trajectory=None
    ) -> bool:
        """The obstacle side owns this stop; returns whether to force the obstacle replan now -- once per
        stall key, however often it is handed back and forth or latched again."""
        force = latch.key not in self._forced
        self._forced.add(latch.key)
        latch.record["disposition"] = "handed_to_obstacle"
        latch.record["events"].append(
            {
                "time_s": now_s,
                "event": "handed_to_obstacle",
                "reason": reason,
                "forced": force,
            }
        )
        latch.mode, latch.hand_reason, latch.hand_direction = (
            "handed",
            reason,
            int(direction) or 1,
        )
        latch.prefix_trajectory = trajectory
        latch.prefix_occupied_now = reason == "prefix_occupied"
        latch.scan = None
        return force

    def _decide(
        self,
        now_s: float,
        phase: str,
        latch: Latch,
        judged: Judgement,
        tracking,
        zeroed,
    ) -> TickResult | None:
        """What a decision without a release does: the pose-bound outcome at once when far or after the
        wait, an occupied prefix after occupied_scans, the sensor hand-off after the wait; None: wait."""
        stall, record, tracker = self.stall, latch.record, latch.tracker
        waited = now_s - latch.stopped_s >= stall.wait_s
        if judged.far or (not judged.pose_ok and waited):
            if phase == "observe":
                record["disposition"] = "observe_rule"
                record["events"].append(
                    {"time_s": now_s, "event": "observe_rule", "far": judged.far}
                )
                latch.mode = "observe_rule"
                return TickResult(zeroed, hold=True)
            record["disposition"] = "ended"
            return TickResult(zeroed, hold=True, end="cusp_position_stall")
        if latch.occupied_scans >= stall.occupied_scans:
            force = self._hand(
                now_s,
                latch,
                "prefix_occupied",
                judged.next_direction,
                judged.trajectory,
            )
            return TickResult(tracking, force_replan=force)
        if waited:
            force = self._hand(now_s, latch, "sensor", tracker.leg_ahead()[1])
            return TickResult(tracking, force_replan=force)
        return None

    def tick(
        self,
        now_s: float,
        phase: str,
        tracker: RearAxlePathTracker,
        raw: TrackingCommand,
        tracking: TrackingCommand,
        *,
        rear,
        excluded: str | None,
        retry_stand: bool,
        standing: Callable[[], bool],
        stopped: Callable[[], bool],
        alive: Callable[[], bool],
        ownership: Callable[[float], tuple[bool, dict | None]],
        judge_now: Callable[[Latch], Judgement],
        scan_id: Callable[[], object],
        replan_pending: Callable[[], bool],
        prefix_occupied: Callable[[Latch], bool],
        release_update: Callable[[], TrackingCommand],
        watch_since_s: float | None = None,
    ) -> TickResult:
        """One tick. raw is the tracker's own result this tick, tracking what the rules before made of
        it. standing: the stop detector; stopped: standing with a zero command applied; alive: every
        obstacle sensor within its timeout (checked every tick, a silent sensor sends no scan);
        ownership(command) -> (whether the obstacle side leaves the stop alone, record); judge_now(latch)
        -> Judgement for this scan; replan_pending: the hand-off's forced replan or a replan wait still
        on; prefix_occupied(latch): the occupied prefix re-walked; release_update: the tracker updated
        again after a release."""
        stall = self.stall
        key = (phase, self.serial(tracker), tracker.leg_index)
        if (
            self._watch is None
            or self._watch[0] != key
            or self._watch[1] is not tracker
        ):
            self._watch = (key, tracker, deque(maxlen=stall.stall_ticks + 1))
        progress = self._watch[2]
        progress.append(float(raw.progress_m))
        latch = self.latch
        if latch is not None:
            # Bound to its key: a leg switch, a new tracker or phase, a status other than tracking or an
            # exclusion ends it -- a failure is never covered.
            why = None
            if latch.key != key or latch.tracker is not tracker:
                why = "key_changed"
            elif raw.status != "tracking":
                why = f"status:{raw.status}"
            elif excluded is not None or retry_stand:
                why = f"excluded:{excluded or 'retry_stand'}"
            if why is not None:
                self._close(now_s, why)
                latch = None
        if phase not in PHASES:
            return TickResult(tracking)
        zeroed = replace(tracking, speed_mps=0.0)
        if latch is None:
            position_tolerance, _ = cusp_tolerances(tracker.config)
            if not (
                excluded is None
                and not retry_stand
                and raw.status == "tracking"
                and raw.at_cusp
                and tracker.leg_remaining_m() <= tracker.leg_end_window_m()
                and len(progress) > stall.stall_ticks
                and progress[-1] - progress[0] < stall.stall_progress_m
                and raw.position_error_m > position_tolerance
            ):
                return TickResult(tracking)
            owned, owned_record = ownership(raw.speed_mps)
            if not owned and not (standing() and raw.speed_mps == 0.0):
                # The obstacle block runs on this command and owns the stop.
                return TickResult(tracking)
            record = {
                "phase": phase,
                "tracker_serial": key[1],
                "leg_index": tracker.leg_index,
                "time_s": now_s,
                "rear": [float(v) for v in rear],
                "leg_remaining_m": tracker.leg_remaining_m(),
                "progress_gain_m": float(progress[-1] - progress[0]),
                "position_error_m": float(raw.position_error_m),
                "yaw_error_rad": float(raw.yaw_error_rad),
                "command_mps": float(raw.speed_mps),
                "ownership": [owned_record] if owned_record is not None else [],
                "scans": [],
                "events": [],
                "disposition": None,
                # The no-progress watch's base: the time latched counts in its 60 s.
                "watch_since_s": watch_since_s,
            }
            self.records.append(record)
            latch = self.latch = Latch(key, tracker, record, now_s)
            tracker.restart_speed_slew(0.0)
            if not owned:
                # Standing on an exact zero the obstacle block does not run: nobody else would own it.
                force = self._hand(now_s, latch, "ownership", tracker.leg_ahead()[1])
                return TickResult(tracking, force_replan=force)
        record = latch.record
        if latch.mode == "handed":
            scan = scan_id()
            if scan == latch.scan:
                return TickResult(tracking)
            latch.scan = scan
            if (
                latch.hand_reason == "prefix_occupied"
                and latch.prefix_trajectory is not None
            ):
                latch.prefix_occupied_now = bool(prefix_occupied(latch))
            # Returned once the forced replan has run, no replan wait is left and the checks pass again.
            if replan_pending():
                return TickResult(tracking)
            owned, owned_record = ownership(raw.speed_mps)
            if not owned or latch.prefix_occupied_now:
                return TickResult(tracking)
            record["events"].append(
                {
                    "time_s": now_s,
                    "event": "ownership_returned",
                    "ownership": owned_record,
                }
            )
            record["disposition"] = None
            latch.mode, latch.since_s, latch.scan, latch.occupied_scans = (
                "brake",
                now_s,
                None,
                0,
            )
            latch.stopped_s, latch.last, latch.prefix_occupied_now = None, None, False
            tracker.restart_speed_slew(0.0)
        if latch.mode == "observe_rule":
            # The observe stop rule runs on the zero command.
            return TickResult(zeroed, hold=True)
        if latch.mode == "judge" and not stopped():
            # Standing is lost: brake and confirm the stop again; the wait still runs from the first stop.
            record["events"].append({"time_s": now_s, "event": "stop_lost"})
            latch.mode, latch.since_s = "brake", now_s
        if latch.mode == "brake":
            if not stopped():
                if now_s - latch.since_s >= stall.wait_s:
                    record["disposition"] = "ended"
                    return TickResult(
                        zeroed, hold=True, end="cusp_position_stall_unconfirmed"
                    )
                return TickResult(zeroed, hold=True)
            record["events"].append({"time_s": now_s, "event": "stop_confirmed"})
            latch.mode, latch.scan = "judge", None
            if latch.stopped_s is None:
                latch.stopped_s = now_s
        # judge: standing, every scan from that scan's control pose -- the obstacle side's ownership
        # first (the confirming tick included); a silent sensor, which sends no scan, every tick.
        if not alive():
            record["events"].append({"time_s": now_s, "event": "sensor_silent"})
            force = self._hand(now_s, latch, "silent", tracker.leg_ahead()[1])
            return TickResult(tracking, force_replan=force)
        scan = scan_id()
        if scan == latch.scan:
            # No new scan: the wait still ends on time, decided on the last scan's judgement.
            if latch.last is not None:
                decided = self._decide(
                    now_s, phase, latch, latch.last, tracking, zeroed
                )
                if decided is not None:
                    return decided
            return TickResult(zeroed, hold=True)
        latch.scan = scan
        owned, owned_record = ownership(raw.speed_mps)
        if not owned:
            record["ownership"].append(owned_record)
            force = self._hand(now_s, latch, "ownership", tracker.leg_ahead()[1])
            return TickResult(tracking, force_replan=force)
        judged = latch.last = judge_now(latch)
        record["scans"].append(
            {"time_s": now_s, **judged.record, "ownership": owned_record}
        )
        if scan != latch.counted_scan:
            # Counted per scan: a stop confirmed again re-judges the same scan from the pose now, but
            # one occupied scan is not three (Codex implementation review 2 P1).
            latch.counted_scan = scan
            latch.occupied_scans = (
                latch.occupied_scans + 1 if judged.prefix_occupied else 0
            )
        if judged.pose_ok and judged.sensor_ok:
            if tracker.release_cusp(stopped=True):
                record["disposition"] = "released"
                record["events"].append({"time_s": now_s, "event": "released"})
                self.latch = None
                # The next leg's command this very tick, so the rules after this see it.
                return TickResult(release_update())
            record["events"].append({"time_s": now_s, "event": "release_refused"})
        decided = self._decide(now_s, phase, latch, judged, tracking, zeroed)
        return decided if decided is not None else TickResult(zeroed, hold=True)


__all__ = [
    "PHASES",
    "DEFAULT_STALL",
    "CuspStallMonitor",
    "Judgement",
    "Latch",
    "StallConfig",
    "TickResult",
    "cusp_tolerances",
    "judge",
    "pose_bounds",
    "prefix_converged",
    "thin",
]
