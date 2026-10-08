"""Plan D8c near-field pocket tracking, shared by the Isaac runner and the CPU replay.

Moved here from sim/isaac/pocket_tracking_handoff.py (it uses core types only):
RoofHandoff qualifies the known-model roof tracking against directly seen pockets and
never extrapolates a missing observation -- after qualification the roof stream is
authoritative and its loss must stop motion. The observation speed budget (D8e) is a pure
function of the travel since the last valid capture.
"""

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.perception.pocket_observation import PocketObservation


@dataclass(frozen=True)
class HandoffSelection:
    observation: PocketObservation
    mode: str
    confirmed_frames: int
    agreement_position_m: float | None
    agreement_yaw_rad: float | None
    agreement_size_m: float | None


class RoofHandoff:
    """Three consecutive, paired observations establish the model continuation."""

    def __init__(
        self,
        *,
        start_front_x_m,
        confirmation_frames=3,
        position_tolerance_m=0.025,
        yaw_tolerance_rad=0.03,
        size_tolerance_m=0.02,
    ):
        if type(confirmation_frames) is not int or confirmation_frames < 1:
            raise ValueError("confirmation_frames must be a positive integer")
        for value in (
            start_front_x_m,
            position_tolerance_m,
            yaw_tolerance_rad,
            size_tolerance_m,
        ):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError("handoff limits must be finite and positive")
        self.start_front_x_m = start_front_x_m
        self.confirmation_frames = confirmation_frames
        self.position_tolerance_m = position_tolerance_m
        self.yaw_tolerance_rad = yaw_tolerance_rad
        self.size_tolerance_m = size_tolerance_m
        self.confirmed = 0
        self.qualified = False
        self.last_stamp = None

    def reset(self) -> None:
        """Break the run of agreeing frames (plan D8c: a frame outside the gate, an invalid
        front or a rejected association is not a paired evaluation -- calling select only
        inside the gate let three non-consecutive frames hand off)."""
        if not self.qualified:
            self.confirmed = 0

    def select(self, front, roof):
        position_error = yaw_error = size_error = None
        if not self.qualified:
            paired = (
                front.status == roof.status == "valid"
                and front.source_provenance != "synthetic_ground_truth"
                and roof.source_provenance != "synthetic_ground_truth"
                and front.frame_id == roof.frame_id == "base_link"
                and front.clock_domain == roof.clock_domain
                and front.stamp_ns == roof.stamp_ns
                and (self.last_stamp is None or front.stamp_ns > self.last_stamp)
            )
            if paired:
                midpoint = (np.array(front.left.center_m) + front.right.center_m) / 2
                pairs = ((front.left, roof.left), (front.right, roof.right))
                position_error = max(
                    float(np.linalg.norm(np.array(a.center_m) - b.center_m))
                    for a, b in pairs
                )
                size_error = max(
                    max(abs(a.width_m - b.width_m), abs(a.height_m - b.height_m))
                    for a, b in pairs
                )
                delta = front.insertion_yaw_rad - roof.insertion_yaw_rad
                yaw_error = abs(math.atan2(math.sin(delta), math.cos(delta)))
                paired = (
                    midpoint[0] <= self.start_front_x_m
                    and position_error <= self.position_tolerance_m
                    and yaw_error <= self.yaw_tolerance_rad
                    and size_error <= self.size_tolerance_m
                )
            self.confirmed = self.confirmed + 1 if paired else 0
            self.last_stamp = front.stamp_ns
            self.qualified = self.confirmed >= self.confirmation_frames
        return HandoffSelection(
            roof if self.qualified else front,
            "roof_model" if self.qualified else "front_pockets",
            self.confirmed,
            position_error,
            yaw_error,
            size_error,
        )


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def held_pockets(observation: PocketObservation, pose) -> dict:
    """The observation's pockets placed in the held frame by pose = held_from_base_link
    (x, y, yaw): centres (z unchanged), sizes and the insertion yaw."""
    c, s = math.cos(pose[2]), math.sin(pose[2])

    def place(p):
        return (pose[0] + c * p[0] - s * p[1], pose[1] + s * p[0] + c * p[1], p[2])

    return {
        "left": place(observation.left.center_m), "right": place(observation.right.center_m),
        "left_size": (observation.left.width_m, observation.left.height_m),
        "right_size": (observation.right.width_m, observation.right.height_m),
        "yaw": _wrap(observation.insertion_yaw_rad + pose[2]),
    }


def _base_prior(held: dict, pose) -> tuple[tuple[float, float], float]:
    """The held-frame midpoint and yaw back in base_link at pose (an XY prior for track_roof)."""
    mx = (held["left"][0] + held["right"][0]) / 2 - pose[0]
    my = (held["left"][1] + held["right"][1]) / 2 - pose[1]
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (c * mx + s * my, -s * mx + c * my), _wrap(held["yaw"] - pose[2])


@dataclass(frozen=True)
class NearFieldConfig:
    """Plan D8c/D8e near-field tracking limits. align_latency_s is L (pixels older than the
    frame stamp), max_latency_s L_max; handoff_start_front_x_m the base_link front-midpoint
    gate g from the CPU replay."""

    align_latency_s: float
    max_latency_s: float
    handoff_start_front_x_m: float
    association_position_m: float = 0.030
    association_yaw_rad: float = 0.030
    association_size_m: float = 0.020
    mismatch_frames: int = 3
    loss_s: float = 0.3
    reacquire_s: float = 3.0
    handoff_frames: int = 3

    def __post_init__(self) -> None:
        for name in ("align_latency_s", "max_latency_s", "handoff_start_front_x_m", "association_position_m",
                     "association_yaw_rad", "association_size_m", "loss_s", "reacquire_s"):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.max_latency_s < self.align_latency_s:
            raise ValueError("max_latency_s must not be below align_latency_s")
        for name in ("mismatch_frames", "handoff_frames"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class FrameResult:
    stamp_s: float
    mode: str  # "front" or "roof" after the frame
    accepted: bool  # a valid observation passed the association and became the estimate
    rejected: bool  # a valid observation failed the association
    reason: str | None
    consecutive_rejects: int
    mismatch: bool
    handoff_confirmed: int
    front_status: str | None
    roof_status: str | None


@dataclass(frozen=True)
class TrackStatus:
    age_s: float  # now - (stamp - L_max) of the last accepted observation
    lost: bool  # older than loss_s: the runner stops
    failed: bool  # lost for more than reacquire_s: near_field_lost
    lost_since_s: float | None


class NearFieldTracker:
    """Plan D8c: pocket tracking from the near capture to the end of insertion.

    The anchor is the near capture's observation in the held frame; every later front or
    roof observation is placed in the same frame with the caller's held_from_base_link pose
    at its stamp - L and accepted only within the anchor's tolerances (each pocket centre,
    insertion yaw, each pocket size) -- comparing with the previous estimate would let it
    walk 30 mm at a time. An accepted observation becomes the estimate. Handoff to the roof
    tracker is judged on every frame (RoofHandoff), reset by any frame that is not a paired,
    associated evaluation inside the gate. The tracker never changes a path: the runner reads
    the last accepted observation and its pose (D8d).
    """

    def __init__(self, config: NearFieldConfig, initial: PocketObservation, initial_pose, initial_stamp_s: float,
                 *, front_fn, roof_fn):
        if initial.status != "valid":
            raise ValueError("the near capture's observation must be valid")
        self.config = config
        self._front_fn = front_fn  # scene -> PocketObservation (detect_pockets with the runner's settings)
        self._roof_fn = roof_fn  # (scene, prior_xy, prior_yaw) -> PocketObservation (track_roof)
        self.anchor = held_pockets(initial, tuple(initial_pose))
        self.latest_observation = initial
        self.latest_pose = tuple(float(v) for v in initial_pose)
        self.latest_stamp_s = float(initial_stamp_s)
        self.latest_held = self.anchor
        self.mode = "front"
        self.handoff = RoofHandoff(start_front_x_m=config.handoff_start_front_x_m,
                                   confirmation_frames=config.handoff_frames)
        self.consecutive_rejects = 0
        self._lost_since = None

    def _associated(self, observation: PocketObservation, pose) -> tuple[bool, str | None, dict]:
        held = held_pockets(observation, pose)
        cfg, a = self.config, self.anchor
        for side in ("left", "right"):
            if math.dist(held[side], a[side]) > cfg.association_position_m:
                return False, f"{side}_position", held
            if max(abs(u - v) for u, v in zip(held[f"{side}_size"], a[f"{side}_size"])) > cfg.association_size_m:
                return False, f"{side}_size", held
        if abs(_wrap(held["yaw"] - a["yaw"])) > cfg.association_yaw_rad:
            return False, "yaw", held
        return True, None, held

    def on_frame(self, scene, stamp_s: float, pose) -> FrameResult:
        """One delivered frame: ``pose`` is held_from_base_link at stamp_s - L."""
        pose = tuple(float(v) for v in pose)
        accepted = rejected = False
        reason = front_status = roof_status = None
        chosen = None
        if self.mode == "front":
            front = self._front_fn(scene)
            front_status = front.status
            paired = False
            if front.status == "valid":
                ok, reason, held = self._associated(front, pose)
                if ok:
                    chosen, accepted = (front, held), True
                    mid_x = (front.left.center_m[0] + front.right.center_m[0]) / 2
                    if mid_x <= self.config.handoff_start_front_x_m:
                        prior_xy, prior_yaw = _base_prior(self.latest_held if chosen is None else held, pose)
                        roof = self._roof_fn(scene, prior_xy, prior_yaw)
                        roof_status = roof.status
                        if roof.status == "valid":
                            roof_ok, roof_reason, roof_held = self._associated(roof, pose)
                            if roof_ok:
                                paired = True
                                self.handoff.select(front, roof)
                                if self.handoff.qualified:
                                    self.mode = "roof"
                                    chosen = (roof, roof_held)
                            else:
                                reason = f"roof_{roof_reason}"
                else:
                    rejected = True
            if not paired:
                self.handoff.reset()
        else:
            prior_xy, prior_yaw = _base_prior(self.latest_held, pose)
            roof = self._roof_fn(scene, prior_xy, prior_yaw)
            roof_status = roof.status
            if roof.status == "valid":
                ok, reason, held = self._associated(roof, pose)
                if ok:
                    chosen, accepted = (roof, held), True
                else:
                    rejected = True
        if accepted:
            self.latest_observation, self.latest_held = chosen
            self.latest_pose, self.latest_stamp_s = pose, float(stamp_s)
            self.consecutive_rejects = 0
            self._lost_since = None
        elif rejected:
            self.consecutive_rejects += 1
        else:
            # An invalid frame breaks the run: the mismatch is three rejections in a row
            # (Codex D8a implementation review P3-3).
            self.consecutive_rejects = 0
        return FrameResult(float(stamp_s), self.mode, accepted, rejected, reason, self.consecutive_rejects,
                           self.consecutive_rejects >= self.config.mismatch_frames, self.handoff.confirmed,
                           front_status, roof_status)

    def status(self, now_s: float) -> TrackStatus:
        """The safety age is always now - (stamp - L_max), never reset by receipt."""
        age = float(now_s) - (self.latest_stamp_s - self.config.max_latency_s)
        lost = age > self.config.loss_s
        if lost and self._lost_since is None:
            self._lost_since = float(now_s)
        if not lost:
            self._lost_since = None
        failed = lost and float(now_s) - self._lost_since > self.config.reacquire_s
        return TrackStatus(age, lost, failed, self._lost_since)


def budget_speed(
    travel_since_capture_m: float,
    time_to_next_result_s: float,
    decel_mps2: float,
    stop_latency_s: float = 0.0,
    budget_m: float = 0.03,
) -> float:
    """Plan D8e: the largest speed v >= 0 with
    d + v * (tau + stop_latency) + v^2 / (2 decel) <= budget.

    d is the travel since the last valid result's conservative pixel time (stamp - L_max),
    tau the time from now until the next capture's result can arrive at the latest (next
    capture - now + L_max): driven until then, braked after the stop latency, the truck has
    not moved more than the budget since that pixel time. 0 when the budget is spent. The
    caller passes as stop latency the drive's reaction time plus the one control tick it
    takes to notice a result that did not come, and stops at once when its current speed
    exceeds the result: the previous tick kept the budget with that tick included, so
    braking now still stops inside it (Codex D8c module re-review P2).
    """
    for value, name in ((travel_since_capture_m, "travel"), (time_to_next_result_s, "time to next result"),
                        (decel_mps2, "deceleration"), (stop_latency_s, "stop latency"), (budget_m, "budget")):
        if isinstance(value, bool) or not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
    if travel_since_capture_m < 0 or time_to_next_result_s < 0 or stop_latency_s < 0:
        raise ValueError("travel, time and stop latency must not be negative")
    if decel_mps2 <= 0 or budget_m <= 0:
        raise ValueError("deceleration and budget must be positive")
    remaining = budget_m - travel_since_capture_m
    if remaining <= 0:
        return 0.0
    tau = time_to_next_result_s + stop_latency_s
    return decel_mps2 * (-tau + math.sqrt(tau * tau + 2.0 * remaining / decel_mps2))


__all__ = ["FrameResult", "HandoffSelection", "NearFieldConfig", "NearFieldTracker", "RoofHandoff",
           "TrackStatus", "budget_speed", "held_pockets"]
