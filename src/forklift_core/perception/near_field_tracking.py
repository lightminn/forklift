"""Plan D8c near-field pocket tracking, shared by the Isaac runner and the recorded replay.

Moved here from sim/isaac/pocket_tracking_handoff.py (it uses core types only):
RoofHandoff qualifies the known-model roof tracking against directly seen pockets and
never extrapolates a missing observation -- after qualification the roof stream is
authoritative and its loss must stop motion. The observation speed budget (D8e) is a pure
function of the travel since the last valid capture.

NearFieldTracker is version 3 (plan "D8c 3판 구현 설계"): its association tolerances come
from the D8b measurements (forklift_core.perception.near_field_bounds), not from fixed
30 mm / 0.03 rad gates; it is not wired into the runner until the recorded-frame replay
(⑥) has passed.
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


CADENCE_TOLERANCE_S = 1e-4  # a stamp interval off the render period breaks the runs


def _finite_time(value, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return value


def _version(token):
    """A held-frame version: a str or int token (not bool), compared by type and value."""
    if isinstance(token, bool) or not isinstance(token, (str, int)):
        raise ValueError("frame_version must be a str or int (an immutable token)")
    return token


def _finite_pose(pose) -> tuple:
    out = tuple(float(v) for v in pose)
    if len(out) != 3 or not all(math.isfinite(v) for v in out):
        raise ValueError("a pose must be three finite numbers (x, y, yaw)")
    return out


def held_pockets(observation: PocketObservation, pose) -> dict:
    """The observation's pockets placed in the held frame by pose = held_from_base_link
    (x, y, yaw): centres (z unchanged), sizes and the insertion yaw."""
    c, s = math.cos(pose[2]), math.sin(pose[2])

    def place(p):
        return (pose[0] + c * p[0] - s * p[1], pose[1] + s * p[0] + c * p[1], p[2])

    return {
        "left": place(observation.left.center_m),
        "right": place(observation.right.center_m),
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


class UnboundedAge(Exception):
    """An association needed the odometry error at an age past the measured table."""


@dataclass(frozen=True)
class NearFieldConfig:
    """Plan D8c 3판: the geometry the tracker needs besides the measured bounds.

    handoff_start_front_x_m is the base_link front-midpoint gate g (the fork tip x + 0.6 -
    0.03 m: after the last correction point); camera_xy_m the optical centre in base_link
    (the camera-face distance of an observation); rear_axle_x_m the rear axle centre x in
    base_link (the lever of the heading error); the pallet model's opening width and
    pocket centre spacing (front model consistency); pallet_term_m the allowed pallet drift
    between two observations (the static condition, plan D8c 3판)."""

    handoff_start_front_x_m: float
    camera_xy_m: tuple
    rear_axle_x_m: float
    opening_width_m: float
    pocket_spacing_m: float
    roof_sigma_max_m: float = 0.020
    pallet_term_m: float = 0.00005
    mismatch_frames: int = 3
    loss_s: float = 0.3
    reacquire_s: float = 3.0
    handoff_frames: int = 3

    def __post_init__(self) -> None:
        for name in (
            "handoff_start_front_x_m",
            "opening_width_m",
            "pocket_spacing_m",
            "roof_sigma_max_m",
            "pallet_term_m",
            "loss_s",
            "reacquire_s",
        ):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value < 0:
                raise ValueError(f"{name} must be finite and non-negative")
        xy = tuple(float(v) for v in self.camera_xy_m)
        if (
            len(xy) != 2
            or not all(math.isfinite(v) for v in xy)
            or not math.isfinite(self.rear_axle_x_m)
        ):
            raise ValueError("camera_xy_m and rear_axle_x_m must be finite")
        object.__setattr__(self, "camera_xy_m", xy)
        for name in ("mismatch_frames", "handoff_frames"):
            if type(getattr(self, name)) is not int or getattr(self, name) < 1:
                raise ValueError(f"{name} must be a positive integer")


@dataclass(frozen=True)
class Accepted:
    """An observation the tracker accepted: where it was placed and what bounds it."""

    observation: PocketObservation
    source: str  # "near_capture", "front" or "roof"
    pose: tuple  # held_from_base_link at its aligned time
    aligned_s: float  # stamp - L: the control time its pixels are placed at
    stamp_s: float
    held: dict  # held_pockets(observation, pose)
    bound: dict  # component -> measured bound (lateral_m, along_m, ...)
    frame_version: object  # the held frame it was placed in


@dataclass(frozen=True)
class FrameResult:
    stamp_s: float
    mode: str  # "front" or "roof" after the frame
    accepted: bool  # a valid observation passed every check and became the estimate
    rejected: bool  # a valid, bounded observation failed a check
    reason: str | None
    consecutive_rejects: int
    mismatch: bool
    unbounded: (
        bool  # an age past the odometry table: the runner stops (near_field_unbounded)
    )
    handoff_confirmed: int
    front_status: str | None  # the detector's status
    roof_status: str | None
    failed: bool  # armed, and the reacquisition deadline passed before this result
    front_eval: str | None = None  # accepted / rejected / invalid (None: not evaluated)
    front_reason: str | None = None
    roof_eval: str | None = None
    roof_reason: str | None = None


@dataclass(frozen=True)
class TrackStatus:
    age_s: float  # now - (stamp - L_max) of the last accepted observation
    lost: bool  # older than loss_s: the runner stops
    failed: bool  # lost for more than reacquire_s: near_field_lost
    lost_since_s: float | None


def wall_erosion_m(
    bound: dict, odometry: tuple, depth_m: float, rho_m: float, pallet_term_m: float
) -> float:
    """Plan D8d b_t: the bound on a pocket wall point's position at depth depth_m behind
    the face, from an observation with these measured bounds carried by an odometry error
    (e, psi) at lever rho_m: the face wall point's error, the observed yaw's error over the
    depth, the carried odometry and the pallet term."""
    e, psi = odometry
    return (
        bound["wall_m"] + depth_m * bound["yaw_rad"] + e + rho_m * psi + pallet_term_m
    )


class NearFieldTracker:
    """Plan D8c 3판: pocket tracking from the near capture to the end of insertion.

    Observations are placed in the held frame with the caller's held_from_base_link pose
    at their aligned time (stamp - L). A new observation is accepted when it is valid,
    its camera-face distance has measured bounds (NearFieldBounds.observation), the front
    agrees with the pallet model, and it agrees -- per pocket in lateral and along, and in
    yaw -- with both the last accepted observation (step) and the near capture (anchor),
    within the sum of the two observations' measured bounds, the measured relative
    odometry error over the time between them (e + rho psi), a mixing term for the
    comparison axis and the pallet term. Three rejections in a row are a mismatch; an age
    past the odometry table is an unbounded event (the runner stops). Handoff to the roof
    is judged on every frame (RoofHandoff), reset by any frame that is not a paired,
    accepted evaluation inside the gate and by a skipped frame. The tracker never changes a
    path: the runner reads ``latest`` (D8d).
    """

    def __init__(
        self,
        config: NearFieldConfig,
        bounds,
        initial: PocketObservation,
        initial_pose,
        initial_aligned_s: float,
        *,
        front_fn,
        roof_fn,
        frame_version,
    ):
        if initial.status != "valid":
            raise ValueError("the near capture's observation must be valid")
        self.config = config
        self.bounds = bounds
        self._front_fn = (
            front_fn  # scene -> PocketObservation (detect_pockets, near-field settings)
        )
        self._roof_fn = (
            roof_fn  # (scene, prior_xy, prior_yaw) -> PocketObservation (track_roof)
        )
        pose = _finite_pose(initial_pose)
        aligned = _finite_time(initial_aligned_s, "initial_aligned_s")
        self.frame_version = _version(frame_version)
        self.anchor = Accepted(
            initial,
            "near_capture",
            pose,
            aligned,
            aligned + bounds.align_latency_s,
            held_pockets(initial, pose),
            dict(bounds.near_capture),
            frame_version,
        )
        self.latest = self.anchor
        self.mode = "front"
        self.handoff = RoofHandoff(
            start_front_x_m=config.handoff_start_front_x_m,
            confirmation_frames=config.handoff_frames,
        )
        self.consecutive_rejects = 0
        self._last_frame_stamp = None
        self._lost_since = None
        self._failed = False
        self._armed_at = (
            None  # the near-field section's entry: failures count from here
        )

    # Geometry ------------------------------------------------------------------------------
    def distance_m(self, observation: PocketObservation) -> float:
        """The camera-face distance: the optical centre to the observed face midpoint along
        the observed insertion axis (base_link)."""
        mid = (
            np.array(observation.left.center_m[:2]) + observation.right.center_m[:2]
        ) / 2
        axis = np.array(
            [
                math.cos(observation.insertion_yaw_rad),
                math.sin(observation.insertion_yaw_rad),
            ]
        )
        return float(np.dot(mid - self.config.camera_xy_m, axis))

    def _lever_m(self, centre) -> float:
        return math.hypot(centre[0] - self.config.rear_axle_x_m, centre[1])

    # Checks --------------------------------------------------------------------------------
    def _compare(
        self, observation, held, bound, ref: Accepted, aligned_s: float
    ) -> str | None:
        dt = aligned_s - ref.aligned_s
        if dt < 0:
            return "older_than_reference"
        odometry = self.bounds.odometry(dt)
        if odometry is None:
            raise UnboundedAge(dt)
        e, psi = odometry
        a = np.array([math.cos(ref.held["yaw"]), math.sin(ref.held["yaw"])])
        n = np.array([-a[1], a[0]])
        mixing = (
            ref.bound["yaw_rad"] + psi
        )  # the comparison axis against the truth axes
        for side in ("left", "right"):
            d = np.array(held[side][:2]) - ref.held[side][:2]
            lever = self._lever_m(getattr(observation, side).center_m)
            t_lat = (
                ref.bound["lateral_m"]
                + bound["lateral_m"]
                + e
                + lever * psi
                + self.config.pallet_term_m
            )
            t_along = (
                ref.bound["along_m"]
                + bound["along_m"]
                + e
                + lever * psi
                + self.config.pallet_term_m
            )
            if abs(float(np.dot(d, n))) > t_lat + t_along * mixing:
                return f"{side}_lateral"
            if abs(float(np.dot(d, a))) > t_along + t_lat * mixing:
                return f"{side}_along"
        if (
            abs(_wrap(held["yaw"] - ref.held["yaw"]))
            > ref.bound["yaw_rad"] + bound["yaw_rad"] + psi
        ):
            return "yaw"
        return None

    def _model(self, observation, bound) -> str | None:
        """Front only: each width and the centre spacing against the pallet model."""
        for side in ("left", "right"):
            if (
                abs(getattr(observation, side).width_m - self.config.opening_width_m)
                > bound["width_m"]
            ):
                return f"{side}_width"
        n = np.array(
            [
                -math.sin(observation.insertion_yaw_rad),
                math.cos(observation.insertion_yaw_rad),
            ]
        )
        spacing = abs(
            float(
                np.dot(
                    np.array(observation.left.center_m[:2])
                    - observation.right.center_m[:2],
                    n,
                )
            )
        )
        if abs(spacing - self.config.pocket_spacing_m) > 2 * bound["lateral_m"]:
            return "spacing"
        return None

    def _evaluate(
        self, observation, source: str, pose, aligned_s: float, stamp_s: float
    ):
        """(status, reason, Accepted or None); status is accepted, rejected or invalid."""
        if observation.status != "valid":
            return "invalid", observation.reason, None
        if source == "roof" and (
            observation.position_sigma_m is None
            or observation.position_sigma_m > self.config.roof_sigma_max_m
        ):
            return "invalid", "roof_sigma", None
        bound = self.bounds.observation(source, self.distance_m(observation))
        if bound is None:
            return "invalid", "unbounded_bin", None
        if source == "front":
            reason = self._model(observation, bound)
            if reason is not None:
                return "rejected", reason, None
        held = held_pockets(observation, pose)
        reason = self._compare(observation, held, bound, self.latest, aligned_s)
        if reason is None and self.latest is not self.anchor:
            anchor_reason = self._compare(
                observation, held, bound, self.anchor, aligned_s
            )
            reason = None if anchor_reason is None else f"anchor_{anchor_reason}"
        if reason is not None:
            return "rejected", reason, None
        return (
            "accepted",
            None,
            Accepted(
                observation,
                source,
                pose,
                aligned_s,
                stamp_s,
                held,
                bound,
                self.frame_version,
            ),
        )

    # Frames --------------------------------------------------------------------------------
    def on_frame(
        self, scene, stamp_s: float, pose, now_s: float, frame_version
    ) -> FrameResult:
        """One delivered frame: ``pose`` is held_from_base_link at stamp_s - L in the held
        frame ``frame_version``, delivered at ``now_s``. The caller delivers every result
        due by now_s before it asks for status() on the same tick (plan D8c 3판: the order
        changes the loss verdict); the loss state is brought to now_s first, so a result
        that arrives after the reacquisition deadline does not erase the failure."""
        frame_version = _version(frame_version)
        if type(frame_version) is not type(self.frame_version) or (
            frame_version != self.frame_version
        ):
            raise ValueError(
                "the held frame changed: rebuild the tracker from a new near capture"
            )
        pose = _finite_pose(pose)
        stamp_s = _finite_time(stamp_s, "stamp_s")
        self.status(now_s)
        aligned = stamp_s - self.bounds.align_latency_s
        if self._last_frame_stamp is not None and (
            abs(stamp_s - self._last_frame_stamp - self.bounds.render_period_s)
            > CADENCE_TOLERANCE_S
        ):
            # A frame that never came (or came off the render cadence) breaks a run of
            # agreeing frames and a run of rejections, as an invalid frame does.
            self.handoff.reset()
            self.consecutive_rejects = 0
        self._last_frame_stamp = stamp_s
        status = reason = front_status = roof_status = None
        front_eval = front_reason = roof_eval = roof_reason = None
        chosen = None
        unbounded = False
        evaluating = (
            None  # the source being evaluated (an unbounded age is its verdict)
        )
        try:
            if self.mode == "front":
                evaluating = "front"
                front = self._front_fn(scene)
                front_status = front.status
                status, reason, chosen = self._evaluate(
                    front, "front", pose, aligned, stamp_s
                )
                front_eval, front_reason = status, reason
                paired = False
                if status == "accepted":
                    mid_x = (front.left.center_m[0] + front.right.center_m[0]) / 2
                    if mid_x <= self.config.handoff_start_front_x_m:
                        prior_xy, prior_yaw = _base_prior(chosen.held, pose)
                        evaluating = "roof"
                        roof = self._roof_fn(scene, prior_xy, prior_yaw)
                        roof_status = roof.status
                        roof_eval, roof_reason, roof_rec = self._evaluate(
                            roof, "roof", pose, aligned, stamp_s
                        )
                        if roof_eval == "invalid" and roof_reason is None:
                            roof_reason = "invalid"
                        if roof_eval == "accepted":
                            paired = True
                            self.handoff.select(front, roof)
                            if self.handoff.qualified:
                                self.mode = "roof"
                                chosen = roof_rec
                        elif roof_eval == "rejected":
                            reason = f"roof_{roof_reason}"
                if not paired:
                    self.handoff.reset()
            else:
                prior_xy, prior_yaw = _base_prior(self.latest.held, pose)
                evaluating = "roof"
                roof = self._roof_fn(scene, prior_xy, prior_yaw)
                roof_status = roof.status
                status, reason, chosen = self._evaluate(
                    roof, "roof", pose, aligned, stamp_s
                )
                roof_eval, roof_reason = status, reason
        except UnboundedAge:
            status, reason, chosen, unbounded = "invalid", "unbounded_age", None, True
            if evaluating == "front":
                front_eval, front_reason = status, reason
            else:
                roof_eval, roof_reason = status, reason
            self.handoff.reset()
        accepted, rejected = status == "accepted", status == "rejected"
        if accepted:
            self.latest = chosen
            self.consecutive_rejects = 0
            if not self._failed:
                self._lost_since = None
        elif rejected:
            self.consecutive_rejects += 1
        else:
            # An invalid frame breaks the run: the mismatch is three rejections in a row
            # (Codex D8a implementation review P3-3).
            self.consecutive_rejects = 0
        return FrameResult(
            stamp_s,
            self.mode,
            accepted,
            rejected,
            reason,
            self.consecutive_rejects,
            self.consecutive_rejects >= self.config.mismatch_frames,
            unbounded,
            self.handoff.confirmed,
            front_status,
            roof_status,
            self._failed,
            front_eval,
            front_reason,
            roof_eval,
            roof_reason,
        )

    @property
    def latest_observation(self) -> PocketObservation:
        return self.latest.observation

    @property
    def latest_held(self) -> dict:
        return self.latest.held

    def arm(self, now_s: float) -> None:
        """The truck enters the near-field section (plan D8c 3판): from here a loss stops
        it and a loss longer than reacquire_s fails (near_field_lost). Outside the section
        a loss is reported but never fails; lost at entry, the wait starts at entry."""
        now_s = _finite_time(now_s, "now_s")
        if self._armed_at is None:
            self._armed_at = now_s
            age = now_s - (self.latest.stamp_s - self.bounds.max_latency_s)
            if age > self.config.loss_s:
                self._lost_since = now_s  # lost at entry: the wait starts here

    @property
    def armed(self) -> bool:
        return self._armed_at is not None

    def status(self, now_s: float) -> TrackStatus:
        """The safety age is always now - (stamp - L_max), never reset by receipt. Once
        armed, a failure (lost for more than reacquire_s) is final: near_field_lost."""
        now_s = _finite_time(now_s, "now_s")
        age = now_s - (self.latest.stamp_s - self.bounds.max_latency_s)
        lost = age > self.config.loss_s
        if lost and self._lost_since is None:
            self._lost_since = now_s
        if not lost and not self._failed:
            self._lost_since = None
        if self.armed and lost and now_s - self._lost_since > self.config.reacquire_s:
            self._failed = True
        return TrackStatus(age, lost or self._failed, self._failed, self._lost_since)


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
    for value, name in (
        (travel_since_capture_m, "travel"),
        (time_to_next_result_s, "time to next result"),
        (decel_mps2, "deceleration"),
        (stop_latency_s, "stop latency"),
        (budget_m, "budget"),
    ):
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


__all__ = [
    "CADENCE_TOLERANCE_S",
    "Accepted",
    "FrameResult",
    "HandoffSelection",
    "NearFieldConfig",
    "NearFieldTracker",
    "RoofHandoff",
    "TrackStatus",
    "UnboundedAge",
    "budget_speed",
    "held_pockets",
    "wall_erosion_m",
]
