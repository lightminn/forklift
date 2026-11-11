"""K6: the real truck's drive conversion (docs/plans/2026-11-11-sim-to-real-bringup.md P0-6).

The real truck drives its two rear wheels and steers one mechanism angle q (mcu-link ``DRIVE``/``STATE``
``steering_rad``: the KTH7801 axis after the MCU's linear ``abs_axis`` calibration). This module owns
the two host-side maps the MCU does not:

- speed and path curvature about the rear axle -> rear wheel rates + q (``rear_drive_command``);
  a curvature the steering cannot reach is refused, not slowed (plan P0-6, Codex P2), and a wheel
  over its rate limit scales both wheels by the same factor so the curvature is kept;
- ``STATE`` wheel counts (32-bit modular, per *wheel* revolution after the gear) + q -> rear-axle
  pose (``RealOdometry``), ordered and gap-checked on the MCU clock.

q <-> curvature is a ``SteeringMap``: ``ideal_bicycle`` (q is the virtual centre-wheel angle,
provisional until S2) or a measured monotonic ``table``. Nominal chassis geometry comes from the
chassis model file; real-only values and their provenance from ``config/real/vehicle.yaml``.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from forklift_core.localization.wheel_odometry import advance_arc
from forklift_core.mcu_link.messages import wheel_count_delta

SCHEMA = "real_vehicle/1"
CURVATURE_EDGE_TOL = 1e-9  # numerical noise at the reachable edge is normalised onto it, nothing more
FLOAT32_EDGE_ULPS = 2  # float32 steps at each end -- an edge angle sent or read as an mcu-link f32
_TOP = {"schema", "chassis_model", "wheel_counts", "wheel_radius", "steering_map", "wheel_rate_limit",
        "steering_control"}
_STEERING_CONTROL = ("steering_rate_rad_s", "creep_threshold_rad", "presteer_threshold_rad", "straight_threshold_rad")
_PROV_KEYS = {"source", "measured", "until"}


def _positive(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def _provenance(entry: dict, name: str) -> dict:
    prov = entry.get("provenance") if isinstance(entry, dict) else None
    if not isinstance(prov, dict) or set(prov) != _PROV_KEYS or not isinstance(prov["measured"], bool) \
            or not isinstance(prov["source"], str) or not prov["source"]:
        raise ValueError(f"{name} needs provenance with exactly {sorted(_PROV_KEYS)}")
    return dict(prov)


@dataclass(frozen=True)
class SteeringMap:
    """Mechanism angle q (rad) <-> rear-axle path curvature (1/m), both strictly increasing."""

    kind: str
    angles_rad: tuple[float, ...]
    curvatures_inv_m: tuple[float, ...]
    wheelbase_m: float  # used by ideal_bicycle only

    @property
    def angle_range(self) -> tuple[float, float]:
        return self.angles_rad[0], self.angles_rad[-1]

    @property
    def curvature_range(self) -> tuple[float, float]:
        return self.curvatures_inv_m[0], self.curvatures_inv_m[-1]

    def curvature(self, angle_rad: float) -> float | None:
        """None outside the map (non-finite or beyond its ends): no extrapolation. An angle that is
        outside only by the float32 rounding of an end (the wire and the firmware header are f32)
        reads as that end."""
        if not math.isfinite(angle_rad):
            return None
        low, high = self.angle_range
        if low - FLOAT32_EDGE_ULPS * float(np.spacing(np.float32(abs(low)))) <= angle_rad < low:
            angle_rad = low
        elif high < angle_rad <= high + FLOAT32_EDGE_ULPS * float(np.spacing(np.float32(abs(high)))):
            angle_rad = high
        if not low <= angle_rad <= high:
            return None
        if self.kind == "ideal_bicycle":
            return math.tan(angle_rad) / self.wheelbase_m
        return float(np.interp(angle_rad, self.angles_rad, self.curvatures_inv_m))

    def angle(self, curvature_inv_m: float) -> float | None:
        if not math.isfinite(curvature_inv_m) or not self.curvature_range[0] <= curvature_inv_m <= self.curvature_range[1]:
            return None
        if self.kind == "ideal_bicycle":
            return math.atan(self.wheelbase_m * curvature_inv_m)
        return float(np.interp(curvature_inv_m, self.curvatures_inv_m, self.angles_rad))


def _steering_map(entry: dict, wheelbase: float, track: float, wheel_limit: float) -> SteeringMap:
    kind = entry.get("kind") if isinstance(entry, dict) else None
    if kind == "ideal_bicycle":
        if set(entry) != {"kind", "provenance"}:
            raise ValueError("an ideal_bicycle steering map takes only kind and provenance")
        tangent = math.tan(wheel_limit)
        kmax = tangent / (wheelbase + tangent * track / 2)  # the inner wheel reaches the limit first
        qmax = math.atan(wheelbase * kmax)
        return SteeringMap(kind, (-qmax, qmax), (-kmax, kmax), wheelbase)
    if kind == "table":
        if set(entry) != {"kind", "points", "provenance"}:
            raise ValueError("a table steering map takes kind, points and provenance")
        try:
            pts = np.asarray(entry["points"], dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError("steering map points must be [[q_rad, curvature_inv_m], ...]") from exc
        if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2 or not np.isfinite(pts).all():
            raise ValueError("steering map points must be at least two finite [q_rad, curvature_inv_m] pairs")
        if np.any(np.diff(pts[:, 0]) <= 0) or np.any(np.diff(pts[:, 1]) <= 0):
            raise ValueError("steering map angles and curvatures must both be strictly increasing")
        if not (pts[0, 0] < 0 < pts[-1, 0] and pts[0, 1] < 0 < pts[-1, 1]):
            raise ValueError("a steering map must reach both sides of straight ahead")
        return SteeringMap(kind, tuple(pts[:, 0].tolist()), tuple(pts[:, 1].tolist()), wheelbase)
    raise ValueError(f"unknown steering map kind {kind!r}")


@dataclass(frozen=True)
class RealVehicleConfig:
    """Resolved real-vehicle values. Pairs are (left, right)."""

    wheelbase_m: float
    track_m: float
    wheel_radius_m: tuple[float, float]
    counts_per_wheel_rev: tuple[float, float]
    wheel_rate_limit_rad_s: float
    steering: SteeringMap
    provenance: dict
    calibration_id: str  # 16 hex digits of the resolved values; the firmware header carries the same
    steering_control: dict  # host-side q limits (4a' D1): not in the calibration id (the firmware does not read them)

    def resolved_values(self) -> dict:
        return {
            "schema": SCHEMA,
            "wheelbase_m": self.wheelbase_m,
            "track_m": self.track_m,
            "wheel_radius_m": list(self.wheel_radius_m),
            "counts_per_wheel_rev": list(self.counts_per_wheel_rev),
            "wheel_rate_limit_rad_s": self.wheel_rate_limit_rad_s,
            "steering": {"kind": self.steering.kind, "angles_rad": list(self.steering.angles_rad),
                         "curvatures_inv_m": list(self.steering.curvatures_inv_m)},
        }


def _calibration_id(values: dict) -> str:
    text = json.dumps(values, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()[:16]


def _counts(entry, side: str) -> float:
    if not isinstance(entry, dict):
        raise ValueError(f"wheel_counts.{side} must be a mapping")
    keys = set(entry) - {"provenance"}
    if keys == {"motor_counts_per_rev", "gear_ratio"}:
        return _positive(entry["motor_counts_per_rev"], "motor_counts_per_rev") * _positive(entry["gear_ratio"], "gear_ratio")
    if keys == {"counts_per_wheel_rev"}:
        return _positive(entry["counts_per_wheel_rev"], "counts_per_wheel_rev")
    raise ValueError(f"wheel_counts.{side} takes either motor_counts_per_rev + gear_ratio or counts_per_wheel_rev")


def real_vehicle_from_mapping(data: dict, chassis: dict) -> RealVehicleConfig:
    """From vehicle.yaml's content and the chassis model's parameters (its ``dimensions``/``assumptions``)."""
    if not isinstance(data, dict) or not set(data) <= _TOP or not (_TOP - {"chassis_model"}) <= set(data):
        raise ValueError(f"vehicle config must hold exactly {sorted(_TOP)} (chassis_model optional here)")
    if data["schema"] != SCHEMA:
        raise ValueError(f"vehicle config schema must be {SCHEMA!r}")
    dims, assum = chassis["dimensions"], chassis["assumptions"]
    wheelbase = _positive(float(dims["front_axle_x_m"]) - float(dims["rear_axle_x_m"]), "wheelbase")
    track = _positive(dims["wheel_track_m"], "wheel_track_m")
    nominal_radius = _positive(dims["wheel_radius_m"], "wheel_radius_m")
    wheel_limit = _positive(assum["steering_limit_rad"], "steering_limit_rad")
    if wheel_limit >= math.pi / 2:
        raise ValueError("steering_limit_rad must be below pi/2")
    prov: dict = {}
    counts = []
    wc = data["wheel_counts"]
    if not isinstance(wc, dict) or set(wc) != {"left", "right"}:
        raise ValueError("wheel_counts must hold left and right")
    for side in ("left", "right"):
        counts.append(_counts(wc[side], side))
        prov[f"wheel_counts.{side}"] = _provenance(wc[side], f"wheel_counts.{side}")
    wr = data["wheel_radius"]
    if not isinstance(wr, dict) or set(wr) != {"left", "right", "provenance"}:
        raise ValueError("wheel_radius must hold left, right and provenance")
    radius = tuple(nominal_radius if wr[s] is None else _positive(wr[s], f"wheel_radius.{s}") for s in ("left", "right"))
    prov["wheel_radius"] = _provenance(wr, "wheel_radius")
    steering = _steering_map(data["steering_map"], wheelbase, track, wheel_limit)
    prov["steering_map"] = _provenance(data["steering_map"], "steering_map")
    rl = data["wheel_rate_limit"]
    if not isinstance(rl, dict) or set(rl) != {"rad_s", "provenance"}:
        raise ValueError("wheel_rate_limit must hold rad_s and provenance")
    rate = _positive(rl["rad_s"], "wheel_rate_limit.rad_s")
    prov["wheel_rate_limit"] = _provenance(rl, "wheel_rate_limit")
    sc = data["steering_control"]
    if not isinstance(sc, dict) or set(sc) != {*_STEERING_CONTROL, "provenance"}:
        raise ValueError(f"steering_control must hold {list(_STEERING_CONTROL)} and provenance")
    control = {k: _positive(sc[k], f"steering_control.{k}") for k in _STEERING_CONTROL}
    prov["steering_control"] = _provenance(sc, "steering_control")
    partial = RealVehicleConfig(wheelbase, track, radius, tuple(counts), rate, steering, prov, "", control)
    return RealVehicleConfig(wheelbase, track, radius, tuple(counts), rate, steering, prov,
                             _calibration_id(partial.resolved_values()), control)


def chassis_model_path(path: str | Path) -> Path:
    """The chassis model file a vehicle.yaml names, resolved relative to that file (absolute paths kept)."""
    import yaml

    path = Path(path).resolve()
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or not isinstance(data.get("chassis_model"), str):
        raise ValueError("vehicle config must name its chassis_model file")
    return (path.parent / data["chassis_model"]).resolve()


def load_real_vehicle(path: str | Path) -> RealVehicleConfig:
    """vehicle.yaml with its chassis model resolved relative to the file (any working directory)."""
    import yaml

    path = Path(path).resolve()
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    chassis = yaml.safe_load(chassis_model_path(path).read_text(encoding="utf-8"))
    return real_vehicle_from_mapping(data, chassis)


@dataclass(frozen=True)
class RearDrive:
    """What the MCU is asked for (mcu-link DRIVE wheels and steering) and what it means."""

    speed_mps: float  # applied rear-axle speed (after a common scale-down)
    curvature_inv_m: float
    left_wheel_rad_s: float
    right_wheel_rad_s: float
    steering_rad: float  # mechanism angle q
    speed_limited: bool


@dataclass(frozen=True)
class DriveRefusal:
    """The request cannot be driven as asked; the caller stops and asks the tracker/planner to replan."""

    reason: str
    requested_curvature_inv_m: float
    curvature_range: tuple[float, float]


def rear_drive_command(speed_mps: float, curvature_inv_m: float, cfg: RealVehicleConfig) -> RearDrive | DriveRefusal:
    """Rear-axle speed and curvature (+ = left when moving forward) -> rear wheel rates and q."""
    if not math.isfinite(speed_mps) or not math.isfinite(curvature_inv_m):
        raise ValueError("speed and curvature must be finite")
    low, high = cfg.steering.curvature_range
    kappa = curvature_inv_m
    if kappa > high:
        if kappa - high > CURVATURE_EDGE_TOL:
            return DriveRefusal("curvature_infeasible", curvature_inv_m, (low, high))
        kappa = high
    elif kappa < low:
        if low - kappa > CURVATURE_EDGE_TOL:
            return DriveRefusal("curvature_infeasible", curvature_inv_m, (low, high))
        kappa = low
    half = cfg.track_m / 2
    factors = ((1 - kappa * half) / cfg.wheel_radius_m[0], (1 + kappa * half) / cfg.wheel_radius_m[1])
    allowed = cfg.wheel_rate_limit_rad_s / max(abs(f) for f in factors)
    speed = min(max(speed_mps, -allowed), allowed)
    angle = cfg.steering.angle(kappa)
    assert angle is not None  # kappa is inside the map by construction
    return RearDrive(speed, kappa, speed * factors[0], speed * factors[1], angle, speed != speed_mps)


@dataclass(frozen=True)
class OdometryStep:
    pose: tuple[float, float, float]  # rear axle x, y (m), unwrapped yaw (rad) in the odometry frame
    integrated: bool  # this sample advanced the pose
    continuity_broken: bool  # motion was lost here (boot, invalid steering, gap): downstream must not trust continuity
    reason: str | None


class RealOdometry:
    """Rear-axle dead reckoning from mcu-link STATE samples, ordered on the MCU clock.

    Each interval advances the mean of the two wheels' rotation x effective radius along the arc of
    its end points' mean curvature (as ``integrate_wheel_odometry_increments``). A new ``boot_id``,
    a steering reading outside the map (or non-finite), or an MCU-clock gap over ``max_gap_s`` drops
    the reference: the counts moved meanwhile are not integrated and the step says continuity was
    broken. A valid sample that ends a gap or starts a new boot is itself the new reference (the
    next sample integrates from it); after an invalid steering reading the next valid sample only
    re-references. The pose itself is never reset here.
    """

    def __init__(self, cfg: RealVehicleConfig, *, max_gap_s: float, initial_pose=(0.0, 0.0, 0.0)) -> None:
        self.cfg = cfg
        self.max_gap_us = _positive(max_gap_s, "max_gap_s") * 1e6
        self.pose = tuple(float(v) for v in initial_pose)
        self._boot: int | None = None
        self._ref: tuple[int, int, int, float] | None = None  # time_us, left, right, curvature
        self._last_time_us: int | None = None

    def _step(self, integrated: bool, broken: bool, reason: str | None) -> OdometryStep:
        return OdometryStep(self.pose, integrated, broken, reason)

    def update(self, mcu_time_us: int, boot_id: int, count_left: int, count_right: int,
               steering_rad: float) -> OdometryStep:
        if boot_id != self._boot:
            first = self._boot is None
            self._boot, self._last_time_us, self._ref = boot_id, None, None
            kappa = self.cfg.steering.curvature(steering_rad)
            self._last_time_us = int(mcu_time_us)
            if kappa is not None:
                self._ref = (int(mcu_time_us), int(count_left), int(count_right), kappa)
            if first:
                return self._step(False, kappa is None, None if kappa is not None else "steering_invalid")
            return self._step(False, True, "boot_changed")
        if self._last_time_us is not None and mcu_time_us <= self._last_time_us:
            return self._step(False, False, "time_not_increasing")
        self._last_time_us = int(mcu_time_us)
        kappa = self.cfg.steering.curvature(steering_rad)
        if kappa is None:
            self._ref = None
            return self._step(False, True, "steering_invalid")
        sample = (int(mcu_time_us), int(count_left), int(count_right), kappa)
        ref, self._ref = self._ref, sample
        if ref is None:
            return self._step(False, False, "rereference")
        if sample[0] - ref[0] > self.max_gap_us:
            return self._step(False, True, "gap")
        c = self.cfg
        rot_l = wheel_count_delta(sample[1], ref[1]) * 2 * math.pi / c.counts_per_wheel_rev[0]
        rot_r = wheel_count_delta(sample[2], ref[2]) * 2 * math.pi / c.counts_per_wheel_rev[1]
        distance = (rot_l * c.wheel_radius_m[0] + rot_r * c.wheel_radius_m[1]) / 2
        self.pose = advance_arc(*self.pose, distance, (ref[3] + kappa) / 2)
        return self._step(True, False, None)


__all__ = ["chassis_model_path", "DriveRefusal", "OdometryStep", "RealOdometry", "RealVehicleConfig", "RearDrive", "SteeringMap",
           "load_real_vehicle", "real_vehicle_from_mapping", "rear_drive_command"]
