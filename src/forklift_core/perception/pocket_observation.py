"""Base-frame pocket geometry for horizontal pallets with vertical openings.

Both rectangular openings share one insertion axis. This contract supports
perception evaluation and tracker input, not insertion-clearance certification.
"""

import math
from dataclasses import dataclass, fields
from numbers import Integral

import numpy as np

from forklift_core._validation import _finite_scalar, _real_array

CLOCK_DOMAINS = frozenset({"ros_sim", "ros_system", "device", "synthetic"})
STATUSES = frozenset({"valid", "no_pallet", "invalid"})
PROVENANCES = frozenset({"synthetic", "replay", "live", "synthetic_ground_truth"})
OBSERVATION_FRAME = "base_link"


@dataclass(frozen=True)
class Pocket:
    """Vertical rectangular opening: front-plane center (x,y,z), width/height in m."""

    center_m: tuple[float, float, float]
    width_m: float
    height_m: float

    def __post_init__(self) -> None:
        center = _real_array(self.center_m, "center_m")
        if center.shape != (3,) or not np.isfinite(center).all():
            raise ValueError("center_m must be a finite three-vector")
        object.__setattr__(self, "center_m", tuple(float(value) for value in center))
        for name in ("width_m", "height_m"):
            value = _finite_scalar(getattr(self, name), name)
            if value <= 0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)


@dataclass(frozen=True)
class PocketObservation:
    """Two pockets in base_link, sharing insertion yaw in (-pi, pi] radians.

    Assumes a horizontal pallet and vertical rectangular openings. Left is along
    (-sin(yaw), cos(yaw), 0); the outward front normal opposes the insertion axis.
    Sigmas are finite nonnegative 1-sigma bounds or None (unknown, never zero).
    stamp_ns is the acquisition time in clock_domain, not the receipt time.
    """

    stamp_ns: int
    clock_domain: str
    frame_id: str
    source_provenance: str
    status: str
    left: Pocket | None
    right: Pocket | None
    insertion_yaw_rad: float | None
    position_sigma_m: float | None
    yaw_sigma_rad: float | None
    reason: str | None

    def __post_init__(self) -> None:
        if (
            isinstance(self.stamp_ns, (bool, np.bool_))
            or not isinstance(self.stamp_ns, Integral)
            or self.stamp_ns < 0
        ):
            raise ValueError("stamp_ns must be a nonnegative integer, excluding bool")
        object.__setattr__(self, "stamp_ns", int(self.stamp_ns))
        for name, allowed in (
            ("clock_domain", CLOCK_DOMAINS),
            ("status", STATUSES),
            ("source_provenance", PROVENANCES),
            ("frame_id", {OBSERVATION_FRAME}),
        ):
            value = getattr(self, name)
            if not isinstance(value, str) or value not in allowed:
                raise ValueError(f"Unsupported {name}: {value!r}")

        if self.status != "valid":
            if any(
                getattr(self, name) is not None
                for name in (
                    "left",
                    "right",
                    "insertion_yaw_rad",
                    "position_sigma_m",
                    "yaw_sigma_rad",
                )
            ):
                raise ValueError("Non-valid observations must omit geometry and sigmas")
            if not isinstance(self.reason, str) or not self.reason.strip():
                raise ValueError("Non-valid observations require a nonempty reason")
            return

        if not isinstance(self.left, Pocket) or not isinstance(self.right, Pocket):
            raise ValueError("Valid observations require left and right Pocket objects")
        if self.reason is not None:
            raise ValueError("Valid observations must omit reason")
        yaw = _finite_scalar(self.insertion_yaw_rad, "insertion_yaw_rad")
        if not -math.pi < yaw <= math.pi:
            raise ValueError("insertion_yaw_rad must lie in (-pi, pi]")
        object.__setattr__(self, "insertion_yaw_rad", yaw)
        for name in ("position_sigma_m", "yaw_sigma_rad"):
            value = getattr(self, name)
            if value is not None:
                value = _finite_scalar(value, name)
                if value < 0:
                    raise ValueError(f"{name} must be nonnegative or None")
                object.__setattr__(self, name, value)

        if not 0.05 < math.dist(self.left.center_m, self.right.center_m) < 2.0:
            raise ValueError("Pocket center distance must lie in (0.05, 2.0) m")
        dx = self.left.center_m[0] - self.right.center_m[0]
        dy = self.left.center_m[1] - self.right.center_m[1]
        separation = -math.sin(yaw) * dx + math.cos(yaw) * dy
        if separation <= (self.left.width_m + self.right.width_m) / 2:
            raise ValueError(
                "Pockets must be ordered along the left axis without overlap"
            )

    def to_json(self) -> dict:
        """Return all contract fields as JSON-compatible Python values (no NaN/Inf)."""
        obj = {field.name: getattr(self, field.name) for field in fields(self)}
        for name in ("left", "right"):
            pocket = getattr(self, name)
            if pocket is not None:
                obj[name] = {
                    "center_m": list(pocket.center_m),
                    "width_m": pocket.width_m,
                    "height_m": pocket.height_m,
                }
        return obj


def pocket_observation_from_json(obj: dict) -> PocketObservation:
    """Parse an observation object; unknown/missing keys and bad values raise ValueError."""
    if not isinstance(obj, dict) or set(obj) != {
        f.name for f in fields(PocketObservation)
    }:
        raise ValueError(
            "Observation must contain exactly the PocketObservation fields"
        )
    values = dict(obj)
    for name in ("left", "right"):
        pocket = values[name]
        if pocket is not None:
            if not isinstance(pocket, dict) or set(pocket) != {
                f.name for f in fields(Pocket)
            }:
                raise ValueError(f"{name} must contain exactly the Pocket fields")
            values[name] = Pocket(**pocket)
    return PocketObservation(**values)


def yaw_difference_rad(estimate_rad: float, reference_rad: float) -> float:
    """Return signed estimate minus reference in (-pi, pi]; -pi maps to +pi."""
    estimate = _finite_scalar(estimate_rad, "estimate_rad")
    reference = _finite_scalar(reference_rad, "reference_rad")
    difference = (
        estimate % math.tau - reference % math.tau + math.pi
    ) % math.tau - math.pi
    return math.pi if difference == -math.pi else difference
