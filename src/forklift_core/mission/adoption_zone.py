"""The adoption-stage zone check (docs/plans/2026-10-28-adoption-zone-check.md, lead decision 2026-10-28).

Wherever a pallet estimate becomes the planned pickup, its centre must lie inside the mission's adoption rectangle
grown by a margin; outside, the mission ends as ``adoption_outside_zone``. The rectangle is a pallet-centre range
aligned with the map axes -- the instructed placement zone at the pallet's first place, a small square about the
put-down centre for a re-pick -- never the obstacle rectangle grown by the pallet's half diagonal, and never the far
selector's circumscribed circle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

OUTSIDE = "adoption_outside_zone"
UNDEFINED = "adoption_zone_undefined"
DEFAULT_MARGIN_M = 0.30  # plan section 5 test 0: the largest per-axis centre error 0.254 m, rounded up to 5 cm
# Plan section 8a (lead 2026-10-28, provisional: observed maxima rounded up, no established bound): a run localizing on
# a survey bundle 0.10 m whatever its encoders (largest error 0.053 m), a run without a bundle 0.30 m (test 0's live
# mapping runs, which used the reported encoders)
MARGIN_LOCALIZED_M = 0.10
MARGIN_MAPPING_M = DEFAULT_MARGIN_M


REPICK_HALF_M = 0.10  # Isaac unloading error 0.02-0.10 m (docs/validation/2026-10-22-session-scene-pallet-props.md)
# docs/plans/2026-10-28-far-target-binding.md F1: the far capture's expected heading -- a re-pick about its put-down
# heading +-15 deg, a zone over its own yaw range + 5 deg, a zone without one +-15 deg
REPICK_YAW_HALF_RAD = math.radians(15.0)
FAR_AXIS_EXTRA_RAD = math.radians(5.0)
FAR_AXIS_DEFAULT_RAD = math.radians(15.0)
FAR_AXIS_MAX_RAD = math.radians(89.0)


def default_margin(*, localized: bool) -> float:
    """The margin when none is given: by whether the run localizes on a survey bundle."""
    return MARGIN_LOCALIZED_M if localized else MARGIN_MAPPING_M


def _finite(value, what: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{what} is not finite: {value!r}")
    return value


@dataclass(frozen=True)
class AdoptionZone:
    """Where the adopted pallet's centre may stand, before the margin."""

    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    source: str  # scene_layout, bay, session_zone or put_down
    # the pallet's heading range (record and the far capture's expected axis only; the adoption check reads position)
    yaw_centre_rad: float = 0.0
    yaw_half_rad: float | None = None

    def __post_init__(self):
        for name in ("x_min_m", "x_max_m", "y_min_m", "y_max_m", "yaw_centre_rad"):
            _finite(getattr(self, name), name)
        if self.x_min_m > self.x_max_m or self.y_min_m > self.y_max_m:
            raise ValueError(f"an empty adoption rectangle: {self}")
        if self.yaw_half_rad is not None and not (math.isfinite(self.yaw_half_rad) and self.yaw_half_rad >= 0):
            raise ValueError(f"a yaw range must be finite and >= 0: {self.yaw_half_rad!r}")

    def bounds(self) -> list:
        return [self.x_min_m, self.x_max_m, self.y_min_m, self.y_max_m]


def mission_zone(pallet, first_zone: AdoptionZone | None) -> AdoptionZone | None:
    """The mission's rectangle, fixed when it is prepared, from the TargetsBook entry: never put down in this session
    -> its first place's zone (an observation moving its pose does not change that); put down (a delivery or a
    recovery) -> a square of REPICK_HALF_M about where (``put_down_pose``); None when the first zone is unknown."""
    if pallet.put_down_pose is None:
        return first_zone
    x, y = (_finite(v, "put_down_pose") for v in (pallet.put_down_pose.x_m, pallet.put_down_pose.y_m))
    return AdoptionZone(x - REPICK_HALF_M, x + REPICK_HALF_M, y - REPICK_HALF_M, y + REPICK_HALF_M, "put_down",
                        yaw_centre_rad=_finite(pallet.put_down_pose.yaw_rad, "put_down_pose"),
                        yaw_half_rad=REPICK_YAW_HALF_RAD)


def far_site(zone: AdoptionZone, margin_m: float, *, base_xy, base_yaw: float):
    """The far capture's expected site in base_link (F1): the rectangle's centre, the radius of the circle around the
    rectangle grown by the margin (every point the adoption check accepts lies in it), and the zone's heading range
    + 5 deg (15 deg without one), placed with the capture's control base."""
    from forklift_core.perception.pocket_detector import ExpectedSite

    margin_m = _finite(margin_m, "margin_m")
    cx, cy = (zone.x_min_m + zone.x_max_m) / 2, (zone.y_min_m + zone.y_max_m) / 2
    dx, dy = cx - float(base_xy[0]), cy - float(base_xy[1])
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    rel = zone.yaw_centre_rad - float(base_yaw)
    radius = math.hypot((zone.x_max_m - zone.x_min_m) / 2 + margin_m, (zone.y_max_m - zone.y_min_m) / 2 + margin_m)
    axis = FAR_AXIS_DEFAULT_RAD if zone.yaw_half_rad is None else zone.yaw_half_rad + FAR_AXIS_EXTRA_RAD
    axis = min(axis, FAR_AXIS_MAX_RAD)  # ExpectedSite's open bound (pi/2): a heading range that wide restricts nothing
    return ExpectedSite(c * dx + s * dy, -s * dx + c * dy, math.atan2(math.sin(rel), math.cos(rel)), radius, axis)


def check(site_xy, zone: AdoptionZone, margin_m: float, *, stage: str) -> dict:
    """The record of one adoption: ``edge_m`` is the signed distance past the rectangle's nearest violated edge
    (negative inside: the distance to the nearest edge); inside when it is at most the margin."""
    x, y = (_finite(v, "site") for v in site_xy)
    margin_m = _finite(margin_m, "margin_m")
    if margin_m < 0:
        raise ValueError(f"a negative margin: {margin_m}")
    edge = max(zone.x_min_m - x, x - zone.x_max_m, zone.y_min_m - y, y - zone.y_max_m)
    return {"stage": stage, "site": [x, y], "zone": zone.bounds(), "zone_source": zone.source,
            "margin_m": margin_m, "edge_m": edge, "inside": edge <= margin_m}
