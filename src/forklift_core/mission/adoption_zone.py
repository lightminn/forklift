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
REPICK_HALF_M = 0.10  # Isaac unloading error 0.02-0.10 m (docs/validation/2026-10-22-session-scene-pallet-props.md)


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

    def __post_init__(self):
        for name in ("x_min_m", "x_max_m", "y_min_m", "y_max_m"):
            _finite(getattr(self, name), name)
        if self.x_min_m > self.x_max_m or self.y_min_m > self.y_max_m:
            raise ValueError(f"an empty adoption rectangle: {self}")

    def bounds(self) -> list:
        return [self.x_min_m, self.x_max_m, self.y_min_m, self.y_max_m]


def mission_zone(pallet, first_zone: AdoptionZone | None) -> AdoptionZone | None:
    """The mission's rectangle, fixed when it is prepared, from the TargetsBook entry: never put down in this session
    -> its first place's zone (an observation moving its pose does not change that); put down (a delivery or a
    recovery) -> a square of REPICK_HALF_M about where (``put_down_pose``); None when the first zone is unknown."""
    if pallet.put_down_pose is None:
        return first_zone
    x, y = (_finite(v, "put_down_pose") for v in (pallet.put_down_pose.x_m, pallet.put_down_pose.y_m))
    return AdoptionZone(x - REPICK_HALF_M, x + REPICK_HALF_M, y - REPICK_HALF_M, y + REPICK_HALF_M, "put_down")


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
