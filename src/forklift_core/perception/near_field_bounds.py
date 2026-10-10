"""Plan D8c 3판: the measured near-field bounds the tracker consumes.

The file (``config/near_field_bounds_measured.json``) is written by
tools/export_near_field_bounds.py from the D8b third matrix; this module reads it and
answers the two lookups the tracker makes. Both are sample maxima of one synthetic
camera model under the contract the file records -- not proven limits, not for the real
D435i.

- ``observation(source, distance_m)``: an observed camera-face distance can come from
  every measured 0.1 m bin whose own range widened by its along bound plus
  DISTANCE_MARGIN_M contains it (the inverse image -- each bin's own along bound, so a
  bin with a large along error is reached from far); the bound is the component-wise
  maximum over those bins. Unmeasured bins and the space outside the grid have no along
  bound, so the source's largest one stands in: when such a region lies within that
  reach the lookup is None. Below 0 m (the face plane) is outside the operating range
  and is not counted as missing.
- ``odometry(age_s)``: the relative odometry error (rear-axle position e, heading psi)
  of every pair of control ticks at most that far apart, from the first stored age at
  or above the query (the table is a running maximum, so that is conservative); None
  past the table.
"""

from __future__ import annotations

import bisect
import json
import math
from dataclasses import dataclass, field
from itertools import pairwise
from pathlib import Path

SCHEMA = "near_field_bounds/v1"
COMPONENTS = ("lateral_m", "along_m", "yaw_rad", "wall_m", "width_m")
REQUIRED = {"front": COMPONENTS, "roof": COMPONENTS[:4]}
BIN_M = 0.1
# Beyond each bin's along bound: a sample maximum is exceeded by new samples.
DISTANCE_MARGIN_M = 0.01


def bin_containing(rows: list[dict], distance_m: float) -> dict | None:
    """The bin holding distance_m ([lo, hi), rows high edge first)."""
    for cell in rows:
        hi, lo = cell["bin_m"]
        if lo <= distance_m < hi:
            return cell
    return None


def interval_bound(
    rows: list[dict], distance_m: float, margin_m: float = DISTANCE_MARGIN_M
) -> dict | None:
    """The component-wise maximum over every measured bin the observed distance can come
    from (see the module docstring); None when an unmeasured bin or the space outside
    the grid is within reach, or when no measured bin is."""
    measured = [c for c in rows if not c.get("unbounded")]
    if not measured:
        return None
    reach = max(c["along_m"] for c in measured) + margin_m
    candidates = [
        c
        for c in measured
        if c["bin_m"][1] - c["along_m"] - margin_m
        <= distance_m
        < c["bin_m"][0] + c["along_m"] + margin_m
    ]
    lo, hi = max(distance_m - reach, 0.0), distance_m + reach
    # Unmeasured: unbounded bins, gaps between bins and the space past either end.
    edges = sorted((c["bin_m"][1], c["bin_m"][0]) for c in rows)
    holes = [(c["bin_m"][1], c["bin_m"][0]) for c in rows if c.get("unbounded")]
    holes += [(a[1], b[0]) for a, b in pairwise(edges) if b[0] - a[1] > 1e-9]
    holes += [(-math.inf, edges[0][0]), (edges[-1][1], math.inf)]
    if any(h_lo < hi and h_hi > lo for h_lo, h_hi in holes if h_hi - h_lo > 1e-9):
        return None
    if not candidates:
        return None
    keys = [k for k in COMPONENTS if all(k in c for c in candidates)]
    return {k: max(c[k] for c in candidates) for k in keys}


def _finite_non_negative(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    value = float(value)
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be finite and non-negative")
    return value


def _check_rows(source: str, rows: list) -> list[dict]:
    if not rows:
        raise ValueError(f"{source}: no bins")
    spans = []
    for cell in rows:
        hi, lo = (float(v) for v in cell["bin_m"])
        if not math.isclose(hi - lo, BIN_M, abs_tol=1e-6) or not math.isclose(
            lo / BIN_M, round(lo / BIN_M), abs_tol=1e-6
        ):
            raise ValueError(
                f"{source}: bin {cell['bin_m']} is not on the {BIN_M} m grid"
            )
        spans.append((lo, hi))
        if not cell.get("unbounded"):
            for k in REQUIRED[source]:
                _finite_non_negative(cell.get(k), f"{source} bin {cell['bin_m']} {k}")
    spans.sort()
    if any(b[0] < a[1] - 1e-9 for a, b in pairwise(spans)):
        raise ValueError(f"{source}: bins overlap")
    return rows


@dataclass(frozen=True)
class NearFieldBounds:
    align_latency_s: float  # L: pixels this much older than the frame stamp (aligned)
    max_latency_s: float  # L_hi: the band's top (the safety age)
    read_delay_s: float  # A: a result arrives this long after its stamp
    render_period_s: float
    observation_rows: dict  # source -> bins (high edge first)
    odometry_age_s: tuple
    odometry_e_m: tuple
    odometry_psi_rad: tuple
    near_capture: dict  # component -> bound of the near-capture observation
    condition: dict = field(default_factory=dict)  # the contract the bounds hold under

    @classmethod
    def from_dict(cls, data: dict) -> NearFieldBounds:
        if data.get("schema") != SCHEMA:
            raise ValueError(f"not a {SCHEMA} file")
        lat, odo = data["latency"], data["odometry"]
        align = _finite_non_negative(lat["align_s"], "align_s")
        top = _finite_non_negative(lat["max_s"], "max_s")
        delay = _finite_non_negative(lat["read_delay_s"], "read_delay_s")
        period = _finite_non_negative(lat["render_period_s"], "render_period_s")
        if align > top or period <= 0:
            raise ValueError("the latency band is inconsistent")
        ages = tuple(_finite_non_negative(a, "odometry age") for a in odo["age_s"])
        if not ages or ages[0] <= 0 or any(b <= a for a, b in pairwise(ages)):
            raise ValueError("odometry ages must be positive and increase")
        tables = {}
        for name in ("e_m", "psi_rad"):
            values = tuple(
                _finite_non_negative(v, f"odometry {name}") for v in odo[name]
            )
            if len(values) != len(ages) or any(b < a for a, b in pairwise(values)):
                raise ValueError(
                    f"odometry {name} must be a running maximum over the ages"
                )
            tables[name] = values
        rows = {s: _check_rows(s, data["observation"][s]) for s in ("front", "roof")}
        near = data["near_capture"]["bound"]
        near = {
            k: _finite_non_negative(near.get(k), f"near capture {k}")
            for k in COMPONENTS
        }
        return cls(
            align,
            top,
            delay,
            period,
            rows,
            ages,
            tables["e_m"],
            tables["psi_rad"],
            near,
            dict(data.get("condition", {})),
        )

    @classmethod
    def load(cls, path) -> NearFieldBounds:
        return cls.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))

    def observation(self, source: str, distance_m: float) -> dict | None:
        return interval_bound(self.observation_rows[source], float(distance_m))

    def odometry(self, age_s: float) -> tuple[float, float] | None:
        if not math.isfinite(age_s) or age_s < 0:
            raise ValueError("age must be finite and non-negative")
        if age_s == 0:
            return 0.0, 0.0
        i = bisect.bisect_left(self.odometry_age_s, age_s - 1e-9)
        if i >= len(self.odometry_age_s):
            return None
        return self.odometry_e_m[i], self.odometry_psi_rad[i]


__all__ = [
    "COMPONENTS",
    "DISTANCE_MARGIN_M",
    "NearFieldBounds",
    "bin_containing",
    "interval_bound",
]
