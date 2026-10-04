"""Emergency-stop probe for the priority-5 plan P0a (docs/plans/2026-10-04-lidar-obstacle-map.md).

At planned moments of a mission run the truck is stopped as the obstacle
layer will stop it: every wheel target to zero in one step, steering held
where it is. The probe records, from ground truth, how long the body takes to
start slowing (command-to-deceleration latency), how far it travels and how
long it takes to stand still, and how far the stopping trajectory leaves the
arc the truck was on (the plan's stopping-envelope offset). Then the run
carries on as before, so one mission gives several stops in different states
(unloaded/loaded, forward/reverse, straight/curve).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field


def parse_spec(text: str) -> list[tuple[str, float]]:
    """'approach@2,transport@5.5' -> [('approach', 2.0), ('transport', 5.5)]."""
    out = []
    for item in filter(None, (part.strip() for part in text.split(","))):
        phase, _, delay = item.partition("@")
        if not phase or not delay:
            raise ValueError(f"bad estop probe item {item!r}: expected phase@seconds")
        value = float(delay)
        if not math.isfinite(value) or value < 0:
            raise ValueError(f"bad estop probe delay in {item!r}")
        out.append((phase, value))
    return out


def arc_offset_m(start, curvature_inv_m: float, point) -> float:
    """Distance from point to the arc (or line) through start (x, y, yaw) with that curvature."""
    x0, y0, yaw = start
    if abs(curvature_inv_m) < 1e-9:
        return abs(-math.sin(yaw) * (point[0] - x0) + math.cos(yaw) * (point[1] - y0))
    radius = 1.0 / curvature_inv_m
    cx = x0 - radius * math.sin(yaw)
    cy = y0 + radius * math.cos(yaw)
    return abs(math.hypot(point[0] - cx, point[1] - cy) - abs(radius))


@dataclass
class EstopProbe:
    pending: list[tuple[str, float]]
    min_speed_mps: float = 0.05
    stop_speed_mps: float = 0.01
    settle_s: float = 0.25
    records: list[dict] = field(default_factory=list)
    _active: dict | None = None

    @property
    def active(self) -> bool:
        return self._active is not None

    def update(self, *, t, phase, phase_elapsed_s, rear, speed_mps, loaded, curvature_inv_m, extra=None) -> bool:
        """Advance one physics tick; True while the probe holds the truck stopped."""
        if self._active is None:
            for index, (name, delay) in enumerate(self.pending):
                if name == phase and phase_elapsed_s >= delay and abs(speed_mps) >= self.min_speed_mps:
                    self.pending.pop(index)
                    self._active = {
                        "phase": phase,
                        "trigger_time_s": t,
                        "trigger_rear": [float(v) for v in rear],
                        "trigger_speed_mps": float(speed_mps),
                        "direction": "reverse" if speed_mps < 0 else "forward",
                        "loaded": bool(loaded),
                        "curvature_inv_m": float(curvature_inv_m),
                        "decel_start_s": None,
                        "path_m": 0.0,
                        "max_arc_offset_m": 0.0,
                        "still_since_s": None,
                        "_last": [float(v) for v in rear],
                        "trace": [],
                    }
                    break
            if self._active is None:
                return False
        a = self._active
        a["path_m"] += math.hypot(rear[0] - a["_last"][0], rear[1] - a["_last"][1])
        a["_last"] = [float(v) for v in rear]
        a["max_arc_offset_m"] = max(
            a["max_arc_offset_m"], arc_offset_m(a["trigger_rear"], a["curvature_inv_m"], rear)
        )
        # Full rear pose every tick (Codex L0 P1): the stopping volume of the
        # whole body, forks and load is rebuilt from it, not just the axle.
        a["trace"].append(
            [t - a["trigger_time_s"], float(speed_mps), *(float(v) for v in rear)]
            + ([float(v) for v in extra] if extra is not None else [])
        )
        if a["decel_start_s"] is None and abs(speed_mps) < 0.98 * abs(a["trigger_speed_mps"]):
            a["decel_start_s"] = t - a["trigger_time_s"]
        if abs(speed_mps) < self.stop_speed_mps:
            if a["still_since_s"] is None:
                a["still_since_s"] = t
            if t - a["still_since_s"] >= self.settle_s:
                record = {k: v for k, v in a.items() if not k.startswith("_")}
                record["stop_time_s"] = a["still_since_s"] - a["trigger_time_s"]
                record["stop_distance_m"] = a["path_m"]
                record["trace_columns"] = ["dt_s", "speed_mps", "rear_x_m", "rear_y_m", "rear_yaw_rad", "extra..."]
                self.records.append(record)
                self._active = None
                return False
        else:
            a["still_since_s"] = None
        return True


__all__ = ["EstopProbe", "arc_offset_m", "parse_spec"]
