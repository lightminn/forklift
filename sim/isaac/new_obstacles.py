"""New-obstacle scenarios for the priority-5 plan, L4 (D6), without Isaac imports.

Each event is fixed before a run as a rule, so the same rule on the same seed
gives the same placement: when the truck has driven ``after_m`` along the
active path of ``phase``, an action happens --

* spawn: a box appears ``ahead_m`` further along that path (rear-axle
  arc length, clamped to the path end), ``lateral_m`` to its left, turned
  with the path; it joins the ground-truth obstacle list of the evaluator,
  never the planner;
* remove: an earlier spawned box disappears;
* silence: an obstacle sensor stops delivering scans (N9, N13).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml


@dataclass
class Event:
    id: str
    action: str  # spawn / remove / silence
    phase: str
    after_m: float
    ahead_m: float = 0.0
    lateral_m: float = 0.0
    size_m: tuple[float, float, float] = (0.4, 0.4, 0.5)
    target: str | None = None  # remove: the spawn id; silence: the sensor name
    fired: bool = False


def load_events(path: Path) -> list[Event]:
    data = yaml.safe_load(Path(path).read_text())
    out = []
    for e in data["events"]:
        out.append(
            Event(
                str(e["id"]), str(e["action"]), str(e["phase"]), float(e["after_m"]),
                float(e.get("ahead_m", 0.0)), float(e.get("lateral_m", 0.0)),
                tuple(float(v) for v in e.get("size_m", (0.4, 0.4, 0.5))), e.get("target"),
            )
        )
        if out[-1].action not in ("spawn", "remove", "silence"):
            raise ValueError(f"unknown action {out[-1].action!r}")
    return out


def pose_along(poses: np.ndarray, distance_m: float, lateral_m: float):
    """(x, y, yaw) distance_m along a rear-axle polyline, shifted lateral_m to its left."""
    xy = poses[:, :2]
    seg = np.hypot(*np.diff(xy, axis=0).T)
    s = np.concatenate(([0.0], np.cumsum(seg)))
    d = float(np.clip(distance_m, 0.0, s[-1]))
    k = int(np.clip(np.searchsorted(s, d, side="right") - 1, 0, len(seg) - 1))
    along = (d - s[k]) / seg[k] if seg[k] > 0 else 0.0
    x, y = xy[k] + along * (xy[k + 1] - xy[k])
    yaw = math.atan2(xy[k + 1, 1] - xy[k, 1], xy[k + 1, 0] - xy[k, 0])
    return x - lateral_m * math.sin(yaw), y + lateral_m * math.cos(yaw), yaw


@dataclass
class Schedule:
    events: list[Event]
    spawned: dict = field(default_factory=dict)  # id -> (x, y, yaw, size)
    silenced: set = field(default_factory=set)
    log: list = field(default_factory=list)

    def update(self, t: float, phase: str, driven_m: float, path_ahead: np.ndarray | None) -> list[tuple]:
        """Actions due now: ('spawn', id, x, y, yaw, size) / ('remove', id) / ('silence', sensor)."""
        due = []
        for e in self.events:
            if e.fired or e.phase != phase or driven_m < e.after_m:
                continue
            if e.action == "spawn":
                if path_ahead is None or len(path_ahead) < 2:
                    continue
                x, y, yaw = pose_along(path_ahead, e.ahead_m, e.lateral_m)
                self.spawned[e.id] = (x, y, yaw, e.size_m)
                due.append(("spawn", e.id, x, y, yaw, e.size_m))
            elif e.action == "remove":
                if e.target not in self.spawned:
                    continue
                self.spawned.pop(e.target)
                due.append(("remove", e.target))
            else:
                self.silenced.add(e.target)
                due.append(("silence", e.target))
            e.fired = True
            self.log.append({"time_s": t, "event": e.id, "action": e.action, "phase": phase,
                             "driven_m": driven_m, "detail": list(due[-1][1:])})
        return due


__all__ = ["Event", "Schedule", "load_events", "pose_along"]
