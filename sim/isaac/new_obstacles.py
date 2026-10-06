"""New-obstacle scenarios for the priority-5 plan, L4 (D6), without Isaac imports.

Each event is fixed before a run as a rule, so the same rule on the same seed
gives the same placement: when the truck has driven ``after_m`` along the
active path of ``phase``, an action happens --

* spawn: a box appears ``ahead_m`` further along that path (rear-axle
  arc length, clamped to the path end), ``lateral_m`` to its left, turned
  with the path; it joins the ground-truth obstacle list of the evaluator,
  never the planner;
* remove: an earlier spawned box disappears;
* silence: an obstacle sensor -- or ``pocket_camera``, the depth check's
  camera -- stops delivering (N9, N13).

A spawn may instead sit in the pallet frame (``frame: pallet``, N11/N12/N14):
``along_m`` from the approach face along the insertion direction (inward
positive) and ``lateral_m`` to its left; it waits until the runner knows the
face. ``on_reverse`` holds a spawn until the phase drives a reverse leg (N4),
and events sharing a ``group`` fire only once between them.
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
    beyond_end_m: float | None = None  # spawn past the path end along its last heading
    after_event: str | None = None  # instead of after_m: delay_s after that event fired
    delay_s: float = 0.0
    frame: str = "path"  # path / pallet
    along_m: float = 0.0  # pallet frame: from the approach face, inward positive
    on_reverse: bool = False  # wait for a reverse leg of the phase (N4)
    group: str | None = None  # events sharing a group fire once between them
    fired: bool = False
    fired_s: float | None = None


def load_events(path: Path) -> list[Event]:
    data = yaml.safe_load(Path(path).read_text())
    out = []
    for e in data["events"]:
        out.append(
            Event(
                str(e["id"]), str(e["action"]), str(e["phase"]), float(e["after_m"]),
                float(e.get("ahead_m", 0.0)), float(e.get("lateral_m", 0.0)),
                tuple(float(v) for v in e.get("size_m", (0.4, 0.4, 0.5))), e.get("target"),
                None if e.get("beyond_end_m") is None else float(e["beyond_end_m"]),
                e.get("after_event"), float(e.get("delay_s", 0.0)),
                str(e.get("frame", "path")), float(e.get("along_m", 0.0)), bool(e.get("on_reverse", False)),
                e.get("group"),
            )
        )
        if out[-1].action not in ("spawn", "remove", "silence"):
            raise ValueError(f"unknown action {out[-1].action!r}")
        if out[-1].frame not in ("path", "pallet"):
            raise ValueError(f"unknown frame {out[-1].frame!r}")
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

    def update(self, t: float, phase: str, driven_m: float, path_ahead: np.ndarray | None, *,
               leg_direction: int = 1, pallet_face: tuple | None = None) -> list[tuple]:
        """Actions due now: ('spawn', id, x, y, yaw, size) / ('remove', id) / ('silence', sensor).

        ``pallet_face`` is (x, y, insertion yaw) of the approach face centre."""
        due = []
        fired_at = {e.id: e.fired_s for e in self.events if e.fired}
        groups = {e.group for e in self.events if e.fired and e.group is not None}
        for e in self.events:
            if e.fired or (e.group is not None and e.group in groups):
                continue
            if e.on_reverse and leg_direction >= 0:
                continue
            if e.after_event is not None:
                if fired_at.get(e.after_event) is None or t - fired_at[e.after_event] < e.delay_s:
                    continue
            elif e.phase != phase or driven_m < e.after_m:
                continue
            if e.action == "spawn":
                if e.frame == "pallet":
                    if pallet_face is None:
                        continue
                    fx, fy, fyaw = pallet_face
                    x = fx + e.along_m * math.cos(fyaw) - e.lateral_m * math.sin(fyaw)
                    y = fy + e.along_m * math.sin(fyaw) + e.lateral_m * math.cos(fyaw)
                    yaw = fyaw
                elif path_ahead is None or len(path_ahead) < 2:
                    continue
                elif e.beyond_end_m is not None:
                    ex, ey = path_ahead[-1, 0], path_ahead[-1, 1]
                    eyaw = math.atan2(path_ahead[-1, 1] - path_ahead[-2, 1], path_ahead[-1, 0] - path_ahead[-2, 0])
                    x = ex + e.beyond_end_m * math.cos(eyaw) - e.lateral_m * math.sin(eyaw)
                    y = ey + e.beyond_end_m * math.sin(eyaw) + e.lateral_m * math.cos(eyaw)
                    yaw = eyaw
                else:
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
            e.fired_s = t
            if e.group is not None:
                groups.add(e.group)
            self.log.append({"time_s": t, "event": e.id, "action": e.action, "phase": phase,
                             "driven_m": driven_m, "detail": list(due[-1][1:])})
        return due


__all__ = ["Event", "Schedule", "load_events", "pose_along"]
