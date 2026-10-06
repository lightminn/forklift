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
    on_reverse: bool = False  # wait for a reverse leg of the phase (N4); same as leg="reverse"
    leg: str = "any"  # any / forward / reverse: the direction of the leg it waits for
    group: str | None = None  # events sharing a group fire once between them
    min_curvature_inv_m: float = 0.0  # path frame: wait until the path curves this much there (N3)
    inside: bool = False  # path frame: lateral_m measured to the inside of that curve
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
                frame=str(e.get("frame", "path")), along_m=float(e.get("along_m", 0.0)),
                on_reverse=bool(e.get("on_reverse", False)),
                leg=str(e.get("leg", "reverse" if e.get("on_reverse") else "any")), group=e.get("group"),
                min_curvature_inv_m=float(e.get("min_curvature_inv_m", 0.0)), inside=bool(e.get("inside", False)),
            )
        )
        if out[-1].action not in ("spawn", "remove", "silence"):
            raise ValueError(f"unknown action {out[-1].action!r}")
        if out[-1].on_reverse:
            out[-1].leg = "reverse"
        if out[-1].leg not in ("any", "forward", "reverse"):
            raise ValueError(f"unknown leg {out[-1].leg!r}")
        if out[-1].frame not in ("path", "pallet"):
            raise ValueError(f"unknown frame {out[-1].frame!r}")
    return out


def _length(poses: np.ndarray) -> float:
    return float(np.sum(np.hypot(*np.diff(poses[:, :2], axis=0).T)))


def _curvature_at(poses: np.ndarray, distance_m: float, window_m: float = 0.3) -> float:
    """Signed curvature (left positive) of the polyline around distance_m."""
    a = pose_along(poses, max(distance_m - window_m, 0.0), 0.0)
    b = pose_along(poses, distance_m + window_m, 0.0)
    xy = poses[:, :2]
    s = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))))
    span = min(distance_m + window_m, s[-1]) - max(distance_m - window_m, 0.0)
    dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
    return float(dyaw / span) if span > 1e-9 else 0.0


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
            leg = "reverse" if e.on_reverse else e.leg
            if (leg == "reverse" and leg_direction >= 0) or (leg == "forward" and leg_direction <= 0):
                continue  # leg_direction 0: no leg of the phase (a backoff)
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
                elif leg != "any" and e.beyond_end_m is None and _length(path_ahead) < e.ahead_m + 0.5:
                    continue  # a leg too short would clamp the box to its end (Codex checkpoint 9 P1)
                elif e.beyond_end_m is not None:
                    ex, ey = path_ahead[-1, 0], path_ahead[-1, 1]
                    eyaw = math.atan2(path_ahead[-1, 1] - path_ahead[-2, 1], path_ahead[-1, 0] - path_ahead[-2, 0])
                    x = ex + e.beyond_end_m * math.cos(eyaw) - e.lateral_m * math.sin(eyaw)
                    y = ey + e.beyond_end_m * math.sin(eyaw) + e.lateral_m * math.cos(eyaw)
                    yaw = eyaw
                else:
                    lateral = e.lateral_m
                    if e.min_curvature_inv_m > 0 or e.inside:
                        kappa = _curvature_at(path_ahead, e.ahead_m)
                        if abs(kappa) < e.min_curvature_inv_m:
                            continue  # not in a curve there yet
                        if e.inside:
                            lateral = math.copysign(abs(e.lateral_m), kappa)
                    x, y, yaw = pose_along(path_ahead, e.ahead_m, lateral)
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
            record = {"time_s": t, "event": e.id, "action": e.action, "phase": phase,
                      "driven_m": driven_m, "detail": list(due[-1][1:])}
            if e.action == "spawn" and e.frame == "path" and path_ahead is not None and len(path_ahead) >= 3:
                record["curvature_inv_m"] = _curvature_at(path_ahead, e.ahead_m)
            self.log.append(record)
        return due


__all__ = ["Event", "Schedule", "load_events", "pose_along"]
