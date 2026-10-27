"""The S-P1 gate (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md, "S-P1 판정 규칙", pre-registered
2026-10-27): the real low layer (forklift_core.safety.low_layer) on geometric depth from the forward camera, the truck
following each path kinematically from a cold start -- no memory, no operator, no truth-injected free space -- and
stopping whenever the footprint up to the stopping distance would enter a cell the layer cannot vouch for.

- positive: paths P1-P5 on an empty floor at 0.3 and 0.6 m/s must all finish with no permanent stop (30 s sim without
  progress) and stopped time <= 50 % of the run;
- negative: the two turning counterexamples (Codex plan review 1 and 2) must never let the footprint overlap the
  pallet-sized box;
- robustness (recorded only): pitch +-1 deg and 5 % depth dropout over 20 seeds.

    python tools/s_p1_gate.py --json OUT.json [--tilt-rad 0.10] [--robustness]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tools"))

from forklift_core.planning.geometry import Footprint  # noqa: E402
from forklift_core.safety import low_layer as L  # noqa: E402

import scene_rig  # noqa: E402

REAR_AXLE_OFFSET_M = 0.34  # base_link is this far ahead of the rear axle (dls08_measured)
MOUNT_BASE_XYZ = (0.619, 0.0, 0.27)  # carriage_low_measured
DEPTH_MIN_M, DEPTH_MAX_M = 0.3, 3.0
DT_S = 0.1  # 10 Hz camera and control
LATENCY_FRAMES = 1  # a frame is used one step after its capture
STOP_DISTANCE_M = {0.3: 0.125, 0.6: 0.260}  # the runner's stop model at these speeds (plan S)
PERMANENT_STOP_S = 30.0
STOPPED_FRACTION_MAX = 0.5
PLANNER_FOOT = Footprint(1.29, 0.17, 0.36)
K = 0.29
PATHS = {
    "P1": [(3.0, 0.0)],
    "P2": [(math.pi / 2 / K, K)],
    "P3": [(math.pi / 2 / K, -K)],
    "P4": [(2.0, 0.0), (math.pi / 2 / K, K), (2.0, 0.0)],
    "P5": [(3.0, 0.0), (-3.0, 0.0)],
}
# Codex plan review: a left turn at 0.29 from the rear axle at the origin; the box's footprint overlaps after
# 0.288 m (body outline) / 0.286 m (operating outline) of travel
NEGATIVES = {
    "N1": (Footprint(0.959, 0.17, 0.315), (0.25, 0.75)),
    "N2": (Footprint(0.944, 0.17, 0.36), (0.25, 0.795)),
}
EPAL = (0.6, 0.8, 0.144)


@dataclass
class World:
    boxes: list = field(default_factory=list)  # (cx, cy, sx, sy, height), axis aligned, standing on the floor


def camera_rotation(tilt_rad: float) -> np.ndarray:
    """rear-axle frame <- optical (the mount's rotation; the translation is added separately)."""
    return np.asarray(scene_rig.Camera(MOUNT_BASE_XYZ, tilt_rad).base_from_optical().rotation, float)


def pixel_dirs(stride: int = 1) -> np.ndarray:
    k = scene_rig.intrinsics()
    us = np.arange(0, k.width, stride) + 0.5
    vs = np.arange(0, k.height, stride) + 0.5
    U, V = np.meshgrid(us, vs)
    return np.stack([(U - k.cx) / k.fx, (V - k.cy) / k.fy, np.ones_like(U)], -1).reshape(-1, 3)


def render_points(world: World, rear_pose, tilt_rad: float, *, pitch_rad: float = 0.0, dropout: float = 0.0,
                  rng=None, stride: int = 1) -> np.ndarray:
    """The valid depth points (odometry frame) a camera on the truck at ``rear_pose`` sees: rays against the floor
    z = 0 and the boxes; depth (optical z) within 0.3-3.0 m; ``pitch_rad`` tips the camera (the truck's pitch);
    ``dropout`` removes that share of pixels at random."""
    x, y, yaw = rear_pose
    c, s = math.cos(yaw), math.sin(yaw)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])  # positive pitch tips the nose down
    rot = rz @ ry @ camera_rotation(tilt_rad)
    cam = np.array([MOUNT_BASE_XYZ[0] + REAR_AXLE_OFFSET_M, MOUNT_BASE_XYZ[1], MOUNT_BASE_XYZ[2]])
    origin = np.array([x, y, 0.0]) + rz @ (ry @ cam)
    d_opt = pixel_dirs(stride)
    d = d_opt @ rot.T  # odometry-frame directions; the parameter along d is the optical depth (d_opt z = 1)
    t = np.full(len(d), np.inf)
    down = d[:, 2] < -1e-9
    t[down] = -origin[2] / d[down, 2]
    for cx, cy, sx, sy, h in world.boxes:
        lo = np.array([cx - sx / 2, cy - sy / 2, 0.0])
        hi = np.array([cx + sx / 2, cy + sy / 2, h])
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (lo - origin) / d
            t2 = (hi - origin) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0)
        t = np.where(hit & (tmin < t), tmin, t)
    valid = (t >= DEPTH_MIN_M) & (t <= DEPTH_MAX_M)
    if dropout > 0:
        valid &= (rng or np.random.default_rng(0)).random(len(t)) >= dropout
    return origin + d[valid] * t[valid, None]


def box_overlaps(footprint: Footprint, rear_pose, box) -> bool:
    """Whether the footprint rectangle at ``rear_pose`` overlaps the box's floor rectangle (separating axes)."""
    x, y, yaw = rear_pose
    c, s = math.cos(yaw), math.sin(yaw)
    fp = [(x + c * a - s * b, y + s * a + c * b) for a, b in
          ((footprint.front_m, footprint.half_width_m), (footprint.front_m, -footprint.half_width_m),
           (-footprint.rear_m, -footprint.half_width_m), (-footprint.rear_m, footprint.half_width_m))]
    cx, cy, sx, sy, _ = box
    bx = [(cx - sx / 2, cy - sy / 2), (cx + sx / 2, cy - sy / 2), (cx + sx / 2, cy + sy / 2), (cx - sx / 2, cy + sy / 2)]
    for poly in (fp, bx):
        for i in range(4):
            (x1, y1), (x2, y2) = poly[i], poly[(i + 1) % 4]
            ax, ay = y2 - y1, x1 - x2
            p1 = [ax * px + ay * py for px, py in fp]
            p2 = [ax * px + ay * py for px, py in bx]
            if max(p1) < min(p2) - 1e-9 or max(p2) < min(p1) - 1e-9:
                return False
    return True


def run(segments, speed: float, *, world: World | None = None, footprint: Footprint = PLANNER_FOOT,
        tilt_rad: float = 0.10, pitch_rad: float = 0.0, dropout: float = 0.0, seed: int = 0, stride: int = 1,
        max_time_s: float = 600.0) -> dict:
    """One cold-start run along ``segments``; the result of the pre-registered rule for it."""
    world = world or World()
    rng = np.random.default_rng(seed)
    cfg = L.LowLayerConfig()
    layer = L.LowLayer(cfg, origin_xy=(-6.0, -6.0), size_cells=(320, 320))
    path = L.path_poses([(0.0, 0.0, 0.0)], segments, step_m=0.01)
    seg_sign = []
    for length, _ in segments:
        seg_sign += [1 if length >= 0 else -1] * max(1, int(math.ceil(abs(length) / 0.01)))
    seg_sign.append(seg_sign[-1])
    i, t = 0, 0.0
    layer.mark_swept(footprint, path[0], t)
    queue = []
    stopped_s = moving_s = still_s = 0.0
    first_block = None
    overlap = False
    stop_m = STOP_DISTANCE_M[speed]
    step = max(1, int(round(speed * DT_S / 0.01)))
    while i < len(path) - 1 and t < max_time_s:
        pose = path[i]
        pitch = pitch_rad * (1 if (rng.random() < 0.5) else -1) if pitch_rad else 0.0
        queue.append((t, render_points(world, pose, tilt_rad, pitch_rad=pitch, dropout=dropout, rng=rng,
                                       stride=stride)))
        if len(queue) > LATENCY_FRAMES:
            _, pts = queue.pop(0)
            layer.integrate(pts, t)
        reverse = seg_sign[i] < 0
        j = min(len(path) - 1, i + step)
        k = min(len(path) - 1, j + int(math.ceil(stop_m / 0.01)))
        # the envelope may not run past a gear change: the stop distance is along the current direction
        k = next((m for m in range(i, k + 1) if seg_sign[m] != seg_sign[i]), k)
        ok, bad = L.envelope_admissible(layer, t, path[i:k + 1], footprint, reverse=reverse)
        if ok:
            for m in range(i + 1, j + 1):
                layer.mark_swept(footprint, path[m], t)
                if any(box_overlaps(footprint, path[m], b) for b in world.boxes):
                    overlap = True
            i = j
            moving_s += DT_S
            still_s = 0.0
        else:
            if first_block is None:
                first_block = {"time_s": round(t, 2), "pose": [round(v, 3) for v in pose],
                               "cells": bad[:6], "cells_total": len(bad)}
            stopped_s += DT_S
            still_s += DT_S
            if still_s >= PERMANENT_STOP_S:
                break
        t += DT_S
    finished = i >= len(path) - 1
    total = stopped_s + moving_s
    return {"finished": finished, "permanent_stop": (not finished) and still_s >= PERMANENT_STOP_S,
            "stopped_fraction": round(stopped_s / total, 3) if total else None, "time_s": round(t, 2),
            "travel_m": round(i * 0.01, 2), "first_block": first_block, "overlap": overlap,
            "pass": finished and still_s < PERMANENT_STOP_S and total > 0 and stopped_s / total <= STOPPED_FRACTION_MAX}


def gate(tilt_rad: float = 0.10, robustness: bool = False, stride: int = 1) -> dict:
    out = {"tilt_rad": tilt_rad, "positive": {}, "negative": {}}
    for name, segs in PATHS.items():
        for v in (0.3, 0.6):
            out["positive"][f"{name}@{v}"] = run(segs, v, tilt_rad=tilt_rad, stride=stride)
    for name, (foot, (bx, by)) in NEGATIVES.items():
        world = World([(bx, by, EPAL[0], EPAL[1], EPAL[2])])
        r = run([(1.0, K)], 0.3, world=world, footprint=foot, tilt_rad=tilt_rad, stride=stride, max_time_s=60.0)
        out["negative"][name] = {**r, "pass": not r["overlap"]}
    out["positive_pass"] = all(r["pass"] for r in out["positive"].values())
    out["negative_pass"] = all(r["pass"] for r in out["negative"].values())
    out["gate_pass"] = out["positive_pass"] and out["negative_pass"]
    if robustness:
        rob = {}
        for name, segs in PATHS.items():
            for v in (0.3, 0.6):
                rob[f"{name}@{v}"] = sum(run(segs, v, tilt_rad=tilt_rad, pitch_rad=math.radians(1.0), dropout=0.05,
                                             seed=s, stride=stride)["finished"] for s in range(20))
        out["robustness_finished_of_20"] = rob
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--tilt-rad", type=float, default=0.10)
    ap.add_argument("--robustness", action="store_true")
    ap.add_argument("--stride", type=int, default=1, help="use every n-th pixel (1: full 640x480)")
    a = ap.parse_args(argv)
    out = gate(a.tilt_rad, a.robustness, a.stride)
    a.json.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: v for k, v in out.items() if k.endswith("pass")}))
    for name, r in {**out["positive"], **out["negative"]}.items():
        print(name, "PASS" if r["pass"] else "FAIL", "travel", r["travel_m"], "stopped", r["stopped_fraction"],
              "first block", (r["first_block"] or {}).get("pose"), (r["first_block"] or {}).get("cells", [])[:3])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
