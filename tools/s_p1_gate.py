"""The S-P1 gate (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md, "S-P1 판정 규칙", pre-registered
2026-10-27 and revised before any result): the real low layer (forklift_core.safety.low_layer) on geometric depth from
the forward camera, the truck following each path kinematically from a cold start -- no memory, no operator, no
truth-injected free space -- refusing to move whenever the footprint up to the stopping distance would enter a cell the
layer cannot vouch for at the end of braking, and braking (reaction 0.15 s, then 1.5 m/s^2) when refused while moving.

- positive: paths P1-P5 on an empty floor at 0.3 and 0.6 m/s must each finish with no permanent stop (30 s sim
  without progress) and a stopped share <= 50 % of the run;
- negative: N1-N6 (the two turning counterexamples, a floating box, a wrong floor plane, a depth dropout band, the
  camera going dark) must never let the footprint -- braking included -- overlap the hazard; the negative runs start
  with the region just ahead of the forks declared clear (``start_clear_m``) so that they move at all;
- robustness (recorded only): pitch +-1 deg and 5 % depth dropout over 20 seeds.

The generator's truth is used only to make depth and to judge overlap; the layer's plane and free-cell decisions are
never bypassed.

    python tools/s_p1_gate.py --json OUT.json [--tilt-rad 0.10] [--start-clear-m 0] [--robustness]
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
STOP_MARGIN_M = 0.05  # drive_permission's stopping distance margin
PERMANENT_STOP_S = 30.0
STOPPED_FRACTION_MAX = 0.5
PATH_STEP_M = 0.01
PLANNER_FOOT = Footprint(1.29, 0.17, 0.36)
K = 0.29
PATHS = {
    "P1": [(3.0, 0.0)],
    "P2": [(math.pi / 2 / K, K)],
    "P3": [(math.pi / 2 / K, -K)],
    "P4": [(2.0, 0.0), (math.pi / 2 / K, K), (2.0, 0.0)],
    "P5": [(3.0, 0.0), (-3.0, 0.0)],
}
EPAL = (0.6, 0.8, 0.144)
# dls08_measured forks in the rear-axle frame: root base x 0.59 -> 0.93, tip 1.29, centres +-0.18625, width 0.055,
# thickness 0.025 about z 0.04 (sim/models/dls08_measured)
FORK_BOXES = [((0.93, c - 0.0275, 0.0275), (1.29, c + 0.0275, 0.0525)) for c in (-0.18625, 0.18625)]


@dataclass
class World:
    boxes: list = field(default_factory=list)  # (cx, cy, sx, sy, z_low, z_high), axis aligned
    dropout: list = field(default_factory=list)  # (x0, x1, y0, y1): no valid depth returns from inside
    hazards: list = field(default_factory=list)  # (cx, cy, sx, sy) judged for overlap only (not rendered)
    camera_off_after_s: float | None = None


def stopping_distance_m(speed: float) -> float:
    return speed * L.REACTION_S + speed * speed / (2 * L.DECEL_MPS2) + STOP_MARGIN_M


def braking_travel_m(speed: float) -> float:
    return speed * L.REACTION_S + speed * speed / (2 * L.DECEL_MPS2)


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
                  rng=None, stride: int = 1) -> tuple:
    """The valid depth points (odometry frame) a camera on the truck at ``rear_pose`` sees, and the camera's origin:
    rays against the floor z = 0 and the boxes; depth (optical z) within 0.3-3.0 m; ``pitch_rad`` tips the camera
    (the truck's pitch); ``dropout`` removes that share of pixels at random; the world's dropout rectangles remove
    every return from inside them."""
    x, y, yaw = rear_pose
    c, s = math.cos(yaw), math.sin(yaw)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])  # positive pitch tips the nose down
    rot = rz @ ry @ camera_rotation(tilt_rad)
    cam = np.array([MOUNT_BASE_XYZ[0] + REAR_AXLE_OFFSET_M, MOUNT_BASE_XYZ[1], MOUNT_BASE_XYZ[2]])
    origin = np.array([x, y, 0.0]) + rz @ (ry @ cam)
    d = pixel_dirs(stride) @ rot.T  # odometry-frame directions; the parameter along d is the optical depth
    t = np.full(len(d), np.inf)
    down = d[:, 2] < -1e-9
    t[down] = -origin[2] / d[down, 2]
    for cx, cy, sx, sy, z0, z1 in world.boxes:
        lo = np.array([cx - sx / 2, cy - sy / 2, z0])
        hi = np.array([cx + sx / 2, cy + sy / 2, z1])
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (lo - origin) / d
            t2 = (hi - origin) / d
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0)
        t = np.where(hit & (tmin < t), tmin, t)
    # the truck's own forks (dls08_measured), in the truck frame: they shade the floor right ahead of the tips
    o_local = rz.T @ (origin - np.array([x, y, 0.0]))
    d_local = d @ rz
    for lo, hi in FORK_BOXES:
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (np.asarray(lo) - o_local) / d_local
            t2 = (np.asarray(hi) - o_local) / d_local
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0)
        t = np.where(hit & (tmin < t), tmin, t)
    valid = (t >= DEPTH_MIN_M) & (t <= DEPTH_MAX_M)
    if dropout > 0:
        valid &= (rng or np.random.default_rng(0)).random(len(t)) >= dropout
    pts = origin + d[valid] * t[valid, None]
    for x0, x1, y0, y1 in world.dropout:
        pts = pts[~((pts[:, 0] >= x0) & (pts[:, 0] <= x1) & (pts[:, 1] >= y0) & (pts[:, 1] <= y1))]
    return pts, origin


def box_overlaps(footprint: Footprint, rear_pose, rect) -> bool:
    """Whether the footprint rectangle at ``rear_pose`` overlaps the axis-aligned floor rectangle ``rect`` =
    (cx, cy, sx, sy, ...) (separating axes)."""
    x, y, yaw = rear_pose
    c, s = math.cos(yaw), math.sin(yaw)
    fp = [(x + c * a - s * b, y + s * a + c * b) for a, b in
          ((footprint.front_m, footprint.half_width_m), (footprint.front_m, -footprint.half_width_m),
           (-footprint.rear_m, -footprint.half_width_m), (-footprint.rear_m, footprint.half_width_m))]
    cx, cy, sx, sy = rect[:4]
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
        tilt_rad: float = 0.10, start_clear_m: float = 0.0, pitch_rad: float = 0.0, dropout: float = 0.0,
        seed: int = 0, stride: int = 1, max_time_s: float = 600.0) -> dict:
    """One cold-start run along ``segments``; the result by the pre-registered rule.

    The truck stops at every gear change (the envelope never runs past one, so it must be standing there), brakes along
    the path in its current gear when refused while moving (the braking travel rounded up to the path step, <= 1 cm
    conservative), and uses each depth frame one step after its capture, stamped with its capture time."""
    world = world or World()
    rng = np.random.default_rng(seed)
    layer = L.LowLayer(L.LowLayerConfig(), origin_xy=(-6.0, -6.0), size_cells=(320, 320))
    path = L.path_poses([(0.0, 0.0, 0.0)], segments, step_m=PATH_STEP_M)
    sign = []  # sign[m]: the direction of the move from pose m to pose m + 1
    for length, _ in segments:
        sign += [1 if length >= 0 else -1] * max(1, int(math.ceil(abs(length) / PATH_STEP_M)))
    sign.append(sign[-1])
    t = 0.0
    if start_clear_m > 0:  # the explicit start condition (alternative 3): the strip ahead of the forks declared clear
        layer.mark_swept_box(path[0], footprint.front_m, footprint.front_m + start_clear_m, footprint.half_width_m, t)
    hazards = [b[:4] for b in world.boxes] + list(world.hazards)
    i, moving, queue = 0, False, []
    stopped_s = moving_s = still_s = 0.0
    first_block, overlap, last_motion_t = None, False, None
    integrated = refused_planes = 0
    stop_m, step = stopping_distance_m(speed), max(1, int(round(speed * DT_S / PATH_STEP_M)))

    def gear_end(m: int) -> int:
        """The index where the move from ``m`` changes gear (or the path ends): the truck must stand there."""
        return next((q for q in range(m + 1, len(path)) if q == len(path) - 1 or sign[q] != sign[m]), len(path) - 1)

    def advance(n: int) -> None:
        nonlocal i, overlap
        end = min(i + n, gear_end(i))
        for m in range(i + 1, end + 1):
            layer.mark_swept(footprint, path[m], t)
            overlap = overlap or any(box_overlaps(footprint, path[m], h) for h in hazards)
        i = end

    while i < len(path) - 1 and t < max_time_s:
        layer.mark_swept(footprint, path[i], t)  # the body stands here now
        layer.prune(t)
        if world.camera_off_after_s is None or t <= world.camera_off_after_s:
            pts, origin = render_points(world, path[i], tilt_rad, rng=rng, stride=stride, dropout=dropout,
                                        pitch_rad=pitch_rad * (1 if rng.random() < 0.5 else -1) if pitch_rad else 0.0)
            queue.append((t, pts, origin, path[i]))
        while queue and (len(queue) > LATENCY_FRAMES or queue[0][0] < t - LATENCY_FRAMES * DT_S + 1e-9):
            t_cap, pts, origin, pose = queue.pop(0)  # used a step after capture, stamped with the capture time
            res = layer.integrate(pts, t_cap, origin=origin, self_mask=(PLANNER_FOOT, pose))
            integrated += 1
            refused_planes += not res.ok
        stop_at = gear_end(i)
        j = min(i + step, stop_at)
        k = min(j + int(math.ceil(stop_m / PATH_STEP_M)), stop_at)  # the stop distance never runs past a gear change
        t_check = t + DT_S + L.braking_end_s(speed)
        ok, bad = L.envelope_admissible(layer, t, path[i:k + 1], footprint, reverse=sign[i] < 0, t_check=t_check)
        if ok:
            advance(step)
            moving_s += DT_S
            still_s, last_motion_t = 0.0, t
            moving = i < stop_at  # arriving at a gear change (or the end) is a planned stop
            t += DT_S
            continue
        if first_block is None:
            first_block = {"time_s": round(t, 2), "pose": [round(v, 3) for v in path[i]], "points": bad[:6],
                           "points_total": len(bad)}
        if moving:  # refused while moving: the truck brakes along the path in this gear, the whole travel judged
            advance(int(math.ceil(braking_travel_m(speed) / PATH_STEP_M)))
            dt_brake = L.braking_end_s(speed)
            moving_s += dt_brake
            t += dt_brake
            last_motion_t = t
            moving = False
            continue
        stopped_s += DT_S
        still_s += DT_S
        t += DT_S
        if still_s >= PERMANENT_STOP_S:
            break
    finished = i >= len(path) - 1
    total = stopped_s + moving_s
    share = stopped_s / total if total else 1.0
    return {"finished": finished, "permanent_stop": (not finished) and still_s >= PERMANENT_STOP_S,
            "stopped_fraction": round(share, 3), "time_s": round(t, 2), "travel_m": round(i * PATH_STEP_M, 2),
            "first_block": first_block, "overlap": overlap, "frames_integrated": integrated,
            "frames_plane_refused": refused_planes,
            "last_motion_s": None if last_motion_t is None else round(last_motion_t, 2),
            "pass": finished and still_s < PERMANENT_STOP_S and share <= STOPPED_FRACTION_MAX}


def negatives(tilt_rad: float, stride: int) -> dict:
    """N1-N6: no run may overlap its hazard, braking included; N3, N5 and N6 must also have moved (else the feature
    under test was never exercised), N4 must have refused every frame's floor plane, N6 must stand still by the time
    the last frame it saw (captured at 0.5 s) runs out (3 s)."""
    out = {}
    turn = [(1.0, K)]
    for name, foot, (bx, by) in (("N1", Footprint(0.959, 0.17, 0.315), (0.25, 0.75)),
                                 ("N2", Footprint(0.944, 0.17, 0.36), (0.25, 0.795))):
        world = World([(bx, by, EPAL[0], EPAL[1], 0.0, EPAL[2])])
        r = run(turn, 0.3, world=world, footprint=foot, tilt_rad=tilt_rad, start_clear_m=0.3, stride=stride,
                max_time_s=60.0)
        out[name] = {**r, "pass": not r["overlap"]}
    r = run([(2.0, 0.0)], 0.3, world=World([(1.9, 0.0, 0.4, 0.8, 0.05, 0.12)]), tilt_rad=tilt_rad,
            start_clear_m=0.3, stride=stride, max_time_s=60.0)
    out["N3"] = {**r, "pass": not r["overlap"] and r["travel_m"] > 0}
    r = run([(1.0, 0.0)], 0.3, world=World([(3.15, 0.0, 3.7, 6.0, 0.0, EPAL[2])]), tilt_rad=tilt_rad, stride=stride,
            max_time_s=60.0)  # a pallet-height plane from 1.30 m: no floor in view
    out["N4"] = {**r, "pass": (not r["overlap"]) and r["frames_integrated"] > 0
                 and r["frames_plane_refused"] == r["frames_integrated"]}
    r = run([(2.0, 0.0)], 0.3, world=World(dropout=[(1.9, 2.6, -1.0, 1.0)], hazards=[(2.25, 0.0, 0.7, 2.0)]),
            tilt_rad=tilt_rad, start_clear_m=0.3, stride=stride, max_time_s=60.0)
    out["N5"] = {**r, "pass": not r["overlap"] and r["travel_m"] > 0}
    r = run([(3.0, 0.0)], 0.3, world=World(camera_off_after_s=0.5), tilt_rad=tilt_rad, start_clear_m=0.3,
            stride=stride, max_time_s=60.0)
    deadline = 0.5 + L.LowLayerConfig().max_age_s
    moved_late = r["last_motion_s"] is not None and r["last_motion_s"] > deadline
    out["N6"] = {**r, "deadline_s": deadline,
                 "pass": r["travel_m"] > 0 and not r["finished"] and not moved_late}
    return out


def gate(tilt_rad: float = 0.10, start_clear_m: float = 0.0, robustness: bool = False, stride: int = 1) -> dict:
    out = {"tilt_rad": tilt_rad, "start_clear_m": start_clear_m, "positive": {}, "negative": negatives(tilt_rad, stride)}
    for name, segs in PATHS.items():
        for v in (0.3, 0.6):
            out["positive"][f"{name}@{v}"] = run(segs, v, tilt_rad=tilt_rad, start_clear_m=start_clear_m, stride=stride)
    out["positive_pass"] = all(r["pass"] for r in out["positive"].values())
    out["negative_pass"] = all(r["pass"] for r in out["negative"].values())
    out["negative_moved"] = {k: r["travel_m"] for k, r in out["negative"].items()}
    out["gate_pass"] = out["positive_pass"] and out["negative_pass"]
    if robustness:
        out["robustness_finished_of_20"] = {
            f"{name}@{v}": sum(run(segs, v, tilt_rad=tilt_rad, start_clear_m=start_clear_m,
                                   pitch_rad=math.radians(1.0), dropout=0.05, seed=s, stride=stride)["finished"]
                               for s in range(20))
            for name, segs in PATHS.items() for v in (0.3, 0.6)}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--tilt-rad", type=float, default=0.10)
    ap.add_argument("--start-clear-m", type=float, default=0.0, help="alternative 3: the strip ahead declared clear")
    ap.add_argument("--robustness", action="store_true")
    ap.add_argument("--stride", type=int, default=1, help="use every n-th pixel (1: full 640x480)")
    a = ap.parse_args(argv)
    out = gate(a.tilt_rad, a.start_clear_m, a.robustness, a.stride)
    a.json.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: v for k, v in out.items() if k.endswith("pass") or k == "negative_moved"}))
    for name, r in {**out["positive"], **out["negative"]}.items():
        print(name, "PASS" if r["pass"] else "FAIL", "travel", r["travel_m"], "stopped", r["stopped_fraction"],
              "first block", (r["first_block"] or {}).get("pose"), (r["first_block"] or {}).get("cells", [])[:3])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
