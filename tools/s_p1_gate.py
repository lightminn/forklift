"""The S-P1 gate (docs/plans/2026-10-27-autonomous-survey-and-relocalization.md, "S-P1 판정 규칙", pre-registered
2026-10-27 and revised before any result): the real low layer (forklift_core.safety.low_layer) on geometric depth from
the forward camera, the truck following each path kinematically from a cold start -- no memory, no operator, no
truth-injected free space -- refusing to move whenever the footprint up to the stopping distance would enter a cell the
layer cannot vouch for at the end of braking, and braking (reaction 0.15 s, then 1.5 m/s^2) when refused while moving.

- positive: paths P1-P5 on an empty floor at 0.3 and 0.6 m/s must each finish with no permanent stop (30 s sim
  without progress) and a stopped share <= 50 % of the run;
- negative: N1-N6 (the two turning counterexamples, a floating box, a wrong floor plane, a depth dropout band, the
  camera going dark) must never let the footprint -- braking included -- overlap the hazard; the negative runs start
  with a strip just ahead of the forks declared clear (``AHEAD_STRIP``) so that they move at all;
- robustness (recorded only): pitch +-1 deg and 5 % depth dropout over 20 seeds.

The generator's truth is used only to make depth and to judge overlap; the layer's plane and free-cell decisions are
never bypassed.

    python tools/s_p1_gate.py --json OUT.json [--tilt-rad 0.10] [--start-clear-m 0] [--robustness] [--jobs N]
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

import scene_rig  # noqa: E402

from forklift_core.planning.geometry import Footprint  # noqa: E402
from forklift_core.safety import low_layer as L  # noqa: E402

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


def sweep_overlaps(footprint: Footprint, poses, rects) -> bool:
    """Whether the footprint's continuous sweep along ``poses`` overlaps any axis-aligned floor rectangle
    (cx, cy, sx, sy, ...): the sweep between neighbouring poses as the low layer bounds it (hull of the two rectangles
    grown by the arc sagitta, a superset), so a hazard between the 1 cm samples is not missed."""
    if not rects:
        return False
    pieces = L._sweep_pieces(list(poses), -footprint.rear_m, footprint.front_m, footprint.half_width_m)
    lo = np.array([[r[0] - r[2] / 2, r[1] - r[3] / 2] for r in rects])
    hi = np.array([[r[0] + r[2] / 2, r[1] + r[3] / 2] for r in rects])
    return any(bool(L._squares_touch(lo, hi, hull, grow).any()) for hull, grow in pieces)


def approach_speed(remaining_m: float, speed: float) -> float:
    """The speed for the next step: the set speed, or less when the truck must stand at a gear change (or the end)
    ``remaining_m`` ahead -- one step at it plus the braking travel from it fits in what remains, so a refusal at any
    point of the approach still stops the truck before the gear change."""
    if remaining_m <= 0.0:
        return 0.0
    lead = DT_S + L.REACTION_S
    a = L.DECEL_MPS2
    v = a * (-lead + math.sqrt(lead * lead + 2.0 * remaining_m / a))  # v * lead + v^2 / (2a) = remaining
    return min(speed, max(0.0, v))


def pose_at(path, s: float) -> tuple:
    """The rear-axle pose ``s`` metres along the path (linear between its 1 cm samples)."""
    q = s / PATH_STEP_M
    m = min(int(math.floor(q + 1e-9)), len(path) - 1)
    if m >= len(path) - 1:
        return tuple(path[-1])
    w = q - m
    a, b = path[m], path[m + 1]
    return tuple(a[k] + w * (b[k] - a[k]) for k in range(3))


def run(segments, speed: float, *, world: World | None = None, footprint: Footprint = PLANNER_FOOT,
        tilt_rad: float = 0.10, start_zone: tuple | None = None, pitch_rad: float = 0.0, dropout: float = 0.0,
        seed: int = 0, stride: int = 1, max_time_s: float = 600.0) -> dict:
    """One cold-start run along ``segments``; the result by the pre-registered rule.

    The truck slows ahead of every gear change (``approach_speed``) and stands there, so the braking travel from its
    current speed always ends before it; when refused while moving it brakes along the path in its current gear and
    the whole travel is judged. The envelope runs from the current pose to the stopping distance (with
    drive_permission's 0.05 m margin), cut at a gear change only -- the truck cannot pass one. Depth frames are used
    one step after their capture, stamped with their capture time. ``start_zone`` = (x0, x1, half width) in the start
    pose's truck frame: a zone declared clear at the start (the explicit start condition)."""
    world = world or World()
    rng = np.random.default_rng(seed)
    layer = L.LowLayer(L.LowLayerConfig(), origin_xy=(-6.0, -6.0), size_cells=(320, 320))
    path = L.path_poses([(0.0, 0.0, 0.0)], segments, step_m=PATH_STEP_M)
    sign = []  # sign[m]: the direction of the move from pose m to pose m + 1
    for length, _ in segments:
        sign += [1 if length >= 0 else -1] * max(1, int(math.ceil(abs(length) / PATH_STEP_M)))
    sign.append(sign[-1])
    s_end = (len(path) - 1) * PATH_STEP_M
    t = 0.0
    if start_zone is not None:
        layer.mark_swept_box(path[0], *start_zone, t)
    hazards = [b[:4] for b in world.boxes] + list(world.hazards)
    s, v, queue = 0.0, 0.0, []
    stopped_s = moving_s = still_s = 0.0
    first_block, overlap, last_motion_t = None, False, None
    integrated = refused_planes = 0

    def gear_stop(at: float) -> float:
        """Where the move from ``at`` changes gear (or the path ends): the truck must stand there."""
        m = min(int(math.floor(at / PATH_STEP_M + 1e-9)), len(path) - 1)
        q = next((q for q in range(m + 1, len(path)) if q == len(path) - 1 or sign[q] != sign[m]), len(path) - 1)
        return q * PATH_STEP_M

    def advance(to: float) -> None:
        nonlocal s, overlap
        poses = [path[m] for m in range(int(math.floor(s / PATH_STEP_M)) + 1, int(math.ceil(to / PATH_STEP_M - 1e-9)))]
        poses.append(pose_at(path, to))
        for p in poses:
            layer.mark_swept(footprint, p, t)
        overlap = overlap or sweep_overlaps(footprint, [pose_at(path, s)] + poses, hazards)
        s = to

    def poses_between(a: float, b: float) -> list:
        inner = [path[m] for m in range(int(math.floor(a / PATH_STEP_M + 1e-9)) + 1,
                                        int(math.ceil(b / PATH_STEP_M - 1e-9)))]
        return [pose_at(path, a)] + inner + [pose_at(path, b)]

    while s < s_end - 1e-9 and t < max_time_s:
        here = pose_at(path, s)
        layer.mark_swept(footprint, here, t)  # the body stands here now
        layer.prune(t)
        if world.camera_off_after_s is None or t <= world.camera_off_after_s:
            pts, origin = render_points(world, here, tilt_rad, rng=rng, stride=stride, dropout=dropout,
                                        pitch_rad=pitch_rad * (1 if rng.random() < 0.5 else -1) if pitch_rad else 0.0)
            queue.append((t, pts, origin, here))
        while queue and (len(queue) > LATENCY_FRAMES or queue[0][0] < t - LATENCY_FRAMES * DT_S + 1e-9):
            t_cap, pts, origin, pose = queue.pop(0)  # used a step after capture, stamped with the capture time
            res = layer.integrate(pts, t_cap, origin=origin, self_mask=(PLANNER_FOOT, pose))
            integrated += 1
            refused_planes += not res.ok
        stop_at = gear_stop(s)
        v_next = approach_speed(stop_at - s, speed)
        reach = min(s + v_next * DT_S + stopping_distance_m(v_next), stop_at)
        t_check = t + DT_S + L.braking_end_s(v_next)
        rev = sign[min(int(math.floor(s / PATH_STEP_M + 1e-9)), len(path) - 1)] < 0
        ok, bad = L.envelope_admissible(layer, t, poses_between(s, reach), footprint, reverse=rev, t_check=t_check)
        if ok:
            to = s + v_next * DT_S
            if stop_at - to < 1e-3:  # within a millimetre of the gear change at a crawl: it stands there
                to = stop_at
            progressed = to - s >= PATH_STEP_M - 1e-9
            advance(to)
            moving_s += DT_S
            v = 0.0 if s >= stop_at - 1e-9 else v_next
            if progressed:
                still_s = 0.0
            else:
                still_s += DT_S
            last_motion_t = t
            t += DT_S
            if still_s >= PERMANENT_STOP_S:
                break
            continue
        if first_block is None:
            first_block = {"time_s": round(t, 2), "pose": [round(c, 3) for c in here], "points": bad[:6]}
        if v > 0.0:  # refused while moving: the truck brakes along the path in this gear, the whole travel judged
            travel = braking_travel_m(v)
            assert s + travel <= stop_at + 1e-9, "the approach speed lets the braking travel pass a gear change"
            advance(s + travel)
            dt_brake = L.braking_end_s(v)
            moving_s += dt_brake
            t += dt_brake
            last_motion_t = t
            v = 0.0
            continue
        stopped_s += DT_S
        still_s += DT_S
        t += DT_S
        if still_s >= PERMANENT_STOP_S:
            break
    finished = s >= s_end - 1e-9
    total = stopped_s + moving_s
    share = stopped_s / total if total else 1.0
    return {"finished": finished, "permanent_stop": (not finished) and still_s >= PERMANENT_STOP_S,
            "stopped_fraction": round(share, 3), "time_s": round(t, 2), "travel_m": round(s, 3),
            "first_block": first_block, "overlap": overlap, "frames_integrated": integrated,
            "frames_plane_refused": refused_planes,
            "last_motion_s": None if last_motion_t is None else round(last_motion_t, 2),
            "pass": finished and still_s < PERMANENT_STOP_S and share <= STOPPED_FRACTION_MAX}


def start_zone_around(footprint: Footprint, radius_m: float) -> tuple:
    """Alternative 3's start condition: the body grown by ``radius_m`` declared clear."""
    return (-footprint.rear_m - radius_m, footprint.front_m + radius_m, footprint.half_width_m + radius_m)


# the negatives' strip ahead of the forks, declared clear so that they move at all (it starts inside the body so the
# shrunk strip still meets the body, and keeps clear of every negative's hazard)
AHEAD_STRIP = (PLANNER_FOOT.front_m - 0.2, PLANNER_FOOT.front_m + 0.3, PLANNER_FOOT.half_width_m + 0.05)


def negative_specs(tilt_rad: float, stride: int) -> dict:
    """N1-N6 as run arguments: the two turning counterexamples, a floating box, a pallet-height plane filling the view
    from 1.30 m (no floor in view), a depth dropout band over a hazard, the camera going dark at 0.5 s."""
    common = {"tilt_rad": tilt_rad, "stride": stride, "max_time_s": 60.0}
    turn = [(1.0, K)]
    specs = {}
    for name, foot, (bx, by) in (("N1", Footprint(0.959, 0.17, 0.315), (0.25, 0.75)),
                                 ("N2", Footprint(0.944, 0.17, 0.36), (0.25, 0.795))):
        specs[name] = (turn, 0.3, {**common, "footprint": foot, "start_zone": AHEAD_STRIP,
                                   "world": World([(bx, by, EPAL[0], EPAL[1], 0.0, EPAL[2])])})
    specs["N3"] = ([(2.0, 0.0)], 0.3, {**common, "start_zone": AHEAD_STRIP,
                                       "world": World([(1.9, 0.0, 0.4, 0.8, 0.05, 0.12)])})
    specs["N4"] = ([(1.0, 0.0)], 0.3, {**common, "world": World([(3.15, 0.0, 3.7, 6.0, 0.0, EPAL[2])])})
    specs["N5"] = ([(2.0, 0.0)], 0.3, {**common, "start_zone": AHEAD_STRIP,
                                       "world": World(dropout=[(1.9, 2.6, -1.0, 1.0)],
                                                      hazards=[(2.25, 0.0, 0.7, 2.0)])})
    specs["N6"] = ([(3.0, 0.0)], 0.3, {**common, "start_zone": AHEAD_STRIP, "world": World(camera_off_after_s=0.5)})
    return specs


def judge_negative(name: str, r: dict) -> dict:
    """No run may overlap its hazard, braking included; N3, N5 and N6 must also have moved (else the feature under
    test was never exercised), N4 must have refused every frame's floor plane, N6 must stand still by the time the
    last frame it saw (captured at 0.5 s) runs out (3 s)."""
    if name in ("N1", "N2"):
        return {**r, "pass": not r["overlap"]}
    if name in ("N3", "N5"):
        return {**r, "pass": not r["overlap"] and r["travel_m"] > 0}
    if name == "N4":
        return {**r, "pass": (not r["overlap"]) and r["frames_integrated"] > 0
                and r["frames_plane_refused"] == r["frames_integrated"]}
    deadline = 0.5 + L.LowLayerConfig().max_age_s
    moved_late = r["last_motion_s"] is not None and r["last_motion_s"] > deadline
    return {**r, "deadline_s": deadline, "pass": r["travel_m"] > 0 and not r["finished"] and not moved_late}


def _run_spec(spec) -> dict:
    segments, speed, kwargs = spec
    return run(segments, speed, **kwargs)


def gate(tilt_rad: float = 0.10, start_clear_m: float = 0.0, robustness: bool = False, stride: int = 1,
         jobs: int = 1) -> dict:
    zone = start_zone_around(PLANNER_FOOT, start_clear_m) if start_clear_m > 0 else None
    specs = {f"neg:{k}": v for k, v in negative_specs(tilt_rad, stride).items()}
    for name, segs in PATHS.items():
        for v in (0.3, 0.6):
            specs[f"pos:{name}@{v}"] = (segs, v, {"tilt_rad": tilt_rad, "start_zone": zone, "stride": stride})
            if robustness:
                for s in range(20):
                    specs[f"rob:{name}@{v}:{s}"] = (segs, v, {"tilt_rad": tilt_rad, "start_zone": zone,
                                                              "stride": stride, "pitch_rad": math.radians(1.0),
                                                              "dropout": 0.05, "seed": s})
    keys = list(specs)
    if jobs > 1:
        from concurrent.futures import ProcessPoolExecutor  # noqa: PLC0415

        with ProcessPoolExecutor(jobs) as pool:
            results = dict(zip(keys, pool.map(_run_spec, [specs[k] for k in keys]), strict=True))
    else:
        results = {k: _run_spec(specs[k]) for k in keys}
    out = {"tilt_rad": tilt_rad, "start_clear_m": start_clear_m, "start_zone": zone,
           "positive": {k[4:]: results[k] for k in keys if k.startswith("pos:")},
           "negative": {k[4:]: judge_negative(k[4:], results[k]) for k in keys if k.startswith("neg:")}}
    out["positive_pass"] = all(r["pass"] for r in out["positive"].values())
    out["negative_pass"] = all(r["pass"] for r in out["negative"].values())
    out["negative_moved"] = {k: r["travel_m"] for k, r in out["negative"].items()}
    out["gate_pass"] = out["positive_pass"] and out["negative_pass"]
    if robustness:
        out["robustness_finished_of_20"] = {
            f"{name}@{v}": sum(results[f"rob:{name}@{v}:{s}"]["finished"] for s in range(20))
            for name in PATHS for v in (0.3, 0.6)}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--tilt-rad", type=float, default=0.10)
    ap.add_argument("--start-clear-m", type=float, default=0.0,
                    help="alternative 3: the body grown by this radius declared clear at the start")
    ap.add_argument("--robustness", action="store_true")
    ap.add_argument("--stride", type=int, default=1, help="use every n-th pixel (1: full 640x480)")
    ap.add_argument("--jobs", type=int, default=1, help="runs in parallel processes")
    a = ap.parse_args(argv)
    out = gate(a.tilt_rad, a.start_clear_m, a.robustness, a.stride, a.jobs)
    a.json.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({k: v for k, v in out.items() if k.endswith("pass") or k == "negative_moved"}))
    for name, r in {**out["positive"], **out["negative"]}.items():
        print(name, "PASS" if r["pass"] else "FAIL", "travel", r["travel_m"], "stopped", r["stopped_fraction"],
              "first block", (r["first_block"] or {}).get("pose"), (r["first_block"] or {}).get("points", [])[:3])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
