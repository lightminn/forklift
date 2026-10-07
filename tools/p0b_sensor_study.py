"""P0b obstacle-sensor study on CPU (priority-5 plan, docs/plans/2026-10-04-lidar-obstacle-map.md).

    python tools/p0b_sensor_study.py sections --run <run dir> --assets <dir> --output sections.npz
    python tools/p0b_sensor_study.py evaluate --run <run dir> --sections sections.npz \
        --candidates config/p0b_candidates.yaml --stop-latency-s .. --stop-decel .. --output result.json

The truck body tilts by about 1e-6 rad in these runs, so a planar LiDAR beam
stays at its mount height: casting it is a 2D ray against the horizontal
cross-section of every collider at that height. ``sections`` cuts the run's
planner obstacles (the Simple_Warehouse prop meshes, triangle colliders with
approximation "none" -- the same geometry PhysX casts against) at each listed
height. ``evaluate`` replays the recorded 10 Hz scan instants: the truck's
own collision shapes (URDF boxes and wheel cylinders at the recorded lift and
steering) and the carried pallet are cut at the same height and block beams
as self hits; the pickup pallet on the floor is an external obstacle. Each
candidate's scans go through the obstacle grid and the drive permission
(forklift_core) with wheel odometry redrawn under the online noise model, and
the result is checked against the ground-truth obstacles.

Reported per candidate (plan D1): unpermitted entries (a truth obstacle in
the stopping volume while the permission let the truck drive at the recorded
speed -- must be 0), events (stopping volume meets an obstacle), the
permission ratio per motion class (the share of moving instants whose allowed
speed is at least the recorded one), and per height the beam pass-through of
each obstacle class (D0's floor-standing assumption).
"""

from __future__ import annotations

import argparse
import json
import math
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ASSET_ROOT = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/Environments/Simple_Warehouse/Props/"
)


# ---------------------------------------------------------------- geometry

def section_segments(triangles: np.ndarray, z: float) -> np.ndarray:
    """(M, 2, 2) segments where the plane at height z cuts the (K, 3, 3) triangles."""
    if not len(triangles):
        return np.zeros((0, 2, 2))
    d = triangles[:, :, 2] - z
    out = []
    for a, b in ((0, 1), (1, 2), (2, 0)):
        cross = (d[:, a] < 0) != (d[:, b] < 0)
        t = np.where(cross, d[:, a] / np.where(cross, d[:, a] - d[:, b], 1.0), 0.0)
        p = triangles[:, a, :2] + t[:, None] * (triangles[:, b, :2] - triangles[:, a, :2])
        out.append((cross, p))
    segs = []
    masks = np.stack([c for c, _ in out], axis=1)
    pts = np.stack([p for _, p in out], axis=1)
    two = masks.sum(1) == 2
    for k in np.nonzero(two)[0]:
        segs.append(pts[k][masks[k]])
    return np.asarray(segs).reshape(-1, 2, 2)


def box_triangles(center, half, rotation=np.eye(3)) -> np.ndarray:
    sx, sy, sz = half
    corners = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    faces = [(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5), (0, 4, 5), (0, 5, 1),
             (2, 3, 7), (2, 7, 6), (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)]
    world = corners @ np.asarray(rotation).T + center
    return world[np.array(faces)]


def cylinder_triangles(center, radius, length, rotation=np.eye(3), n=32) -> np.ndarray:
    """Closed cylinder with its axis along local z."""
    a = np.linspace(0, 2 * math.pi, n, endpoint=False)
    ring = np.column_stack((radius * np.cos(a), radius * np.sin(a)))
    top = np.column_stack((ring, np.full(n, length / 2)))
    bottom = np.column_stack((ring, np.full(n, -length / 2)))
    tris = []
    for i in range(n):
        j = (i + 1) % n
        tris += [(bottom[i], bottom[j], top[j]), (bottom[i], top[j], top[i])]
        tris += [((0, 0, length / 2), top[i], top[j]), ((0, 0, -length / 2), bottom[j], bottom[i])]
    t = np.asarray(tris, dtype=float)
    return t @ np.asarray(rotation).T + center


def rpy_matrix(roll, pitch, yaw) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = map(lambda f: f, (math.cos(roll), math.sin(roll), math.cos(pitch),
                                               math.sin(pitch), math.cos(yaw), math.sin(yaw)))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _origin(element):
    if element is None:
        return np.zeros(3), np.eye(3)
    xyz = np.array([float(v) for v in element.get("xyz", "0 0 0").split()])
    rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
    return xyz, rpy_matrix(*rpy)


def urdf_triangles(urdf: Path, joint_values: dict) -> dict:
    """Collision triangles per link in the base_link frame at the given joint values."""
    root = ET.parse(urdf).getroot()
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    links = {l.get("name"): l for l in root.findall("link")}
    roots = [name for name in links if name not in joints]
    cache = {name: (np.zeros(3), np.eye(3)) for name in roots}

    def pose(link):
        if link in cache:
            return cache[link]
        joint = joints[link]
        p_pos, p_rot = pose(joint.find("parent").get("link"))
        o_pos, o_rot = _origin(joint.find("origin"))
        rot = p_rot @ o_rot
        pos = p_pos + p_rot @ o_pos
        axis_el = joint.find("axis")
        axis = np.array([float(v) for v in axis_el.get("xyz").split()]) if axis_el is not None else np.array([1.0, 0, 0])
        q = float(joint_values.get(joint.get("name"), 0.0))
        if joint.get("type") == "prismatic":
            pos = pos + rot @ (axis * q)
        elif joint.get("type") in ("revolute", "continuous") and q:
            k = axis / np.linalg.norm(axis)
            K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
            rot = rot @ (np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K)
        cache[link] = (pos, rot)
        return cache[link]

    out = {}
    for name, link in links.items():
        tris = []
        for collision in link.findall("collision"):
            lpos, lrot = pose(name)
            o_pos, o_rot = _origin(collision.find("origin"))
            center = lpos + lrot @ o_pos
            rot = lrot @ o_rot
            box = collision.find("geometry/box")
            cyl = collision.find("geometry/cylinder")
            if box is not None:
                half = np.array([float(v) for v in box.get("size").split()]) / 2
                tris.append(box_triangles(center, half, rot))
            elif cyl is not None:
                tris.append(cylinder_triangles(center, float(cyl.get("radius")), float(cyl.get("length")), rot))
        if tris:
            out[name] = np.concatenate(tris)
    return out


def cast_rays_2d(origin, angles, segments, range_max):
    """Nearest hit distance per ray against (M, 2, 2) segments; inf when none, and the segment index."""
    n = len(angles)
    best = np.full(n, np.inf)
    which = np.full(n, -1)
    if not len(segments):
        return best, which
    a = segments[:, 0, :] - origin
    e = segments[:, 1, :] - segments[:, 0, :]
    keep = np.minimum(np.hypot(*a.T), np.hypot(*(a + e).T)) <= range_max + np.hypot(*e.T)
    a, e = a[keep], e[keep]
    idx = np.nonzero(keep)[0]
    if not len(idx):
        return best, which
    d = np.column_stack((np.cos(angles), np.sin(angles)))
    for start in range(0, n, 256):
        dd = d[start : start + 256]
        denom = dd[:, None, 0] * e[None, :, 1] - dd[:, None, 1] * e[None, :, 0]
        with np.errstate(divide="ignore", invalid="ignore"):
            t = (a[None, :, 0] * e[None, :, 1] - a[None, :, 1] * e[None, :, 0]) / denom
            u = (a[None, :, 0] * dd[:, None, 1] - a[None, :, 1] * dd[:, None, 0]) / denom
        ok = (np.abs(denom) > 1e-12) & (t > 1e-9) & (u >= 0) & (u <= 1) & (t <= range_max)
        t = np.where(ok, t, np.inf)
        k = np.argmin(t, axis=1)
        best[start : start + 256] = t[np.arange(len(dd)), k]
        hit = np.isfinite(best[start : start + 256])
        which[start : start + 256] = np.where(hit, idx[k], -1)
    return best, which


def transform_segments(segments, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return segments @ rot.T + [x, y]


# ---------------------------------------------------------------- sections

def usd_triangles(path: Path) -> np.ndarray:
    from pxr import Usd, UsdGeom

    stage = Usd.Stage.Open(str(path))
    cache = UsdGeom.XformCache()
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    out = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        pts = np.array(mesh.GetPointsAttr().Get(), dtype=float)
        xf = np.array(cache.GetLocalToWorldTransform(prim))
        world = (np.c_[pts, np.ones(len(pts))] @ xf)[:, :3] * mpu
        counts = np.array(mesh.GetFaceVertexCountsAttr().Get())
        idx = np.array(mesh.GetFaceVertexIndicesAttr().Get())
        k = 0
        for n in counts:
            face = idx[k : k + n]
            k += n
            for t in range(1, n - 1):
                out.append(world[[face[0], face[t], face[t + 1]]])
    return np.asarray(out)


def scene_props(scene_usda: Path):
    """(asset filename, 4x4 world transform, prim path) for every prop referenced in the run's scene."""
    from pxr import Sdf, Usd, UsdGeom

    layer = Sdf.Layer.FindOrOpen(str(scene_usda))
    stage = Usd.Stage.Open(layer, load=Usd.Stage.LoadNone)
    cache = UsdGeom.XformCache()
    out = []
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not (path.startswith("/World/Factory") or path.startswith("/World/RandomProps")):
            continue
        spec_refs = []
        for spec in prim.GetPrimStack():
            spec_refs += [r.assetPath for r in spec.referenceList.GetAddedOrExplicitItems()]
        props = [r for r in spec_refs if "Simple_Warehouse/Props/" in r]
        if props:
            out.append((props[0].rsplit("/", 1)[-1], np.array(cache.GetLocalToWorldTransform(prim)), path))
    return out


def clip_triangle_z(tri: np.ndarray, z_min: float) -> list:
    """The part of a 3D triangle with z >= z_min, as 0-2 triangles (Sutherland-Hodgman)."""
    if z_min <= -np.inf or tri[:, 2].min() >= z_min:
        return [tri]
    poly = []
    for i in range(3):
        a, b = tri[i], tri[(i + 1) % 3]
        a_in, b_in = a[2] >= z_min, b[2] >= z_min
        if a_in:
            poly.append(a)
        if a_in != b_in:
            t = (z_min - a[2]) / (b[2] - a[2])
            poly.append(a + t * (b - a))
    if len(poly) < 3:
        return []
    return [np.array([poly[0], poly[i], poly[i + 1]]) for i in range(1, len(poly) - 1)]


def triangle_cells(tri2d: np.ndarray, res: float) -> np.ndarray:
    """Cells (i, j) on a world grid of size res (origin 0) whose square overlaps a 2D triangle (separating axes)."""
    lo = np.floor(tri2d.min(axis=0) / res).astype(int)
    hi = np.floor(tri2d.max(axis=0) / res).astype(int)
    ii, jj = np.meshgrid(np.arange(lo[0], hi[0] + 1), np.arange(lo[1], hi[1] + 1), indexing="ij")
    ii, jj = ii.ravel(), jj.ravel()
    cx, cy = (ii + 0.5) * res, (jj + 0.5) * res
    half = res / 2
    keep = np.ones(len(ii), dtype=bool)
    for k in range(3):
        a, b = tri2d[k], tri2d[(k + 1) % 3]
        n = np.array([b[1] - a[1], a[0] - b[0]])
        norm = np.hypot(*n)
        if norm < 1e-12:
            continue
        n = n / norm
        tri_proj = tri2d @ n
        centre = cx * n[0] + cy * n[1]
        extent = half * (abs(n[0]) + abs(n[1]))
        keep &= (centre + extent >= tri_proj.min() - 1e-12) & (centre - extent <= tri_proj.max() + 1e-12)
    return np.column_stack((ii[keep], jj[keep]))


def sections_command(args) -> dict:
    args.assets.mkdir(parents=True, exist_ok=True)
    props = scene_props(args.run / "scene.usda")
    names = sorted({p[0] for p in props})
    local = {}
    for name in names:
        target = args.assets / name
        if not target.exists():
            urllib.request.urlretrieve(ASSET_ROOT + name, target)
        local[name] = usd_triangles(target)
    heights = [float(h) for h in args.heights.split(",")]
    owners, kinds, seg_by_h = [], [], {h: [] for h in heights}
    owner_of = {h: [] for h in heights}
    for k, (name, xf, path) in enumerate(props):
        tris = local[name]
        world = (np.c_[tris.reshape(-1, 3), np.ones(len(tris) * 3)] @ xf)[:, :3].reshape(-1, 3, 3)
        owners.append(path)
        kinds.append(name)
        for h in heights:
            segs = section_segments(world, h)
            seg_by_h[h].append(segs)
            owner_of[h].append(np.full(len(segs), k))
    payload = {"heights": np.array(heights), "owners": np.array(owners), "kinds": np.array(kinds)}
    # Floor projection of every collider triangle in the truck's height band
    # (Codex checkpoint P2): conservative, no gaps between sections.
    proj = {}
    for k, (name, xf, path) in enumerate(props):
        tris = local[name]
        world = (np.c_[tris.reshape(-1, 3), np.ones(len(tris) * 3)] @ xf)[:, :3].reshape(-1, 3, 3)
        zlo, zhi = world[:, :, 2].min(axis=1), world[:, :, 2].max(axis=1)
        for tri in world[(zhi >= args.projection_bottom_m) & (zlo <= args.projection_top_m)]:
            # Only the part at or above projection_bottom_m: a protrusion lower
            # than that (a cone's base plate under 0.03 m) is outside the
            # operating scope (D0, user decision 2026-10-05).
            for part in clip_triangle_z(tri, args.projection_bottom_m):
                for c in triangle_cells(part[:, :2], 0.05):
                    proj[(int(c[0]), int(c[1]))] = k
    payload["proj_cells"] = np.array(list(proj.keys()), dtype=int).reshape(-1, 2)
    payload["proj_owner"] = np.array(list(proj.values()), dtype=int)
    for h in heights:
        payload[f"seg_{h:.3f}"] = np.concatenate(seg_by_h[h]) if seg_by_h[h] else np.zeros((0, 2, 2))
        payload[f"own_{h:.3f}"] = np.concatenate(owner_of[h]) if owner_of[h] else np.zeros(0, int)
    np.savez_compressed(args.output, **payload)
    return {"props": len(props), "assets": names, "heights": heights,
            "segments": {f"{h:.3f}": int(len(payload[f'seg_{h:.3f}'])) for h in heights}}


# ---------------------------------------------------------------- evaluate

def _yaw(q_wxyz):
    w, x, y, z = np.asarray(q_wxyz, float).T
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def load_candidates(path: Path) -> dict:
    import yaml

    data = yaml.safe_load(path.read_text())
    out = {}
    for name, spec in data["candidates"].items():
        out[name] = {
            "sensors": [
                {"name": m["name"], "xyz": tuple(float(v) for v in m["xyz"]), "yaw": math.radians(float(m["yaw_deg"]))}
                for m in spec["sensors"]
            ],
            "lift_offset_m": float(spec.get("lift_offset_m", 0.0)),
        }
    return out


def _rect_cells_free(snapshot, rect, margin=0.0):
    """Count FREE cells whose square overlaps a truth rectangle (exact separating-axis test)."""
    from forklift_core.perception.obstacle_grid import FREE

    res = snapshot.resolution_m
    nx, ny = snapshot.state.shape
    c, s = math.cos(rect["yaw_rad"]), math.sin(rect["yaw_rad"])
    hl, hw = rect["length_m"] / 2 + margin, rect["width_m"] / 2 + margin
    r = math.hypot(hl, hw) + res
    i0 = max(int((rect["x_m"] - r - snapshot.origin_x_m) / res), 0)
    i1 = min(int((rect["x_m"] + r - snapshot.origin_x_m) / res) + 1, nx)
    j0 = max(int((rect["y_m"] - r - snapshot.origin_y_m) / res), 0)
    j1 = min(int((rect["y_m"] + r - snapshot.origin_y_m) / res) + 1, ny)
    if i0 >= i1 or j0 >= j1:
        return 0
    ii, jj = np.meshgrid(np.arange(i0, i1), np.arange(j0, j1), indexing="ij")
    dx = snapshot.origin_x_m + (ii + 0.5) * res - rect["x_m"]
    dy = snapshot.origin_y_m + (jj + 0.5) * res - rect["y_m"]
    half = res / 2
    ex = abs(c) * hl + abs(s) * hw
    ey = abs(s) * hl + abs(c) * hw
    proj = half * (abs(c) + abs(s))
    overlap = (
        (np.abs(dx) <= ex + half) & (np.abs(dy) <= ey + half)
        & (np.abs(dx * c + dy * s) <= hl + proj) & (np.abs(-dx * s + dy * c) <= hw + proj)
    )
    return int((snapshot.state[i0:i1, j0:j1][overlap] == FREE).sum())


def _volume_unobserved(permission, snap, samples, arc, footprint, own_cells, now, free_age, *, direction=0,
                       own_pose=None, own_footprint=None):
    """Why the swept check volume up to its first OCCUPIED sample is unobserved (an
    UNKNOWN cell, leaving the grid, expired FREE), or None when it is covered.
    It walks like the permission (user decision 2026-10-06, replacing the Codex
    design P1 reading that counted past OCCUPIED).
    Cells and exemptions as the permission takes them (shape parts, direction,
    partial own cells only where the stop reaches past the body)."""
    from forklift_core.control.drive_permission import RETAINED, parts_of, shape_cells
    from forklift_core.perception.obstacle_grid import FREE, OCCUPIED

    cfg = permission.config
    radius = max(float(np.hypot(abs(lon) + max(fp.front_m, fp.rear_m), abs(lat) + fp.half_width_m))
                 for fp, lat, lon in parts_of(footprint))
    oldest = np.inf
    for i in range(len(samples)):
        if i == 0:
            pose, pad = samples[0], 0.0
        else:
            a_, b_ = samples[i - 1], samples[i]
            dyaw = float(np.arctan2(np.sin(b_[2] - a_[2]), np.cos(b_[2] - a_[2])))
            pad = (float(np.hypot(b_[0] - a_[0], b_[1] - a_[1])) + radius * abs(dyaw)) / 2
            pose = np.array([(a_[0] + b_[0]) / 2, (a_[1] + b_[1]) / 2, a_[2] + dyaw / 2])
        margin = cfg.envelope_offset_m + pad
        cells, outside = shape_cells(snap, pose, footprint, margin, direction=direction)
        if outside:
            return {"why": "outside", "sample": i}
        whole, partial = own_cells
        keep = []
        for c in cells:
            key = (int(c[0]), int(c[1]))
            if key in whole:
                continue
            if key in partial and own_pose is not None and not permission._enters(
                    snap, key, own_pose, own_footprint, pose, footprint, margin, direction):
                continue
            keep.append(c)
        cells = np.array(keep).reshape(-1, 2)
        if not len(cells):
            continue
        st = snap.state[cells[:, 0], cells[:, 1]]
        if (st == OCCUPIED).any():
            # User decision 2026-10-06: coverage walks the stop volume as the
            # permission does -- it stops at the first sample holding an
            # OCCUPIED cell, so what lies there and beyond is never entered.
            break
        bad = (st != FREE) & (st != OCCUPIED) & (st != RETAINED)
        if bad.any():
            c0 = cells[np.argmax(bad)]
            wx = snap.origin_x_m + (c0[0] + 0.5) * snap.resolution_m
            wy = snap.origin_y_m + (c0[1] + 0.5) * snap.resolution_m
            return {"why": "unknown", "sample": i, "cells": int(bad.sum()), "cell_m": [float(wx), float(wy)]}
        seen = (st == FREE) | (st == RETAINED)
        if seen.any():
            oldest = min(oldest, float(np.min(snap.free_stamp[cells[seen, 0], cells[seen, 1]])))
    if now - oldest > free_age:
        return {"why": "expired", "age_s": float(now - oldest)}
    return None


def evaluate_command(args) -> dict:
    from dataclasses import replace

    from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel, arc_poses
    from forklift_core.control.drive_permission import shape_meets
    from forklift_core.control.shadow_memory import ShadowMemory
    from forklift_core.localization.slam_pose import OdometryNoise
    from forklift_core.localization.wheel_odometry import AckermannOdometryGeometry, integrate_wheel_odometry
    from forklift_core.perception.obstacle_grid import AgeErrorTable, GridConfig, ObstacleGrid, ObstacleScan
    from forklift_core.planning.geometry import Footprint, FootprintCollisionChecker, Rectangle

    run = args.run
    log = np.load(run / "slam_log.npz")
    meta = json.loads((run / "meta.json").read_text())
    result = json.loads((run / "result.json").read_text())
    sections = np.load(args.sections)
    candidates = load_candidates(args.candidates)
    age = json.loads(args.odometry_age.read_text())
    table = AgeErrorTable(tuple(age["ages_s"]), tuple(age["cumulative_position_m"]), tuple(age["cumulative_yaw_rad"]))
    g = meta["odometry_geometry"]
    rear_x = float(g["rear_axle_x_in_base_m"])
    geometry = result["geometry"]
    unloaded = Footprint(**geometry["unloaded_footprint"])
    loaded = Footprint(**geometry["loaded_footprint"])
    body = Footprint(args.body_front_m, unloaded.rear_m, unloaded.half_width_m)
    # Body and fork blades, not their hull, as the runner's permission (D4).
    import sys as _sys

    _sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "isaac"))
    from insertion_geometry import read_fork_blades_m

    unloaded_shape = [(body, 0.0, 0.0)] + [
        (Footprint((x1 - x0) / 2, (x1 - x0) / 2, (y1 - y0) / 2), (y0 + y1) / 2, (x0 + x1) / 2 - rear_x)
        for x0, x1, y0, y1 in read_fork_blades_m(args.forklift_urdf)
    ]
    stopping = StoppingModel(args.stop_latency_s, args.stop_decel_mps2, args.stop_margin_m)
    pconfig = PermissionConfig(stopping, args.envelope_m, evidence_max_age_s=args.free_age_s,
                               envelope_ramp_m=args.envelope_ramp_m, shadow_band_m=args.shadow_band_m)

    # 120 Hz truth and wheel odometry (online noise model, fixed draw).
    stamps = log["joint_stamps_s"]
    base = log["base_pose_world"].astype(float)
    yaw = np.unwrap(_yaw(base[:, 3:7]))
    truth_rear = np.column_stack((base[:, 0] + rear_x * np.cos(yaw), base[:, 1] + rear_x * np.sin(yaw), yaw))
    rates = log["wheel_rates_rad_s"].astype(float)[:, 2:4]
    steer = log["steering_rad"].astype(float)
    noise = OdometryNoise(seed=args.noise_seed)
    odom = integrate_wheel_odometry(
        stamps,
        rates + noise._wheels.normal(0, noise.wheel_rate_std_rad_s, rates.shape),
        steer + noise._steering.normal(0, noise.steering_std_rad, steer.shape),
        AckermannOdometryGeometry(g["wheelbase_m"], g["track_m"], g["wheel_radius_m"]),
        initial_pose=tuple(truth_rear[0]),
    )
    velocity = np.gradient(base[:, :2], stamps, axis=0)
    signed_speed = velocity[:, 0] * np.cos(yaw) + velocity[:, 1] * np.sin(yaw)

    samples = result["samples"]
    sample_t = np.array([x["time_s"] for x in samples])
    phases = [x["phase"] for x in samples]
    obstacles = [o for o in meta["obstacles"] if o.get("base_m", 0.0) == 0.0]
    kinds = list(sections["kinds"])
    pallet_urdf = run / "pallet_with_synthetic_inertia.urdf"
    pallet_tris = np.concatenate(list(urdf_triangles(pallet_urdf, {}).values()))
    truck_cache: dict = {}
    # Which asset each floor rectangle is: the kind of the section segments inside it.
    probe_h = float(sections["heights"][0])
    probe_seg = sections[f"seg_{probe_h:.3f}"].mean(axis=1)
    probe_own = sections[f"own_{probe_h:.3f}"]
    rect_kind = []
    for o in obstacles:
        c, s_ = math.cos(o["yaw_rad"]), math.sin(o["yaw_rad"])
        d = probe_seg - [o["x_m"], o["y_m"]]
        inside = (np.abs(d[:, 0] * c + d[:, 1] * s_) <= o["length_m"] / 2 + 0.02) & (
            np.abs(-d[:, 0] * s_ + d[:, 1] * c) <= o["width_m"] / 2 + 0.02
        )
        owners_in = probe_own[inside]
        rect_kind.append(str(kinds[int(np.bincount(owners_in).argmax())]) if len(owners_in) else "none")
    obstacle_rects = [Rectangle(o["x_m"], o["y_m"], o["length_m"], o["width_m"], o["yaw_rad"]) for o in obstacles]
    # Floor projection of every prop's collider within the truck's height band
    # (Codex design P1: "3D collision shape floor projection n FREE = 0"): the
    # union over every section height of the cells inside each closed section
    # (even-odd) or crossed by a section edge, on a world grid aligned like the
    # snapshots (origin 0): the cells whose square overlaps the shape.
    res_w = 0.05
    proj = {}
    for key in [k for k in sections.files if k.startswith("seg_")]:
        if float(key[4:]) > args.projection_top_m:
            continue
        segs = sections[key]
        if not len(segs):
            continue
        lo = np.floor(np.minimum(segs[:, 0], segs[:, 1]) / res_w).astype(int)
        hi = np.floor(np.maximum(segs[:, 0], segs[:, 1]) / res_w).astype(int)
        own_k = sections[key.replace("seg_", "own_")]
        for owner in np.unique(own_k):
            sel = own_k == owner
            os_ = segs[sel]
            i0, j0 = lo[sel].min(axis=0)
            i1, j1 = hi[sel].max(axis=0)
            ci, cj = np.meshgrid(np.arange(i0, i1 + 1), np.arange(j0, j1 + 1), indexing="ij")
            cx, cy = (ci + 0.5) * res_w, (cj + 0.5) * res_w
            ax, ay, bx_, by_ = os_[:, 0, 0], os_[:, 0, 1], os_[:, 1, 0], os_[:, 1, 1]
            crosses = ((ay[None, None, :] > cy[..., None]) != (by_[None, None, :] > cy[..., None]))
            with np.errstate(divide="ignore", invalid="ignore"):
                xint = ax + (cy[..., None] - ay) * (bx_ - ax) / (by_ - ay)
            inside = (crosses & (cx[..., None] < xint)).sum(axis=-1) % 2 == 1
            for a_, b_ in zip(ci[inside], cj[inside]):
                proj[(int(a_), int(b_))] = int(owner)
            # Plus every cell a section edge passes through (supercover by fine
            # sampling): with the inside cells, exactly the cells whose square
            # overlaps the section.
            n = np.maximum(np.ceil(np.hypot(bx_ - ax, by_ - ay) / (res_w / 10)).astype(int), 1)
            for k_ in range(len(ax)):
                t_ = np.linspace(0.0, 1.0, n[k_] + 1)
                pi = np.floor((ax[k_] + t_ * (bx_[k_] - ax[k_])) / res_w).astype(int)
                pj = np.floor((ay[k_] + t_ * (by_[k_] - ay[k_])) / res_w).astype(int)
                for a_, b_ in zip(pi, pj):
                    proj[(int(a_), int(b_))] = int(owner)
    proj_cells = np.array(list(proj.keys()), dtype=int).reshape(-1, 2)
    proj_owner = np.array(list(proj.values()), dtype=int)
    if "proj_cells" in sections.files:
        # The conservative triangle projection, when the sections carry it.
        proj_cells, proj_owner = sections["proj_cells"], sections["proj_owner"]
    hall = meta["hall"]
    rng = np.random.default_rng(args.noise_seed)
    scan_stamps = log["scan_stamps_s"]
    beam_angles = np.linspace(-math.pi, math.pi, args.beams, endpoint=False)
    evaluated_phases = set(args.phases.split(","))

    def truck_segments(h, lift, steering):
        key = (round(h, 3), round(lift, 2), round(steering[0], 2), round(steering[1], 2))
        if key not in truck_cache:
            parts = urdf_triangles(args.forklift_urdf, {"fork_lift": lift, "left_steer": steering[0], "right_steer": steering[1]})
            truck_cache[key] = section_segments(np.concatenate(list(parts.values())), h)
        return truck_cache[key]

    # Synthetic boxes on the recorded path (plan D1 "빈 통과 방지"): the
    # recorded runs never bring an obstacle into the short stopping volume, so
    # boxes are placed every inject_every_m of travel-phase path, alternating
    # lateral offsets, from the start of the run. A box counts until the
    # truck's footprint first touches it (the replay drives on through it).
    injected = []
    if args.inject_every_m > 0:
        travel = np.array([phases[int(np.clip(np.searchsorted(sample_t, tt), 0, len(samples) - 1))]
                           in evaluated_phases for tt in stamps])
        seg_len = np.r_[0.0, np.hypot(*np.diff(truth_rear[:, :2], axis=0).T)]
        s_travel = np.cumsum(seg_len * travel)
        offsets = (0.0, 0.25, -0.25)
        nxt = args.inject_every_m
        for idx in range(len(stamps)):
            if travel[idx] and s_travel[idx] >= nxt:
                x0, y0, yaw0 = truth_rear[idx]
                ahead = 2.5  # beyond the fork tips at the placing instant
                lat = offsets[len(injected) % len(offsets)]
                cx = x0 + ahead * math.cos(yaw0) - lat * math.sin(yaw0)
                cy = y0 + ahead * math.sin(yaw0) + lat * math.cos(yaw0)
                injected.append(Rectangle(float(cx), float(cy), 0.4, 0.4, float(yaw0)))
                nxt += args.inject_every_m
    def box_segments(rect):
        c, s_ = math.cos(rect.yaw_rad), math.sin(rect.yaw_rad)
        hx, hy = rect.length_m / 2, rect.width_m / 2
        pts = [(rect.x_m + c * u - s_ * v, rect.y_m + s_ * u + c * v) for u, v in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]
        return np.array([[pts[i], pts[(i + 1) % 4]] for i in range(4)])

    # L2 (plan): false occupancy -- an OCCUPIED cell farther than the tolerance
    # from every true obstacle (the floor projection above, the pallet and the
    # injected boxes). Tolerance: half the cell diagonal twice (cell to cell),
    # the odometry bound over the occupied memory's age, and 0.05 m.
    occ_bound = table.at(args.occupied_age_s)
    false_tol = res_w * math.sqrt(2) + (occ_bound[0] if occ_bound is not None else 0.3) + 0.05
    k_tol = int(math.ceil(false_tol / res_w))
    true_mask = None
    if len(proj_cells):
        mi0, mj0 = proj_cells.min(axis=0) - k_tol - 1
        mi1, mj1 = proj_cells.max(axis=0) + k_tol + 2
        base_mask = np.zeros((mi1 - mi0, mj1 - mj0), dtype=bool)
        base_mask[proj_cells[:, 0] - mi0, proj_cells[:, 1] - mj0] = True
        grown = base_mask.copy()
        for dx_ in range(-k_tol, k_tol + 1):
            for dy_ in range(-k_tol, k_tol + 1):
                if res_w * math.hypot(max(abs(dx_) - 1, 0), max(abs(dy_) - 1, 0)) > false_tol:
                    continue
                grown |= np.roll(np.roll(base_mask, dx_, axis=0), dy_, axis=1)
        true_mask = (grown, mi0, mj0)

    def near_rect(px, py, rect, tol):
        c_, s_ = math.cos(rect.yaw_rad), math.sin(rect.yaw_rad)
        u_ = (px - rect.x_m) * c_ + (py - rect.y_m) * s_
        v_ = -(px - rect.x_m) * s_ + (py - rect.y_m) * c_
        du = np.maximum(np.abs(u_) - rect.length_m / 2, 0.0)
        dv = np.maximum(np.abs(v_) - rect.width_m / 2, 0.0)
        return np.hypot(du, dv) <= tol

    report = {}
    for cname, cand in candidates.items():
        alive = list(range(len(injected)))
        grid = None
        grid_cfg = GridConfig(
            hall["x_min_m"], hall["x_max_m"], hall["y_min_m"], hall["y_max_m"], table,
            free_max_age_s=args.free_age_s, occupied_max_age_s=args.occupied_age_s,
            free_r_cap_m=args.free_r_cap_m, free_rho_m=args.free_rho_m, close_gap_m=args.close_gap_m,
            free_min_width_m=args.free_min_width_m, taper_m=args.taper_m,
        )
        grid = ObstacleGrid(grid_cfg)
        permission = DrivePermission(pconfig)
        # Shadow-band memory (D4 delta 2026-10-05); its first snapshot, at the
        # first scan, is start-up.
        shadow = ShadowMemory(args.shadow_band_m, table, evidence_max_age_s=args.free_age_s) if args.shadow_band_m > 0 else None
        stats = {"events": 0, "unpermitted": [], "moving": {}, "permitted": {}, "free_inside": {}, "scans": 0,
                 "false_occupied": {"scans": 0, "scans_with": 0, "max": 0, "total": 0, "examples": []}}
        for k, t in enumerate(scan_stamps[: args.max_scans]):
            j = int(np.searchsorted(stamps, t))
            if j >= len(stamps):
                break
            si = int(np.clip(np.searchsorted(sample_t, t), 0, len(samples) - 1))
            sample = samples[si]
            phase = phases[si]
            lift = float(sample["lift_m"])
            ppos = sample["pallet_position_m"]
            pyaw = float(sample["pallet_yaw_rad"])
            carried = phase in ("lift", "extract", "transport", "lower") and ppos[2] > 0.01
            bx, by, byaw = base[j, 0], base[j, 1], yaw[j]
            st = steer[j]
            for sensor in cand["sensors"]:
                h = sensor["xyz"][2]
                key = f"seg_{h:.3f}"
                if key not in sections:
                    raise SystemExit(f"no section at {h} m in {args.sections}")
                ext = sections[key]
                if alive:
                    ext = np.concatenate([ext] + [box_segments(injected[b_]) for b_ in alive])
                self_local = truck_segments(h, lift + (cand["lift_offset_m"] if carried else 0.0), st)
                self_world = transform_segments(self_local, bx, by, byaw)
                pz = ppos[2] + (cand["lift_offset_m"] if carried else 0.0)
                pal = section_segments(pallet_tris + [0, 0, pz], h)
                pal_world = transform_segments(pal, ppos[0], ppos[1], pyaw) if len(pal) else pal
                c, s = math.cos(byaw), math.sin(byaw)
                ox = bx + c * sensor["xyz"][0] - s * sensor["xyz"][1]
                oy = by + s * sensor["xyz"][0] + c * sensor["xyz"][1]
                origin = np.array([ox, oy])
                angles = beam_angles + byaw + sensor["yaw"]
                d_ext, _ = cast_rays_2d(origin, angles, ext, args.cast_m)
                d_self, _ = cast_rays_2d(origin, angles, self_world, args.cast_m)
                d_pal, _ = cast_rays_2d(origin, angles, pal_world, args.cast_m) if len(pal_world) else (np.full(len(angles), np.inf), None)
                if carried:
                    d_self = np.minimum(d_self, d_pal)
                else:
                    d_ext = np.minimum(d_ext, d_pal)
                ranges = np.minimum(d_ext, d_self)
                self_hit = d_self < d_ext
                finite = np.isfinite(ranges)
                ranges[finite] += np.clip(rng.normal(0, 0.02, int(finite.sum())), -0.06, 0.06)
                ranges[finite & (ranges < 0.15)] = -np.inf
                laser_in_rear = (sensor["xyz"][0] - rear_x, sensor["xyz"][1], sensor["yaw"])
                # The fork gap is not the truck's own shape (plan D4); a carried pallet is.
                own_fp = loaded if carried else body
                grid.add_scan(ObstacleScan(float(t), sensor["name"], tuple(odom[j]), laser_in_rear,
                                           beam_angles, ranges, self_hit, h <= args.clear_max_height_m,
                                           (own_fp.front_m, own_fp.rear_m, own_fp.half_width_m)))
            stats["scans"] += 1
            # The correction control would use now: truth o odom^-1 (SLAM error is studied separately).
            tr, od = truth_rear[j], odom[j]
            own_now = loaded if carried else body  # the same outline the grid withholds
            v = float(signed_speed[j])
            evaluated = phase in evaluated_phases and abs(v) >= args.moving_mps
            if not evaluated and shadow is None:
                continue
            dyaw = tr[2] - od[2]
            cc, ss = math.cos(dyaw), math.sin(dyaw)
            correction = (tr[0] - (cc * od[0] - ss * od[1]), tr[1] - (ss * od[0] + cc * od[1]), dyaw)
            window = replace(grid_cfg, x_min_m=round(tr[0] - 6, 1), x_max_m=round(tr[0] + 6, 1),
                             y_min_m=round(tr[1] - 6, 1), y_max_m=round(tr[1] + 6, 1))
            grid.config = window
            snap = grid.snapshot(float(t), correction)
            grid.config = grid_cfg
            if shadow is not None:
                # The memory sees every scan, as the runner's does; only the
                # statistics are filtered (Codex P2).
                snap = shadow.apply(snap, tuple(tr), own_now)
            if k % args.free_check_every == 0:
                from forklift_core.perception.obstacle_grid import OCCUPIED as _OCC

                occ = np.argwhere(snap.state == _OCC)
                if len(occ):
                    wx = snap.origin_x_m + (occ[:, 0] + 0.5) * snap.resolution_m
                    wy = snap.origin_y_m + (occ[:, 1] + 0.5) * snap.resolution_m
                    true_ = np.zeros(len(occ), dtype=bool)
                    if true_mask is not None:  # no static projection: pallet and boxes still judged (Codex checkpoint 14)
                        gi_ = np.floor(wx / res_w).astype(int) - true_mask[1]
                        gj_ = np.floor(wy / res_w).astype(int) - true_mask[2]
                        inside_ = (gi_ >= 0) & (gi_ < true_mask[0].shape[0]) & (gj_ >= 0) & (gj_ < true_mask[0].shape[1])
                        true_[inside_] = true_mask[0][gi_[inside_], gj_[inside_]]
                    pal_rect = Rectangle(ppos[0], ppos[1], geometry["pallet_depth_m"], geometry["pallet_width_m"], pyaw)
                    true_ |= near_rect(wx, wy, pal_rect, false_tol)
                    for b_ in alive:
                        true_ |= near_rect(wx, wy, injected[b_], false_tol)
                    fo = stats["false_occupied"]
                    n_false = int((~true_).sum())
                    fo["scans"] += 1
                    fo["total"] += n_false
                    fo["max"] = max(fo["max"], n_false)
                    if n_false:
                        fo["scans_with"] += 1
                        if len(fo["examples"]) < 30:
                            fo["examples"].append({"t": float(t), "phase": phase, "cells": n_false,
                                                   "first_m": [float(wx[~true_][0]), float(wy[~true_][0])],
                                                   "truck": [float(v_) for v_ in tr]})
            if not evaluated:
                continue
            # Path ahead = what the truck actually drove next (rear axle).
            seg = np.hypot(*np.diff(truth_rear[j:, :2], axis=0).T)
            s_cum = np.concatenate(([0.0], np.cumsum(seg)))
            end = int(np.searchsorted(s_cum, pconfig.lookahead_m)) + 1
            ahead = truth_rear[j : j + max(end, 2)]
            # The shape the runner's permission checks (obstacle_layer.footprints).
            if carried:
                fp = loaded
            elif args.shape == "parts" or (args.shape == "hull_forward" and v < 0):
                fp = unloaded_shape
                if args.shape == "hull_forward":
                    own_now = unloaded_shape  # reversing, the present body + blades are the truck
            else:
                fp = unloaded
            permission.update(snap, ahead, fp, own_now, current_pose=tuple(tr), direction=-1 if v < 0 else 1)
            # The steering held by an emergency stop gives the stopping arc.
            kappa = float(np.mean([math.tan(a) / (g["wheelbase_m"] + math.tan(a) * side * g["track_m"] / 2)
                                   for a, side in ((st[0], 1), (st[1], -1))]))
            direction = -1 if v < 0 else 1
            allowed, reason = permission.allowed_speed(
                float(t), {sd["name"]: float(t) for sd in cand["sensors"]}, current_pose=tuple(tr),
                curvature_inv_m=kappa, direction=direction, footprint=fp, own_footprint=own_now,
                speed_cap_mps=abs(v),
            )
            dump_now = args.dump_at is not None and k == args.dump_at
            if args.dump_class is not None and not stats.get("dumped") and reason != "ok":
                cls_now = f"{'loaded' if carried else 'unloaded'}_{'reverse' if v < 0 else 'forward'}"
                dump_now = cls_now == args.dump_class and stats["scans"] > 50
                stats["dumped"] = dump_now
            if dump_now:
                np.savez(args.dump_path, state=snap.state, origin=[snap.origin_x_m, snap.origin_y_m],
                         res=snap.resolution_m, ahead=ahead, pose=tr, verified=permission.path_check.verified_m,
                         blocked=str(permission.path_check.blocked), allowed=allowed, reason=reason,
                         kappa=kappa, v=v, free_stamp=snap.free_stamp, t=float(t), oldest=permission.path_check.oldest_free_s,
                         loaded=carried)
            arc_samples, arc_s = arc_poses(tuple(tr), kappa, direction, stopping.distance_m(v) + pconfig.step_m, pconfig.step_m)
            own_set = permission._own_cells(snap, tuple(tr), own_now)
            unobserved = _volume_unobserved(permission, snap, arc_samples, arc_s, fp, own_set, float(t), args.free_age_s,
                                            direction=direction, own_pose=tuple(tr), own_footprint=own_now)
            curve = abs(kappa) > 0.1
            cls = f"{'loaded' if carried else 'unloaded'}_{'reverse' if v < 0 else 'forward'}_{'curve' if curve else 'straight'}"
            stats["moving"][cls] = stats["moving"].get(cls, 0) + 1
            if not unobserved:
                stats.setdefault("covered", {})[cls] = stats.setdefault("covered", {}).get(cls, 0) + 1
            else:
                miss = stats.setdefault("uncovered_by_phase", {}).setdefault(cls, {})
                miss[phase] = miss.get(phase, 0) + 1
                ex = stats.setdefault("uncovered_examples", [])
                if len(ex) < 60:
                    info = dict(unobserved)
                    if "cell_m" in info:
                        dx, dy = info["cell_m"][0] - tr[0], info["cell_m"][1] - tr[1]
                        c_, s_ = math.cos(tr[2]), math.sin(tr[2])
                        info["cell_truck_m"] = [round(dx * c_ + dy * s_, 3), round(-dx * s_ + dy * c_, 3)]
                    ex.append({"t": float(t), "phase": phase, "cls": cls, "v": float(v), **info})
            permitted = allowed >= abs(v) - 1e-9
            stats["permitted"][cls] = stats["permitted"].get(cls, 0) + int(permitted)
            if not permitted:
                why = stats.setdefault("blocked_reasons", {}).setdefault(cls, {})
                why[reason] = why.get(reason, 0) + 1
            # Truth: which floor obstacles (or the floor pallet) meet the
            # steering-held stopping volume at the recorded speed (Codex L0 P1),
            # one event per obstacle (Codex L0 P2).
            vol, _ = arc_poses(tuple(tr), kappa, direction, stopping.distance_m(v), 0.025)
            truths = list(enumerate(obstacle_rects)) + [(1000 + b_, injected[b_]) for b_ in alive]
            if not carried and phase != "approach":
                truths.append((-1, Rectangle(ppos[0], ppos[1], geometry["pallet_depth_m"], geometry["pallet_width_m"], pyaw)))
            for oid, rect in truths:
                # The real shape (body + blades unloaded), as the permission checks it.
                if any(shape_meets(rect, fp, tuple(p), args.envelope_m) for p in vol[1:]):
                    stats["events"] += 1
                    obj = stats.setdefault("event_objects", {})
                    obj[oid] = obj.get(oid, 0) + 1
                    if permitted:
                        stats["unpermitted"].append({"t": float(t), "phase": phase, "v": v, "object": oid, "reason": reason})
            if alive:
                for b_ in list(alive):
                    if shape_meets(injected[b_], fp, tuple(tr)):
                        alive.remove(b_)
            if k % args.free_check_every == 0 and len(proj_cells):
                oi = np.round(snap.origin_x_m / res_w).astype(int)
                oj = np.round(snap.origin_y_m / res_w).astype(int)
                li, lj = proj_cells[:, 0] - oi, proj_cells[:, 1] - oj
                ok = (li >= 0) & (li < snap.state.shape[0]) & (lj >= 0) & (lj < snap.state.shape[1])
                from forklift_core.perception.obstacle_grid import FREE as _FREE
                bad = ok.copy()
                bad[ok] = snap.state[li[ok], lj[ok]] == _FREE
                for owner, ci, cj in zip(proj_owner[bad], proj_cells[bad, 0], proj_cells[bad, 1]):
                    kind = str(kinds[owner])
                    stats["free_inside"][kind] = stats["free_inside"].get(kind, 0) + 1
                    ex = stats.setdefault("free_inside_examples", [])
                    if len(ex) < 40:
                        mine = proj_cells[proj_owner == owner]
                        centre = (mine.mean(axis=0) + 0.5) * res_w
                        ex.append({"t": float(t), "phase": phase, "owner": int(owner), "kind": kind,
                                   "cell_m": [float((ci + 0.5) * res_w), float((cj + 0.5) * res_w)],
                                   "owner_centre_m": centre.tolist(), "owner_cells": int(len(mine)),
                                   "truck": [float(v_) for v_ in tr]})
        ratio = {c: stats["permitted"][c] / stats["moving"][c] for c in stats["moving"]}
        # Coverage: not stopped for lack of observation (occupied blocks depend on
        # the recorded truth-planned path, which a grid plan would route around).
        coverage = {c: stats.get("covered", {}).get(c, 0) / stats["moving"][c] for c in stats["moving"]}
        report[cname] = {
            "rect_kinds": {k: rect_kind.count(k) for k in sorted(set(rect_kind))},
            "sensors": cand["sensors"], "lift_offset_m": cand["lift_offset_m"],
            "scans": stats["scans"], "events": stats["events"],
            "unpermitted_entries": len(stats["unpermitted"]), "unpermitted": stats["unpermitted"][:20],
            "event_objects": len(stats.get("event_objects", {})),
            "injected_boxes": len(injected),
            "injected_event_objects": len([o for o in stats.get("event_objects", {}) if o >= 1000]),
            "event_counts_by_object": {str(k): v for k, v in stats.get("event_objects", {}).items()},
            "free_inside_examples": stats.get("free_inside_examples", []),
            "moving_by_class": stats["moving"], "permission_ratio": ratio, "coverage_ratio": coverage,
            "free_cells_inside_obstacles": stats["free_inside"],
            "blocked_reasons": stats.get("blocked_reasons", {}),
            "uncovered_by_phase": stats.get("uncovered_by_phase", {}),
            "uncovered_examples": stats.get("uncovered_examples", []),
            "false_occupied": {**stats["false_occupied"], "tolerance_m": false_tol},
        }
        print(cname, json.dumps({k: report[cname][k] for k in ("events", "event_objects", "unpermitted_entries", "permission_ratio", "coverage_ratio")}), flush=True)
    return {"run": str(run), "stopping": vars(stopping), "envelope_m": args.envelope_m, "candidates": report}


# ---------------------------------------------------------------- L1′: the single configuration

ROOT = Path(__file__).resolve().parents[1]
# The code a single verdict depends on; merge refuses parts computed by different versions.
CODE_FILES = ("tools/p0b_sensor_study.py", "sim/isaac/obstacle_layer.py", "sim/isaac/planar_lidar.py",
              "sim/isaac/insertion_geometry.py", "src/forklift_core/control/drive_permission.py",
              "src/forklift_core/control/shadow_memory.py", "src/forklift_core/perception/obstacle_grid.py",
              "src/forklift_core/planning/geometry.py", "src/forklift_core/planning/factory_layout.py")
def code_hashes() -> dict:
    """sha256 of every file in CODE_FILES."""
    import hashlib

    return {rel: hashlib.sha256((ROOT / rel).read_bytes()).hexdigest() for rel in CODE_FILES}


MOTION_CLASSES = tuple(f"{l}_{d}_{c}" for l in ("unloaded", "loaded") for d in ("forward", "reverse")
                       for c in ("curve", "straight"))


def rect_segments(rect) -> np.ndarray:
    """The four edges of a Rectangle as (4, 2, 2) segments."""
    c, s_ = math.cos(rect.yaw_rad), math.sin(rect.yaw_rad)
    hx, hy = rect.length_m / 2, rect.width_m / 2
    pts = [(rect.x_m + c * u - s_ * v, rect.y_m + s_ * u + c * v) for u, v in ((hx, hy), (-hx, hy), (-hx, -hy), (hx, -hy))]
    return np.array([[pts[i], pts[(i + 1) % 4]] for i in range(4)])


def run_prisms(result: dict, meta: dict) -> list:
    """(Rectangle, top) of a run's prism colliders (plan v10 D0): every kept floor prop of the
    recorded scenario up to its highest load -- factory_layout.prism_columns rebuilt from the
    record, checked against the column count the runner wrote."""
    from forklift_core.planning.factory_layout import _inside
    from forklift_core.planning.geometry import Rectangle

    record = result.get("prism_colliders")
    if not record:
        raise SystemExit("L1′ replays prism-collider runs only (--prism-colliders, plan v10 D0)")
    loads = [(Rectangle(o["x_m"], o["y_m"], o["length_m"], o["width_m"], o["yaw_rad"]), o["base_m"] + o["height_m"])
             for o in meta["obstacles"] if o.get("base_m", 0.0) > 0.0]
    out = []
    for prop in result["scenario"]["props"]:
        rect = Rectangle(**prop["rectangle"])
        top = float(prop["asset"]["height_m"])
        for load, load_top in loads:
            if _inside(load, rect):
                top = max(top, float(load_top))
        out.append((rect, top))
    if len(out) != int(record["columns"]):
        raise SystemExit(f"{len(out)} recorded props but {record['columns']} prism columns")
    return out


def run_new_obstacles(result: dict) -> list:
    """(id, Rectangle, height, spawned at, removed at or inf) of every box the run spawned (L4 events)."""
    from forklift_core.planning.geometry import Rectangle

    boxes = {}
    for e in (result.get("new_obstacles") or {}).get("log") or []:
        if e["action"] == "spawn":
            oid, x, y, yaw, size = e["detail"]
            boxes[oid] = [oid, Rectangle(float(x), float(y), float(size[0]), float(size[1]), float(yaw)),
                          float(size[2]), float(e["time_s"]), math.inf]
        elif e["action"] == "remove":
            if e["detail"][0] in boxes:
                boxes[e["detail"][0]][4] = float(e["time_s"])
        else:
            # N9 silences the one LiDAR: no scans to cover with (a separate safety case).
            raise SystemExit(f"{e['action']} event: not a coverage trajectory")
    return [tuple(b) for b in boxes.values()]


def face_unobserved_m(snapshot, pose, outline, now_s: float, free_age_s: float, *, depth_m: float = 0.5,
                      step_m: float = 0.005) -> dict:
    """Per face of the own outline (a Footprint at the rear-axle pose): the thickest band,
    over probe lines step_m apart along the face, between the face and the first observed
    cell outward (OCCUPIED, or FREE within free_age_s) -- what a stopped truck cannot see in
    front of its own faces and the shadow memory has to bridge (plan v10 D4). Sampled, not
    exact: probes start 1 mm off the face and step step_m outward, so a sliver of a cell
    narrower than step_m along the face can be missed. Cells past the first observed one
    (an obstacle's inside) do not count. None: unobserved all the way to depth_m."""
    from forklift_core.perception.obstacle_grid import FREE, OCCUPIED

    x0, y0, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    res = snapshot.resolution_m
    nx, ny = snapshot.state.shape
    f, r, w = outline.front_m, outline.rear_m, outline.half_width_m
    offsets = np.append(np.arange(0.0, depth_m, step_m) + 1e-3, depth_m)
    along = {"front": np.arange(-w, w + 1e-9, step_m), "rear": np.arange(-w, w + 1e-9, step_m),
             "left": np.arange(-r, f + 1e-9, step_m), "right": np.arange(-r, f + 1e-9, step_m)}
    out = {}
    for face, t in along.items():
        d, a = np.meshgrid(offsets, t, indexing="ij")  # (offset, point along the face)
        if face == "front":
            u, v = f + d, a
        elif face == "rear":
            u, v = -r - d, a
        elif face == "left":
            u, v = a, w + d
        else:
            u, v = a, -w - d
        i = np.floor((x0 + c * u - s * v - snapshot.origin_x_m) / res).astype(int)
        j = np.floor((y0 + s * u + c * v - snapshot.origin_y_m) / res).astype(int)
        inside = (i >= 0) & (i < nx) & (j >= 0) & (j < ny)
        observed = np.zeros(i.shape, dtype=bool)
        ii, jj = i[inside], j[inside]
        st = snapshot.state[ii, jj]
        observed[inside] = (st == OCCUPIED) | ((st == FREE) & (now_s - snapshot.free_stamp[ii, jj] <= free_age_s + 1e-9))
        if not observed.any(axis=0).all():
            out[face] = None
            continue
        first = observed.argmax(axis=0).max()
        out[face] = 0.0 if first == 0 else float(offsets[first])
    return out


def _raw_snapshot(layer, now_s: float, correction, version: int, pose):
    """The layer's grid snapshot before the shadow memory, on the window refresh() uses."""
    from dataclasses import replace

    w, res = layer.window_m, layer.grid_config.resolution_m
    x0 = math.floor((pose[0] - w) / res) * res
    y0 = math.floor((pose[1] - w) / res) * res
    layer.grid.config = replace(layer.grid_config, x_min_m=x0, x_max_m=x0 + 2 * w, y_min_m=y0, y_max_m=y0 + 2 * w)
    try:
        return layer.grid.snapshot(now_s, correction, version)
    finally:
        layer.grid.config = layer.grid_config


def injection_poses(stamps, truth_rear, signed_speed, travel, every_m: float, *, ahead_m=(2.5, 1.75, 1.0),
                    lead_m: tuple = (1.29, 0.17), loaded_lead_m: tuple | None = None, loaded=None,
                    offsets=(0.0, 0.25, -0.25), min_leg_m: float = 0.5):
    """Synthetic boxes (x, y, yaw, index placed at) every every_m of evaluated travel, on the
    path the truck is about to drive: ahead_m of recorded arc on (cycling through the given
    distances, so short straights -- a docking approach ends at the pallet -- meet boxes too),
    or the rest of the leg if it reverses or ends sooner, then past the leading face there
    (lead_m = (front, rear) from the rear axle, loaded_lead_m where loaded[idx] -- the outline
    the truck has when the box is placed; from the
    rear axle, plus 0.25 m) and shifted by the next lateral offset. A box exists from the
    tick it is placed at (an obstacle that appears ahead, like L4's), never before -- placed
    from the start, a box on the return path sat on the truck at the first scan. Curves and
    reverse legs meet their boxes too (plan v10 L1′: 30 objects per motion class; this stands
    in for the plan's synthetic trajectories moved toward obstacles). A leg shorter than
    min_leg_m from there gets none. The distance to the truck is not bounded here (an arc
    is not a distance); single drops any box within 0.5 m of the outline at placement."""
    out = []
    if every_m <= 0:
        return out
    seg_len = np.r_[0.0, np.hypot(*np.diff(truth_rear[:, :2], axis=0).T)]
    s_travel = np.cumsum(seg_len * travel)
    nxt = every_m
    for idx in range(len(stamps)):
        if not (travel[idx] and s_travel[idx] >= nxt):
            continue
        nxt += every_m
        ahead = ahead_m[len(out) % len(ahead_m)] if isinstance(ahead_m, (tuple, list)) else ahead_m
        leg, direction = leg_ahead(truth_rear, signed_speed, idx, ahead)
        length = float(np.sum(np.hypot(*np.diff(leg[:, :2], axis=0).T)))
        if direction == 0 or length < min_leg_m:
            continue
        x0, y0, yaw0 = leg[-1]
        faces = loaded_lead_m if (loaded is not None and loaded_lead_m is not None and loaded[idx]) else lead_m
        lead = (faces[0] if direction > 0 else faces[1]) + 0.25
        lat = offsets[len(out) % len(offsets)]
        out.append((float(x0 + direction * lead * math.cos(yaw0) - lat * math.sin(yaw0)),
                    float(y0 + direction * lead * math.sin(yaw0) + lat * math.cos(yaw0)), float(yaw0), idx))
    return out


def phase_at(transitions: list, t: float, first: str = "observe") -> str:
    """The phase at t from the recorded transitions (exact times, never a later sample's)."""
    phase = transitions[0]["from"] if transitions else first
    for tr in transitions:
        if tr["time_s"] <= t + 1e-9:
            phase = tr["to"]
        else:
            break
    return phase


def leg_ahead(rear: np.ndarray, speed: np.ndarray, j: int, lookahead_m: float, *, still_mps: float = 0.02):
    """(path, direction) the truck drove from tick j until it reverses or lookahead_m is
    covered -- the recorded counterpart of the tracker's leg_ahead (a cusp ends the leg)."""
    sgn = np.where(np.abs(speed[j:]) > still_mps, np.sign(speed[j:]), 0.0)
    moving = np.flatnonzero(sgn)
    direction = int(sgn[moving[0]]) if len(moving) else 0
    stop = len(sgn)
    if direction:
        flips = np.flatnonzero(sgn == -direction)
        if len(flips):
            stop = int(flips[0])
    leg = rear[j : j + max(stop, 2)]
    s_cum = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(leg[:, :2], axis=0).T))))
    end = int(np.searchsorted(s_cum, lookahead_m)) + 1
    return leg[: max(end, 2)], direction


TILT_BINS_DEG = (0.0, 0.02, 0.05, 0.1, 0.2, 0.5, 1.0, 180.0)


def sweep_meets(rect, shape, samples, margin_m: float, direction: int) -> bool:
    """Whether the continuous sweep of shape along the sample poses, grown by margin_m as the
    permission grows it, may meet rect. The permission (footprint_cells) grows the leading
    face and the sides, never the trailing face: below curvature 1 / half width no body point
    moves against the travel. So each step a -> b is covered by the shape at a grown, on the
    leading face, by margin + the step's arc + the turning reach (radius |dyaw|) and, on the
    sides, by margin + the turning reach + the chord's sideways part -- conservative between
    samples (Codex L1′ 4th P1) without the trailing growth that counted posts the truck was
    leaving (5th P2-1)."""
    from forklift_core.control.drive_permission import parts_of, shape_meets
    from forklift_core.planning.geometry import Footprint

    parts = parts_of(shape)
    radius = max(float(np.hypot(abs(lon) + max(fp.front_m, fp.rear_m), abs(lat) + fp.half_width_m))
                 for fp, lat, lon in parts)
    for a, b in zip(samples[:-1], samples[1:]):
        dyaw = abs(float(np.arctan2(np.sin(b[2] - a[2]), np.cos(b[2] - a[2]))))
        step = float(np.hypot(b[0] - a[0], b[1] - a[1]))
        lead = margin_m + step + radius * dyaw
        side = margin_m + radius * dyaw + step * dyaw / 2
        grown = [(Footprint(fp.front_m + (lead if direction >= 0 else 0.0), fp.rear_m + (lead if direction <= 0 else 0.0),
                            fp.half_width_m + side), lat, lon) for fp, lat, lon in parts]
        if shape_meets(rect, grown, tuple(a), 0.0):
            return True
    return False


def judge_classes(classes: dict, *, coverage_min: float, objects_min: int) -> dict:
    """Plan v10 L1′ per motion class: coverage >= coverage_min, at least objects_min event
    objects (a class with none does not pass on 'no unpermitted entry'), no unpermitted entry.
    A class fails on coverage or an unpermitted entry; one that only lacks event objects is
    'insufficient' (not enough evidence, not a failure), one without motion 'absent'.
    Status: 'fail' if any class fails, else 'incomplete' if any is insufficient or absent,
    else 'pass' (all eight present and passing)."""
    out = {}
    for cls in MOTION_CLASSES:
        c = classes.get(cls)
        if not c or not c["moving"]:
            out[cls] = {"moving": 0, "verdict": "absent"}
            continue
        coverage = c["covered"] / c["moving"]
        row = {"moving": c["moving"], "coverage": coverage, "event_objects": c["event_objects"],
               "unpermitted": c["unpermitted"], "coverage_ok": coverage >= coverage_min,
               "events_ok": c["event_objects"] >= objects_min, "unpermitted_ok": c["unpermitted"] == 0}
        if not (row["coverage_ok"] and row["unpermitted_ok"]):
            row["verdict"] = "fail"
        else:
            row["verdict"] = "pass" if row["events_ok"] else "insufficient"
        out[cls] = row
    verdicts = {r["verdict"] for r in out.values()}
    status = "fail" if "fail" in verdicts else ("incomplete" if verdicts & {"absent", "insufficient"} else "pass")
    return {"classes": out, "status": status, "pass": status == "pass",
            "absent": [c for c, r in out.items() if r["verdict"] == "absent"],
            "insufficient": [c for c, r in out.items() if r["verdict"] == "insufficient"],
            "coverage_min": coverage_min, "objects_min": objects_min}


def _rect_dict(rect) -> dict:
    return {"x_m": rect.x_m, "y_m": rect.y_m, "length_m": rect.length_m, "width_m": rect.width_m, "yaw_rad": rect.yaw_rad}


def single_command(args) -> dict:
    """L1′ (plan v10): the brief's one LiDAR through the runner's own ObstacleLayer.

    Sensor, grid and permission values come from the layer config, the chassis from the
    URDF the run used (hash-checked, measured chassis only), the scene from the run's prism
    colliders and spawned boxes. Scans go into the layer at their 10 Hz stamps (add_scans,
    then refresh on the leg ahead, as the runner's scan callback); the permission and the
    truth are judged every control tick between them (limit with the last snapshot, so
    sensor timeout and evidence age are exercised, as the runner's loop).

    Pose inputs are the run's own: at each scan the odometry and the applied correction
    the runner fed its layer (slam_records.json -- applied o odom equals the recorded
    control pose), at every tick the recorded control pose (slam_control.npy). A SLAM
    release between scans re-refreshes at its tick (as run_transport's loop) with the
    correction the tracker applies there -- the last estimate received before it, i.e. the
    map_from_odom of the latest scan record at or before the release (SlamPoseTracker.
    release); each such value is checked against the next scan's applied correction and
    the matches are reported. A truth-fed run uses the truth pose and no correction. Not
    replayed: the range noise (the layer's own draw, independent of the run's -- a cell
    edge can fall differently) and the tracker's planned leg (the runner's path_ahead):
    the path check gets the recorded leg, so the verdict judges the stopping-arc check
    alone (permitted = the arc's own limit >= |v|), which the runner's allowed speed never
    exceeds -- stricter than the runner, never laxer.
    Pallets are outside the 1.05 m plane's reach and
    known pallets are off (plan 2026-10-08): pallet events are counted apart, by phase.
    Objects lower than h_det are outside the operating assumption: a run with one is refused.
    """
    import atexit
    import hashlib
    import shutil
    import sys as _sys
    import tempfile

    _sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import obstacle_layer as OL
    import planar_lidar
    from insertion_geometry import read_fork_blades_m

    from forklift_core.control.drive_permission import arc_poses, shape_meets
    from forklift_core.perception.known_obstacles import invert_pose
    from forklift_core.perception.obstacle_grid import FREE, AgeErrorTable, compose
    from forklift_core.planning.geometry import Bounds, Footprint, Rectangle

    code = code_hashes()  # before anything runs; checked again at the end
    # The config and the odometry table are hashed as read and checked again at the end,
    # like the code (Codex L1′ 11th P2): the result names the inputs it used.
    config_bytes = Path(args.layer_config).read_bytes()
    config_sha = hashlib.sha256(config_bytes).hexdigest()
    with tempfile.TemporaryDirectory(prefix="l1p_config_") as tmp:
        (Path(tmp) / "layer.yaml").write_bytes(config_bytes)
        config = OL.load_layer_config(Path(tmp) / "layer.yaml")
    if (config.get("known_pallets") or {}).get("enabled"):
        raise SystemExit("known pallets are off until D8 (plan 2026-10-08); L1′ counts pallets apart")
    # The planning memory feeds the planner only; the permission never sees it.
    config = {**config, "grid": {**config["grid"], "planning_memory": False}}
    if len(config["sensors"]) != 1:
        raise SystemExit("L1′ is the single configuration: one sensor")
    sensor = config["sensors"][0]
    age_path = Path(config["odometry_age"])
    age_path = age_path if age_path.is_absolute() else ROOT / age_path
    age_bytes = age_path.read_bytes()
    age_sha = hashlib.sha256(age_bytes).hexdigest()
    age = json.loads(age_bytes)
    table = AgeErrorTable(tuple(age["ages_s"]), tuple(age["cumulative_position_m"]), tuple(age["cumulative_yaw_rad"]))
    evaluated_phases = set(args.phases.split(","))
    loaded_phases = ("lift", "extract", "transport", "lower")  # the runner's loaded state
    classes = {cls: {"moving": 0, "covered": 0, "permitted": 0, "event_objects": 0, "unpermitted": 0}
               for cls in MOTION_CLASSES}
    runs, seen = [], set()
    for run in args.run:
        # Every input of this run is read once as bytes, hashed, and parsed from a private
        # copy of those bytes: what the result is computed from is what it names, whatever
        # happens to the originals meanwhile (Codex L1′ 12th/13th P2 -- the URDF is parsed
        # per lift and steering state, long after the start).
        snap = Path(tempfile.mkdtemp(prefix="l1p_inputs_"))
        atexit.register(shutil.rmtree, snap, True)  # also when the run is refused part way
        result_bytes = (run / "result.json").read_bytes()
        result_sha = hashlib.sha256(result_bytes).hexdigest()
        result = json.loads(result_bytes)
        recorded = result["arguments"]
        sources = {"forklift.urdf": ROOT / recorded["forklift_urdf"], "meta.json": run / "meta.json",
                   "slam_log.npz": run / "slam_log.npz", "pallet.urdf": run / "pallet_with_synthetic_inertia.urdf"}
        sources.update({n: run / n for n in ("slam_records.json", "slam_control.npy") if (run / n).exists()})
        inputs_sha = {str(run / "result.json"): result_sha}
        for name, src in sources.items():
            data = src.read_bytes()
            inputs_sha[str(src)] = hashlib.sha256(data).hexdigest()
            (snap / name).write_bytes(data)
        urdf = snap / "forklift.urdf"
        log = np.load(snap / "slam_log.npz")
        # The run's identity is its recorded motion (the 120 Hz stamps and base poses), not the
        # bytes of a file: a copy of a run with other whitespace is still the same run (Codex
        # L1′ 14th P2).
        run_id = hashlib.sha256(np.ascontiguousarray(log["joint_stamps_s"], dtype=np.float64).tobytes()
                                + np.ascontiguousarray(log["base_pose_world"], dtype=np.float64).tobytes()).hexdigest()
        if run_id in seen:
            raise SystemExit(f"{run}: the same run twice")
        seen.add(run_id)
        meta = json.loads((snap / "meta.json").read_bytes())
        if inputs_sha[str(sources["forklift.urdf"])] != result["forklift_urdf_sha256"]:
            raise SystemExit(f"{sources['forklift.urdf']} differs from the URDF {run} ran with")
        if "dls08_measured" not in str((result.get("chassis_model") or {}).get("forklift_urdf", "")) or \
                result.get("scene_chassis_matches_urdf") is False:
            raise SystemExit(f"{run}: not the measured chassis (plan v10: every judgement on dls08_measured)")
        laser = meta["laser"]
        if config.get("shared_with_slam") and (
                tuple(float(v) for v in laser["mount_xyz_m"]) != tuple(sensor.xyz_m)
                or int(laser["beam_count"]) != int(config["beams"])
                or float(laser["range_min_m"]) != float(config["range_min_m"])
                or float(laser["range_max_m"]) != float(config["range_max_m"])):
            raise SystemExit("the layer's sensor is not the run's recorded LiDAR")
        if "silenced_scan_stamps_s" in log.files and len(log["silenced_scan_stamps_s"]):
            raise SystemExit(f"{run}: silenced scans -- not a coverage trajectory")
        rear_off = abs(float(recorded["rear_axle_offset_m"]))
        geometry = result["geometry"]
        unloaded = Footprint(**geometry["unloaded_footprint"])
        loaded = Footprint(**geometry["loaded_footprint"])
        b = result["scenario"]["bounds"]
        noise_seed = args.noise_seed if args.noise_seed is not None else int(result["seed"])  # the runner's layer seed
        layer = OL.ObstacleLayer(
            config, hall=Bounds(b["x_min_m"], b["x_max_m"], b["y_min_m"], b["y_max_m"]), error_table=table,
            unloaded=unloaded, loaded=loaded,
            body_front_m=geometry["axle_to_fork_tip_m"] - geometry["carriage_limit_m"],
            rear_axle_x_in_base_m=-rear_off, noise_seed=noise_seed,
            blades_rear_m=tuple((x0 + rear_off, x1 + rear_off, y0, y1) for x0, x1, y0, y1 in read_fork_blades_m(urdf)),
        )
        stopping = layer.permission.config.stopping
        step = layer.permission.config.step_m
        free_age = layer.grid_config.free_max_age_s
        envelope = layer.permission.config.envelope_offset_m
        lookahead = layer.permission.config.lookahead_m
        h = float(sensor.xyz_m[2])
        mount = planar_lidar.LaserMount(tuple(sensor.xyz_m), float(sensor.yaw_rad))

        # Operating assumption A (plan v10 D0): every kept object reaches h_det. A lower one
        # (N5) is a limit experiment, not coverage evidence.
        prisms = run_prisms(result, meta)
        low = [round(top, 3) for _, top in prisms if top < layer.band_top_m]
        spawned = run_new_obstacles(result)
        low += [round(height, 3) for _, _, height, _, _ in spawned if height < layer.band_top_m]
        if low:
            raise SystemExit(f"{run}: objects below h_det {layer.band_top_m} m ({low[:5]}) -- outside D0 A")
        static = [rect for rect, _ in prisms]
        static_segs = np.concatenate([rect_segments(r) for r in static]) if static else np.zeros((0, 2, 2))
        static_centres = np.array([[r.x_m, r.y_m, math.hypot(r.length_m, r.width_m) / 2] for r in static]).reshape(-1, 3)

        # 120 Hz truth; the runner's own pose inputs (see the docstring).
        stamps = log["joint_stamps_s"]
        base = log["base_pose_world"].astype(float)
        yaw = np.unwrap(_yaw(base[:, 3:7]))
        g = meta["odometry_geometry"]
        rear_x = float(g["rear_axle_x_in_base_m"])
        truth_rear = np.column_stack((base[:, 0] + rear_x * np.cos(yaw), base[:, 1] + rear_x * np.sin(yaw), yaw))
        steer = log["steering_rad"].astype(float)
        feedback = result.get("feedback")
        if feedback == "slam_estimate":
            recs = json.loads((snap / "slam_records.json").read_bytes())
            rec_by_stamp = {round(float(x["stamp_s"]), 6): x for x in recs}
            control = np.load(snap / "slam_control.npy")  # t, control rear (x, y, yaw), truth rear
            releases = sorted(float(e["time_s"]) for e in (result.get("slam_summary") or {}).get("holds", ())
                              if e["event"] != "hold")
        elif feedback == "simulator_ground_truth":
            recs, rec_by_stamp, control, releases = None, None, None, []
        else:
            raise SystemExit(f"{run}: feedback {feedback!r} has no recorded pose inputs")

        def scan_inputs(ts, js):
            """(odom rear, applied correction) the runner fed its layer at this scan."""
            if recs is None:
                return tuple(truth_rear[js]), (0.0, 0.0, 0.0)
            rec = rec_by_stamp.get(round(float(ts), 6))
            if rec is None:
                raise SystemExit(f"{run}: no SLAM record for the scan at {ts}")
            return (compose(tuple(rec["odom_base"]), (rear_x, 0.0, 0.0)),
                    tuple(float(v) for v in rec["applied_map_from_odom"]))

        rec_stamps = np.array([float(x["stamp_s"]) for x in recs]) if recs is not None else np.zeros(0)

        def release_correction(t):
            """(applied after a release at t, the next scan's applied or None): the tracker
            applies the last estimate it received, the latest record's map_from_odom."""
            k = int(np.searchsorted(rec_stamps, t + 1e-9, side="right") - 1)
            if k < 0:
                return None, None
            nxt = recs[k + 1]["applied_map_from_odom"] if k + 1 < len(recs) else None
            return (tuple(float(v) for v in recs[k]["map_from_odom"]),
                    None if nxt is None else tuple(float(v) for v in nxt))

        def control_pose(j, t):
            if control is None:
                return tuple(truth_rear[j])
            k = int(np.clip(np.searchsorted(control[:, 0], t + 1e-9, side="right") - 1, 0, len(control) - 1))
            return tuple(float(v) for v in control[k, 1:4])
        velocity = np.gradient(base[:, :2], stamps, axis=0)
        signed_speed = velocity[:, 0] * np.cos(yaw) + velocity[:, 1] * np.sin(yaw)
        transitions = result.get("transitions") or []
        phases = [phase_at(transitions, float(tt)) for tt in stamps]
        samples = result["samples"]
        sample_t = np.array([x["time_s"] for x in samples])

        def sample_before(t):
            return samples[int(np.clip(np.searchsorted(sample_t, t, side="right") - 1, 0, len(samples) - 1))]

        travel = np.array([ph in evaluated_phases for ph in phases])
        injected, placed_s = [], []
        loaded_mask = np.array([ph in loaded_phases for ph in phases])
        for x, y, yy, idx in injection_poses(stamps, truth_rear, signed_speed, travel, args.inject_every_m,
                                             lead_m=(unloaded.front_m, unloaded.rear_m),
                                             loaded_lead_m=(loaded.front_m, loaded.rear_m), loaded=loaded_mask):
            box = Rectangle(x, y, 0.4, 0.4, yy)
            if shape_meets(box, loaded if loaded_mask[idx] else unloaded, tuple(truth_rear[idx]), 0.5):
                # Never closer than 0.5 m to the outline at placement: about 1.9 stopping
                # distances at 0.6 m/s (Codex L1′ 10th P3 -- the arc did not bound it).
                continue
            injected.append(box)
            placed_s.append(float(stamps[idx]))
        alive = set(range(len(injected)))  # not yet touched
        active: set = set()  # placed, and not dropped as a duplicate
        alias: dict = {}  # a spawn's object id -> the object it shares a collision shape with
        seen_spawns: list = []  # spawn ids in order of appearance
        spawn_rect: dict = {}

        def present_keys(t):
            return {f"new:{oid}" for oid, _ in boxes_at(t)}
        pending = list(range(len(injected)))  # in placement order

        def boxes_present(t):
            """Boxes present at t. A box whose placement comes due on a spot already taken
            -- by a present box, a prism or a spawned obstacle there at that time, within
            0.05 m -- is dropped: one collision shape must not count as two objects (Codex
            L1′ 5th P2-2, 6th P2-1)."""
            while pending and placed_s[pending[0]] <= t + 1e-9:
                b_ = pending.pop(0)
                box = injected[b_]
                as_shape = Footprint(box.length_m / 2, box.length_m / 2, box.width_m / 2)
                taken = ([injected[o] for o in active & alive] + static + [r for _, r in boxes_at(t)])
                if any(shape_meets(r, as_shape, (box.x_m, box.y_m, box.yaw_rad), 0.05) for r in taken):
                    alive.discard(b_)
                    st["injected_duplicates"] = st.get("injected_duplicates", 0) + 1
                else:
                    active.add(b_)
            # One collision shape is one object (Codex L1′ 9th P2): a spawn appearing on a
            # prism or on an earlier spawn present then counts under that one's object.
            for oid, r in boxes_at(t):
                key = f"new:{oid}"
                if key in seen_spawns:
                    continue
                as_shape = Footprint(r.length_m / 2, r.length_m / 2, r.width_m / 2)
                pose_r = (r.x_m, r.y_m, r.yaw_rad)
                for i_, pr in enumerate(static):
                    if shape_meets(pr, as_shape, pose_r, 0.05):
                        alias[key] = f"prism{i_}"
                        break
                else:
                    for k2 in seen_spawns:
                        r2 = spawn_rect[k2]
                        if k2 in present_keys(t) and shape_meets(r2, as_shape, pose_r, 0.05):
                            alias[key] = k2
                            break
                seen_spawns.append(key)
                spawn_rect[key] = r
            # A spawned obstacle appearing on present boxes replaces every one of them: they
            # go, and the spawn's events count under the first one's object (Codex L1′ 7th
            # P2-1, 8th P2: a spawn covering two boxes).
            for oid, r in boxes_at(t):
                key = f"new:{oid}"
                for b_ in sorted(active & alive):
                    box = injected[b_]
                    as_shape = Footprint(box.length_m / 2, box.length_m / 2, box.width_m / 2)
                    if shape_meets(r, as_shape, (box.x_m, box.y_m, box.yaw_rad), 0.05):
                        alive.discard(b_)
                        alias.setdefault(key, f"box{b_}")
                        st["injected_duplicates"] = st.get("injected_duplicates", 0) + 1
            return sorted(active & alive)
        pallet_tris = np.concatenate(list(urdf_triangles(snap / "pallet.urdf", {}).values()))
        truck_cache: dict = {}

        def truck_segments(lift, steering):
            key = (round(lift, 2), round(steering[0], 2), round(steering[1], 2))
            if key not in truck_cache:
                parts = urdf_triangles(urdf, {"fork_lift": lift, "left_steer": steering[0], "right_steer": steering[1]})
                truck_cache[key] = section_segments(np.concatenate(list(parts.values())), h)
            return truck_cache[key]

        st = {"scans": 0, "ticks": 0, "self_hit_beams_max": 0, "self_hit_scans": 0, "max_tilt_rad": 0.0,
              "tilt_bins": {}, "agreement": {"beams": 0, "within": 0, "missing_in_hall": 0, "missing_outside_hall": 0,
                                             "extra": 0, "abs_m": []},
              "face_moving": {}, "face_stopped": {}, "free_cells_overlapping_prisms": 0,
              "free_centres_in_prisms": 0, "free_in_prism_examples": [], "pallet_by_phase": {},
              "uncovered_examples": [], "unpermitted": [], "blocked_reasons": {},
              "classes": {cls: {"moving": 0, "covered": 0, "permitted": 0, "events": 0, "objects": set(),
                                "unpermitted": 0} for cls in MOTION_CLASSES}}
        scan_stamps = log["scan_stamps_s"][: args.max_scans]
        rec_ranges = log["scan_ranges_m"] if "scan_ranges_m" in log.files else None
        rec_pose = log["laser_pose_world"] if "laser_pose_world" in log.files else None
        last = {"correction": None, "scan": -1, "version": 0, "release": 0}

        def to_control(x, y, truth_pose, ctrl_pose):
            """A world point where the truck at truth_pose sees it, in the frame where it is at ctrl_pose."""
            rel = compose(invert_pose(truth_pose), (x, y, 0.0))
            out = compose(ctrl_pose, rel)
            return out[0], out[1]

        def leg_in_frame(leg, truth_pose, ctrl_pose):
            """The recorded leg (truth) as the control frame sees it from the truck."""
            off = compose(ctrl_pose, invert_pose(truth_pose))
            return np.array([compose(off, tuple(p)) for p in leg])

        def boxes_at(t):
            return [(oid, rect) for oid, rect, height, t0, t1 in spawned if t0 <= t < t1]

        def take_scan(k, ts):
            """The runner's scan callback: cast, add_scans, refresh on the leg ahead."""
            js = min(int(np.searchsorted(stamps, ts)), len(stamps) - 1)
            phase = phases[js]
            carried = phase in loaded_phases
            sample = sample_before(ts)
            lift, ppos, pyaw = float(sample["lift_m"]), sample["pallet_position_m"], float(sample["pallet_yaw_rad"])
            origin, directions = planar_lidar.laser_rays_world(base[js, :3], base[js, 3:7], mount, layer.beam_angles)
            angles = np.arctan2(directions[:, 1], directions[:, 0])
            o2 = origin[:2]
            boxes_now = boxes_at(ts)
            segs = np.concatenate([static_segs] + [rect_segments(r) for _, r in boxes_now]) if boxes_now else static_segs
            d_static, _ = cast_rays_2d(o2, angles, segs, layer.range_max_m)
            present = boxes_present(ts)
            d_inj = (cast_rays_2d(o2, angles, np.concatenate([rect_segments(injected[b_]) for b_ in present]),
                                  layer.range_max_m)[0] if present else np.full(len(angles), np.inf))
            self_world = transform_segments(truck_segments(lift, steer[js]), base[js, 0], base[js, 1], yaw[js])
            d_self, _ = cast_rays_2d(o2, angles, self_world, layer.range_max_m)
            pal = section_segments(pallet_tris + [0, 0, ppos[2]], h)
            d_pal = (cast_rays_2d(o2, angles, transform_segments(pal, ppos[0], ppos[1], pyaw), layer.range_max_m)[0]
                     if len(pal) else np.full(len(angles), np.inf))
            if carried:
                d_self = np.minimum(d_self, d_pal)
                d_scene = d_static
            else:
                d_scene = np.minimum(d_static, d_pal)
            d_ext = np.minimum(d_scene, d_inj)
            dist = np.minimum(d_ext, d_self)
            hits = np.isfinite(dist)
            own = d_self < d_ext
            limit = OL.beam_limits(origin, directions, band_top_m=layer.band_top_m, band_bottom_m=layer.band_bottom_m)
            ranges = layer.ranges_from(dist, hits, layer.beam_noise(len(dist)))
            odom_rear, correction = scan_inputs(ts, js)
            layer.add_scans(float(ts), {sensor.name: (dist, hits, own, limit, ranges)}, odom_rear=odom_rear,
                            loaded=carried)
            st["scans"] += 1
            n_self = int(own.sum())
            st["self_hit_beams_max"] = max(st["self_hit_beams_max"], n_self)
            st["self_hit_scans"] += int(n_self > 0)
            w_, x_, y_, z_ = base[js, 3:7]
            tilt = float(math.acos(min(1.0, abs(1 - 2 * (x_ * x_ + y_ * y_)))))
            st["max_tilt_rad"] = max(st["max_tilt_rad"], tilt)
            k_bin = int(np.searchsorted(TILT_BINS_DEG, math.degrees(tilt), side="right") - 1)
            label = f"{TILT_BINS_DEG[k_bin]}-{TILT_BINS_DEG[k_bin + 1]}deg"
            row_t = st["tilt_bins"].setdefault(label, {"scans": 0, "min_valid_range_m": layer.range_max_m})
            row_t["scans"] += 1
            row_t["min_valid_range_m"] = min(row_t["min_valid_range_m"], float(min(np.min(limit), layer.range_max_m)))
            if rec_ranges is not None and k < len(rec_ranges) and k % args.free_check_every == 0:
                # Isaac's recorded raycast (pre-noise distances) against this cast, without
                # the synthetic boxes: the sections reproduce the scene to the millimetre.
                rec = rec_ranges[k].astype(float)
                cmp_ = np.minimum(d_scene, d_self)
                ag = st["agreement"]
                both = np.isfinite(rec) & np.isfinite(cmp_) & (cmp_ <= 5.0)
                diff = np.abs(rec[both] - cmp_[both])
                ag["beams"] += int(both.sum())
                ag["within"] += int((diff <= 0.005).sum())
                ag["abs_m"].append(diff)
                missing = np.isfinite(rec) & (rec > 0) & (rec <= 5.0) & (cmp_ > rec + 0.005)
                if missing.any():
                    # Hits the scene here does not hold: the hall walls lie outside the
                    # scenario bounds (the planner never goes there); anything inside them is
                    # geometry this replay lacks.
                    lp = rec_pose[k] if rec_pose is not None else (o2[0], o2[1], float(angles[0] - layer.beam_angles[0]))
                    a_ = lp[2] + layer.beam_angles[missing]
                    hx, hy = lp[0] + rec[missing] * np.cos(a_), lp[1] + rec[missing] * np.sin(a_)
                    in_hall = (hx >= b["x_min_m"]) & (hx <= b["x_max_m"]) & (hy >= b["y_min_m"]) & (hy <= b["y_max_m"])
                    ag["missing_in_hall"] += int(in_hall.sum())
                    ag["missing_outside_hall"] += int((~in_hall).sum())
                ag["extra"] += int(((cmp_ <= 5.0) & ((rec > cmp_ + 0.005) | (rec == np.inf))).sum())
            tr = truth_rear[js]
            if correction != last["correction"]:
                last["version"] += 1
            ctrl = compose(correction, odom_rear)
            ahead, leg_dir = leg_ahead(truth_rear, signed_speed, js, lookahead)
            layer.refresh(float(ts), correction, last["version"], current_pose=ctrl,
                          path_ahead=leg_in_frame(ahead, tr, ctrl), loaded=carried, direction=leg_dir)
            last["correction"], last["scan"] = correction, k
            if phase in evaluated_phases and k % args.free_check_every == 0:
                moving = abs(float(signed_speed[js])) >= args.moving_mps
                outline = loaded if carried else layer.body
                faces = face_unobserved_m(_raw_snapshot(layer, float(ts), correction, last["version"], ctrl), ctrl,
                                          outline, float(ts), free_age)
                bucket = st["face_moving" if moving else "face_stopped"]
                for face, d in faces.items():
                    prev = bucket.get(face, 0.0)
                    bucket[face] = None if (d is None or prev is None) else max(prev, d)
                snap = layer.snapshot
                for i_, rect in enumerate(static):
                    if math.hypot(rect.x_m - tr[0], rect.y_m - tr[1]) > layer.window_m + 2.0:
                        continue
                    # The grid is in the control frame: the prism as the truck sees it there.
                    rect = Rectangle(*to_control(rect.x_m, rect.y_m, tr, ctrl), rect.length_m, rect.width_m,
                                     rect.yaw_rad + (ctrl[2] - tr[2]))
                    n_overlap = _rect_cells_free(snap, _rect_dict(rect))
                    if not n_overlap:
                        continue
                    st["free_cells_overlapping_prisms"] += n_overlap
                    ii, jj = np.nonzero(snap.state == FREE)
                    cx = snap.origin_x_m + (ii + 0.5) * snap.resolution_m
                    cy = snap.origin_y_m + (jj + 0.5) * snap.resolution_m
                    c_, s_ = math.cos(rect.yaw_rad), math.sin(rect.yaw_rad)
                    u_ = (cx - rect.x_m) * c_ + (cy - rect.y_m) * s_
                    v_ = -(cx - rect.x_m) * s_ + (cy - rect.y_m) * c_
                    n_in = int(((np.abs(u_) < rect.length_m / 2) & (np.abs(v_) < rect.width_m / 2)).sum())
                    st["free_centres_in_prisms"] += n_in
                    if len(st["free_in_prism_examples"]) < 20:
                        st["free_in_prism_examples"].append({"t": float(ts), "prism": i_, "overlapping": n_overlap,
                                                             "centres_inside": n_in, "truck": [float(q) for q in tr]})

        k_next = 0
        st["releases"] = {"count": len(releases), "matched_next_scan": 0}
        # Every tick of the control record, from the first: before the first scan there is
        # no snapshot (moving there is uncovered and never permitted), after the last the
        # sensor timeout acts (Codex L1′ 2nd P1-2, 3rd P1-2).
        for j in range(0, len(stamps), args.tick_every):
            t = float(stamps[j])
            while k_next < len(scan_stamps) and scan_stamps[k_next] <= t + 1e-9:
                take_scan(k_next, float(scan_stamps[k_next]))
                k_next += 1
            while last["correction"] is not None and last["release"] < len(releases) \
                    and releases[last["release"]] <= t + 1e-9:
                # A SLAM release between scans: the runner bumps the version and refreshes
                # at that tick with the new applied correction (run_transport.py loop).
                last["release"] += 1
                new, then = release_correction(t)
                st["releases"]["matched_next_scan"] += int(new is not None and then == new)
                if new is not None and new != last["correction"]:
                    last["version"] += 1
                    last["correction"] = new
                    phase_r = phases[j]
                    ctrl_r = control_pose(j, t)
                    ahead_r, dir_r = leg_ahead(truth_rear, signed_speed, j, lookahead)
                    layer.refresh(t, new, last["version"], current_pose=ctrl_r,
                                  path_ahead=leg_in_frame(ahead_r, truth_rear[j], ctrl_r),
                                  loaded=phase_r in loaded_phases, direction=dir_r)
                    st["release_refreshes"] = st.get("release_refreshes", 0) + 1
            phase = phases[j]
            v = float(signed_speed[j])
            if phase not in evaluated_phases or abs(v) < args.moving_mps:
                continue
            st["ticks"] += 1
            carried = phase in loaded_phases
            tr = truth_rear[j]
            ctrl = control_pose(j, t)  # the runner's control pose at this tick
            direction = -1 if v < 0 else 1
            kappa = float(np.mean([math.tan(a) / (g["wheelbase_m"] + math.tan(a) * side * g["track_m"] / 2)
                                   for a, side in ((steer[j][0], 1), (steer[j][1], -1))]))
            layer.permission.last_estop = None
            allowed, reason = layer.limit(t, current_pose=tuple(ctrl), curvature_inv_m=kappa, direction=direction,
                                          loaded=carried, cap_mps=abs(v))
            estop = layer.permission.last_estop
            fp, own_now = layer.footprints(carried, direction)
            cls = f"{'loaded' if carried else 'unloaded'}_{'reverse' if v < 0 else 'forward'}_{'curve' if abs(kappa) > 0.1 else 'straight'}"
            row = st["classes"][cls]
            row["moving"] += 1
            # Coverage (user decision 2026-10-06) read off the permission's own stopping-arc
            # walk: it stops at the first OCCUPIED sample; UNKNOWN (RETAINED outside the
            # present band included), leaving the grid, stale evidence or no scan at all
            # leave it uncovered (Codex L1′ 2nd P2-4).
            if estop is None:
                unobserved = {"why": reason}
            elif estop.blocked in ("unknown", "edge"):
                unobserved = {"why": estop.blocked, "verified_m": estop.verified_m}
            elif t - estop.oldest_free_s > free_age:
                unobserved = {"why": "expired", "age_s": t - estop.oldest_free_s}
            else:
                unobserved = None
            if not unobserved:
                row["covered"] += 1
            elif len(st["uncovered_examples"]) < 40:
                st["uncovered_examples"].append({"t": t, "phase": phase, "cls": cls, "v": v, **unobserved})
            # The verdict judges the stopping-arc check alone (its own limit, as allowed_speed
            # computes it): the runner's allowed speed is at most this, and the path check
            # would get the recorded leg, not the tracker's plan (Codex L1′ 3rd P1-1).
            if estop is None or t - estop.oldest_free_s > layer.permission.config.evidence_max_age_s:
                arc_allowed = 0.0
            else:
                arc_allowed = stopping.speed_for(estop.verified_m)
            permitted = arc_allowed >= abs(v) - 1e-9
            row["permitted"] += int(permitted)
            row["permitted_with_path"] = row.get("permitted_with_path", 0) + int(allowed >= abs(v) - 1e-9)
            if not permitted:
                why = st["blocked_reasons"].setdefault(cls, {})
                why[reason] = why.get(reason, 0) + 1
            # Truth at this tick: what the steering-held stopping volume (world, truth pose) meets.
            dist_stop = stopping.distance_m(v)
            vol, _ = arc_poses(tuple(tr), kappa, direction, dist_stop, 0.025)
            reach = dist_stop + 2.0
            near = np.flatnonzero(np.hypot(static_centres[:, 0] - tr[0], static_centres[:, 1] - tr[1])
                                  <= reach + static_centres[:, 2]) if len(static_centres) else []
            present = boxes_present(t)
            truths = ([(f"prism{i}", static[i]) for i in near] + [(f"new:{oid}", r) for oid, r in boxes_at(t)]
                      + [(f"box{b_}", injected[b_]) for b_ in present])
            for oid, rect in truths:
                if sweep_meets(rect, fp, vol, envelope, direction):
                    while oid in alias:  # to the end of the chain (Codex L1′ 10th P2)
                        oid = alias[oid]
                    row["events"] += 1
                    row["objects"].add(oid)
                    if permitted:
                        row["unpermitted"] += 1
                        if len(st["unpermitted"]) < 40:
                            st["unpermitted"].append({"t": t, "phase": phase, "cls": cls, "v": v,
                                                      "object": oid, "reason": reason})
            if not carried:
                sample = sample_before(t)
                ppos, pyaw = sample["pallet_position_m"], float(sample["pallet_yaw_rad"])
                pal_rect = Rectangle(ppos[0], ppos[1], geometry["pallet_depth_m"], geometry["pallet_width_m"], pyaw)
                if sweep_meets(pal_rect, fp, vol, envelope, direction):
                    pb = st["pallet_by_phase"].setdefault(phase, {"events": 0, "unpermitted": 0})
                    pb["events"] += 1
                    pb["unpermitted"] += int(permitted)
            for b_ in present:
                if shape_meets(injected[b_], fp, tuple(tr)):
                    alive.discard(b_)
        ag = st["agreement"]
        diffs = np.concatenate(ag.pop("abs_m")) if ag["abs_m"] else np.zeros(0)
        ag["within_share"] = ag["within"] / ag["beams"] if ag["beams"] else None
        ag["abs_m"] = ({"p50": float(np.median(diffs)), "p99": float(np.percentile(diffs, 99)), "max": float(diffs.max())}
                       if len(diffs) else None)
        for cls, row in st["classes"].items():
            agg = classes[cls]
            for key in ("moving", "covered", "permitted", "unpermitted"):
                agg[key] += row[key]
            agg["event_objects"] += len(row["objects"])
            row["objects"] = len(row["objects"])
        st["injected_boxes"] = len(injected)
        st["new_obstacles"] = [oid for oid, *_ in spawned]
        st["prisms"] = {"columns": len(prisms), "min_top_m": min((top for _, top in prisms), default=None)}
        shutil.rmtree(snap, ignore_errors=True)
        runs.append({"run": str(run), "run_id": run_id, "result_sha256": result_sha, "inputs_sha256": inputs_sha,
                     "run_success": result.get("success"),
                     "run_failure": result.get("failure_reason"),
                     "evaluated_s": [float(stamps[0]), float(stamps[-1])] if len(stamps) else None,
                     "noise_seed": noise_seed, **st})
        print(run, json.dumps({c: (r["moving"], r["covered"], r["objects"], r["unpermitted"])
                               for c, r in st["classes"].items() if r["moving"]}), flush=True)
    verdict = judge_classes(classes, coverage_min=args.coverage_min, objects_min=args.min_event_objects)
    if code_hashes() != code:
        # The code changed while this ran: the result cannot name the code that made it
        # (Codex L1′ 6th P2-2). ws1 jobs run from their own copy of the tree. The config,
        # the odometry table and the run inputs were parsed from the bytes they are named by.
        raise SystemExit("the code changed during the run -- result discarded")
    if args.tick_every != 1 or args.max_scans is not None:
        # Skipped ticks or a cut record can hide events (Codex L1′ 2nd P1-1/P1-2): a diagnostic.
        verdict = {**verdict, "status": "diagnostic", "pass": False}
    return {"layer_config": str(args.layer_config),
            "layer_config_sha256": config_sha,
            "odometry_age_sha256": age_sha,
            "code_sha256": code,
            "settings": {"inject_every_m": args.inject_every_m, "phases": sorted(evaluated_phases),
                         "moving_mps": args.moving_mps, "tick_every": args.tick_every, "max_scans": args.max_scans,
                         "free_check_every": args.free_check_every},
            "verdict": verdict, "totals": classes, "runs": runs}


def merge_command(args) -> dict:
    """The L1′ verdict over several single outputs (one per run, run in parallel)."""
    classes = {cls: {"moving": 0, "covered": 0, "permitted": 0, "event_objects": 0, "unpermitted": 0}
               for cls in MOTION_CLASSES}
    runs, same = [], set()
    for path in args.inputs:
        part = json.loads(Path(path).read_text())
        if "code_sha256" not in part:
            raise SystemExit(f"{path}: no code version -- rerun it with the current tool")
        same.add(json.dumps([part["layer_config_sha256"], part["odometry_age_sha256"], part["settings"],
                             part["code_sha256"]], sort_keys=True))
        runs += part["runs"]
        for cls, row in part["totals"].items():
            for key in classes[cls]:
                classes[cls][key] += row[key]
    if len(same) != 1:
        raise SystemExit("the parts ran different layer configs, odometry tables, settings or code")
    verdict = judge_classes(classes, coverage_min=args.coverage_min, objects_min=args.min_event_objects)
    settings = json.loads(same.copy().pop())[2]
    if settings.get("tick_every", 1) != 1 or settings.get("max_scans") is not None:
        verdict = {**verdict, "status": "diagnostic", "pass": False}
    ids = [r.get("run_id") for r in runs]
    if None in ids or len(ids) != len(set(ids)):
        raise SystemExit("a run appears twice (or a part has no run_id)")
    return {"parts": [str(p) for p in args.inputs], "same": json.loads(same.pop()), "verdict": verdict,
            "totals": classes, "runs": runs}


def replace_bounds(hall):
    from forklift_core.planning.geometry import Bounds

    # Generous: the study checks obstacles, not the hall outline.
    return Bounds(hall["x_min_m"] - 50, hall["x_max_m"] + 50, hall["y_min_m"] - 50, hall["y_max_m"] + 50)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    s = sub.add_parser("sections")
    s.add_argument("--run", type=Path, required=True)
    s.add_argument("--assets", type=Path, required=True)
    s.add_argument("--heights", default="0.03,0.06,0.08,0.10,0.12,0.14,0.18,0.20,0.25,0.35,0.50,0.75,1.00,1.05")
    s.add_argument("--projection-top-m", type=float, default=1.05)
    s.add_argument("--projection-bottom-m", type=float, default=0.0)
    s.add_argument("--output", type=Path, required=True)
    e = sub.add_parser("evaluate")
    e.add_argument("--run", type=Path, required=True)
    e.add_argument("--sections", type=Path, required=True)
    e.add_argument("--candidates", type=Path, required=True)
    e.add_argument("--odometry-age", type=Path, required=True)
    e.add_argument("--forklift-urdf", type=Path, default=Path("sim/models/dls08_provisional/forklift.urdf"))
    e.add_argument("--stop-latency-s", type=float, required=True)
    e.add_argument("--stop-decel-mps2", type=float, required=True)
    e.add_argument("--stop-margin-m", type=float, default=0.05)
    e.add_argument("--envelope-m", type=float, default=0.01)
    e.add_argument("--envelope-ramp-m", type=float, default=0.0)
    e.add_argument("--body-front-m", type=float, default=0.884)
    e.add_argument("--free-age-s", type=float, default=0.2)
    e.add_argument("--occupied-age-s", type=float, default=0.3)
    e.add_argument("--free-r-cap-m", type=float, default=0.20)
    e.add_argument("--free-rho-m", type=float, default=4.0)
    e.add_argument("--beams", type=int, default=800)
    e.add_argument("--cast-m", type=float, default=5.5)
    e.add_argument("--moving-mps", type=float, default=0.05)
    e.add_argument("--phases", default="observe,approach,transport,return_home")
    e.add_argument("--free-check-every", type=int, default=1)
    e.add_argument("--noise-seed", type=int, default=7)
    e.add_argument("--max-scans", type=int, default=None)
    e.add_argument("--clear-max-height-m", type=float, default=0.15)
    e.add_argument("--close-gap-m", type=float, default=0.0)
    e.add_argument("--free-min-width-m", type=float, default=0.0)
    e.add_argument("--shadow-band-m", type=float, default=0.0)
    e.add_argument("--taper-m", type=float, default=0.0)
    e.add_argument("--shape", choices=("hull", "parts", "hull_forward"), default="hull_forward")
    e.add_argument("--inject-every-m", type=float, default=0.0)
    e.add_argument("--projection-top-m", type=float, default=1.05)
    e.add_argument("--dump-at", type=int, default=None)
    e.add_argument("--dump-path", type=Path, default=Path("p0b_dump.npz"))
    e.add_argument("--dump-class", default=None, help="dump the first non-ok instant of e.g. unloaded_reverse")
    e.add_argument("--output", type=Path)
    # L1′ (plan v10): the single configuration through the runner's ObstacleLayer.
    q = sub.add_parser("single")
    q.add_argument("--run", type=Path, action="append", required=True, help="a prism-collider run dir (repeat)")
    q.add_argument("--layer-config", type=Path, default=ROOT / "config/obstacle_layer_single.yaml")
    q.add_argument("--phases", default="observe,approach,transport,return_home")
    q.add_argument("--moving-mps", type=float, default=0.05)
    q.add_argument("--inject-every-m", type=float, default=0.5)
    q.add_argument("--free-check-every", type=int, default=1)
    q.add_argument("--noise-seed", type=int, default=None, help="default: the run's seed")
    q.add_argument("--max-scans", type=int, default=None)
    q.add_argument("--tick-every", type=int, default=1, help="judge every n-th 120 Hz control tick")
    q.add_argument("--coverage-min", type=float, default=0.80)
    q.add_argument("--min-event-objects", type=int, default=30)
    q.add_argument("--output", type=Path)
    m = sub.add_parser("merge")
    m.add_argument("inputs", type=Path, nargs="+")
    m.add_argument("--coverage-min", type=float, default=0.80)
    m.add_argument("--min-event-objects", type=int, default=30)
    m.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = {"sections": sections_command, "evaluate": evaluate_command, "single": single_command,
              "merge": merge_command}[args.command](args)
    text = json.dumps(result, indent=2)
    if getattr(args, "output", None) and args.command in ("evaluate", "single", "merge"):
        args.output.write_text(text + "\n")
    print(text if args.command == "sections" else "done")


if __name__ == "__main__":
    main()
