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
    args = parser.parse_args()
    result = {"sections": sections_command, "evaluate": evaluate_command}[args.command](args)
    text = json.dumps(result, indent=2)
    if getattr(args, "output", None) and args.command == "evaluate":
        args.output.write_text(text + "\n")
    print(text if args.command == "sections" else "done")


if __name__ == "__main__":
    main()
