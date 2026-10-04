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
    """Count FREE cells whose centre lies inside a truth rectangle (shrunk by margin)."""
    from forklift_core.perception.obstacle_grid import FREE

    res = snapshot.resolution_m
    nx, ny = snapshot.state.shape
    c, s = math.cos(rect["yaw_rad"]), math.sin(rect["yaw_rad"])
    hl, hw = rect["length_m"] / 2 - margin, rect["width_m"] / 2 - margin
    if hl <= 0 or hw <= 0:
        return 0
    r = math.hypot(hl, hw)
    i0 = max(int((rect["x_m"] - r - snapshot.origin_x_m) / res), 0)
    i1 = min(int((rect["x_m"] + r - snapshot.origin_x_m) / res) + 1, nx)
    j0 = max(int((rect["y_m"] - r - snapshot.origin_y_m) / res), 0)
    j1 = min(int((rect["y_m"] + r - snapshot.origin_y_m) / res) + 1, ny)
    if i0 >= i1 or j0 >= j1:
        return 0
    ii, jj = np.meshgrid(np.arange(i0, i1), np.arange(j0, j1), indexing="ij")
    x = snapshot.origin_x_m + (ii + 0.5) * res - rect["x_m"]
    y = snapshot.origin_y_m + (jj + 0.5) * res - rect["y_m"]
    inside = (np.abs(x * c + y * s) <= hl) & (np.abs(-x * s + y * c) <= hw)
    return int((snapshot.state[i0:i1, j0:j1][inside] == FREE).sum())


def evaluate_command(args) -> dict:
    from dataclasses import replace

    from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
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
    stopping = StoppingModel(args.stop_latency_s, args.stop_decel_mps2, args.stop_margin_m)
    pconfig = PermissionConfig(stopping, args.envelope_m, evidence_max_age_s=args.free_age_s)

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
    owners = list(sections["owners"])
    pickup = meta["mission"]["pickup_truth"]
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

    report = {}
    for cname, cand in candidates.items():
        grid = None
        grid_cfg = GridConfig(
            hall["x_min_m"], hall["x_max_m"], hall["y_min_m"], hall["y_max_m"], table,
            free_max_age_s=args.free_age_s, occupied_max_age_s=args.occupied_age_s,
            free_r_cap_m=args.free_r_cap_m, free_rho_m=args.free_rho_m,
        )
        grid = ObstacleGrid(grid_cfg)
        permission = DrivePermission(pconfig)
        stats = {"events": 0, "unpermitted": [], "moving": {}, "permitted": {}, "free_inside": {}, "scans": 0}
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
            if phase not in evaluated_phases:
                continue
            v = float(signed_speed[j])
            if abs(v) < args.moving_mps:
                continue
            # The correction control would use now: truth o odom^-1 (SLAM error is studied separately).
            tr, od = truth_rear[j], odom[j]
            dyaw = tr[2] - od[2]
            cc, ss = math.cos(dyaw), math.sin(dyaw)
            correction = (tr[0] - (cc * od[0] - ss * od[1]), tr[1] - (ss * od[0] + cc * od[1]), dyaw)
            window = replace(grid_cfg, x_min_m=round(tr[0] - 6, 1), x_max_m=round(tr[0] + 6, 1),
                             y_min_m=round(tr[1] - 6, 1), y_max_m=round(tr[1] + 6, 1))
            grid.config = window
            snap = grid.snapshot(float(t), correction)
            grid.config = grid_cfg
            # Path ahead = what the truck actually drove next (rear axle).
            seg = np.hypot(*np.diff(truth_rear[j:, :2], axis=0).T)
            s_cum = np.concatenate(([0.0], np.cumsum(seg)))
            end = int(np.searchsorted(s_cum, pconfig.lookahead_m)) + 1
            ahead = truth_rear[j : j + max(end, 2)]
            fp = loaded if carried else unloaded
            permission.evaluate(snap, ahead, fp, body, current_pose=tuple(tr))
            if args.dump_at is not None and k == args.dump_at:
                only = ObstacleGrid(window)
                for sc in list(grid.scans)[-len(cand["sensors"]):]:
                    only.add_scan(sc)
                alone = only.snapshot(float(t), correction)
                np.savez(str(args.dump_path) + ".alone.npz", state=alone.state,
                         stamps=[sc.stamp_s for sc in grid.scans], t=float(t))
                np.savez(args.dump_path, state=snap.state, origin=[snap.origin_x_m, snap.origin_y_m],
                         res=snap.resolution_m, ahead=ahead, pose=tr, verified=permission.evaluation.verified_m,
                         blocked=str(permission.evaluation.blocked))
            allowed, reason = permission.allowed_speed(float(t), {s["name"]: float(t) for s in cand["sensors"]})
            curve = abs(float(np.tan(st).mean())) > 0.1
            cls = f"{'loaded' if carried else 'unloaded'}_{'reverse' if v < 0 else 'forward'}_{'curve' if curve else 'straight'}"
            stats["moving"][cls] = stats["moving"].get(cls, 0) + 1
            permitted = allowed >= abs(v) - 1e-9
            stats["permitted"][cls] = stats["permitted"].get(cls, 0) + int(permitted)
            if not permitted:
                why = stats.setdefault("blocked_reasons", {}).setdefault(cls, {})
                why[reason] = why.get(reason, 0) + 1
            # Truth: does any floor obstacle (or the floor pallet) meet the stopping volume?
            s_stop = stopping.distance_m(v)
            send = int(np.searchsorted(s_cum, s_stop)) + 1
            vol = truth_rear[j : j + max(send, 2)]
            truths = list(obstacle_rects)
            if not carried and phase != "approach":
                truths.append(Rectangle(ppos[0], ppos[1], geometry["pallet_depth_m"], geometry["pallet_width_m"], pyaw))
            checker = FootprintCollisionChecker(truths, fp, replace_bounds(hall))
            hit = any(not checker.free(tuple(p), args.envelope_m) for p in vol[1:])
            if hit:
                stats["events"] += 1
                if permitted:
                    stats["unpermitted"].append({"t": float(t), "phase": phase, "v": v, "reason": reason})
            if k % args.free_check_every == 0:
                for o, kind in zip(obstacles, rect_kind):
                    n = _rect_cells_free(snap, o, margin=0.05)
                    if n:
                        stats["free_inside"][kind] = stats["free_inside"].get(kind, 0) + n
        ratio = {c: stats["permitted"][c] / stats["moving"][c] for c in stats["moving"]}
        reasons = stats.get("blocked_reasons", {})
        # Coverage: not stopped for lack of observation (occupied blocks depend on
        # the recorded truth-planned path, which a grid plan would route around).
        coverage = {c: 1 - reasons.get(c, {}).get("unknown", 0) / stats["moving"][c] for c in stats["moving"]}
        report[cname] = {
            "rect_kinds": {k: rect_kind.count(k) for k in sorted(set(rect_kind))},
            "sensors": cand["sensors"], "lift_offset_m": cand["lift_offset_m"],
            "scans": stats["scans"], "events": stats["events"],
            "unpermitted_entries": len(stats["unpermitted"]), "unpermitted": stats["unpermitted"][:20],
            "moving_by_class": stats["moving"], "permission_ratio": ratio, "coverage_ratio": coverage,
            "free_cells_inside_obstacles": stats["free_inside"],
            "blocked_reasons": stats.get("blocked_reasons", {}),
        }
        print(cname, json.dumps({k: report[cname][k] for k in ("events", "unpermitted_entries", "permission_ratio", "coverage_ratio")}), flush=True)
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
    s.add_argument("--heights", default="0.08,0.10,0.12,0.14,1.05")
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
    e.add_argument("--envelope-m", type=float, default=0.05)
    e.add_argument("--body-front-m", type=float, default=0.884)
    e.add_argument("--free-age-s", type=float, default=0.2)
    e.add_argument("--occupied-age-s", type=float, default=3.0)
    e.add_argument("--free-r-cap-m", type=float, default=0.20)
    e.add_argument("--free-rho-m", type=float, default=4.0)
    e.add_argument("--beams", type=int, default=800)
    e.add_argument("--cast-m", type=float, default=5.5)
    e.add_argument("--moving-mps", type=float, default=0.05)
    e.add_argument("--phases", default="observe,approach,transport,return_home")
    e.add_argument("--free-check-every", type=int, default=5)
    e.add_argument("--noise-seed", type=int, default=7)
    e.add_argument("--max-scans", type=int, default=None)
    e.add_argument("--clear-max-height-m", type=float, default=0.15)
    e.add_argument("--dump-at", type=int, default=None)
    e.add_argument("--dump-path", type=Path, default=Path("p0b_dump.npz"))
    e.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = {"sections": sections_command, "evaluate": evaluate_command}[args.command](args)
    text = json.dumps(result, indent=2)
    if getattr(args, "output", None) and args.command == "evaluate":
        args.output.write_text(text + "\n")
    print(text if args.command == "sections" else "done")


if __name__ == "__main__":
    main()
