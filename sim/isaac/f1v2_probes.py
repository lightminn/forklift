"""F1 v2 controlled probes in Isaac (docs/plans/2026-11-04-f1-v2-registration.md sections 2-3), run from
run_transport's --f1v2-probe branch (off by default; nothing here runs in a mission).

delivery: the delivery match at fixed poses on the line_2 scene. The reference scan is taught as the runner teaches it
(the delivery rear pose, the yaw-only quaternion, the spawn-settled base height, the truck left out), with the scene
pallet still at the pickup. The pallet is then seated on the forks at a nominal pose and lifted with the operating lift
command; at every grid point the truck and the pallet are moved together, held (wheels at zero velocity, steering at
zero, the lift at its command) until settled, and scanned: (a) actual pose, (b) (a) plus K fixed noise draws, (c) the
actual origin with a yaw-only orientation, (d) the actual orientation with the origin at the nominal base height 0.
Each scan is matched as the mission core matches it (f1v2_probe_core.evaluate), with the truth estimate and with v1's
representative estimate error.
"""

import json
import math
import os
from pathlib import Path

import numpy as np

import f1v2_probe_core as core_

PALLET_ON_FORKS = (0.929, -0.012, -0.003)  # pallet origin in the base frame at transport (v1 line_2 stop samples)
LIFT_COMMAND_M = None  # the settings' lift_target_m (the operating transport command), read at run time
SETTLE_WINDOW_STEPS = 30  # 0.25 s at 120 Hz
SETTLE_MAX_STEPS = 360  # 3 s
SETTLE_TOL = {"base_z_m": 5e-5, "pitch_rad": 1e-7, "roll_rad": 1e-7, "lift_m": 5e-5}
POSE_TOL_M, POSE_TOL_RAD = 1e-3, 1e-4  # the laser pose after settling against the grid target


def _yaw_quat(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def _rpy(q_wxyz) -> tuple:
    w, x, y, z = (float(v) for v in q_wxyz)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def _hold(robot, action_type, wheels, steers, lift_index, lift_command):
    robot.apply_action(action_type(joint_velocities=np.zeros(len(wheels)), joint_indices=wheels))
    robot.apply_action(action_type(joint_positions=np.zeros(len(steers)), joint_indices=steers))
    robot.apply_action(action_type(joint_positions=np.array([lift_command]), joint_indices=lift_index))


def _state(robot, lift_index) -> dict:
    base, q = robot.get_world_pose()
    roll, pitch, yaw = _rpy(q)
    return {"base": [float(v) for v in base], "q": [float(v) for v in q], "roll_rad": roll, "pitch_rad": pitch,
            "yaw_rad": yaw, "lift_m": float(robot.get_joint_positions()[lift_index[0]])}


def _settle(world, robot, action_type, wheels, steers, lift_index, lift_command) -> dict:
    """Hold and step until the base height, pitch, roll and lift change less than SETTLE_TOL over a window."""
    prev = _state(robot, lift_index)
    steps, deltas = 0, None
    while steps < SETTLE_MAX_STEPS:
        for _ in range(SETTLE_WINDOW_STEPS):
            _hold(robot, action_type, wheels, steers, lift_index, lift_command)
            world.step(render=False)
        steps += SETTLE_WINDOW_STEPS
        now = _state(robot, lift_index)
        deltas = {"base_z_m": abs(now["base"][2] - prev["base"][2]),
                  "pitch_rad": abs(now["pitch_rad"] - prev["pitch_rad"]),
                  "roll_rad": abs(now["roll_rad"] - prev["roll_rad"]), "lift_m": abs(now["lift_m"] - prev["lift_m"])}
        prev = now
        if all(deltas[k] < SETTLE_TOL[k] for k in SETTLE_TOL):
            return {"converged": True, "steps": steps, "last_window": deltas}
    return {"converged": False, "steps": steps, "last_window": deltas}


def _relative(base, base_yaw, pos, q):
    """A pose in the base frame (planar position, height, yaw)."""
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    d = np.asarray(pos, float) - np.asarray(base, float)
    return (c * d[0] + s * d[1], -s * d[0] + c * d[1], float(d[2]), _rpy(q)[2] - base_yaw)


def _place(robot, pallet, base_xy, yaw, base_z, pallet_rel):
    """Move the truck and the pallet together (the pallet at pallet_rel in the new base frame), velocities zero."""
    robot.set_world_pose(position=np.array([base_xy[0], base_xy[1], base_z]), orientation=_yaw_quat(yaw))
    robot.set_linear_velocity(np.zeros(3))
    robot.set_angular_velocity(np.zeros(3))
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    c, s = math.cos(yaw), math.sin(yaw)
    px = base_xy[0] + c * pallet_rel[0] - s * pallet_rel[1]
    py = base_xy[1] + s * pallet_rel[0] + c * pallet_rel[1]
    pallet.set_world_pose(position=np.array([px, py, base_z + pallet_rel[2]]), orientation=_yaw_quat(yaw + pallet_rel[3]))
    pallet.set_linear_velocity(np.zeros(3))
    pallet.set_angular_velocity(np.zeros(3))


def run_delivery_probe(world, robot, pallet, *, args, settings, geometry, destination, initial_base, lift_index, wheels,
                       steers, action_type, out_dir: Path, order: str = "forward", extra=None) -> dict:
    import planar_lidar
    import yaml

    from forklift_core.planning.pallet_mission import site_poses
    from forklift_core.sensors.lidar import PlanarScanPattern

    cfg = yaml.safe_load(Path(args.lidar).read_text())
    pattern = PlanarScanPattern(cfg["beam_count"], float(cfg["range_min_m"]), float(cfg["range_max_m"]))
    mount = planar_lidar.LaserMount(tuple(map(float, cfg["mount_xyz_m"])), float(cfg["mount_yaw_rad"]))
    angles = pattern.beam_angles_rad()
    offset = abs(float(args.rear_axle_offset_m))
    lift_command = float(settings["lift_target_m"])
    taught = site_poses(destination, geometry)["delivery"]
    delivery = (float(taught.x_m), float(taught.y_m), float(taught.yaw_rad))
    base_z = float(initial_base[2])
    # the reference, as teach_docking_reference casts it (pallet at the pickup, the truck left out)
    base_d = np.array([delivery[0] + offset * math.cos(delivery[2]), delivery[1] + offset * math.sin(delivery[2]), base_z])
    origin, dirs = planar_lidar.laser_rays_world(base_d, _yaw_quat(delivery[2]), mount, angles)
    dist, hits, own = planar_lidar.cast_scan(origin, dirs, pattern.range_max_m, ignore_self=True)
    reference_ranges = pattern.ranges_from_hits(dist, hits)
    reference_points = core_.laser_points(reference_ranges, angles)
    rear_from_laser = (offset + float(mount.xyz_m[0]), float(mount.xyz_m[1]), float(mount.yaw_rad))
    points = core_.grid(delivery, offset, mount.xyz_m[:2], mount.yaw_rad)
    for e in extra or []:  # v1 operating stop poses (the operating comparison)
        base, rear = core_.poses_from_laser(e["laser"], offset, mount.xyz_m[:2], mount.yaw_rad)
        points.append({"label": e["label"], "d_m": delivery[0] - rear[0], "laser": list(e["laser"]), "base": base,
                       "rear": rear, "v1": e})
    if order == "reverse":
        points = points[::-1]
    # seat the pallet on the forks at the first point (lift down, forks in the openings), then lift and settle
    limit = os.environ.get("FORKLIFT_F1V2_PROBE_LIMIT")  # smoke runs only: the first N points
    if limit:
        points = points[:int(limit)]
    first = points[0]
    _place(robot, pallet, first["base"][:2], first["base"][2], base_z, (*PALLET_ON_FORKS[:2], 0.0, PALLET_ON_FORKS[2]))
    seat = _settle(world, robot, action_type, wheels, steers, lift_index, 0.0)
    lift = _settle(world, robot, action_type, wheels, steers, lift_index, lift_command)
    s0 = _state(robot, lift_index)
    pos_p, q_p = pallet.get_world_pose()
    pallet_rel = _relative(s0["base"], s0["yaw_rad"], pos_p, q_p)
    rows, raw = [], {}
    for index, p in enumerate(points):
        _place(robot, pallet, p["base"][:2], p["base"][2], s0["base"][2], pallet_rel)
        settle = _settle(world, robot, action_type, wheels, steers, lift_index, lift_command)
        st = _state(robot, lift_index)
        base, q = np.asarray(st["base"]), np.asarray(st["q"])
        laser = planar_lidar.laser_pose_2d(base, q, mount)
        pos_p, q_p = pallet.get_world_pose()
        truth_rear = (base[0] - offset * math.cos(st["yaw_rad"]), base[1] - offset * math.sin(st["yaw_rad"]),
                      st["yaw_rad"])
        casts = {}
        o, dv = planar_lidar.laser_rays_world(base, q, mount, angles)
        d_a, h_a, _ = planar_lidar.cast_scan_flags(o, dv, pattern.range_max_m,
                                                    own_prefixes=(planar_lidar.SELF_PREFIX, "/World/Pallet"))
        r_a = pattern.ranges_from_hits(d_a, h_a)
        casts["a"] = core_.sent_ranges(r_a, np.zeros(len(r_a)), pattern.range_min_m, pattern.range_max_m)
        for k in range(core_.NOISE_K):
            casts[f"b{k}"] = core_.sent_ranges(r_a, core_.noise_vector(len(r_a), index if order == "forward"
                                                                       else len(points) - 1 - index, k),
                                               pattern.range_min_m, pattern.range_max_m)
        o, dv = planar_lidar.laser_rays_world(base, _yaw_quat(st["yaw_rad"]), mount, angles)
        d_c, h_c, _ = planar_lidar.cast_scan_flags(o, dv, pattern.range_max_m,
                                                    own_prefixes=(planar_lidar.SELF_PREFIX, "/World/Pallet"))
        casts["c"] = core_.sent_ranges(pattern.ranges_from_hits(d_c, h_c), np.zeros(len(r_a)), pattern.range_min_m,
                                       pattern.range_max_m)
        o, dv = planar_lidar.laser_rays_world(np.array([base[0], base[1], 0.0]), q, mount, angles)
        d_d, h_d, _ = planar_lidar.cast_scan_flags(o, dv, pattern.range_max_m,
                                                    own_prefixes=(planar_lidar.SELF_PREFIX, "/World/Pallet"))
        casts["d"] = core_.sent_ranges(pattern.ranges_from_hits(d_d, h_d), np.zeros(len(r_a)), pattern.range_min_m,
                                       pattern.range_max_m)
        evals = {}
        for name, sent in casts.items():
            for ei, off in enumerate(core_.ESTIMATE_OFFSETS):
                estimate = (truth_rear[0] + off[0], truth_rear[1] + off[1], truth_rear[2] + off[2])
                evals[f"{name}/e{ei}"] = core_.evaluate(
                    reference_points, sent, angles, rear_from_laser=rear_from_laser, estimate_rear=estimate,
                    truth_rear=truth_rear, delivery_rear=delivery, reference_rear=delivery)
        raw[p["label"]] = np.stack([casts[n] for n in sorted(casts)]).astype(np.float32)
        rows.append({
            "label": p["label"], "grid_index": index if order == "forward" else len(points) - 1 - index,
            "d_m": p["d_m"], "target_laser": [float(v) for v in p["laser"]],
            "laser": [float(v) for v in laser], "pose_ok": (math.hypot(laser[0] - p["laser"][0], laser[1] - p["laser"][1])
                                                           <= POSE_TOL_M and abs(laser[2] - p["laser"][2]) <= POSE_TOL_RAD),
            "settle": settle, "state": st,
            "pallet_rel": [float(v) for v in _relative(st["base"], st["yaw_rad"], pos_p, q_p)],
            "v1": p.get("v1"), "evals": evals})
    np.savez_compressed(out_dir / "f1v2_delivery_ranges.npz", order=np.array(sorted(casts)),
                        **{k.replace(".", "p"): v for k, v in raw.items()})
    out = {"probe": "delivery", "order": order, "delivery_rear": list(delivery), "base_z_reference_m": base_z,
           "rear_from_laser": list(rear_from_laser), "lift_command_m": lift_command,
           "pallet_on_forks_nominal": list(PALLET_ON_FORKS), "seat": seat, "lift_settle": lift,
           "pallet_rel_after_lift": [float(v) for v in pallet_rel],
           "reference_beams": int(np.isfinite(reference_ranges).sum()), "reference_self_hits": int(own),
           "points": rows}
    (out_dir / "f1v2_delivery_probe.json").write_text(json.dumps(out) + "\n")
    return {"points": len(rows), "settle_failures": sum(not r["settle"]["converged"] for r in rows),
            "pose_failures": sum(not r["pose_ok"] for r in rows)}
