"""F1 v2 controlled probes in Isaac (docs/plans/2026-11-04-f1-v2-registration.md sections 2-3), run from
run_transport's --f1v2-probe branch (off by default; nothing here runs in a mission).

delivery: the delivery match at fixed poses on the line_2 scene. The reference scan is taught as the runner teaches it
(the delivery rear pose, the yaw-only quaternion, the spawn-settled base height, the truck left out), with the scene
pallet still at the pickup. The pallet is then seated on the forks at a nominal pose and lifted with the operating lift
command; at every grid point the truck and the pallet are moved together, held (wheels at zero velocity, steering at
zero, the lift at its command) until settled, and scanned: (a) actual pose, (b) (a) plus K fixed noise draws, (c) the
actual origin with a yaw-only orientation, (d) the actual orientation with the base at the O arm's settled height at
the same grid point (``reference``: label -> the O run's laser origin z; the O run itself uses its own, so its (d) is
its (a)). (c) keeps (a)'s origin and levels only the directions.
Each scan is matched as the mission core matches it (f1v2_probe_core.evaluate), with the truth estimate and with v1's
representative estimate error.

cusp: every registered cusp leg (f1v2_probe_core.registered_paths, plus the four synthetic paths from the scene start)
driven by f1v2_probe_core.CuspTrial on the truth rear pose, per tracker setting (the v1 transport and observe settings)
and load (the pallet seated on the forks at the transport lift, then the truck alone with the pallet made kinematic and
lifted away), each trial from its leg's start pose after a settle, at the v1 paths' own coordinates in the aisle_rack
scene with its walls' and pallets' collisions off (the runner's probe branch). Contacts between the truck or the pallet
and anything else (the tyres' floor contacts aside) are counted per trial.
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
SETTLE_MAX_STEPS = 720  # 6 s (plan amendment 1: 3 s did not reach the angle tolerance at PhysX jitter)
SETTLE_TOL = {"base_z_m": 5e-5, "pitch_rad": 5e-7, "roll_rad": 5e-7, "lift_m": 5e-5}  # amendment 1: 1e-7 rad < jitter
POSE_TOL_M, POSE_TOL_RAD = 3e-3, 1e-3  # amendment 1: the truck shifts ~1 mm while settling after the move


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


SETTLE_WINDOW_S, SETTLE_MAX_S = 0.25, 15.0  # F2 plan 2 + amendment 1: seconds; 15 s (a compliant truck rocks longer)
# F2 amendment 1: pitch/roll within 2e-6 rad per window for the F2 probes (a compliant truck's lateral rocking did
# not reach 5e-7 in 6 s; 2e-6 rad moves a wheel centre by 0.5 um over the half track, a laser by 2 um at 1.05 m)
SETTLE_TOL_F2 = {"base_z_m": 5e-5, "pitch_rad": 2e-6, "roll_rad": 2e-6, "lift_m": 5e-5}
SETTLE_EXTRA_TOL = {"steer_rad": 5e-5, "pallet_rel_m": 5e-5, "pallet_rel_yaw_rad": 5e-5}  # F2 plan 4.1


def _settle(world, robot, action_type, wheels, steers, lift_index, lift_command, on_step=None, hz=None,
            pallet=None, check_steer=False) -> dict:
    """Hold and step until the base height, pitch, roll and lift change less than SETTLE_TOL over a window;
    ``on_step`` is called after every physics step (the cusp probe's contact count). ``hz`` sets the window and the
    cap in seconds (None: the 120 Hz step counts, as F1 v2 ran). ``check_steer`` adds the steering joint angles and
    ``pallet`` the pallet's pose relative to the truck (F2 plan 4.1)."""
    window = SETTLE_WINDOW_STEPS if hz is None else int(round(SETTLE_WINDOW_S * hz))
    cap = SETTLE_MAX_STEPS if hz is None else int(round(SETTLE_MAX_S * hz))

    def extra():
        out = {}
        if check_steer:
            out["steer"] = np.asarray(robot.get_joint_positions()[steers], dtype=float)
        if pallet is not None:
            st = _state(robot, lift_index)
            pos_p, q_p = pallet.get_world_pose()
            out["pallet_rel"] = np.asarray(_relative(st["base"], st["yaw_rad"], pos_p, q_p), dtype=float)
        return out

    prev, prev_x = _state(robot, lift_index), extra()
    steps, deltas = 0, None
    while steps < cap:
        for _ in range(window):
            _hold(robot, action_type, wheels, steers, lift_index, lift_command)
            world.step(render=False)
            if on_step is not None:
                on_step()
        steps += window
        now, now_x = _state(robot, lift_index), extra()
        deltas = {"base_z_m": abs(now["base"][2] - prev["base"][2]),
                  "pitch_rad": abs(now["pitch_rad"] - prev["pitch_rad"]),
                  "roll_rad": abs(now["roll_rad"] - prev["roll_rad"]), "lift_m": abs(now["lift_m"] - prev["lift_m"])}
        tol = SETTLE_TOL if hz is None else SETTLE_TOL_F2  # F1 v2 (hz None) keeps its own
        ok = all(deltas[k] < tol[k] for k in tol)
        if check_steer:
            deltas["steer_rad"] = float(np.max(np.abs(now_x["steer"] - prev_x["steer"])))
            ok = ok and deltas["steer_rad"] < SETTLE_EXTRA_TOL["steer_rad"]
        if pallet is not None:
            d = now_x["pallet_rel"] - prev_x["pallet_rel"]
            deltas["pallet_rel_m"] = float(np.max(np.abs(d[:3])))
            deltas["pallet_rel_yaw_rad"] = float(abs(d[3]))
            ok = (ok and deltas["pallet_rel_m"] < SETTLE_EXTRA_TOL["pallet_rel_m"]
                  and deltas["pallet_rel_yaw_rad"] < SETTLE_EXTRA_TOL["pallet_rel_yaw_rad"])
        prev, prev_x = now, now_x
        if ok:
            return {"converged": True, "steps": steps, "last_window": deltas}
    return {"converged": False, "steps": steps, "last_window": deltas}


def _common_start(world, robot, pallet, action_type, wheels, steers, lift_index, *, base_xy, yaw, base_z, load: str,
                  lift_cmd: float, hz: float, on_step=None) -> dict:
    """F2 plan 4.1: the common initial state of a trial. The truck goes to the pose with every joint's velocity zero,
    the steering at 0 and the lift at 0 (the spin joints left where they are); loaded, the pallet goes to its nominal
    seat on the forks (PALLET_ON_FORKS at floor height) with the lift down, settles, and is lifted with the operating
    command and settles again -- so no joint is set into the pallet. Returns the settle records."""
    _place(robot, pallet if load == "loaded" else None, base_xy, yaw, base_z,
           (PALLET_ON_FORKS[0], PALLET_ON_FORKS[1], 0.0, PALLET_ON_FORKS[2]))
    q = np.asarray(robot.get_joint_positions(), dtype=float).copy()
    q[np.asarray(steers)] = 0.0
    q[np.asarray(lift_index)] = 0.0
    robot.set_joint_positions(q)
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    held = pallet if load == "loaded" else None
    # ``on_step(include_pallet)``: while the pallet is seated and lifted its floor contact is allowed ("prep") -- the
    # forks take it off the floor during the lift -- and anything else it touches counts; after the settles, every
    # pallet contact but the truck's counts (the caller's own counting; Codex reviews of P2)
    seat_step = None if on_step is None else (lambda: on_step("prep"))
    lift_step = None if on_step is None else (lambda: on_step("prep"))
    # the seat settle does not hold the pallet's pose relative to the truck: resting on the floor it stays while the
    # truck settles (smoke: 6e-5 m drift); its pose on the forks is held once lifted
    out = {"seat": _settle(world, robot, action_type, wheels, steers, lift_index, 0.0, on_step=seat_step, hz=hz,
                           pallet=None, check_steer=True)}
    if load == "loaded":
        out["lift"] = _settle(world, robot, action_type, wheels, steers, lift_index, lift_cmd, on_step=lift_step, hz=hz,
                              pallet=held, check_steer=True)
    # the trial starts from the last settle (loaded: after the lift); the seat's own convergence is recorded
    out["converged"] = out["lift" if load == "loaded" else "seat"]["converged"]
    st = _state(robot, lift_index)
    out["start_steer_rad"] = [float(v) for v in robot.get_joint_positions()[steers]]
    out["lift_m"] = st["lift_m"]
    if held is not None:
        pos_p, q_p = pallet.get_world_pose()
        out["pallet_rel"] = [float(v) for v in _relative(st["base"], st["yaw_rad"], pos_p, q_p)]
    return out


PLANAR_TOL_M, PLANAR_TOL_RAD, PLANAR_TRIES = 5e-5, 1e-5, 5  # F2 plan 4.2: the laser planar pose matching


def _quat_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2),
                              math.cos(yaw / 2), math.sin(yaw / 2))
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                     cr * cp * sy - sr * sp * cy])


def _match_planar(world, robot, pallet, action_type, wheels, steers, lift_index, lift_cmd, hz, mount, target) -> dict:
    """F2 plan 4.2: shift truck and pallet together in the plane (about the laser point, height, roll, pitch and the
    pallet's full relative pose kept) until the laser planar pose is within PLANAR_TOL of the target, settling after
    each shift; at most PLANAR_TRIES shifts."""
    import planar_lidar

    def laser_now():
        base, q = robot.get_world_pose()
        return planar_lidar.laser_pose_2d(np.asarray(base), np.asarray(q), mount)

    tries, settles, history = 0, [], []
    drift = (0.0, 0.0, 0.0)  # the last settle's own planar drift, taken off the next shift in advance (smoke: the
    # truck rolled about 0.5 mm forward on every re-settle, so a plain shift never got within 0.05 mm)
    while True:
        laser = laser_now()
        corr = core_.planar_correction(laser, target)
        history.append([float(v) for v in corr])
        ok = math.hypot(corr[0], corr[1]) <= PLANAR_TOL_M and abs(corr[2]) <= PLANAR_TOL_RAD
        if ok or tries >= PLANAR_TRIES:
            # a point is matched when the pose is within the tolerance and every settle after a shift converged
            return {"matched": bool(ok and all(settles)), "within_tolerance": bool(ok), "shifts": tries,
                    "residual": [float(v) for v in corr], "history": history, "settles": settles}
        pivot = (float(laser[0]), float(laser[1]))
        move = (corr[0] + drift[0], corr[1] + drift[1], corr[2] + drift[2])
        for body in (robot, pallet):
            pos, qb = body.get_world_pose()
            new_pos, new_q = core_.shift_planar(pos, qb, pivot, move)
            body.set_world_pose(position=new_pos, orientation=new_q)
            body.set_linear_velocity(np.zeros(3))
            body.set_angular_velocity(np.zeros(3))
        robot.set_joint_velocities(np.zeros(robot.num_dof))
        placed = laser_now()
        settles.append(_settle(world, robot, action_type, wheels, steers, lift_index, lift_cmd, hz=hz, pallet=pallet,
                               check_steer=True)["converged"])
        after = laser_now()
        drift = core_.planar_correction(after, placed)  # after - placed
        tries += 1


def _relative(base, base_yaw, pos, q):
    """A pose in the base frame (planar position, height, yaw)."""
    c, s = math.cos(base_yaw), math.sin(base_yaw)
    d = np.asarray(pos, float) - np.asarray(base, float)
    return (c * d[0] + s * d[1], -s * d[0] + c * d[1], float(d[2]), _rpy(q)[2] - base_yaw)


def _place(robot, pallet, base_xy, yaw, base_z, pallet_rel):
    """Move the truck and the pallet together (the pallet at pallet_rel in the new base frame), velocities zero; the
    truck alone when ``pallet`` is None."""
    robot.set_world_pose(position=np.array([base_xy[0], base_xy[1], base_z]), orientation=_yaw_quat(yaw))
    robot.set_linear_velocity(np.zeros(3))
    robot.set_angular_velocity(np.zeros(3))
    robot.set_joint_velocities(np.zeros(robot.num_dof))
    if pallet is None:
        return
    c, s = math.cos(yaw), math.sin(yaw)
    px = base_xy[0] + c * pallet_rel[0] - s * pallet_rel[1]
    py = base_xy[1] + s * pallet_rel[0] + c * pallet_rel[1]
    pallet.set_world_pose(position=np.array([px, py, base_z + pallet_rel[2]]), orientation=_yaw_quat(yaw + pallet_rel[3]))
    pallet.set_linear_velocity(np.zeros(3))
    pallet.set_angular_velocity(np.zeros(3))


def run_delivery_probe(world, robot, pallet, *, args, settings, geometry, destination, initial_base, lift_index, wheels,
                       steers, action_type, out_dir: Path, order: str = "forward", extra=None, reference=None,
                       reference_sha256=None, grid_name: str = "full") -> dict:
    """``grid_name`` "full" (F1 v2: 132 points, one seat and lift, poses re-placed with the pallet's planar pose) or
    "f2" (F2 plan 4.2: 22 points, each from the common start of 4.1, the laser planar pose matched by rigid planar
    shifts of truck and pallet to PLANAR_TOL in at most PLANAR_TRIES, a point that does not get there invalid; cast
    (e) adds the target planar laser pose at the actual height, roll and pitch)."""
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
    f2 = grid_name == "f2"
    hz = float(settings["physics_hz"])
    points = (core_.f2_grid if f2 else core_.grid)(delivery, offset, mount.xyz_m[:2], mount.yaw_rad)
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
    seat = lift = None
    if not f2:
        _place(robot, pallet, first["base"][:2], first["base"][2], base_z,
               (*PALLET_ON_FORKS[:2], 0.0, PALLET_ON_FORKS[2]))
        seat = _settle(world, robot, action_type, wheels, steers, lift_index, 0.0)
        lift = _settle(world, robot, action_type, wheels, steers, lift_index, lift_command)
        s0 = _state(robot, lift_index)
        pos_p, q_p = pallet.get_world_pose()
        pallet_rel = _relative(s0["base"], s0["yaw_rad"], pos_p, q_p)
    else:
        pallet_rel = None
    rows, raw = [], {}
    for index, p in enumerate(points):
        planar = None
        if f2:
            settle = _common_start(world, robot, pallet, action_type, wheels, steers, lift_index, base_xy=p["base"][:2],
                                   yaw=p["base"][2], base_z=base_z, load="loaded", lift_cmd=lift_command, hz=hz)
            planar = _match_planar(world, robot, pallet, action_type, wheels, steers, lift_index, lift_command, hz,
                                   mount, p["laser"])
        else:
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
        o_a = o
        # (c): (a)'s own origin, the directions levelled (the yaw-only orientation's beams) -- recomputing the origin
        # from the yaw-only orientation would move it with the mount as well (Codex review P2-4)
        _, dv = planar_lidar.laser_rays_world(base, _yaw_quat(st["yaw_rad"]), mount, angles)
        d_c, h_c, _ = planar_lidar.cast_scan_flags(o_a, dv, pattern.range_max_m,
                                                    own_prefixes=(planar_lidar.SELF_PREFIX, "/World/Pallet"))
        casts["c"] = core_.sent_ranges(pattern.ranges_from_hits(d_c, h_c), np.zeros(len(r_a)), pattern.range_min_m,
                                       pattern.range_max_m)
        # (d): (a)'s origin and directions with the origin's height replaced by the O arm's laser origin height at
        # this point (the O run: its own, so its (d) is its (a))
        ref_z = float(o_a[2]) if reference is None else float(reference[p["label"]])
        o_d = np.array([o_a[0], o_a[1], ref_z])
        _, dv = planar_lidar.laser_rays_world(base, q, mount, angles)
        d_d, h_d, _ = planar_lidar.cast_scan_flags(o_d, dv, pattern.range_max_m,
                                                    own_prefixes=(planar_lidar.SELF_PREFIX, "/World/Pallet"))
        casts["d"] = core_.sent_ranges(pattern.ranges_from_hits(d_d, h_d), np.zeros(len(r_a)), pattern.range_min_m,
                                       pattern.range_max_m)
        if f2:
            # (e): the target planar laser pose at the actual height, roll and pitch (the residual planar error's share)
            roll_, pitch_, yaw_ = _rpy(q)
            q_e = _quat_rpy(roll_, pitch_, yaw_ - core_._wrap(laser[2] - p["laser"][2]))
            o_e, dv = planar_lidar.laser_rays_world(base, q_e, mount, angles)
            o_e = np.array([p["laser"][0], p["laser"][1], o_e[2]])
            d_e, h_e, _ = planar_lidar.cast_scan_flags(o_e, dv, pattern.range_max_m,
                                                        own_prefixes=(planar_lidar.SELF_PREFIX, "/World/Pallet"))
            casts["e"] = core_.sent_ranges(pattern.ranges_from_hits(d_e, h_e), np.zeros(len(r_a)),
                                           pattern.range_min_m, pattern.range_max_m)
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
            "laser": [float(v) for v in laser],
            "pose_ok": (planar["matched"] if f2 else
                        (math.hypot(laser[0] - p["laser"][0], laser[1] - p["laser"][1]) <= POSE_TOL_M
                         and abs(laser[2] - p["laser"][2]) <= POSE_TOL_RAD)),
            "settle": settle, "planar": planar, "state": st, "laser_origin_world": [float(v) for v in o_a], "d_origin_z_m": ref_z,
            "pallet_rel": [float(v) for v in _relative(st["base"], st["yaw_rad"], pos_p, q_p)],
            "v1": p.get("v1"), "evals": evals})
    np.savez_compressed(out_dir / "f1v2_delivery_ranges.npz", order=np.array(sorted(casts)),
                        **{k.replace(".", "p"): v for k, v in raw.items()})
    out = {"probe": "delivery", "grid": grid_name, "physics_hz": hz, "order": order,
           "d_reference": "own" if reference is None else "O",
           "d_reference_sha256": reference_sha256,
           "delivery_rear": list(delivery), "base_z_reference_m": base_z,
           "rear_from_laser": list(rear_from_laser), "lift_command_m": lift_command,
           "pallet_on_forks_nominal": list(PALLET_ON_FORKS), "seat": seat, "lift_settle": lift,
           "pallet_rel_after_lift": None if pallet_rel is None else [float(v) for v in pallet_rel],
           "reference_beams": int(np.isfinite(reference_ranges).sum()), "reference_self_hits": int(own),
           "points": rows}
    (out_dir / "f1v2_delivery_probe.json").write_text(json.dumps(out) + "\n")
    return {"points": len(rows), "settle_failures": sum(not r["settle"]["converged"] for r in rows),
            "pose_failures": sum(not r["pose_ok"] for r in rows)}


ROBOT_PREFIX = "/World/Forklift"
PALLET_PREFIX = "/World/Pallet"
LOAD_STEPS = 12  # the wheel load check: the floor impulses averaged over this many held steps after the settle
LOAD_S = 0.1  # F2 plan 2: the same window in seconds (LOAD_STEPS at 120 Hz)
LOAD_TOL = 0.01  # plan section 3: the wheel loads sum to the truck's (+ the pallet's) weight within 1 %


FLOOR_TOKEN = "/GroundPlane/"  # the warehouse floor collider (steering_probe_arms.FLOOR_TOKEN)


def _pallet_skip(mine: str, other: str, include_pallet) -> bool:
    """Whether a pallet contact is left out: ``include_pallet`` False (a pallet not involved) leaves every pallet
    contact out; "prep" (being seated or lifted) leaves only the pallet's floor contact out -- anything else it touches
    still counts; True counts all."""
    if not mine.startswith(PALLET_PREFIX):
        return False
    if include_pallet is False:
        return True
    return include_pallet == "prep" and FLOOR_TOKEN in other


def _contacts(stage_wheels, include_pallet=True) -> dict:
    """This step's contact report: the tyres' summed impulse z against the floor collider (by the other actor, as
    steering_probe_arms classifies the clean window) and the count of contacts between the truck or the pallet and
    anything else, a tyre on the floor aside."""
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    headers, data = get_physx_simulation_interface().get_contact_report()
    jz, foreign, pairs = 0.0, 0, set()
    for header in headers:
        a0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
        a1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
        ours = [p.startswith(ROBOT_PREFIX) or p.startswith(PALLET_PREFIX) for p in (a0, a1)]
        if all(ours) or not any(ours):
            continue
        mine, other = (a0, a1) if ours[0] else (a1, a0)
        if _pallet_skip(mine, other, include_pallet):
            continue  # a pallet not on the forks (left at the pickup), or its floor contact while seated or lifted
        sign = 1.0 if mine == a1 else -1.0
        for k in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
            c = data[k]
            if mine in stage_wheels and FLOOR_TOKEN in other:
                jz += sign * float(c.impulse[2])
            else:
                foreign += 1
                pairs.add((mine, other))
    return {"jz": jz, "foreign": foreign, "pairs": sorted(pairs)}


def _pallet_away(stage, pallet, world) -> list:
    """The pallet out of the way for empty trials: its bodies kinematic and lifted 10 m (one physics step)."""
    from pxr import Usd, UsdPhysics

    bodies = [p for p in Usd.PrimRange(stage.GetPrimAtPath(PALLET_PREFIX)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for body in bodies:
        UsdPhysics.RigidBodyAPI(body).CreateKinematicEnabledAttr(True)
    position, orientation = pallet.get_world_pose()
    pallet.set_world_pose(position=np.asarray(position, float) + np.array([0.0, 0.0, 10.0]), orientation=orientation)
    world.step(render=False)
    return [str(p.GetPath()) for p in bodies]


def _gravity(stage) -> float:
    from pxr import UsdPhysics

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            g = UsdPhysics.Scene(prim).GetGravityMagnitudeAttr().Get()
            return 9.81 if g is None or not math.isfinite(float(g)) else float(g)
    return 9.81


def run_cusp_probe(world, robot, pallet, *, args, settings, stage, initial_base, lift_index, wheels, steers,
                   action_type, out_dir: Path, registry: dict, robot_mass_kg: float, accel_mps2: float,
                   start_rear, registry_sha256: str, record: bool = True, order: str = "forward",
                   subset: str = "all") -> dict:
    """``subset``: "all" (every registered leg) or "every8" (F2 plan 4.2: the synthetic legs and the registry's cusp
    legs whose running number -- registry order, versions then legs -- is a multiple of 8). Every trial starts from the
    common state (F2 plan 4.1, ``_common_start``): loaded, the pallet is seated and lifted again each time."""
    from forklift_core.control.path_tracking import TrackerConfig

    stage_wheels = tuple(f"{ROBOT_PREFIX}/{n}" for n in ("front_left_wheel", "front_right_wheel", "rear_left_wheel",
                                                          "rear_right_wheel"))
    hz = float(settings["physics_hz"])
    dt = 1.0 / hz
    offset = abs(float(args.rear_axle_offset_m))
    lift_command = float(settings["lift_target_m"])
    pallet_kg = float(settings["pallet_mass_kg"])
    g = _gravity(stage)
    layer = yaml_load(args.obstacle_layer)
    decel = float(layer["permission"]["decel_mps2"])
    accel = float(accel_mps2)  # the runner's COMMAND_ACCEL_MPS2
    configs = {k: TrackerConfig(**registry["tracker_configs"][k]) for k in ("transport", "observe")}
    # the synthetic paths start at the scene's registered start (the same in both arms, not the settled spawn)
    start_rear = tuple(float(v) for v in start_rear)
    synthetic = []
    for kappa in core_.SYNTHETIC_CURVATURES:
        for side in (1, -1):
            p = core_.synthetic_path(start_rear, side * kappa)
            synthetic.append({"id": f"synthetic_k{kappa:.2f}_{'left' if side > 0 else 'right'}", "run": None,
                              "source": "synthetic", "sha256": core_.path_sha256(p), **p, "cusp_legs": 1})
    versions = list(registry["versions"]) + synthetic
    limit = os.environ.get("FORKLIFT_F1V2_PROBE_LIMIT")  # smoke runs only: the first N trials per load
    trials, results, ticks = [], [], {}
    running = {}
    for v in registry["versions"]:
        for leg in range(v["cusp_legs"]):
            running[(v["id"], leg)] = len(running)
    keep = (lambda v, leg: True) if subset == "all" else (
        lambda v, leg: v["source"] == "synthetic" or running[(v["id"], leg)] % 8 == 0)
    for load in ("loaded", "empty"):
        plan = [(load, cname, v, leg) for cname in ("transport", "observe") for v in versions
                for leg in range(v["cusp_legs"]) if keep(v, leg)]
        if order == "reverse":
            plan = plan[::-1]
        if limit:
            plan = plan[:int(limit)]
        trials.extend(plan)
    setup = {}
    base_z = float(initial_base[2])
    current_load = None
    for index, (load, cname, version, leg) in enumerate(trials):
        if load != current_load:
            current_load = load
            if load == "loaded":
                setup["loaded"] = {"init": "common start per trial (F2 plan 4.1)"}
            else:
                bodies = _pallet_away(stage, pallet, world)
                base_z = float(initial_base[2])
                setup["empty"] = {"pallet_bodies_kinematic": bodies,
                                  "pallet_lifted_to_m": np.asarray(pallet.get_world_pose()[0], float).tolist(),
                                  "base_z_m": base_z}
        lift_cmd = lift_command if load == "loaded" else 0.0
        trial = core_.CuspTrial(version, configs[cname], leg, geometry=args.drive_geometry, decel_mps2=decel,
                                accel_mps2=accel, steering_rate_rad_s=float(settings["steering_command_rate_rad_s"]),
                                dt=dt, steering_now=np.zeros(len(steers)), record=record)
        sr = trial.start_pose()
        sb = (sr[0] + offset * math.cos(sr[2]), sr[1] + offset * math.sin(sr[2]))
        foreign, pairs, k = 0, set(), 0
        tally = {"settle": 0, "load": 0, "drive": 0}

        def count(phase_, c_):
            nonlocal foreign
            foreign += c_["foreign"]
            tally[phase_] += c_["foreign"]
            pairs.update(map(tuple, c_["pairs"]))
            return c_

        settle = _common_start(world, robot, pallet, action_type, wheels, steers, lift_index, base_xy=sb, yaw=sr[2],
                               base_z=base_z, load=load, lift_cmd=lift_cmd, hz=hz,
                               on_step=lambda inc: count("settle", _contacts(stage_wheels,
                                                                             inc if load == "loaded" else False)))
        jz = 0.0
        load_steps = int(round(LOAD_S * hz))  # F2 plan 2: seconds (12 steps at 120 Hz)
        for _ in range(load_steps):
            _hold(robot, action_type, wheels, steers, lift_index, lift_cmd)
            world.step(render=False)
            jz += count("load", _contacts(stage_wheels, load == "loaded"))["jz"]
        weight = (robot_mass_kg + (pallet_kg if load == "loaded" else 0.0)) * g
        load_ratio = -jz / (load_steps * dt) / weight
        st = _state(robot, lift_index)
        start_actual = (st["base"][0] - offset * math.cos(st["yaw_rad"]), st["base"][1] - offset * math.sin(st["yaw_rad"]),
                        st["yaw_rad"])
        while True:
            base, q = robot.get_world_pose()
            yaw = _rpy(q)[2]
            rear = (float(base[0]) - offset * math.cos(yaw), float(base[1]) - offset * math.sin(yaw), yaw)
            v = np.asarray(robot.get_linear_velocity(), dtype=float)
            signed = float(v[0] * math.cos(yaw) + v[1] * math.sin(yaw))
            roll_, pitch_, _ = _rpy(q)
            aux = [float(base[2]), roll_, pitch_, float(robot.get_joint_positions()[lift_index[0]])]
            if load == "loaded":
                pp_, pq_ = pallet.get_world_pose()
                aux += [float(v) for v in pp_] + [float(v) for v in pq_] + [float(v) for v in pallet.get_linear_velocity()]
            out = trial.step(k * dt, rear, signed, float(np.linalg.norm(v[:2])),
                             float(robot.get_angular_velocity()[2]), robot.get_joint_positions()[steers], aux=aux)
            if out is None:
                break
            rates, steer_cmd = out
            robot.apply_action(action_type(joint_velocities=rates, joint_indices=wheels))
            robot.apply_action(action_type(joint_positions=steer_cmd, joint_indices=steers))
            robot.apply_action(action_type(joint_positions=np.array([lift_cmd]), joint_indices=lift_index))
            world.step(render=False)
            count("drive", _contacts(stage_wheels, load == "loaded"))
            k += 1
        tid = f"{version['id']}/leg{leg}/{cname}/{load}"
        ticks[f"t{index:04d}"] = np.asarray(trial.rows, dtype=np.float64)
        ticks[f"a{index:04d}"] = np.asarray(trial.aux_rows, dtype=np.float64)
        results.append({"index": index, "id": tid, "version": version["id"], "path_sha256": version["sha256"],
                        "leg": leg, "config": cname, "load": load, "start_target": sr,
                        "start_actual": [float(v) for v in start_actual],
                        "start_offset_m": float(math.hypot(start_actual[0] - sr[0], start_actual[1] - sr[1])),
                        "start_yaw_offset_rad": float(core_._wrap(start_actual[2] - sr[2])),
                        "settle": settle, "wheel_load_ratio": float(load_ratio),
                        "wheel_load_ok": bool(abs(load_ratio - 1.0) <= LOAD_TOL),
                        "foreign_contacts": foreign, "foreign_by_phase": tally, "foreign_pairs": sorted(pairs),
                        **trial.summary()})
    np.savez_compressed(out_dir / "f1v2_cusp_ticks.npz", columns=np.array(core_.TICK_COLUMNS),
                        aux_columns=np.array(core_.AUX_COLUMNS), **ticks)
    out = {"probe": "cusp", "order": order, "record": record, "dt_s": dt, "gravity_mps2": g,
           "robot_mass_kg": robot_mass_kg, "pallet_mass_kg": pallet_kg, "lift_command_m": lift_command,
           "decel_mps2": decel, "accel_mps2": accel, "tracker_configs": registry["tracker_configs"],
           "scene_start_rear": list(start_rear), "setup": setup, "subset": subset, "init": "common",
           "registry_sha256": registry_sha256,
           "versions": [{k: v[k] for k in ("id", "run", "source", "sha256", "cusp_legs")} for v in versions],
           "omitted": registry.get("omitted", []), "trials": results}
    (out_dir / "f1v2_cusp_probe.json").write_text(json.dumps(out) + "\n")
    return {"trials": len(results), "settle_failures": sum(not r["settle"]["converged"] for r in results),
            "load_failures": sum(not r["wheel_load_ok"] for r in results),
            "contact_trials": sum(r["foreign_contacts"] > 0 for r in results),
            "timeouts": sum(r["outcome"] == "timeout" for r in results)}


def yaml_load(path) -> dict:
    import yaml

    return yaml.safe_load(Path(path).read_text())


# --- F2 plan section 3 / M1: the static probe (per-wheel floor force, height, axle tilt) ---

WHEEL_LINKS = ("front_left_wheel", "front_right_wheel", "rear_left_wheel", "rear_right_wheel")
STATIC_WINDOW_S = 0.5  # F2 plan 3: the window after the settle


def _wheel_floor(stage_wheels, include_pallet=True) -> dict:
    """This step's contact report per tyre against the floor collider: the summed impulse z on the tyre (sign: on the
    truck), the contact point count and the smallest separation; and the foreign contact count (the truck or the pallet
    against anything but each other, a tyre on the floor aside)."""
    from omni.physx import get_physx_simulation_interface
    from pxr import PhysicsSchemaTools

    headers, data = get_physx_simulation_interface().get_contact_report()
    rows = {w: {"jz": 0.0, "n": 0, "min_sep": math.inf} for w in stage_wheels}
    foreign = 0
    for header in headers:
        a0 = str(PhysicsSchemaTools.intToSdfPath(header.actor0))
        a1 = str(PhysicsSchemaTools.intToSdfPath(header.actor1))
        ours = [p.startswith(ROBOT_PREFIX) or p.startswith(PALLET_PREFIX) for p in (a0, a1)]
        if all(ours) or not any(ours):
            continue
        mine, other = (a0, a1) if ours[0] else (a1, a0)
        if _pallet_skip(mine, other, include_pallet):
            continue
        sign = 1.0 if mine == a1 else -1.0
        for k in range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data):
            c = data[k]
            if mine in rows and FLOOR_TOKEN in other:
                r = rows[mine]
                r["jz"] += sign * float(c.impulse[2])
                r["n"] += 1
                r["min_sep"] = min(r["min_sep"], float(c.separation))
            else:
                foreign += 1
    return {"wheels": rows, "foreign": foreign}


def _axle_tilt(q_wxyz) -> float:
    """The wheel axle's angle from the horizontal: the link's local y (the spin axis, URDF axis 0 1 0) in the world."""
    w, x, y, z = (float(v) for v in q_wxyz)
    axle_z = 2 * (y * z + w * x)  # the z component of R @ (0, 1, 0)
    return math.asin(max(-1.0, min(1.0, abs(axle_z))))


def run_static_probe(world, robot, pallet, *, args, settings, stage, initial_base, lift_index, wheels, steers,
                     action_type, out_dir: Path, robot_mass_kg: float, order: str = "forward") -> dict:
    """At the scene's start pose, empty then loaded: the common start (4.1), then a STATIC_WINDOW_S window recording
    each wheel's floor force (impulse z / dt), contact point count, smallest separation, link-centre height and axle
    tilt, and the body height, roll, pitch and lift. The calibration judge compares an arm with the rigid arm of the
    same mass (F2 plan 3)."""
    from steering_probe_arms import _link_poses

    hz = float(settings["physics_hz"])
    dt = 1.0 / hz
    stage_wheels = tuple(f"{ROBOT_PREFIX}/{n}" for n in WHEEL_LINKS)
    g = _gravity(stage)
    lift_command = float(settings["lift_target_m"])
    pallet_kg = float(settings["pallet_mass_kg"])
    base0, q0 = robot.get_world_pose()
    yaw0 = _rpy(q0)[2]
    base_z = float(initial_base[2])
    from scene import tyre_contact_readback

    readback = tyre_contact_readback(stage)
    out = {"probe": "static", "physics_hz": hz, "tyre_readback": readback, "order": order, "gravity_mps2": g, "robot_mass_kg": robot_mass_kg,
           "pallet_mass_kg": pallet_kg, "lift_command_m": lift_command,
           "start_base": [float(base0[0]), float(base0[1]), yaw0], "loads": {}}
    for load in (("empty", "loaded") if order == "forward" else ("loaded", "empty")):
        if load == "empty" and order != "forward":
            _pallet_away(stage, pallet, world)  # the reversed run: the seated pallet leaves before the empty truck
        foreign = {"n": 0}
        loaded = load == "loaded"

        def count(include_pallet):
            foreign["n"] += _wheel_floor(stage_wheels, include_pallet if loaded else False)["foreign"]

        start = _common_start(world, robot, pallet, action_type, wheels, steers, lift_index, base_xy=base0[:2],
                              yaw=yaw0, base_z=base_z, load=load, lift_cmd=lift_command if load == "loaded" else 0.0,
                              hz=hz, on_step=count)
        lift_cmd = lift_command if load == "loaded" else 0.0
        steps = int(round(STATIC_WINDOW_S * hz))
        series = {w: {"fz": [], "n": [], "min_sep": [], "z": [], "tilt": []} for w in WHEEL_LINKS}
        body = {"z": [], "roll": [], "pitch": [], "lift": []}
        for _ in range(steps):
            _hold(robot, action_type, wheels, steers, lift_index, lift_cmd)
            world.step(render=False)
            c = _wheel_floor(stage_wheels, loaded)
            foreign["n"] += c["foreign"]
            poses = _link_poses(robot)
            for w, path in zip(WHEEL_LINKS, stage_wheels):
                r = c["wheels"][path]
                series[w]["fz"].append(-r["jz"] / dt)  # the floor's push on the tyre, upward positive
                series[w]["n"].append(r["n"])
                series[w]["min_sep"].append(r["min_sep"])
                p, q = poses[w]
                series[w]["z"].append(float(p[2]))
                series[w]["tilt"].append(_axle_tilt(q))
            st = _state(robot, lift_index)
            body["z"].append(st["base"][2])
            body["roll"].append(st["roll_rad"])
            body["pitch"].append(st["pitch_rad"])
            body["lift"].append(st["lift_m"])

        def summary(xs):
            a = np.asarray(xs, dtype=float)
            fin = a[np.isfinite(a)]
            return {"mean": float(fin.mean()) if len(fin) else None, "min": float(fin.min()) if len(fin) else None,
                    "max": float(fin.max()) if len(fin) else None}

        weight = (robot_mass_kg + (pallet_kg if load == "loaded" else 0.0)) * g
        total = sum(summary(series[w]["fz"])["mean"] or 0.0 for w in WHEEL_LINKS)
        out["loads"][load] = {
            "start": start, "window_steps": steps, "foreign_contacts": foreign["n"], "tyre_readback": readback,
            "wheel_load_ratio": total / weight,
            "wheels": {w: {k: summary(v) for k, v in series[w].items()} for w in WHEEL_LINKS},
            "body": {k: summary(v) for k, v in body.items()}}
    (out_dir / "f1v2_static_probe.json").write_text(json.dumps(out) + "\n")
    return {"loads": {k: {"ratio": v["wheel_load_ratio"], "converged": v["start"]["converged"],
                          "foreign": v["foreign_contacts"]} for k, v in out["loads"].items()}}
