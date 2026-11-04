"""F1 v2 delivery-match probe (docs/plans/2026-11-04-f1-v2-registration.md section 2): the parts that need no Isaac --
the pose grid, the fixed noise vectors and the match evaluation (the runner's scan_docking.dock() with the same
arguments as the mission core, plus the weak direction at every start's converged pose and the correction's error
against the truth, as the core's docking record computes it)."""

import math

import numpy as np

from forklift_core.localization import scan_match
from forklift_core.localization.scan_docking import dock, laser_points
from forklift_core.localization.slam_pose import compose, invert

ROUND1_D_M = tuple(round(1.5 + 0.02 * i, 2) for i in range(-6, 6))  # 1.38 .. 1.60: 12 along-track points
ROUND1_LASER_Y_M = (-9.00, -8.955, -8.91)
ROUND1_YAW_RAD = (0.0, 0.02, 0.04)
ROUND2_D_M = tuple(round(0.6 + 0.02 * i, 2) for i in range(-5, 3))  # 0.50 .. 0.64: 8 points
ROUND2_LATERAL_M = (-0.08, 0.0, 0.08)
ROUND2_YAW_RAD = 0.02  # the round-1 yaw set's centre
NOISE_SIGMA_M, NOISE_CUT_M, NOISE_K = 0.02, 0.06, 8  # the operating constant model (obstacle_layer_single_team_spec)
ESTIMATE_OFFSETS = ((0.0, 0.0, 0.0), (-0.06, 0.03, 0.0))  # truth, and v1's representative estimate error at the match


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def poses_from_laser(laser, rear_offset_m: float, mount_xy, mount_yaw: float) -> tuple:
    """(base, rear) for a laser pose in the world: base yaw = laser yaw - mount yaw, base = laser - R(yaw) mount_xy,
    rear = base - rear_offset (cos, sin)(yaw) -- the inverse of planar_lidar.laser_pose_2d with the rear axle behind."""
    yaw = _wrap(float(laser[2]) - float(mount_yaw))
    c, s = math.cos(yaw), math.sin(yaw)
    bx = float(laser[0]) - (c * mount_xy[0] - s * mount_xy[1])
    by = float(laser[1]) - (s * mount_xy[0] + c * mount_xy[1])
    return (bx, by, yaw), (bx - rear_offset_m * c, by - rear_offset_m * s, yaw)


def grid(delivery_rear, rear_offset_m: float, mount_xy, mount_yaw: float) -> list:
    """The registered grid: 12 x 3 x 3 round-1 points (stop distance from the delivery rear, laser y, yaw) and 8 x 3
    round-2 points (distance, lateral offset of the laser from the delivery line, yaw at the centre). The stop distance d
    puts the rear axle d behind the delivery rear along the delivery heading (line_2's is 0)."""
    dx, dy, dyaw = (float(v) for v in delivery_rear)
    if abs(dyaw) > 1e-6:
        raise ValueError(f"the probe grid assumes a delivery heading of 0, got {dyaw}")
    out = []

    def point(label, d, laser_y, yaw):
        # the rear x is fixed by d; the laser y is the grid's (the rear y follows from it)
        base, rear = poses_from_laser((0.0, laser_y, yaw), rear_offset_m, mount_xy, mount_yaw)
        rear_x = dx - d
        shift = rear_x - rear[0]
        base = (base[0] + shift, base[1], base[2])
        rear = (rear_x, rear[1], rear[2])
        laser = (shift, laser_y, yaw)
        out.append({"label": label, "d_m": d, "laser": laser, "base": base, "rear": rear})

    for d in ROUND1_D_M:
        for y in ROUND1_LASER_Y_M:
            for yaw in ROUND1_YAW_RAD:
                point(f"r1_d{d:.2f}_y{y:.3f}_w{yaw:.2f}", d, y, yaw)
    for d in ROUND2_D_M:
        for lat in ROUND2_LATERAL_M:
            point(f"r2_d{d:.2f}_l{lat:+.2f}", d, dy + lat, ROUND2_YAW_RAD)
    return out


def noise_vector(beams: int, point_index: int, k: int) -> np.ndarray:
    """The k-th fixed noise draw at a grid point, the same in both arms (seeded by point and k)."""
    rng = np.random.default_rng([20261104, int(point_index), int(k)])
    return np.clip(rng.normal(0.0, NOISE_SIGMA_M, int(beams)), -NOISE_CUT_M, NOISE_CUT_M)


def sent_ranges(ranges, noise, range_min_m: float, range_max_m: float) -> np.ndarray:
    """The ranges the runner sends from the shared LiDAR (scan_capture.shared_sent_ranges, inlined so this module needs
    no Isaac path): noise on the finite ranges, clipped to the sensor limits."""
    sent = np.asarray(ranges, dtype=float).copy()
    measured = np.isfinite(sent)
    sent[measured] = np.clip(sent[measured] + np.asarray(noise, dtype=float)[measured], range_min_m, range_max_m)
    return sent


def weak_direction(reference, live, pose, inlier_m: float = 0.05) -> dict:
    """The information matrix at a converged pose (scan_match's own rows): smallest eigenvalue and its eigenvector."""
    ref = np.asarray(reference, dtype=float).reshape(-1, 2)
    liv = np.asarray(live, dtype=float).reshape(-1, 2)
    normals = scan_match._normals(ref)
    jac, _, inl, _, _ = scan_match._information_rows(ref, normals, liv, np.asarray(pose, dtype=float), inlier_m)
    if int(inl.sum()) < 3:
        return {"inliers": int(inl.sum()), "min_eigenvalue": 0.0, "weak_vector": None}
    w, v = np.linalg.eigh(jac.T @ jac)
    return {"inliers": int(inl.sum()), "min_eigenvalue": float(w[0]), "weak_vector": [float(x) for x in v[:, 0]]}


def evaluate(reference_points, ranges, beam_angles, *, rear_from_laser, estimate_rear, truth_rear, delivery_rear,
             reference_rear) -> dict:
    """One match as the mission core makes it (core.py 6855-6915): live points from the sent ranges, dock() with the
    core's arguments, then the evaluation-only goal error against the truth and, here, every start's weak direction."""
    live = laser_points(ranges, beam_angles)
    result = dock(reference_points, live, rear_from_laser=tuple(rear_from_laser), estimate_rear=tuple(estimate_rear),
                  delivery_rear=tuple(delivery_rear))
    expected = compose(tuple(estimate_rear), compose(invert(tuple(truth_rear)), tuple(reference_rear)))
    goal_error = None if result.goal_estimate is None else [
        float(result.goal_estimate[0] - expected[0]), float(result.goal_estimate[1] - expected[1]),
        _wrap(float(result.goal_estimate[2] - expected[2]))]
    starts = []
    for (ok, why, q), detail in zip(result.starts, result.start_details):
        starts.append({"accepted": bool(ok), "reason": str(why), "pose": [float(x) for x in q],
                       "information": float(detail["min_information"]),
                       "inlier_fraction": float(detail["inlier_fraction"]),
                       "median_residual_m": detail["median_residual_m"],
                       **weak_direction(reference_points, live, q)})
    return {"accepted": bool(result.accepted), "reason": result.reason, "live_beams": int(len(live)),
            "min_information": min(s["information"] for s in starts) if starts else None,
            "goal_error_vs_truth": goal_error,
            "correction": None if result.correction is None else [float(x) for x in result.correction],
            "starts": starts}


# --- (ii') the cusp arrival probe (plan section 3, amendment 2): the registered paths and one cusp leg's trial ---

CUSP_STOP_NEEDED = 12  # the runner's cusp_stop_needed (run_transport.py, ms.cusp_stop_ticks)
CUSP_STOP_YAW_RATE_RADPS = 0.02  # the runner's cusp_stop_yaw_rate_radps
CUSP_TIME_SPEED_MPS = 0.05  # the time cap is the leg length over this speed (plan section 3)
SYNTHETIC_STEP_M = 0.05  # the planner's sample spacing
SYNTHETIC_CURVATURES = (0.20, 0.36)
SYNTHETIC_LEGS_M = (1.0, 0.6, 0.8)  # straight, arc, then the reverse straight after the cusp
STATUS_CODE = {"tracking": 0, "braking": 1, "arrived": 2, "failed": 3}
FAILURE_CODE = {None: 0, "cross_track": 1, "endpoint_heading": 2}


def path_sha256(path: dict) -> str:
    """The hash of what the tracker reads (poses, directions, curvatures) as float64/int8 bytes."""
    import hashlib

    h = hashlib.sha256()
    h.update(np.asarray(path["poses"], dtype=np.float64).tobytes())
    h.update(np.asarray(path["directions"], dtype=np.int8).tobytes())
    h.update(np.asarray(path["curvatures_inv_m"], dtype=np.float64).tobytes())
    return h.hexdigest()


def leg_bounds(directions) -> list:
    """(start sample, end sample) of every leg, as the tracker splits the path at its gear changes."""
    d = np.asarray(directions)
    ends = (np.flatnonzero(d[1:-1] != d[2:]) + 1).tolist() + [len(d) - 1]
    starts = [0] + ends[:-1]
    return list(zip(starts, ends))


def first_transport_plan(result: dict) -> tuple:
    """The run's first transport plan and where it came from (plan section 3): the first cusp replan's replaced path,
    else the first delivery-straight replacement's original (docking overwrites paths.transport), else paths.transport."""
    replans = result.get("cusp_replans") or []
    if replans:
        return replans[0]["replaced_path"], "cusp_replans[0].replaced_path"
    for e in result.get("paths_replaced") or []:
        if e.get("leg") == "transport" and e.get("by") == "delivery_straight":
            return e["path"], "paths_replaced[first transport/delivery_straight].path"
    return (result.get("paths") or {})["transport"], "paths.transport"


def registered_paths(results: dict) -> dict:
    """Every registered path version of the v1 runs ({run name: result.json dict}): the first transport plan and each
    cusp replan's path, deduplicated by hash, with the versions the rule leaves out listed (the transport plans a
    scene replan replaced before any of the registered ones)."""
    versions, omitted = [], []
    seen = set()
    for run in sorted(results):
        r = results[run]
        first, source = first_transport_plan(r)
        entries = [(f"{run}/first", source, first)]
        entries += [(f"{run}/replan{i}", f"cusp_replans[{i}].path", c["path"])
                    for i, c in enumerate(r.get("cusp_replans") or [])]
        for vid, src, p in entries:
            digest = path_sha256(p)
            if digest in seen:
                omitted.append({"id": vid, "source": src, "sha256": digest, "why": "duplicate"})
                continue
            seen.add(digest)
            versions.append({"id": vid, "run": run, "source": src, "sha256": digest,
                             "poses": [[float(v) for v in q] for q in p["poses"]],
                             "directions": [int(v) for v in p["directions"]],
                             "curvatures_inv_m": [float(v) for v in p["curvatures_inv_m"]],
                             "cusp_legs": len(leg_bounds(p["directions"])) - 1})
        if not (r.get("cusp_replans") or []):
            # a scene replan before the first delivery straight replaced a transport plan the rule cannot reach
            for e in r.get("paths_replaced") or []:
                if e.get("leg") == "transport" and e.get("by") == "delivery_straight":
                    break
                if e.get("leg") == "transport":
                    omitted.append({"id": f"{run}/{e.get('by')}@{float(e.get('time_s', 0.0)):.1f}",
                                    "source": "paths_replaced", "sha256": path_sha256(e["path"]),
                                    "why": "replaced before the first registered plan"})
    return {"versions": versions, "omitted": omitted}


def synthetic_path(start_rear, curvature: float) -> dict:
    """Plan section 3 (P2): 1.0 m straight, 0.6 m arc of ``curvature`` (sign = side), a cusp, a 0.8 m reverse straight,
    sampled every SYNTHETIC_STEP_M from ``start_rear``."""
    x, y, yaw = (float(v) for v in start_rear)
    poses, directions, curvatures = [[x, y, yaw]], [1], [0.0]
    pieces = ((SYNTHETIC_LEGS_M[0], 1, 0.0), (SYNTHETIC_LEGS_M[1], 1, float(curvature)), (SYNTHETIC_LEGS_M[2], -1, 0.0))
    for length, direction, kappa in pieces:
        for _ in range(int(round(length / SYNTHETIC_STEP_M))):
            s = direction * SYNTHETIC_STEP_M
            if kappa == 0.0:
                x, y = x + s * math.cos(yaw), y + s * math.sin(yaw)
            else:
                yaw1 = yaw + kappa * s
                x, y = x + (math.sin(yaw1) - math.sin(yaw)) / kappa, y + (math.cos(yaw) - math.cos(yaw1)) / kappa
                yaw = yaw1
            poses.append([x, y, yaw])
            directions.append(direction)
            curvatures.append(kappa)
    directions[0], curvatures[0] = directions[1], curvatures[1]
    return {"poses": poses, "directions": directions, "curvatures_inv_m": curvatures}


class CuspTrial:
    """One cusp leg driven as the mission core drives it, on the truth rear pose (plan section 3).

    The tracker gets the whole path and starts at ``leg`` (skip_legs); every step is the core's tick: tracker update,
    the cusp stop count (core.py cusp_and_observe_recovery: the tracker's speed 0, the planar truth speed within the
    tracker's stop speed, the yaw rate within CUSP_STOP_YAW_RATE_RADPS, CUSP_STOP_NEEDED ticks), then assemble_command
    with the obstacle layer acting (Ackermann, creep, slew_cap_stop with the stopping deceleration and the command
    acceleration, the steering rate limit) and no permission, low-clearance or hold input. A leg the tracker itself
    accepts (its own leg switch) is held from there: zero speed, steering held, until the stop count completes.

    ``record`` adds the endpoint events, read from the update's own results and the tracker's views (status,
    braking_against_motion, cusp_pending, failure, leg_switches): no call changes the tracker or the commands, and the
    command/trajectory hash is the same with it off (the instrumentation equivalence).
    """

    def __init__(self, path: dict, config, leg: int, *, geometry, decel_mps2: float, accel_mps2: float,
                 steering_rate_rad_s: float, dt: float, steering_now, record: bool = True):
        from forklift_core.control.path_tracking import RearAxlePathTracker

        self.poses = np.asarray(path["poses"], dtype=float)
        self.tracker = RearAxlePathTracker(path["poses"], path["directions"], path["curvatures_inv_m"], config)
        bounds = leg_bounds(path["directions"])
        if not 0 <= leg < len(bounds) - 1:
            raise ValueError(f"leg {leg} does not end at a cusp ({len(bounds) - 1} cusp legs)")
        if leg:
            self.tracker.skip_legs(leg)
        self.leg, (self.start_index, self.end_index) = leg, bounds[leg]
        assert self.tracker.leg_end_index == self.end_index
        self.goal = self.poses[self.end_index]
        d = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(self.poses[:, :2], axis=0), axis=1))]
        self.leg_length_m = float(d[self.end_index] - d[self.start_index])
        self.time_limit_s = self.leg_length_m / CUSP_TIME_SPEED_MPS
        self.config, self.geometry, self.dt = config, geometry, float(dt)
        self.decel, self.accel, self.rate = float(decel_mps2), float(accel_mps2), float(steering_rate_rad_s)
        self.steering_command = np.asarray(steering_now, dtype=float).copy()
        self.command_speed = 0.0
        self.held_curvature = 0.0
        self.stop_ticks = 0
        self.ticks = 0
        self.advanced = False
        self.done = False
        self.outcome = None
        self.e_stop = None
        self.stop = None
        self.record = record
        self.events = {"first_at_endpoint": None, "first_exceed_after_endpoint": None, "first_pending": None,
                       "first_failed": None, "leg_switch": None} if record else None
        self._prev_pending = False
        self._prev_failed = False
        self.rows = []
        self.aux_rows = []

    def start_pose(self) -> list:
        return [float(v) for v in self.poses[self.start_index]]

    def yaw_error(self, rear) -> float:
        return _wrap(float(self.goal[2]) - float(rear[2]))

    def position_error(self, rear) -> float:
        return float(math.hypot(float(self.goal[0]) - float(rear[0]), float(self.goal[1]) - float(rear[1])))

    def _note(self, key, t, rear, tracking, **more):
        if self.events[key] is None:
            self.events[key] = {"tick": self.ticks, "t_s": float(t), "rear": [float(v) for v in rear],
                                "position_error_m": self.position_error(rear), "yaw_error_rad": self.yaw_error(rear),
                                "tracker_yaw_error_rad": None if tracking is None else float(tracking.yaw_error_rad),
                                **more}

    def step(self, t: float, rear, signed_speed: float, planar_speed: float, yaw_rate: float, steering_actual,
             aux=()):
        """One tick from the state read after the last physics step. Returns (wheel rates, steering command) to apply,
        or None when the trial has ended (stop confirmed or time cap reached; nothing is applied then). ``aux``: the
        rest of the physical state the hash covers (body height and tilt, lift, the pallet's pose and velocity)."""
        from forklift_core.control.path_tracking import ackermann_command
        from forklift_core.control.speed_chain import creep, slew_cap_stop

        if self.done:
            return None
        rear = np.asarray(rear, dtype=float)
        tracking = None
        if not self.advanced:
            leg_before = self.tracker.leg_index
            tracking = self.tracker.update(rear, float(signed_speed), self.dt)
            if self.tracker.leg_index != leg_before:
                self.advanced = True  # the tracker's own leg switch (an accepted cusp)
            requested, curvature = float(tracking.speed_mps), float(tracking.curvature_inv_m)
            speed_zero = self.advanced or tracking.speed_mps == 0.0
            if self.record:
                self._record_events(t, rear, tracking)
        else:
            requested, curvature, speed_zero = 0.0, self.held_curvature, True
            if self.record and abs(self.yaw_error(rear)) > self._yaw_tolerance():
                self._note("first_exceed_after_endpoint", t, rear, None)  # held after the switch, still the leg's end
        # the core's stop (its condition only: a tracker speed of 0 while driving a leg comes only at its end)
        stopped = (speed_zero and float(planar_speed) <= self.config.stop_speed_mps
                   and abs(float(yaw_rate)) <= CUSP_STOP_YAW_RATE_RADPS)
        self.stop_ticks = self.stop_ticks + 1 if stopped else 0
        self.rows.append([float(t), *[float(v) for v in rear], float(signed_speed), float(planar_speed),
                          float(yaw_rate), *[float(v) for v in steering_actual],
                          float("nan") if tracking is None else float(tracking.speed_mps),
                          float("nan") if tracking is None else float(tracking.curvature_inv_m),
                          -1 if tracking is None else STATUS_CODE[tracking.status],
                          -1 if tracking is None else FAILURE_CODE[tracking.failure],
                          -1 if tracking is None else int(tracking.cusp_pending), int(self.advanced),
                          float("nan") if tracking is None else float(tracking.progress_m),
                          -1 if tracking is None else int(tracking.segment_index)])
        self.aux_rows.append([float(v) for v in aux])
        self.ticks += 1
        if self.stop_ticks >= CUSP_STOP_NEEDED:
            self.done = True
            self.e_stop = self.yaw_error(rear)
            self.stop = {"tick": self.ticks - 1, "t_s": float(t), "rear": [float(v) for v in rear],
                         "yaw_error_rad": self.e_stop, "position_error_m": self.position_error(rear)}
            if self.advanced:
                self.outcome = "release"
            elif tracking is not None and tracking.status == "failed":
                self.outcome = f"failed_{tracking.failure}"
            elif tracking is not None and tracking.cusp_pending:
                self.outcome = "pending"
            else:
                self.outcome = "stopped_not_released"
            self.rows[-1].extend([0.0, *self.steering_command])  # nothing applied after the confirmation
            return None
        if self.ticks * self.dt >= self.time_limit_s:
            self.done, self.outcome = True, "timeout"
            self.rows[-1].extend([0.0, *self.steering_command])
            return None
        hold = self.advanced
        drive = ackermann_command(requested, curvature, self.geometry)
        target = self.steering_command.copy() if hold else np.asarray(drive.steering_rad, dtype=float)
        error = float(np.max(np.abs(target - np.asarray(steering_actual, dtype=float))))
        wheel = creep(requested, error)
        wheel = slew_cap_stop(self.command_speed, wheel, decel_mps2=self.decel, accel_mps2=self.accel, dt=self.dt)
        self.command_speed = wheel
        drive = ackermann_command(wheel, curvature, self.geometry)
        self.steering_command = self.steering_command + np.clip(target - self.steering_command, -self.rate * self.dt,
                                                                self.rate * self.dt)
        if not self.advanced:
            self.held_curvature = curvature
        self.rows[-1].extend([float(drive.speed_mps), *self.steering_command])
        return np.asarray(drive.wheel_rates_rad_s, dtype=float), self.steering_command.copy()

    def _record_events(self, t, rear, tracking):
        """The endpoint events of this update, inferred from its results (amendment 2): at_endpoint holds exactly
        when the update braked for the endpoint -- status braking that is neither a brake against the motion nor a held
        cusp carried over from an earlier update -- or latched endpoint_heading or a held cusp this update, or switched
        the leg this update (status braking, set before the switch)."""
        pending_now, failed_now = bool(tracking.cusp_pending), tracking.status == "failed"
        at_endpoint = ((tracking.status == "braking" and not self.tracker.braking_against_motion
                        and not self._prev_pending)
                       or (failed_now and not self._prev_failed and tracking.failure == "endpoint_heading")
                       or (pending_now and not self._prev_pending))
        if at_endpoint:
            self._note("first_at_endpoint", t, rear, tracking)
        if (self.events["first_at_endpoint"] is not None
                and abs(self.yaw_error(rear)) > self._yaw_tolerance()):
            self._note("first_exceed_after_endpoint", t, rear, tracking)
        if pending_now and not self._prev_pending:
            self._note("first_pending", t, rear, tracking)
        if failed_now and not self._prev_failed:
            self._note("first_failed", t, rear, tracking, failure=tracking.failure)
        if self.advanced and self.events["leg_switch"] is None:
            sw = self.tracker.leg_switches[-1]
            self._note("leg_switch", t, rear, tracking, via=sw["via"])
        self._prev_pending, self._prev_failed = pending_now, failed_now

    def _yaw_tolerance(self) -> float:
        c = self.config
        return c.yaw_tolerance_rad if c.cusp_yaw_tolerance_rad is None else c.cusp_yaw_tolerance_rad

    def trajectory_sha256(self) -> str:
        import hashlib

        h = hashlib.sha256(np.asarray(self.rows, dtype=np.float64).tobytes())
        h.update(np.asarray(self.aux_rows, dtype=np.float64).tobytes())
        return h.hexdigest()

    def summary(self) -> dict:
        return {"leg": self.leg, "start_index": self.start_index, "end_index": self.end_index,
                "leg_length_m": self.leg_length_m, "time_limit_s": self.time_limit_s, "ticks": self.ticks,
                "outcome": self.outcome, "e_stop_rad": self.e_stop, "stop": self.stop, "events": self.events,
                "trajectory_sha256": self.trajectory_sha256()}


TICK_COLUMNS = ("t", "rear_x", "rear_y", "rear_yaw", "signed_speed", "planar_speed", "yaw_rate", "steer_l", "steer_r",
                "trk_speed", "trk_curvature", "trk_status", "trk_failure", "trk_pending", "advanced", "trk_progress",
                "trk_segment", "wheel_speed_cmd", "steer_cmd_l", "steer_cmd_r")
AUX_COLUMNS = ("base_z", "roll", "pitch", "lift", "pallet_x", "pallet_y", "pallet_z", "pallet_qw", "pallet_qx",
               "pallet_qy", "pallet_qz", "pallet_vx", "pallet_vy", "pallet_vz")  # the pallet's only when loaded
