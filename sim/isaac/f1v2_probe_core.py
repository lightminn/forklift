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
