"""RearAxlePathTracker.set_input_offset (docs/plans/2026-11-03-near-end-stand-c2-injection.md 1.2, 2): the rigid
input offset about the final goal follows the moved path exactly, leaves the path arrays alone, and is cleared inside
update() before an update that would leave its clear distance -- so no arrival is latched under it."""

import math

import numpy as np

from forklift_core.control import RearAxlePathTracker, TrackerConfig


def reverse_arc(length=1.6, curvature=0.29, n=60):
    s = np.linspace(0.0, length, n)
    yaw = -curvature * s  # reversing: heading changes by -kappa per metre driven backwards along +x
    x = -np.cumsum(np.r_[0.0, np.diff(s)] * np.cos(yaw))
    y = -np.cumsum(np.r_[0.0, np.diff(s)] * np.sin(yaw))
    poses = np.column_stack((x, y, yaw))
    return poses, np.full(n, -1), np.full(n, curvature)


def moved(poses, dx, dpsi):
    g = poses[-1]
    c, s = math.cos(dpsi), math.sin(dpsi)
    rel = poses[:, :2] - g[:2]
    out = poses.copy()
    out[:, 0] = g[0] + c * rel[:, 0] - s * rel[:, 1] - dx * math.sin(g[2])
    out[:, 1] = g[1] + s * rel[:, 0] + c * rel[:, 1] + dx * math.cos(g[2])
    out[:, 2] = poses[:, 2] + dpsi
    return out


def config():
    return TrackerConfig(max_curvature_inv_m=0.36, lookahead_m=0.28, position_tolerance_m=0.03, yaw_tolerance_rad=0.05)


def lockstep(a, b, pose, steps=4000, dt=1 / 120):
    """Drive a's commands (a midpoint bicycle), feeding b the same pose and measured speed each tick."""
    pose = np.array(pose, dtype=float)
    speed = 0.0
    for _ in range(steps):
        ca, cb = a.update(pose, speed, dt), b.update(pose, speed, dt)
        yield ca, cb
        speed = ca.speed_mps
        change = speed * ca.curvature_inv_m * dt
        pose[:2] += speed * dt * np.array([math.cos(pose[2] + change / 2), math.sin(pose[2] + change / 2)])
        pose[2] += change
        if ca.status in {"arrived", "failed"}:
            break


def same(ca, cb):
    for field in ("speed_mps", "curvature_inv_m", "position_error_m", "yaw_error_rad"):
        assert math.isclose(getattr(ca, field), getattr(cb, field), abs_tol=1e-9), field
    assert ca.status == cb.status


def test_the_offset_tracker_follows_the_moved_path_exactly():
    poses, gears, curvatures = reverse_arc()
    dx, dpsi = -0.06, 0.15
    reference = RearAxlePathTracker(moved(poses, dx, dpsi), gears, curvatures, config())
    offset = RearAxlePathTracker(poses, gears, curvatures, config())
    offset.set_input_offset(dx, dpsi, -1.0)  # never cleared: the whole run under the offset
    statuses = []
    for ca, cb in lockstep(reference, offset, moved(poses, dx, dpsi)[0] + [0.01, 0.02, -0.02]):
        same(ca, cb)
        assert math.isclose(offset.progress_m(), reference.progress_m(), abs_tol=1e-9)
        assert offset.leg_index == reference.leg_index
        statuses.append(ca.status)
    assert statuses[-1] == "arrived"
    # the arrival came under the offset here (clear distance -1): the tracker says so
    assert [e["event"] for e in offset.input_offset_events] == ["arrived_under_offset"]


def test_once_cleared_it_is_the_plain_tracker():
    poses, gears, curvatures = reverse_arc()
    plain = RearAxlePathTracker(poses, gears, curvatures, config())
    cleared = RearAxlePathTracker(poses, gears, curvatures, config())
    cleared.set_input_offset(-0.06, 0.15, 1e9)  # cleared before the first update runs
    for ca, cb in lockstep(plain, cleared, poses[0] + [0.02, -0.01, 0.03]):
        same(ca, cb)
        assert plain._last_eval == cleared._last_eval
    assert [e["event"] for e in cleared.input_offset_events] == ["cleared"] and not cleared.input_offset_active()
    assert plain.input_offset_events == [] and not plain.input_offset_active()


def test_the_path_arrays_and_leg_ahead_stay_the_original_path():
    poses, gears, curvatures = reverse_arc()
    tracker = RearAxlePathTracker(poses, gears, curvatures, config())
    tracker.set_input_offset(-0.06, 0.15, 0.10)
    tracker.update(poses[0], 0.0, 1 / 120)
    assert np.array_equal(tracker.path_arrays()[0], poses)
    ahead, _ = tracker.leg_ahead()
    assert np.allclose(ahead[1:], poses[tracker._segment + 1:])
    assert tracker.input_offset_active()


def test_the_offset_is_cleared_before_an_update_that_would_arrive_under_it():
    # Codex second review: 0.11 m left before the update, the next input the moved goal -- under the offset its
    # error is 0 and it would arrive; cleared first, the update reads the given pose 0.06 m / 0.15 rad off.
    poses, gears, curvatures = reverse_arc()
    tracker = RearAxlePathTracker(poses, gears, curvatures, config())
    tracker._progress = float(tracker._distance[-1] - 0.11)
    tracker._segment = int(np.searchsorted(tracker._distance, tracker._progress, side="right") - 1)
    tracker.set_input_offset(-0.06, 0.15, 0.10)
    at_moved_goal = moved(poses, -0.06, 0.15)[-1]
    cmd = tracker.update(at_moved_goal, 0.0, 1 / 120)
    assert not tracker.input_offset_active()
    assert [e["event"] for e in tracker.input_offset_events] == ["cleared"]
    assert cmd.status != "arrived" and not tracker._arrived
    assert tracker._last_eval[0] == [float(v) for v in at_moved_goal]
    assert math.isclose(cmd.position_error_m, 0.06, abs_tol=1e-9)
    for _ in range(50):
        assert tracker.update(at_moved_goal, 0.0, 1 / 120).status != "arrived"
