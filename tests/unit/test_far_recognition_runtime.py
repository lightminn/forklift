"""The far recognition's runner side on the CPU rig (docs/plans/2026-10-18-isaac-recognition-far-capture.md)."""
import importlib.util
import math
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from tools import instance_rig
from tools import recognition_scenes as rs
from tools import recognition_truth as rt

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("far_recognition_runtime", ROOT / "sim/isaac/far_recognition_runtime.py")
R = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R)
EPAL = rs.known_spec("epal6")
DT = 1 / 120
PALLET = (4.2, 0.05, 0.02)  # world x, y, yaw of the run's pallet
CAMERA_X = instance_rig.PERCEPTION_CAMERA.xyz_m[0]


def _runtime(**kw):
    return R.FarRuntime(fork_spacing=0.3725, pallet_depth_m=EPAL.overall_depth_m, pallet_shape="epal6",
                        zone_centre_xy=PALLET[:2], zone_radius_m=0.6, camera_x_m=CAMERA_X, dt=DT, **kw)


def _render(base):
    x, y, yaw = R.relative(base, PALLET)
    placed = [rt.placed_boxes(rt.instance_boxes(EPAL, ()), rt.Placement(x, y, yaw, 0.0))]
    out = instance_rig.render_instances(placed)
    return out.scene, out.instance_ids


def _leg(n=480):
    """Rear-axle-free base x(t): accelerate to 0.3 m/s, cruise, decelerate to rest (a speed change at each end)."""
    v = np.where(np.arange(n) < 120, 0.3 * np.arange(n) / 120,
                 np.where(np.arange(n) < 360, 0.3, 0.3 * (n - np.arange(n)) / 120))
    return np.concatenate(([0.0], np.cumsum(v) * DT))[:n]


def _drive(run, lag, xs, every=24):
    for tick, x in enumerate(xs):
        base = (float(x), 0.0, 0.0)
        run.record_step(tick, tick * DT, base, base, PALLET)
        if run.due(tick, observing=True) and tick >= lag:
            scene, ids = _render((float(xs[tick - lag]), 0.0, 0.0))  # the pixels of lag steps ago
            run.drive_frame(tick, tick * DT, scene.rgb, scene.depth_m, ids, scene.intrinsics, scene.base_from_optical)


def _stop(run, xs):
    base = (float(xs[-1]), 0.0, 0.0)
    scene, ids = _render(base)
    scene = replace(scene, stamp_ns=int(round((len(xs) * DT + 0.1) * 1e9)))  # the capture right after the leg
    return run.stop(scene, ids, initial_time_s=0.0, control_base=base, odom_base=base, truth_base=base,
                    truth_pallet=PALLET)


def test_a_driven_leg_gives_a_fused_far_observation_close_to_the_truth():
    xs = _leg()
    run = _runtime(lag_steps=11)
    _drive(run, 11, xs)
    obs, rec = _stop(run, xs)
    assert obs.status == "valid" and rec["source"] == "fused" and rec["observations"] >= 2
    err = rec["used_error"]["errors"]
    assert err["lateral_m"] < 0.02 and err["along_m"] < 0.02 and err["yaw_rad"] < math.radians(2)
    assert rec["truth_face"]["insertion_yaw_rad"] == pytest.approx(PALLET[2], abs=1e-9)
    assert run.leg == 1 and run.far.tracker.selected is None  # the next leg starts afresh


def test_the_leg_recovers_the_pixel_lag_it_was_driven_with():
    xs = _leg()
    run = _runtime(lag_steps=11)
    _drive(run, 11, xs)
    fit = run.lag_fit()
    assert fit is not None and fit["tau_steps"] == 11 and fit["accepted"]


def test_a_wrong_lag_puts_the_frames_at_the_wrong_poses_and_the_fit_says_so():
    xs = _leg()
    run = _runtime(lag_steps=0)
    _drive(run, 11, xs)
    fit = run.lag_fit()
    assert fit is not None and fit["tau_steps"] == 11  # measured against the logged truth, whatever was assumed


def test_paused_or_off_leg_takes_no_driving_frames():
    run = _runtime()
    assert run.due(24, observing=True) and not run.due(25, observing=True)
    run.paused = True
    assert not run.due(24, observing=True)
    assert not _runtime(drive_frames=False).due(24, observing=True)
    assert not _runtime().due(24, observing=False)


def test_a_frame_without_a_logged_pose_or_ready_semantics_is_skipped():
    run = _runtime(lag_steps=11)
    scene, ids = _render((0.0, 0.0, 0.0))
    assert run.drive_frame(24, 24 * DT, scene.rgb, scene.depth_m, ids, scene.intrinsics,
                           scene.base_from_optical)["skipped"] == "no_pose"
    run.record_step(13, 13 * DT, (0, 0, 0), (0, 0, 0), PALLET)
    assert run.drive_frame(24, 24 * DT, scene.rgb, scene.depth_m, None, scene.intrinsics,
                           scene.base_from_optical)["skipped"] == "semantic_not_ready"


def test_the_fork_spacing_comes_from_the_blade_centres():
    assert R.fork_spacing_m(((0.1, 0.9, 0.15, 0.21), (0.1, 0.9, -0.21, -0.15))) == pytest.approx(0.36)


def test_the_saved_frames_recompute_every_stop_exactly(tmp_path):
    from tools import recompute_far_recognition as RC

    xs = _leg()
    run = _runtime(lag_steps=11, out_dir=tmp_path / "recognition")
    _drive(run, 11, xs)
    _stop(run, xs)
    pairs = RC.recompute(tmp_path, recognition=run.summary())
    assert len(pairs) == 1 and RC.differences(*pairs[0]) == []


def test_a_frame_before_the_cameras_first_render_is_skipped():
    run = _runtime()
    run.record_step(24, 24 * DT, (0, 0, 0), (0, 0, 0), PALLET)
    scene, ids = _render((0.0, 0.0, 0.0))
    assert run.drive_frame(24, 24 * DT, None, None, ids, scene.intrinsics,
                           scene.base_from_optical)["skipped"] == "depth_not_ready"
