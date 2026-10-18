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


def _payload(grid, labels):
    return {"data": np.asarray(grid, dtype=np.uint32), "info": {"idToLabels": labels}}


def test_the_mission_semantics_keep_this_runs_labels_among_the_assets_own():
    grid = np.zeros((480, 640), dtype=np.uint32)
    grid[:, 300:] = 2
    grid[100:200, 100:200] = 3
    out = R.decode_mission_semantic(_payload(grid, {"0": {"class": "floor,pillar"}, "2": {"class": "pillar"},
                                                     "3": {"class": "inst_4,pallet"}}))
    assert out.ready and int(out.instance_ids[150, 150]) == 4 and int(out.instance_ids[0, 400]) == -1
    assert int(out.instance_ids[0, 0]) == -1  # the background is not an instance


def test_the_mission_semantics_need_the_background_label():
    grid = np.zeros((480, 640), dtype=np.uint32)
    assert not R.decode_mission_semantic(_payload(grid, {"0": {"class": "pillar"}})).ready
    assert not R.decode_mission_semantic(None).ready


def test_two_instance_labels_on_one_id_are_a_contract_error():
    grid = np.zeros((480, 640), dtype=np.uint32)
    with pytest.raises(ValueError):
        R.decode_mission_semantic(_payload(grid, {"0": {"class": "inst_0,inst_1"}}))


def test_the_lag_fit_reads_only_counted_readings():
    xs = _leg()
    run = _runtime(lag_steps=11)
    _drive(run, 11, xs)
    for f in run.leg_frames[::2]:  # half the readings uncounted and far off: they must not move the fit
        for c in f["candidates"]:
            c["geometry"], c["pallet_centre_m"] = "ambiguous", [c["pallet_centre_m"][0] + 0.5, 0.0, 0.0]
    fit = run.lag_fit()
    assert fit is not None and fit["tau_steps"] == 11 and fit["accepted"]


def test_a_target_lost_while_driving_starts_a_new_observation_leg():
    xs = _leg()
    run = _runtime(lag_steps=11)
    _drive(run, 11, xs[:200])
    first = run.leg
    assert run.far.tracker.selected is not None
    empty = np.full_like(_render((0.0, 0.0, 0.0))[1], -1)  # the pallet out of view for 1.3 s
    scene, _ = _render((float(xs[199]), 0.0, 0.0))
    for k in range(200, 360):
        run.record_step(k, k * DT, (float(xs[199]), 0.0, 0.0), (float(xs[199]), 0.0, 0.0), PALLET)
        if run.due(k, observing=True):
            run.drive_frame(k, k * DT, scene.rgb, scene.depth_m, empty, scene.intrinsics, scene.base_from_optical)
    assert run.leg == first + 1
    ended = run.leg_endings[-1]
    assert ended["leg"] == first and ended["reason"] == "target_lost_while_driving" and ended["selection_history"]
    for k in range(360, 480):  # the pallet back in view: the new leg chooses afresh
        run.record_step(k, k * DT, (float(xs[199]), 0.0, 0.0), (float(xs[199]), 0.0, 0.0), PALLET)
        if run.due(k, observing=True):
            s2, ids2 = _render((float(xs[199]), 0.0, 0.0))
            run.drive_frame(k, k * DT, s2.rgb, s2.depth_m, ids2, s2.intrinsics, s2.base_from_optical)
    assert run.far.tracker.selected is not None


def test_the_lag_fit_says_whether_it_agrees_with_the_lag_used():
    xs = _leg()
    run = _runtime(lag_steps=11)
    _drive(run, 11, xs)
    assert run.lag_fit()["agrees_with_used"] is True
    wrong = _runtime(lag_steps=0)
    _drive(wrong, 11, xs)
    assert wrong.lag_fit()["agrees_with_used"] is False


def test_the_recompute_reproduces_a_leg_ended_by_a_loss(tmp_path):
    from tools import recompute_far_recognition as RC

    xs = _leg()
    run = _runtime(lag_steps=11, out_dir=tmp_path / "recognition")
    _drive(run, 11, xs[:200])
    x = float(xs[199])
    scene, ids = _render((x, 0.0, 0.0))
    for k in range(200, 480):
        run.record_step(k, k * DT, (x, 0.0, 0.0), (x, 0.0, 0.0), PALLET)
        if run.due(k, observing=True):
            shown = ids if k >= 360 else np.full_like(ids, -1)
            run.drive_frame(k, k * DT, scene.rgb, scene.depth_m, shown, scene.intrinsics, scene.base_from_optical)
    stop = replace(scene, stamp_ns=int(round(480 * DT * 1e9)))
    obs, rec = run.stop(stop, ids, initial_time_s=0.0, control_base=(x, 0.0, 0.0), odom_base=(x, 0.0, 0.0),
                        truth_base=(x, 0.0, 0.0), truth_pallet=PALLET)
    assert obs.status == "valid" and run.leg_endings and rec["leg"] == 1
    pairs = RC.recompute(tmp_path, recognition=run.summary())
    assert [RC.differences(a, b) for a, b in pairs] == [[]]


def test_a_lag_that_changes_between_the_halves_disagrees_with_the_lag_used():
    # Codex: first half 24, second half 11 -- the whole leg fits nothing, each half fits exactly
    xs = _leg()
    run = _runtime(lag_steps=24)
    for tick, x in enumerate(xs):
        base = (float(x), 0.0, 0.0)
        run.record_step(tick, tick * DT, base, base, PALLET)
        if run.due(tick, observing=True) and tick >= 24:
            lag = 24 if tick < 240 else 11
            scene, ids = _render((float(xs[tick - lag]), 0.0, 0.0))
            run.drive_frame(tick, tick * DT, scene.rgb, scene.depth_m, ids, scene.intrinsics, scene.base_from_optical)
    assert run.lag_fit()["agrees_with_used"] is False


def test_the_recompute_fails_when_saved_frames_are_missing(tmp_path):
    from tools import recompute_far_recognition as RC

    xs = _leg()
    run = _runtime(lag_steps=11, out_dir=tmp_path / "recognition")
    _drive(run, 11, xs)
    _stop(run, xs)
    stop_file = sorted((tmp_path / "recognition").glob("frame_*.npz"))[-1]
    stop_file.unlink()
    pairs = RC.recompute(tmp_path, recognition=run.summary())
    assert any(RC.differences(a, b) for a, b in pairs)
    for f in (tmp_path / "recognition").glob("frame_*.npz"):
        f.unlink()
    assert any(RC.differences(a, b) for a, b in RC.recompute(tmp_path, recognition=run.summary()))
