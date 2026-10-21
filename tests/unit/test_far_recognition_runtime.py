"""The far recognition's runner side on the CPU rig (docs/plans/2026-10-18-isaac-recognition-far-capture.md)."""
import importlib.util
import json
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


def test_the_lag_fit_does_not_depend_on_proposal_ids_or_order():
    # Codex v2: renaming the same reading to learned_0 turned an accepted 11-step lag into None
    from forklift_core.perception.recognition_bridge import proposals_from_ids
    from forklift_core.perception.instance_proposals import InstanceProposal

    def renamed(scene, ids):
        props = proposals_from_ids(ids)
        return [InstanceProposal(f"learned_{9 - k}", p.mask, 1.0, "oracle") for k, p in enumerate(reversed(props))], ()

    xs = _leg()
    plain, other = _runtime(lag_steps=11), _runtime(lag_steps=11, proposer=renamed)
    _drive(plain, 11, xs)
    _drive(other, 11, xs)
    a, b = plain.lag_fit(), other.lag_fit()
    assert a is not None and b is not None and (a["tau_steps"], a["accepted"]) == (b["tau_steps"], b["accepted"]) == (11, True)


class _TruthModel:
    """A learned model stand-in: the truth masks of the rendered pallet at a fixed score (no truth in the API --
    it reads the RGB's non-background pixels, which the rig paints only on the pallet)."""

    def __init__(self, score=0.9, fail_after=None):
        self.score, self.calls, self.fail_after = score, 0, fail_after

    def propose(self, scene):
        from forklift_core.perception.instance_proposals import InstanceProposal

        self.calls += 1
        if self.fail_after is not None and self.calls > self.fail_after:
            raise RuntimeError("model gone")
        _, ids = self._ids
        return [InstanceProposal("l0", ids == 0, self.score, "learned")] if (ids == 0).any() else []


def _learned_runtime(model, tmp_path=None, tau=0.5):
    source = R.LearnedSource(model)
    run = _runtime(lag_steps=11, out_dir=None if tmp_path is None else tmp_path / "recognition",
                   proposer=source, identity_threshold=tau, proposer_record={"kind": "learned", "tau": tau})
    return run, source


def _drive_learned(run, model, xs, lag=11):
    for tick, x in enumerate(xs):
        base = (float(x), 0.0, 0.0)
        run.record_step(tick, tick * DT, base, base, PALLET)
        if run.due(tick, observing=True) and tick >= lag:
            model._ids = _render((float(xs[tick - lag]), 0.0, 0.0))
            scene, ids = model._ids
            run.drive_frame(tick, tick * DT, scene.rgb, scene.depth_m, ids, scene.intrinsics, scene.base_from_optical)


def _stop_learned(run, model, xs):
    base = (float(xs[-1]), 0.0, 0.0)
    model._ids = _render(base)
    scene, ids = model._ids
    scene = replace(scene, stamp_ns=int(round((len(xs) * DT + 0.1) * 1e9)))
    return run.stop(scene, ids, initial_time_s=0.0, control_base=base, odom_base=base, truth_base=base,
                    truth_pallet=PALLET)


def test_a_confident_learned_reading_is_selected_and_one_below_tau_is_not():
    xs = _leg()
    hi, lo = _TruthModel(0.9), _TruthModel(0.3)
    run_hi, _ = _learned_runtime(hi)
    run_lo, _ = _learned_runtime(lo)
    _drive_learned(run_hi, hi, xs)
    _drive_learned(run_lo, lo, xs)
    assert _stop_learned(run_hi, hi, xs)[0].status == "valid"
    obs, rec = _stop_learned(run_lo, lo, xs)
    assert obs.status != "valid" and rec["status"] != "selected"


def test_a_model_failure_falls_back_to_the_geometric_proposals_and_is_counted():
    xs = _leg()
    model = _TruthModel(0.9, fail_after=3)
    run, source = _learned_runtime(model)
    _drive_learned(run, model, xs)
    obs, rec = _stop_learned(run, model, xs)
    assert source.fallbacks and all("model gone" in r for r in source.fallbacks)
    assert all(p.source == "geometric" for p in run.far.last_proposals)
    assert obs.status != "valid" and rec["status"] != "selected"  # geometric proposals are never confirmed
    assert run.summary()["proposer"]["fallbacks"] == len(source.fallbacks)
    timing = run.summary()["proposer"]["model_ms"]  # plan task 6: the model's time per call (design §6: 200 ms)
    assert timing["n"] == model.calls and 0 <= timing["p50"] <= timing["max"]
    assert timing["first"] <= timing["max"] and timing["max_after_first"] <= timing["max"]


def test_a_learned_run_saves_rgb_and_proposals_and_recomputes_exactly(tmp_path):
    from tools import recompute_far_recognition as RC

    xs = _leg()
    model = _TruthModel(0.9, fail_after=8)  # learned frames, then geometric fallback frames
    run, _ = _learned_runtime(model, tmp_path)
    _drive_learned(run, model, xs)
    _stop_learned(run, model, xs)
    z = np.load(sorted((tmp_path / "recognition").glob("frame_*.npz"))[0])
    assert z["rgb"].shape == (*z["depth_m"].shape, 3) and z["proposal_masks"].dtype == bool
    assert str(z["proposer_source"]) == "learned"
    pairs = RC.recompute(tmp_path, recognition=run.summary())
    assert len(pairs) == 1 and RC.differences(*pairs[0]) == []


def test_an_oracle_run_saves_no_rgb_or_proposals(tmp_path):
    xs = _leg()
    run = _runtime(lag_steps=11, out_dir=tmp_path / "recognition")
    _drive(run, 11, xs)
    z = np.load(sorted((tmp_path / "recognition").glob("frame_*.npz"))[0])
    assert "rgb" not in z.files and "proposal_masks" not in z.files


class _DepthModel:
    """A learned stand-in that reads only the saved frame (the depth components, relabelled learned at a fixed
    score), so a recompute can rerun it."""

    def __init__(self, score):
        self.score = score

    def propose(self, scene):
        from forklift_core.perception import geometric_proposals as G

        return [replace(p, source="learned", score=self.score, proposal_id=f"l{k}")
                for k, p in enumerate(G.propose(scene).proposals)]


def test_rerunning_the_model_recomputes_the_decision_not_only_the_numbers(tmp_path):
    # Codex review 1: scores 0.50025 -> 0.49975 at tau 0.5 flip selected -> reobserve, yet the numeric check passed
    from tools import recompute_far_recognition as RC

    xs = _leg()
    run, _ = _learned_runtime(_DepthModel(0.50025), tmp_path, tau=0.5)
    _drive(run, 11, xs)
    obs, _ = _stop(run, xs)
    assert obs.status == "valid"
    summary = run.summary()
    same = RC.recompute(tmp_path, recognition=summary, model=_DepthModel(0.50025))
    assert [RC.differences(*p) for p in same] == [[]]
    flipped = RC.recompute(tmp_path, recognition=summary, model=_DepthModel(0.49975))
    assert any(RC.differences(*p) for p in flipped)


def test_a_zone_narrowed_for_a_later_leg_is_recorded_and_recomputed_per_leg(tmp_path):
    """Plan 2026-10-19 (the bottom-deck re-observation): the zone becomes the judged pallet's centre for the legs
    after the change; the summary keeps the change by leg and the recompute replays every stop with its zone."""
    from tools import recompute_far_recognition as RC

    run = _runtime(out_dir=tmp_path / "recognition")
    xs = _leg()
    _drive(run, 0, xs)
    obs0, rec0 = _stop(run, xs)
    run.set_zone(PALLET[:2], 0.3)
    assert run.zone_radius_m == 0.3 and run.zone_changes == [{"leg": 1, "centre_xy": list(PALLET[:2]),
                                                             "radius_m": 0.3}]
    _drive(run, 0, xs)
    obs1, rec1 = _stop(run, xs)
    assert obs1.status == "valid"
    summary = run.summary()
    assert summary["zone"]["radius_m"] == 0.6 and summary["zone_changes"][0]["leg"] == 1  # the first zone kept
    (tmp_path / "result.json").write_text(json.dumps({"recognition": summary}))
    pairs = RC.recompute(tmp_path)
    assert len(pairs) == 2 and all(a.get("status") == b.get("status") for a, b in pairs)


def test_the_recompute_needs_the_zone_change_to_match_a_stop_the_narrowed_zone_turns_away(tmp_path):
    """Codex ecf4be5 P2: a zone moved 0.4 m off the pallet with radius 0.3 m turns the second stop away; the
    recompute matches only with the recorded change, and without it reads the first zone and selects again."""
    from tools import recompute_far_recognition as RC

    run = _runtime(out_dir=tmp_path / "recognition")
    xs = _leg()
    _drive(run, 0, xs)
    obs0, _ = _stop(run, xs)
    assert obs0.status == "valid"
    run.set_zone((PALLET[0] + 0.4, PALLET[1]), 0.3)
    _drive(run, 0, xs)
    obs1, _ = _stop(run, xs)
    assert obs1.status != "valid"
    summary = run.summary()
    (tmp_path / "result.json").write_text(json.dumps({"recognition": summary}))
    assert all(not RC.differences(a, b) for a, b in RC.recompute(tmp_path))
    stale = {**summary, "zone_changes": []}
    assert any(RC.differences(a, b) for a, b in RC.recompute(tmp_path, recognition=stale))


def test_a_stop_the_forks_do_not_fit_records_the_candidate_decided_on():
    # Codex T11 review P2: the mission ends at once on fork_spacing_adjust only for a known shape's fixed dimensions,
    # so the stop names the candidate (and its hypothesis) the not_pickable decision was made on.
    run = R.FarRuntime(fork_spacing=0.29, pallet_depth_m=EPAL.overall_depth_m, pallet_shape="epal6",
                       zone_centre_xy=PALLET[:2], zone_radius_m=0.6, camera_x_m=CAMERA_X, dt=DT)
    xs = _leg()
    _drive(run, 0, xs)
    obs, rec = _stop(run, xs)
    assert rec["status"] == "not_pickable" and "fork_spacing_adjust" in rec["reasons"]
    assert rec["decided"]["hypothesis"].startswith("epal6") and rec["decided"]["pickable"] is False
