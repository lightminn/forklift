"""Plan D8b ③ observation errors and ⑤ correspondence on a synthetic recorded run."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams
from tools import d8b_calibration as CAL
from tools import d8b_observations as OBS
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("pocket_frame_recorder", ROOT / "sim/isaac/pocket_frame_recorder.py")
RECORDER = importlib.util.module_from_spec(SPEC)
sys.modules["pocket_frame_recorder"] = RECORDER
SPEC.loader.exec_module(RECORDER)

GEOMETRY = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
PRIOR = load_pallet_prior(ROOT / "config/pallet_prior_epal6.yaml")
MODEL = CAL.PalletModel.from_geometry(GEOMETRY)
FRONT = DetectorParams.derived_for(PRIOR, range_min_m=0.1)
URDF = "sim/models/dls08_measured/forklift.urdf"
K = {"fx": 465.741156, "fy": 465.741156, "cx": 319.5, "cy": 239.5, "width": 640, "height": 480}
MOUNT = scene_rig.Camera((0.619, 0.0, 0.27), 0.10).base_from_optical()
PALLET_X = 5.0
DELAY = 11 / 120


class Stand:
    """A Run-like holder for rendering the frames before the run is written."""

    def __init__(self, truth):
        self.truth = truth
        self.intrinsics = CAL.Intrinsics.from_meta(K)


def make_run(tmp_path, *, speed=0.15, start_d=1.3, end_d=0.7, every=36):
    face = PALLET_X - GEOMETRY.overall_depth_m / 2
    times, xs, x, v, t = [], [], 0.0, 0.0, 0.0
    while start_d - x > end_d:  # the camera starts start_d from the face
        times.append(t)
        xs.append(x)
        v = min(speed, v + 0.3 / 120)
        x += v / 120
        t += 1 / 120
    times, xs = np.array(times), np.array(xs)
    base_x = face - start_d - MOUNT.translation_m[0] + xs
    pallet = np.tile([PALLET_X, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0], (len(times), 1))
    base = np.column_stack((base_x, np.zeros((len(times), 2)), np.tile([1.0, 0, 0, 0], (len(times), 1))))
    data = {"stamp_s": times, "base_pose": base, "lift_m": np.zeros(len(times)), "pallet_pose": pallet,
            "phase": np.full(len(times), 2, dtype=np.int8), "phases": np.asarray(RECORDER.PHASES)}
    stand = Stand(CAL.Truth(data, CAL.Mount(np.asarray(MOUNT.rotation), np.asarray(MOUNT.translation_m))))
    truck = scene_rig.truck_boxes(ROOT / URDF, lift_m=0.0)
    recorder = RECORDER.PocketFrameRecorder(tmp_path)
    recorder.phase = "approach"
    for k in range(len(times)):
        recorder.record_tick(stamp_s=times[k], rendered=False, base_pose=base[k].tolist(), lift_m=0.0,
                             pallet_pose=pallet[k].tolist())
    for k in range(every, len(times), every):
        s = times[k]
        depth, _ = OBS.render_world(stand, s - DELAY, GEOMETRY, truck)
        recorder.record_read(read_s=s, stamp_s=s, rendering_frame=k, depth_m=depth, lift_m=0.0)
    recorder.close({"camera": {"intrinsics": K}, "mount": {"rotation": np.asarray(MOUNT.rotation).tolist(),
                                                          "translation_m": list(MOUNT.translation_m)},
                    "rear_axle_offset_m": -0.34, "forklift_urdf": URDF,
                    "near_capture": {"near_estimate_m": {"x_m": PALLET_X + 0.01, "y_m": 0.015, "yaw_rad": 0.004}}}, [])
    rear = np.column_stack((base_x - 0.34, np.zeros(len(times)), np.zeros(len(times))))
    np.save(tmp_path / "slam_control.npy", np.column_stack((times, rear, rear)))
    (tmp_path / "result.json").write_text(json.dumps({"seed": 1, "arguments": {"slam_noise_seed": 1}}))
    return CAL.Run.load(tmp_path)


@pytest.fixture(scope="module")
def run(tmp_path_factory):
    return make_run(tmp_path_factory.mktemp("obs"))


def test_the_truth_pockets_sit_on_the_face_left_first(run):
    truth = OBS.truth_in_base(run, 1.0, GEOMETRY)
    assert truth["left"][1] > 0 > truth["right"][1]
    assert truth["left"][1] == pytest.approx(GEOMETRY.opening_centre_offset_m)
    assert truth["left"][2] == pytest.approx(GEOMETRY.opening_centre_height_m)
    assert truth["yaw"] == pytest.approx(0.0)
    face_x = PALLET_X - GEOMETRY.overall_depth_m / 2
    base_x = CAL.pose_at(run.truth.t, run.truth.base, 1.0)[1][0]
    assert truth["left"][0] == pytest.approx(face_x - base_x)


def test_carry_moves_a_point_by_the_relative_transform():
    point, yaw = OBS.carry(np.array([1.0, 0.0, 0.1]), 0.0, np.array([0.0, 0.0, 0.0]),
                           np.array([0.5, 0.0, 0.0]))
    np.testing.assert_allclose(point, [0.5, 0.0, 0.1], atol=1e-12)
    point, yaw = OBS.carry(np.array([1.0, 0.0, 0.0]), 0.0, np.zeros(3), np.array([0.0, 0.0, math.pi / 2]))
    np.testing.assert_allclose(point[:2], [0.0, -1.0], atol=1e-12)
    assert yaw == pytest.approx(-math.pi / 2)


def test_the_chain_observes_every_frame_and_its_errors_are_small(run):
    rows = OBS.observe_run(run, DELAY, GEOMETRY, PRIOR, FRONT, MODEL)
    kept = [r for r in rows if "skipped" not in r]
    assert len(kept) >= 4
    # The near-capture prior starts the chain; the roof tracker runs on every frame.
    assert all(r["roof"] is not None for r in kept)
    valid_front = [r for r in kept if r["front"]["errors"]]
    assert valid_front, [r["front"]["reason"] for r in kept]
    for r in valid_front:
        e = r["front"]["errors"]
        assert max(e["left"]["centre_m"], e["right"]["centre_m"]) < 0.03
        assert abs(e["yaw_rad"]) < 0.03
    assert any(r["roof_truth_prior"]["errors"] for r in kept)
    assert any(r["chain_updated"] for r in kept)
    bounds = OBS.run_bounds(run, MODEL)
    bins = OBS.bin_summary(rows, bounds)
    assert sum(b["frames"] for b in bins) == len(kept)
    assert OBS.losses(rows, bounds[:4])["longest_s"] >= 0.0


def test_the_cpu_replay_of_its_own_truth_corresponds_exactly(run):
    truck = scene_rig.truck_boxes(ROOT / URDF, lift_m=0.0)
    read = [r for r in run.reads if CAL.usable(r)][-1]  # at cruise: 0.1 s is 15 mm of travel
    out = OBS.depth_agreement(run, read, DELAY, GEOMETRY, truck)
    assert out["common"] > 1000 and out["depth_max_m"] == pytest.approx(0.0, abs=1e-9)
    assert out["validity_mismatch"] == 0.0
    assert OBS.depth_agreement(run, read, DELAY + 0.1, GEOMETRY, truck)["depth_p95_m"] > 0.005
    rows = OBS.observe_run(run, DELAY, GEOMETRY, PRIOR, FRONT, MODEL)

    def cpu(r):
        return OBS.render_world(run, r["stamp_s"] - DELAY, GEOMETRY, truck)[0]

    cpu_rows = OBS.observe_run(run, DELAY, GEOMETRY, PRIOR, FRONT, MODEL, depth_of=cpu)
    verdict = OBS.correspondence(rows, cpu_rows, [out])
    assert verdict["agreement"] == 1.0 and verdict["longest_disagreement"] == 0
    assert verdict["checks"]["agreement"] and verdict["checks"]["disagree_run"]
    assert verdict["spatial_m_max"] == pytest.approx(0.0, abs=1e-9)


def test_the_validity_mismatch_counts_the_silhouette_band(run, monkeypatch):
    # Codex D8b impl P1: invalid boundary pixels must show, though depth skips the band.
    truck = scene_rig.truck_boxes(ROOT / URDF, lift_m=0.0)
    read = [r for r in run.reads if CAL.usable(r)][-1]
    _, region = OBS.render_world(run, read["stamp_s"] - DELAY, GEOMETRY, truck)
    band = region & ~CAL.erode(region, OBS.SILHOUETTE_PX)
    depth = run.depth(read)
    depth[band] = np.nan
    monkeypatch.setattr(run, "depth", lambda r: depth)
    out = OBS.depth_agreement(run, read, DELAY, GEOMETRY, truck)
    assert out["validity_mismatch"] == pytest.approx(band.sum() / region.sum())
    assert out["depth_max_m"] == pytest.approx(0.0, abs=1e-9)


def row(t, updated, distance=1.0, travelled=None, mode="front", **extra):
    return {"index": int(t * 10), "aligned_s": t, "chain_updated": updated, "distance_m": distance,
            "travelled_m": t if travelled is None else travelled, "mode": mode, "standing": False,
            "speed_mps": 0.1, "handoff_agree": False, **extra}


def test_losses_include_the_stretch_before_the_first_update_and_the_terminal_one():
    # Codex D8b impl P1: 10 s and 1 m without any update must not read as 0.
    never = [row(t, False, travelled=0.1 * t) for t in np.arange(0, 10.01, 0.1)]
    out = OBS.losses(never)
    assert out["longest_s"] == pytest.approx(10.0) and out["longest_m"] == pytest.approx(1.0)
    assert out["terminal_s"] == pytest.approx(10.0)
    once = [row(0.0, True)] + [row(t, False) for t in (0.1, 0.2, 0.3)]
    assert OBS.losses(once)["longest_s"] == pytest.approx(0.3) and OBS.losses(once)["terminal_s"] == pytest.approx(0.3)
    middle = [row(0.0, True), row(0.1, False), row(0.2, False), row(0.3, True)]
    out = OBS.losses(middle)
    assert out["longest_s"] == pytest.approx(0.3) and out["terminal_s"] is None


class FakePocket:
    def __init__(self, centre, width=0.2275, height=0.078):
        self.center_m, self.width_m, self.height_m = tuple(centre), width, height


class FakeObservation:
    status = "valid"

    def __init__(self, left, right, yaw):
        self.left, self.right, self.insertion_yaw_rad = left, right, yaw


def test_the_walls_follow_the_observed_yaw_not_only_the_centre():
    # Same centres and width, yaw 0.05 rad off: the wall points move sideways by
    # (w/2)(cos 0.05 - 1) along the truth's lateral axis (Codex D8b impl P1).
    w = 0.2275
    truth = {"left": np.array([1.0, 0.35, 0.061]), "right": np.array([1.0, -0.35, 0.061]), "yaw": 0.0,
             "width": w, "height": 0.078,
             "left_walls": {"outer": np.array([1.0, 0.35 + w / 2, 0.061]), "inner": np.array([1.0, 0.35 - w / 2, 0.061])},
             "right_walls": {"outer": np.array([1.0, -0.35 - w / 2, 0.061]), "inner": np.array([1.0, -0.35 + w / 2, 0.061])}}
    obs = FakeObservation(FakePocket(truth["left"]), FakePocket(truth["right"]), 0.05)
    errors = OBS.pocket_errors(obs, truth)
    expected = w / 2 * (math.cos(0.05) - 1)
    assert errors["left"]["wall_m"]["outer"] == pytest.approx(expected)
    assert errors["left"]["wall_m"]["inner"] == pytest.approx(-expected)
    assert errors["yaw_rad"] == pytest.approx(0.05)
    same = OBS.pocket_errors(FakeObservation(FakePocket(truth["left"]), FakePocket(truth["right"]), 0.0), truth)
    assert same["left"]["wall_m"]["outer"] == pytest.approx(0.0, abs=1e-15)


def test_the_truth_walls_carry_the_pallets_roll_and_pitch(run):
    t = 1.0
    flat = OBS.truth_in_base(run, t, GEOMETRY)
    original = run.truth.pallet.copy()
    try:
        q = np.array([math.cos(0.01), math.sin(0.01), 0.0, 0.0])  # roll 0.02 rad about x
        run.truth.pallet[:, 3:] = q
        rolled = OBS.truth_in_base(run, t, GEOMETRY)
    finally:
        run.truth.pallet[:] = original
    assert rolled["left_walls"]["outer"][1] != pytest.approx(flat["left_walls"]["outer"][1], abs=1e-6)


def test_the_correspondence_verdict_fails_each_tolerance_on_its_own():
    a = [row(t, True, front={"errors": None}, roof=None) for t in (0.1, 0.2, 0.3, 0.4)]
    b = [dict(r) for r in a]
    b[1] = {**b[1], "chain_updated": False}
    b[2] = {**b[2], "chain_updated": False}
    depth = [{"depth_p95_m": 0.006, "validity_mismatch": 0.01}]
    out = OBS.correspondence(a, b, depth)
    assert out["longest_disagreement"] == 2 and out["checks"]["disagree_run"] is False
    assert out["checks"]["agreement"] is False and out["checks"]["depth_p95"] is False
    assert out["checks"]["mismatch"] is True and not out["pass"]


def test_run_files_never_collide():
    assert OBS.run_name("/a/batch/run") != OBS.run_name("/b/batch/run")
    assert OBS.run_name("/a/seed_1/run").startswith("seed_1_")


def test_losses_run_from_the_anchor_to_the_insertion_end():
    # Re-review P1: observations 0.1-0.7 s inside a 0-1 s span leave 0.1 s before the first
    # update and 0.3 s after the last.
    rows = [row(t, True) for t in (0.1, 0.4, 0.7)]
    out = OBS.losses(rows, (0.0, 1.0, 0.0, 1.0))
    assert out["terminal_s"] == pytest.approx(0.3) and out["longest_s"] == pytest.approx(0.3)
    intervals = OBS.loss_intervals(rows, (0.0, 1.0, 0.0, 1.0))
    assert intervals[0][:2] == pytest.approx((0.0, 0.1)) and intervals[-1][:2] == pytest.approx((0.7, 1.0))


def test_the_handoff_needs_an_unbroken_run_of_accepted_agreeing_pairs():
    handoff = OBS.Handoff()
    assert [handoff.observe(True), handoff.observe(True)] == ["front", "front"]
    handoff.reset()  # a lost read, a read without a valid time or a control gap
    assert handoff.observe(True) == "front"  # not the third in a row
    assert [handoff.observe(True), handoff.observe(True)] == ["front", "roof"]
    assert handoff.observe(False) == "roof"  # roof mode is kept


def test_every_unobserved_read_resets_the_handoff_and_both_chains_lose_the_same_reads():
    import inspect

    source = inspect.getsource(OBS.observe_run)
    for marker in ('"skipped": "no_valid_time"', '"skipped": "control_gap"', '"loss": read["result"]'):
        before = source[:source.index(marker)]
        assert "handoff.reset()" in before[before.rindex("        if "):], marker
    assert "if not CAL.usable(read):" in source and "depth_of is None and not CAL.usable" not in source


def test_the_centre_difference_is_a_distance_not_per_axis():
    def errors(d):
        side = {"lateral_m": d, "along_m": d, "vertical_m": d, "height_m": 0.0, "wall_m": {"outer": 0.0, "inner": 0.0}}
        return {"errors": {"yaw_rad": 0.0, "left": side, "right": dict(side)}}

    diff = OBS.geometry_difference(errors(0.0), errors(0.004))
    assert diff["spatial_m"] == pytest.approx(0.004 * math.sqrt(3))
    a = [row(0.1, True, front=errors(0.0), roof=None)]
    b = [row(0.1, True, front=errors(0.004), roof=None)]
    verdict = OBS.correspondence(a, b, [{"depth_p95_m": 0.001, "validity_mismatch": 0.0}])
    assert verdict["checks"]["geometry"] is False and verdict["margin_m"] == pytest.approx(0.004 * math.sqrt(3))


def test_empty_bins_along_the_path_are_marked_unbounded():
    rows = [row(t, True, distance=d) for t, d in ((0.1, 1.05), (0.2, 1.04), (0.3, 0.85))]
    bins = OBS.bin_summary(rows, (0.0, 1.0, 0.0, 1.0, 1.05, 0.85))
    by_bin = {tuple(b["bin_m"]): b for b in bins}
    assert by_bin[(1.0, 0.9)]["frames"] == 0 and by_bin[(1.0, 0.9)]["unbounded"]


def test_a_run_outside_its_calibration_contract_is_refused(run):
    calibration = {"contract": {**{k: run.meta.get(k) for k in CAL.CONTRACT_KEYS},
                                **{k: run.result.get(k) for k in CAL.HASH_KEYS}},
                   "inputs": {run.key: {"seed": 1, "approach_straight_speed_mps": run.meta.get("approach_straight_speed_mps"),
                                        "pocket_check": run.meta.get("pocket_check"),
                                        "dwells": [d["kind"] for d in run.dwells], "reads": len(run.reads)}}}
    OBS.check_calibration(run, run.key, calibration)
    calibration["inputs"][run.key]["seed"] = 3
    with pytest.raises(ValueError, match="filed under 3"):
        OBS.check_calibration(run, run.key, calibration)
    calibration["inputs"][run.key]["seed"] = 1
    calibration["contract"]["fps"] = 30
    with pytest.raises(ValueError, match="fps"):
        OBS.check_calibration(run, run.key, calibration)


def test_the_collector_refuses_a_file_from_another_calibration(tmp_path):
    calibration = {"L_s": 0.0917, "inputs": {"/r/run": {}}, "verdict": {"complete": True, "pass": False}}
    (tmp_path / f"{OBS.run_name('/r/run')}.json").write_text(json.dumps(
        {"key": "/r/run", "calibration_sha256": "old", "L_s": 0.0917, "bins": [], "losses": {}}))
    with pytest.raises(ValueError, match="under this calibration"):
        OBS.collect(calibration, "new", tmp_path)
    summary = OBS.collect({**calibration, "verdict": {"complete": False}}, "old", tmp_path)
    assert not summary["complete"] and not summary["calibration_matrix_complete"]


def test_the_correspondence_counts_across_lost_reads_and_fails_missing_cpu_rows():
    def obs(updated):
        return row(0.0, updated, front={"errors": None}, roof=None)

    isaac = [{**obs(True), "index": 0}, {**obs(True), "index": 1, "loss": "no_depth"}, {**obs(True), "index": 2}]
    cpu = [{**obs(False), "index": 0}, {**obs(True), "index": 1, "loss": "no_depth"}, {**obs(False), "index": 2}]
    out = OBS.correspondence(isaac, cpu, [])
    # Disagree, both lost (agree), disagree: the longest run is 1, not 2.
    assert out["longest_disagreement"] == 1 and out["checks"]["complete"]
    short = OBS.correspondence(isaac, cpu[:1], [])
    assert short["checks"]["complete"] is False and not short["pass"]


def test_results_with_numpy_counts_serialise():
    rows = [row(0.1, True, distance=1.05), row(0.2, False, distance=1.04)]
    rows[0]["standing"] = np.bool_(True)
    summary = OBS.bin_summary(rows, (0.0, 0.3, 0.0, 0.3, 1.05, 1.04))
    assert json.loads(json.dumps(summary, default=OBS.json_default))[0]["standing"] == 1


def test_the_matrix_margin_takes_each_maximum_over_all_runs(tmp_path):
    # Codex D8b ③⑤ consult: 10.60 mm spatial in one run and 6.72 mrad in another give
    # 10.60 + 6.72 x 0.6 = 14.64 mm, not the larger per-run sum.
    calibration = {"L_s": 0.09, "inputs": {"/a/run": {}, "/b/run": {}}, "verdict": {"complete": True, "pass": True}}
    for key, (spatial, yaw) in (("/a/run", (0.0106, 0.001)), ("/b/run", (0.002, 0.00672))):
        corr = {"spatial_m_max": spatial, "yaw_rad_max": yaw, "margin_m": spatial + yaw * 0.6, "pass": False,
                "checks": {"geometry": False}}
        (tmp_path / f"{OBS.run_name(key)}.json").write_text(json.dumps(
            {"key": key, "calibration_sha256": "c", "L_s": 0.09, "bins": [], "losses": {}, "correspondence": corr}))
    summary = OBS.collect(calibration, "c", tmp_path)
    assert summary["margin_m"] == pytest.approx(0.0106 + 0.00672 * 0.6)
    assert summary["transfer_to_isaac"] is False and summary["check_failures"]["geometry"] == 2


def test_disagreements_are_listed_for_diagnosis():
    a = [row(0.1, True, front={"errors": None, "refused": None}, roof=None)]
    b = [row(0.1, False, front={"errors": None, "refused": "step"}, roof=None)]
    out = OBS.correspondence(a, b, [{"index": 1, "depth_p95_m": 0.012, "validity_mismatch": 0.0, "common": 9,
                                     "region": 10}])
    assert out["disagreements"][0]["index"] == a[0]["index"] and out["disagreements"][0]["refused"]["cpu"][0] == "step"
    assert out["worst_depth"]["depth_p95_m"] == 0.012


def test_no_transfer_without_the_whole_observed_matrix_and_a_passed_calibration(tmp_path):
    corr = {"spatial_m_max": 0.001, "yaw_rad_max": 0.001, "margin_m": 0.0016, "pass": True, "checks": {}}
    calibration = {"L_s": 0.09, "inputs": {"/a/run": {}, "/b/run": {}}, "verdict": {"complete": True, "pass": True}}
    (tmp_path / f"{OBS.run_name('/a/run')}.json").write_text(json.dumps(
        {"key": "/a/run", "calibration_sha256": "c", "L_s": 0.09, "bins": [], "losses": {}, "correspondence": corr}))
    assert OBS.collect(calibration, "c", tmp_path)["transfer_to_isaac"] is False  # /b/run missing
    (tmp_path / f"{OBS.run_name('/b/run')}.json").write_text(json.dumps(
        {"key": "/b/run", "calibration_sha256": "c", "L_s": 0.09, "bins": [], "losses": {}, "correspondence": corr}))
    assert OBS.collect(calibration, "c", tmp_path)["transfer_to_isaac"] is True
    failed = {**calibration, "verdict": {"complete": True, "pass": False}}
    assert OBS.collect(failed, "c", tmp_path)["transfer_to_isaac"] is False
