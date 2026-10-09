"""Plan D8b offline calibration: delay pieces, rigid render, synthetic runs (no Isaac)."""

import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.pallet_geometry import load_pallet_geometry
from tools import d8b_calibration as CAL

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("pocket_frame_recorder", ROOT / "sim/isaac/pocket_frame_recorder.py")
RECORDER = importlib.util.module_from_spec(SPEC)
sys.modules["pocket_frame_recorder"] = RECORDER
SPEC.loader.exec_module(RECORDER)

MODEL = CAL.PalletModel.from_geometry(load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml"))
# A coarse camera keeps the synthetic runs fast; same field of view as the 640 x 480 rig.
K = CAL.Intrinsics(fx=116.4, fy=116.4, cx=79.5, cy=59.5, width=160, height=120)
TILT = 0.10
MOUNT_T = np.array([0.619, 0.0, 0.27])


@pytest.fixture(autouse=True)
def coarse_pixels(monkeypatch):
    # The coarse camera's 22 mm boards are ~2 px at 1.5 m; the 640 x 480 runs keep 3 / 300.
    monkeypatch.setattr(CAL, "ERODE_PX", 1)
    monkeypatch.setattr(CAL, "MIN_PIXELS", 60)


def mount_rotation(tilt=TILT):
    # optical z forward (+x base) tilted down, x right (-y base), y down.
    c, s = math.cos(tilt), math.sin(tilt)
    forward = np.array([c, 0.0, -s])
    right = np.array([0.0, -1.0, 0.0])
    down = np.cross(forward, right)
    return np.column_stack((right, down, forward))


def test_pieces_find_a_crossing_between_two_ticks_neither_inside():
    taus = np.array([0.0, 0.1, 0.2])
    r = np.array([2.0, -3.0, -8.0]) * 1e-3  # zero at 0.04 s, tolerance 0.55 mm
    (lo, hi), = CAL.pieces(r, 0.55e-3, taus)
    assert lo == pytest.approx(0.1 * (2.0 - 0.55) / 5.0) and hi == pytest.approx(0.1 * (2.0 + 0.55) / 5.0)


def test_pieces_keep_two_separate_feasible_pieces_and_skip_undefined_ticks():
    taus = np.arange(7) * 0.1
    r = np.array([1.0, 0.0, 1.0, np.nan, 1.0, 0.0, 1.0]) * 1e-3
    found = CAL.pieces(r, 0.5e-3, taus)
    assert len(found) == 2
    assert found[0] == pytest.approx((0.05, 0.15)) and found[1] == pytest.approx((0.45, 0.55))
    assert CAL.pieces(np.full(7, np.nan), 1.0, taus) == []


def test_erosion_by_three_pixels_shrinks_a_block_by_three_on_each_side():
    mask = np.zeros((20, 20), dtype=bool)
    mask[5:15, 4:14] = True
    eroded = CAL.erode(mask, 3)
    assert eroded.sum() == 4 * 4 and eroded[8:12, 7:11].all()


def camera_pose_facing(distance_m, lateral_m=0.0):
    """pallet <- optical for a camera distance_m from the outer face at x = -0.3."""
    r_pc = mount_rotation()
    p_pc = np.array([-MODEL.depth_m / 2 - distance_m, lateral_m, MOUNT_T[2]])
    return r_pc, p_pc


def test_the_rigid_render_matches_the_analytic_front_plane_depth():
    r_pc, p_pc = camera_pose_facing(1.0)
    window = CAL.roi(K, r_pc, p_pc, MODEL)
    depth, front = CAL.render(K, r_pc, p_pc, MODEL, window, -MODEL.depth_m / 2)
    assert front.sum() > 500
    v, u = np.argwhere(front)[len(np.argwhere(front)) // 2]
    ray = r_pc @ np.array([(u - K.cx) / K.fx, (v - K.cy) / K.fy, 1.0])
    expected = (-MODEL.depth_m / 2 - p_pc[0]) / ray[0]
    assert depth[v, u] == pytest.approx(expected, abs=1e-9)
    # The inner block faces at x = -0.0725 share the normal but are not the outer face.
    assert not front[depth > 1.0 + 0.2].any()


@pytest.mark.parametrize("distance, lateral", [(1.0, 0.0), (0.6, 0.05), (2.4, -0.1)])
def test_the_front_plane_fast_path_equals_the_full_render(distance, lateral):
    r_pc, p_pc = camera_pose_facing(distance, lateral)
    window = CAL.roi(K, r_pc, p_pc, MODEL)
    depth, front = CAL.render(K, r_pc, p_pc, MODEL, window, -MODEL.depth_m / 2)
    fast, mask = CAL.front_plane(K, r_pc, p_pc, MODEL, window, -MODEL.depth_m / 2)
    v0, v1, u0, u1 = window
    assert mask.sum() > 50
    np.testing.assert_array_equal(mask, front[v0:v1, u0:u1])
    np.testing.assert_allclose(fast[mask], depth[v0:v1, u0:u1][mask], atol=1e-12)


def test_erosion_drops_pixels_at_the_array_edge():
    assert not CAL.erode(np.ones((10, 10), dtype=bool), 3)[:3].any()
    assert CAL.erode(np.ones((10, 10), dtype=bool), 3).sum() == 16
    assert not CAL.erode(np.ones((5, 5), dtype=bool), 3).any()


GEOMETRY_SHA = hashlib.sha256((ROOT / "config/pallet_geometry_epal6.yaml").read_bytes()).hexdigest()


def sha(text):
    return hashlib.sha256(text.encode()).hexdigest()


def write_run(tmp_path, *, speed, delay_s, bias_m=0.0, dwell=None, start_d=1.6, end_d=0.5, end_hold_s=0.0,
              pocket_check=False, video=False, seed=1, noise_seed=1, script_sha="s", control_until=None,
              blank_reads=(), control_gap=None):
    """A straight approach recorded through the real recorder; frames show s - delay_s.
    ``dwell`` = (kind, seconds) stands at the start; ``end_hold_s`` stands at the end."""
    recorder = RECORDER.PocketFrameRecorder(tmp_path)
    pallet = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    face = -MODEL.depth_m / 2
    camera_x0 = face - start_d
    x, v, t = 0.0, 0.0, 0.0
    times, xs = [], []
    hold_until = dwell[1] if dwell else 0.0
    arrived = None
    while arrived is None or t < arrived + end_hold_s:
        times.append(t)
        xs.append(x)
        if arrived is None and camera_x0 + x >= face - end_d:
            arrived = t
            if not end_hold_s:
                break
        target = 0.0 if t < hold_until or arrived is not None else speed
        v = min(target, v + 0.3 / 120) if target >= v else max(target, v - 0.3 / 120)
        x += v / 120
        t += 1 / 120
    times, xs = np.array(times), np.array(xs)
    base_x = camera_x0 - MOUNT_T[0] + xs
    rotation = mount_rotation()
    for k, (tk, bx) in enumerate(zip(times, base_x)):
        recorder.phase = "approach"
        recorder.record_tick(stamp_s=tk, rendered=k % 12 == 0, base_pose=[bx, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                             lift_m=0.0, pallet_pose=pallet)
    for k in range(0, len(times), 12):
        s = times[k]
        shown = s - delay_s
        if shown < 0:
            continue
        cam_x = camera_x0 + float(np.interp(shown, times, xs))
        r_pc, p_pc = rotation, np.array([cam_x, 0.0, MOUNT_T[2]])
        depth, _ = CAL.render(K, r_pc, p_pc, MODEL, (0, K.height, 0, K.width), face)
        blank = any(abs(s - b) < 1e-6 for b in blank_reads)
        recorder.record_read(read_s=s, stamp_s=s, rendering_frame=k, depth_m=None if blank else depth + bias_m,
                             lift_m=0.0, no_depth=blank)
    dwells = [] if dwell is None else [{"kind": "start", "trigger_s": 0.0, "still_since_s": 0.0, "end_s": dwell[1],
                                        "d_est_m": start_d}]
    if end_hold_s:
        dwells.append({"kind": "arrival", "trigger_s": arrived, "still_since_s": arrived, "end_s": times[-1],
                       "d_est_m": end_d})
    recorder.close({"camera": {"intrinsics": {"fx": K.fx, "fy": K.fy, "cx": K.cx, "cy": K.cy, "width": K.width,
                                              "height": K.height}},
                    "mount": {"rotation": rotation.tolist(), "translation_m": MOUNT_T.tolist()},
                    "approach_straight_speed_mps": speed, "pocket_check": pocket_check, "video": video, "fps": 60},
                   dwells)
    rear = np.column_stack((base_x, np.zeros_like(base_x), np.zeros_like(base_x)))
    control = np.column_stack((times, rear, rear))
    if control_until is not None:
        control = control[times <= control_until]
    if control_gap is not None:
        control = control[(control[:, 0] < control_gap[0]) | (control[:, 0] > control_gap[1])]
    np.save(tmp_path / "slam_control.npy", control)
    (tmp_path / "result.json").write_text(json.dumps({
        "seed": seed, "arguments": {"slam_noise_seed": noise_seed}, "script_sha256": sha(script_sha),
        "source_sha256": {"a.py": sha("a")}, "pallet_geometry_sha256": GEOMETRY_SHA, "forklift_urdf_sha256": sha("f"),
        "pallet_urdf_sha256": sha("p")}))
    return CAL.Run.load(tmp_path)


def test_a_known_delay_lies_in_every_piece_of_a_synthetic_run(tmp_path):
    run = write_run(tmp_path, speed=0.3, delay_s=0.075)
    rows = CAL.analyse_frames(run, MODEL, [(1.6, 0.0), (0.5, 0.0)])
    eligible = [r for r in rows if r["eligible"]]
    assert len(eligible) >= 5
    for row in eligible:
        assert len(row["pieces"]) == 1
        lo, hi = row["pieces"][0]
        assert lo - 1e-9 <= 0.075 <= hi + 1e-9
    summary = CAL.summarise_run(rows)
    assert summary["mismatch"] == 0 and summary["edge"] == 0
    assert summary["L_run_s"] == pytest.approx(0.075, abs=CAL.TICK_S / 2)
    # The window's history after the start is never eligible: it holds the stand.
    assert all(r["stamp_s"] >= CAL.TAUS[-1] - 1e-9 for r in eligible)


def test_a_standing_frame_measures_the_bias_and_not_the_delay(tmp_path):
    run = write_run(tmp_path, speed=0.3, delay_s=0.075, bias_m=0.1e-3, dwell=("start", 2.0))
    records = CAL.analyse_dwells(run, MODEL)
    assert records[0]["static_frames"] >= 5
    assert records[0]["bias_m"] == pytest.approx(0.1e-3, abs=1e-6)
    assert records[0]["spread_p95_m"] < 1e-6


def test_a_bias_larger_than_its_bound_shows_as_a_mismatch_or_a_shifted_piece(tmp_path):
    # 2 mm of bias is far beyond eps + B: at 0.3 m/s it reads as ~6.7 ms of extra delay.
    run = write_run(tmp_path, speed=0.3, delay_s=0.075, bias_m=2e-3)
    rows = CAL.analyse_frames(run, MODEL, [(1.6, 0.0), (0.5, 0.0)])
    mids = [(r["pieces"][0][0] + r["pieces"][0][1]) / 2 for r in rows if r["eligible"] and r["pieces"]]
    assert mids and all(m > 0.075 + 0.004 for m in mids)


def test_bias_bound_is_the_larger_neighbour_plus_the_allowance():
    biases = [(2.0, 0.1e-3), (1.5, -0.2e-3), (1.0, 0.05e-3)]
    assert CAL.bias_bound(1.7, 1.7, biases) == pytest.approx(0.2e-3 + CAL.M_U_M)
    assert CAL.bias_bound(1.2, 1.2, biases) == pytest.approx(0.2e-3 + CAL.M_U_M)
    assert CAL.bias_bound(2.05, 2.1, biases) is None
    assert CAL.bias_bound(0.99, 1.2, biases) is None


def test_the_bias_bound_covers_the_whole_window_not_the_stamp_distance():
    # Codex D8b impl P1: dwells (2.0, 0.25 mm), (1.75, 0), (1.5, 0); a frame stamped at
    # 1.74 m whose pixels may be from 1.7625 m must carry the 0.25 mm bin's bound.
    biases = [(2.0, 0.25e-3), (1.75, 0.0), (1.5, 0.0)]
    assert CAL.bias_bound(1.74, 1.74, biases) == pytest.approx(CAL.M_U_M)
    assert CAL.bias_bound(1.71, 2.04, biases) is None  # leaves the dwells: no bound
    assert CAL.bias_bound(1.71, 1.95, biases) == pytest.approx(0.25e-3 + CAL.M_U_M)


def test_odometry_tables_are_zero_for_exact_odometry_and_grow_with_drift():
    t = np.arange(0, 3, CAL.TICK_S)
    truth = np.column_stack((0.1 * t, np.zeros_like(t), np.zeros_like(t)))
    exact = np.column_stack((t, truth, truth))
    tables = CAL.odometry_tables(exact, 0.0, 3.0, max_age_ticks=12)
    assert all(row["e_m"] == pytest.approx(0.0, abs=1e-12) for row in tables["by_age"])
    drifting = np.column_stack((t, truth * [1.01, 1, 1], truth))  # 1 % scale error
    tables = CAL.odometry_tables(drifting, 0.0, 3.0, max_age_ticks=12)
    assert tables["by_age"][-1]["e_m"] == pytest.approx(0.01 * 0.1 * 0.1, rel=1e-3)
    # The anchor table always ends at the last row (Codex D8b impl P1).
    assert tables["anchor"]["age_s"][-1] == pytest.approx(t[-1])
    assert tables["anchor"]["e_m"][-1] == pytest.approx(0.01 * 0.1 * t[-1], rel=1e-3)
    assert tables["anchor"]["max_age_s"] == pytest.approx(t[-1])


def test_a_last_tick_jump_is_kept_in_the_anchor_table():
    t = np.arange(14) * CAL.TICK_S
    truth = np.column_stack((0.1 * t, np.zeros_like(t), np.zeros_like(t)))
    control = truth.copy()
    control[-1, 1] += 0.02
    tables = CAL.odometry_tables(np.column_stack((t, control, truth)), 0.0, 1.0)
    assert tables["anchor"]["e_m"][-1] == pytest.approx(0.02)


def test_the_cli_reports_a_partial_set_without_passing_it(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(CAL, "MIN_ELIGIBLE", 5)
    monkeypatch.setattr(CAL, "MIN_SINGLE", 5)
    monkeypatch.setattr(CAL, "CELL_MIN", 1)
    slow = write_run(tmp_path / "slow" / "run", speed=0.08, delay_s=0.075, dwell=("start", 2.0), end_hold_s=2.0,
                     start_d=1.1, end_d=0.7)
    fast = write_run(tmp_path / "fast" / "run", speed=0.30, delay_s=0.075, start_d=1.1, end_d=0.7)
    out = tmp_path / "out"
    assert CAL.main(["--seed-runs", f"1:{slow.directory},{fast.directory}", "--output", str(out)]) == 0
    result = json.loads((out / "d8b_calibration.json").read_text())
    # Same basename "run": both kept under their full paths (Codex D8b impl P1).
    assert set(result["runs"]) == {str(slow.directory), str(fast.directory)}
    assert [d["kind"] for d in result["dwells"]["1"]] == ["start", "arrival"]
    assert all(d["static_frames"] >= 5 and abs(d["bias_m"]) < 1e-6 for d in result["dwells"]["1"])
    assert result["L_s"] == pytest.approx(0.075, abs=CAL.TICK_S / 2)
    assert result["band_s"][0] <= 0.075 <= result["band_s"][1]
    cells = result["cells"]["1"]
    assert len(cells) == 1 and cells[0]["slow"] >= 1 and cells[0]["fast"] >= 1
    verdict = result["verdict"]
    # Two runs of one seed are not the plan's matrix: reported, never passed.
    assert not verdict["complete"] and not verdict["pass"] and verdict["missing"]
    assert [f for f in verdict["failures"] if not f.startswith("(i) seed 1: 2 dwells")] == []
    assert verdict["analysis_complete"]
    errors = result["extra"][str(fast.directory)]["control_error"]
    assert errors["all"]["position_m"]["max"] == pytest.approx(0.0, abs=1e-12)
    assert result["extra"][str(fast.directory)]["odometry"]["anchor"]["max_age_s"] > 1.0


def test_a_run_given_twice_or_another_camera_is_refused(tmp_path):
    run = write_run(tmp_path / "a", speed=0.30, delay_s=0.075, dwell=("start", 2.0), start_d=1.1, end_d=0.7)
    with pytest.raises(ValueError, match="twice"):
        CAL.analyse([(1, [run.directory, run.directory])], MODEL)
    other = write_run(tmp_path / "b", speed=0.30, delay_s=0.075, start_d=1.1, end_d=0.7, video=True)
    with pytest.raises(ValueError, match="video"):
        CAL.analyse([(1, [run.directory, other.directory])], MODEL)


def test_the_manifest_names_what_the_matrix_lacks():
    class Fake:
        def __init__(self, speed, d5=False, dwells=()):
            self.meta = {"approach_straight_speed_mps": speed, "pocket_check": d5}
            self.dwells = [{"kind": k} for k in dwells]

    full = {seed: [Fake(0.055, dwells=CAL.REQUIRED_DWELLS)] + [Fake(v) for v in CAL.REQUIRED_SPEEDS[1:]]
            for seed in CAL.REQUIRED_SEEDS}
    full[1].append(Fake(0.08, d5=True))
    assert CAL.manifest(full) == []
    partial = {1: full[1][:-1], 3: full[3][1:], 7: full[5]}
    missing = CAL.manifest(partial)
    assert any("D5-on" in m for m in missing) and any("seed 3: 0 runs at 0.055" in m for m in missing)
    assert any(m.startswith("seed 5: no runs") for m in missing) and any("seed 7" in m for m in missing)


def test_a_window_past_the_record_is_not_eligible(tmp_path):
    run = write_run(tmp_path, speed=0.30, delay_s=0.075, start_d=1.1, end_d=0.7)
    last = run.truth.t[-1]
    assert CAL.window_times(run.truth, last - 0.1) is not None
    assert CAL.window_times(run.truth, last - 0.1 + CAL.TICK_S / 2) is None
    assert CAL.window_times(run.truth, CAL.TAUS[-1] - CAL.TICK_S / 2, CAL.TAUS[-1]) is None  # before the record
    assert CAL.window_times(run.truth, CAL.IN_RANGE_S + CAL.TICK_S) is not None  # eligibility needs only 0.4 s


def test_parallel_rays_and_a_camera_past_the_face_give_no_front_pixels():
    r_pc = np.column_stack(([0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]))  # level, along +x
    face = -MODEL.depth_m / 2
    window = (0, K.height, 0, K.width)
    with np.errstate(all="raise"):
        depth, mask = CAL.front_plane(K, r_pc, np.array([face + 0.01, 0.0, 0.07]), MODEL, window, face)
    assert not mask.any() and np.isnan(depth).all()
    sideways = np.column_stack(([1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]))  # looking along +y
    with np.errstate(all="raise"):
        _, mask = CAL.front_plane(K, sideways, np.array([face - 1.0, 0.0, 0.07]), MODEL, window, face)
    assert mask.sum() < 0.6 * mask.size  # half the rays point away: no warning, no hit


def test_a_tau_whose_render_explains_few_front_pixels_is_undefined():
    # Smoke 1857: at an alias delay only 2-3 % of the predicted front pixels lay within
    # 20 mm (inner block faces seen through the pockets), yet more than 300 of them.
    r_pc, p_pc = camera_pose_facing(1.0)
    face = -MODEL.depth_m / 2
    window = (0, K.height, 0, K.width)
    truth_depth, front = CAL.render(K, r_pc, p_pc, MODEL, window, face)
    isaac = np.where(front, truth_depth, np.nan)
    r, n = CAL.residual(isaac, K, r_pc, p_pc, MODEL, face)
    assert r == pytest.approx(0.0, abs=1e-12) and n > 0
    pixels = np.argwhere(CAL.erode(front, CAL.ERODE_PX))
    far = isaac.copy()
    keep = pixels[: int(0.05 * len(pixels))]
    far[CAL.erode(front, CAL.ERODE_PX)] += 0.2275  # most pixels see a surface 0.2275 m behind
    far[keep[:, 0], keep[:, 1]] = truth_depth[keep[:, 0], keep[:, 1]]
    assert math.isnan(CAL.residual(far, K, r_pc, p_pc, MODEL, face)[0])
    near = isaac.copy()
    drop = pixels[: int(0.05 * len(pixels))]
    near[drop[:, 0], drop[:, 1]] += 0.2275  # 5 % occluded / other surface: still defined
    assert CAL.residual(near, K, r_pc, p_pc, MODEL, face)[0] == pytest.approx(0.0, abs=1e-12)


def test_a_run_filed_under_another_seed_or_built_from_other_code_is_refused(tmp_path):
    good = write_run(tmp_path / "a", speed=0.30, delay_s=0.075, dwell=("start", 2.0), start_d=1.1, end_d=0.7)
    with pytest.raises(ValueError, match="filed under 3"):
        CAL.analyse([(3, [good.directory])], MODEL)
    noisy = write_run(tmp_path / "b", speed=0.30, delay_s=0.075, start_d=1.1, end_d=0.7, noise_seed=2)
    with pytest.raises(ValueError, match="slam noise seed 2"):
        CAL.analyse([(1, [good.directory, noisy.directory])], MODEL)
    other = write_run(tmp_path / "c", speed=0.30, delay_s=0.075, start_d=1.1, end_d=0.7, script_sha="t")
    with pytest.raises(ValueError, match="script_sha256"):
        CAL.analyse([(1, [good.directory, other.directory])], MODEL)
    with pytest.raises(ValueError, match="pallet geometry analysed"):
        CAL.analyse([(1, [good.directory])], MODEL, geometry_sha256="0" * 64)


def test_reads_without_depth_still_count_in_the_control_error_and_gaps_are_reported(tmp_path, monkeypatch):
    monkeypatch.setattr(CAL, "MIN_ELIGIBLE", 1)
    monkeypatch.setattr(CAL, "MIN_SINGLE", 1)
    dwell = write_run(tmp_path / "d", speed=0.08, delay_s=0.075, dwell=("start", 2.0), end_hold_s=2.0,
                      start_d=1.1, end_d=0.7, blank_reads=(4.0,))
    short = write_run(tmp_path / "s", speed=0.30, delay_s=0.075, start_d=1.1, end_d=0.7, control_until=1.0)
    result, _ = CAL.analyse([(1, [dwell.directory, short.directory])], MODEL, geometry_sha256=GEOMETRY_SHA)
    samples = CAL.read_samples(dwell, MODEL)
    assert any(abs(x["stamp_s"] - 4.0) < 1e-6 for x in samples)  # the no_depth read is a ② target
    assert result["extra"][str(dwell.directory)]["control_error"]["expected"] == len(samples)
    missing = result["analysis_missing"]
    assert any(str(short.directory) in m and "outside the control record" in m for m in missing)
    assert any(str(short.directory) in m and "④ control covers" in m for m in missing)
    assert not result["verdict"]["analysis_complete"]


def test_front_pixels_without_an_isaac_return_count_against_the_coverage():
    r_pc, p_pc = camera_pose_facing(1.0)
    face = -MODEL.depth_m / 2
    truth_depth, front = CAL.render(K, r_pc, p_pc, MODEL, (0, K.height, 0, K.width), face)
    isaac = np.where(front, truth_depth, np.nan)
    pixels = np.argwhere(CAL.erode(front, CAL.ERODE_PX))
    holes = pixels[int(0.2 * len(pixels)):]
    isaac[holes[:, 0], holes[:, 1]] = np.nan  # 80 % of the face returned nothing
    assert math.isnan(CAL.residual(isaac, K, r_pc, p_pc, MODEL, face)[0])


def test_missing_hashes_are_refused_even_when_every_run_lacks_them(tmp_path):
    run = write_run(tmp_path / "a", speed=0.30, delay_s=0.075, dwell=("start", 2.0), start_d=1.1, end_d=0.7)
    record = json.loads((run.directory / "result.json").read_text())
    for key in ("script_sha256", "source_sha256"):
        broken = {k: v for k, v in record.items() if k != key}
        (run.directory / "result.json").write_text(json.dumps(broken))
        with pytest.raises(ValueError, match=key):
            CAL.analyse([(1, [run.directory])], MODEL)
    (run.directory / "result.json").write_text(json.dumps({**record, "source_sha256": {}}))
    with pytest.raises(ValueError, match="source_sha256"):
        CAL.analyse([(1, [run.directory])], MODEL)


def test_a_gap_inside_the_control_record_is_reported_not_interpolated(tmp_path, monkeypatch):
    monkeypatch.setattr(CAL, "MIN_ELIGIBLE", 1)
    monkeypatch.setattr(CAL, "MIN_SINGLE", 1)
    dwell = write_run(tmp_path / "d", speed=0.08, delay_s=0.075, dwell=("start", 2.0), end_hold_s=2.0,
                      start_d=1.1, end_d=0.7, control_gap=(3.0, 4.5))
    result, _ = CAL.analyse([(1, [dwell.directory])], MODEL, geometry_sha256=GEOMETRY_SHA)
    errors = result["extra"][str(dwell.directory)]["control_error"]
    assert errors["outside_control"] > 0
    missing = result["analysis_missing"]
    assert any("gaps up to" in m for m in missing) and any("outside the control record" in m for m in missing)


def test_a_sample_aligned_before_the_anchor_is_not_a_gap():
    # The first approach read shows the capture moment, which the control record skips.
    t = np.r_[np.arange(0, 1.0, CAL.TICK_S), np.arange(1.1, 2.0, CAL.TICK_S)]
    rows = np.column_stack((t, np.zeros((len(t), 6))))
    frames = [{"stamp_s": 1.12, "phase": "approach", "distance_m": 1.0, "speed_mps": 0.0},
              {"stamp_s": 1.5, "phase": "approach", "distance_m": 1.0, "speed_mps": 0.0}]
    errors = CAL.control_error(rows, frames, 0.0917, [(2.0, 0.0), (0.5, 0.0)], span=(1.1, 2.0))
    assert (errors["before_anchor"], errors["outside_control"], errors["all"]["samples"]) == (1, 0, 1)
    errors = CAL.control_error(rows, frames, 0.0917, [(2.0, 0.0), (0.5, 0.0)], span=(0.0, 2.0))
    assert errors["outside_control"] == 1  # inside the span the same gap is missing data


def test_a_non_finite_control_or_truth_record_is_refused(tmp_path):
    run = write_run(tmp_path, speed=0.30, delay_s=0.075, start_d=1.1, end_d=0.7)
    control = np.load(run.directory / "slam_control.npy")
    control[60, 1] = np.nan
    np.save(run.directory / "slam_control.npy", control)
    with pytest.raises(ValueError, match="slam_control"):
        CAL.Run.load(run.directory)
    control[60, 1] = 0.0
    control[61, 0] = control[60, 0]  # a repeated time
    np.save(run.directory / "slam_control.npy", control)
    with pytest.raises(ValueError, match="time order"):
        CAL.Run.load(run.directory)


@pytest.mark.parametrize("cut", ["start", "end"])
def test_missing_control_at_either_end_of_the_span_is_reported(tmp_path, monkeypatch, cut):
    monkeypatch.setattr(CAL, "MIN_ELIGIBLE", 1)
    monkeypatch.setattr(CAL, "MIN_SINGLE", 1)
    run = write_run(tmp_path, speed=0.08, delay_s=0.075, dwell=("start", 2.0), end_hold_s=2.0, start_d=1.1, end_d=0.7)
    # Phases: "other" for the first second, so the record goes on outside the span.
    truth = dict(np.load(run.directory / "pocket_frames/truth.npz"))
    truth["phase"] = np.where(truth["stamp_s"] < 1.0, 0, truth["phase"]).astype(np.int8)
    np.savez_compressed(run.directory / "pocket_frames/truth.npz", **truth)
    span = CAL.Run.load(run.directory).phase_span()
    control = np.load(run.directory / "slam_control.npy")
    if cut == "start":
        keep = ~((control[:, 0] >= span[0] - 1e-9) & (control[:, 0] < span[0] + 2.5 * CAL.TICK_S))
    else:
        keep = control[:, 0] <= span[1] - 2.5 * CAL.TICK_S
    np.save(run.directory / "slam_control.npy", control[keep])
    result, _ = CAL.analyse([(1, [run.directory])], MODEL, geometry_sha256=GEOMETRY_SHA)
    assert any("④ control covers" in m for m in result["analysis_missing"]), result["analysis_missing"]


def test_a_delay_beyond_the_band_range_fails_instead_of_aliasing(tmp_path):
    # Confirmation review P1: with only a shrunk search, a 0.6 s delay could alias into a
    # piece inside it; the whole -0.1 .. 1.0 s range is evaluated and a piece past 0.4 s
    # fails the calibration.
    run = write_run(tmp_path, speed=0.30, delay_s=0.6, start_d=1.9, end_d=0.7)
    rows = CAL.analyse_frames(run, MODEL, [(1.9, 0.0), (0.6, 0.0)])
    eligible = [r for r in rows if r["eligible"]]
    assert eligible
    assert all(r["out_of_range"] for r in eligible)
    assert all(lo - 1e-9 <= 0.6 <= hi + 1e-9 for r in eligible for lo, hi in r["out_of_range"])
    summary = CAL.summarise_run(rows)
    assert summary["out_of_range"] == len(eligible) and summary["L_run_s"] is None


def test_a_creeping_stand_is_judged_over_the_whole_evaluated_range(tmp_path):
    # Confirmation review P2: 0.2 mm/s of creep moves 0.08 mm in a 0.4 s window but
    # 0.22 mm in the 1.1 s one; only the latter is the static rule.
    run = write_run(tmp_path, speed=0.30, delay_s=0.075, dwell=("start", 3.0), start_d=1.1, end_d=0.7)
    times = run.truth.t
    creep = np.where(times < 3.0, 0.0002 * times, 0.0)
    run.truth.base[:, 0] += creep
    records = CAL.analyse_dwells(run, MODEL)
    assert records[0]["static_frames"] == 0
    assert records[0]["window_shift_m"]["min"] > CAL.STATIC_POS_M
    narrow = CAL.motion_window(run.truth, 2.0, MODEL)
    assert narrow.max_shift_m < CAL.STATIC_POS_M  # the 0.4 s window alone would have passed it
