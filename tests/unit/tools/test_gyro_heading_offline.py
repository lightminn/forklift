"""Gyro-heading plan V-G0 tools: the offline gyro model, the standstill bias update and the verdict's counting rules."""

import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import gyro_heading_offline as gh  # noqa: E402
import gyro_v0  # noqa: E402
import slam_remap_drive  # noqa: E402

QUIET = gh.GyroModel(scale=0.0, initial_bias_dps=0.0, temperature_dps=0.0, random_walk_dps_rts=0.0, noise_dps_rthz=0.0,
                     lsb_dps=1e-9, mount_error_deg=0.0)
T = np.arange(0, 60.0, 1 / 120.0)


def test_a_vehicle_yaw_reads_on_the_sensors_minus_y_and_comes_back_vertical():
    """D435i IMU axes are the depth axes (x right, y down, z forward): the camera looks ahead, yaw is about its -y."""
    known = gh.camera_rotation()
    rates = np.full(len(T) - 1, 0.2)
    samples = gh.gyro_samples(T, rates, QUIET, np.random.default_rng(0), known)
    assert np.allclose(samples[:, 1], -0.2) and np.allclose(samples[:, [0, 2]], 0.0, atol=1e-9)
    assert np.allclose((samples @ known.T)[:, 2], 0.2)


def test_the_gyro_samples_carry_the_datasheet_noise_scale_and_quantisation():
    model = replace(gh.GyroModel(), initial_bias_dps=0.0, temperature_dps=0.0, random_walk_dps_rts=0.0, scale=0.01)
    known = gh.camera_rotation()
    rates = np.full(len(T) - 1, 0.3)
    samples = gh.gyro_samples(T, rates, model, np.random.default_rng(1), known)
    vertical = (samples @ known.T)[:, 2]
    assert abs(vertical.mean() / 0.3 - 1.01) < 1e-3  # the scale error
    density = vertical.std() * math.sqrt(1 / 120.0) * 180 / math.pi  # deg/s/sqrt(Hz)
    assert abs(density / 0.014 - 1) < 0.05
    lsb = model.lsb_dps * math.pi / 180
    assert np.allclose(np.round(samples / lsb) * lsb, samples)


def test_a_standstill_block_replaces_the_bias_and_motion_or_a_varying_rate_does_not():
    model = gh.GyroModel(zupt_s=0.5)
    dt = np.full(600, 1 / 120.0)
    noise = np.random.default_rng(2).normal(0, model.noise_dps_rthz * math.pi / 180 * math.sqrt(120), 600)
    bias = math.radians(0.8)
    still = np.ones(600, bool)
    est, updates = gh.standstill_bias(bias + noise, dt, still, model)
    assert updates == 10 and abs(est[-1] - bias) < math.radians(0.05)
    # the wheels' counts change: no update at all
    _, updates = gh.standstill_bias(bias + noise, dt, np.zeros(600, bool), model)
    assert updates == 0
    # a rate that varies within the block (a turn starting) is not white noise: no update
    ramp = bias + noise + np.linspace(0, 0.2, 600)
    _, updates = gh.standstill_bias(ramp, dt, still, replace(model, zupt_s=5.0))
    assert updates == 0


def test_a_steady_rotation_with_the_wheels_still_is_taken_as_bias_the_plans_stated_premise():
    """Plan G2: with the rear wheels still the chassis is taken not to turn -- a gyro cannot tell (Codex round-2 P1)."""
    model = gh.GyroModel(zupt_s=0.5)
    dt = np.full(120, 1 / 120.0)
    est, updates = gh.standstill_bias(np.full(120, 0.01), dt, np.ones(120, bool), model)
    assert updates == 2 and abs(est[-1] - 0.01) < 1e-12


def test_a_truck_that_moves_before_the_first_bias_update_fails_the_gyro_arm():
    stamps = np.arange(0, 1.0, 1 / 120.0)
    increments = np.zeros((len(stamps), 2))
    increments[10:] = 0.01
    vertical = np.zeros(len(stamps) - 1)
    still = np.all(increments[1:] == 0, axis=1)
    with pytest.raises(RuntimeError, match="before the first standstill bias update"):
        gh.gyro_odometry(stamps, increments, vertical, still, gh.GyroModel(), (0.0, 0.0, 0.0), 0.125)


def test_the_gyro_heading_integrates_the_rate_less_its_bias_and_the_wheels_give_the_distance():
    stamps = np.arange(0, 3.0, 1 / 120.0)
    increments = np.zeros((len(stamps), 2))
    increments[121:] = 0.008  # moving after 1 s
    bias = 0.002
    vertical = np.full(len(stamps) - 1, bias)
    vertical[120:] += 0.1  # turning at 0.1 rad/s once moving
    still = np.all(increments[1:] == 0, axis=1)
    pose, updates = gh.gyro_odometry(stamps, increments, vertical, still, replace(QUIET, zupt_s=0.5), (0.0, 0.0, 0.0),
                                     0.125)
    assert updates == 2
    moving = len(stamps) - 1 - 120
    assert abs(pose[-1, 2] - 0.1 * moving / 120.0) < 1e-9
    assert abs(np.sum(np.hypot(*np.diff(pose[:, :2], axis=0).T)) - 0.008 * 0.125 * moving) < 1e-9


def _entry(tmp_path, run, arm, bundle, seed, *, position, yaw=0.01, ok=True, sent=10, noise_seed=None, stamps=(0.1, 0.2, 0.3),
           digest="d" * 64, vertices=3, ranges_bin=None):
    out = tmp_path / run / f"{arm}_{bundle}_s{seed}"
    (out / "bridge" / "dump").mkdir(parents=True)
    (out / "status.json").write_text(json.dumps({"state": "done", "driver_exit": 0 if ok else 1, "container_exit": 0,
                                                 "eval_exit": 0, "sent": sent, "serialize_ok": True}))
    (out / "v2.json").write_text(json.dumps({"nodes": 3, "paired": 3, "position_max_m": position, "yaw_max_rad": yaw,
                                             "graph": {"vertices": vertices}}))
    (out / "drive.jsonl").write_text("".join(json.dumps({"stamp_s": t}) + "\n" for t in stamps))
    (out / "bridge" / "dump" / "nodes.csv").write_text("stamp_s\n0.1\n0.2\n0.3\n")
    run_dir = tmp_path / f"rec_{run}"
    (out / "drive.log").write_text("config " + json.dumps({"run": str(run_dir.resolve()), "noise_seed": seed if noise_seed is None else noise_seed,
                                                          "odom_file": str(out / "odom.npz"), "ranges_bin": ranges_bin}) + "\n")
    (out / "odom.json").write_text(json.dumps({"arm": arm, "bundle": bundle or "A+", "seed": seed,
                                               "record_sha256": digest}) + "\n")
    return {"run": run, "run_dir": str(run_dir), "arm": arm, "bundle": bundle, "seed": seed, "out": str(out),
            "noise_seed": seed}


@pytest.fixture
def small_matrix(monkeypatch):
    monkeypatch.setattr(gyro_v0, "SEEDS", (1, 2, 3, 4, 5))
    monkeypatch.setattr(gyro_v0, "WINS", 4)
    monkeypatch.setattr(gyro_v0, "JUDGED", ("A+",))
    monkeypatch.setattr(gyro_v0, "RECORDED", ())
    monkeypatch.setattr(gyro_v0, "REQUIRED_RUNS", ("r6",))
    monkeypatch.setattr(gyro_v0, "record_facts", lambda run_dir, cache: (10, "d" * 64))


def _matrix(tmp_path, **gyro):
    entries = []
    for s in (1, 2, 3, 4, 5):
        entries.append(_entry(tmp_path, "r6", "steering", None, s, position=0.15))
        entries.append(_entry(tmp_path, "r6", "gyro", "A+", s, position=0.05, ok=s != 5, **(gyro if s == 1 else {})))
    return entries


def test_the_verdict_counts_a_failed_or_short_run_as_infinite_and_judges_each_bundle_and_run(tmp_path, small_matrix):
    entries = _matrix(tmp_path)
    result = gyro_v0.verdict(entries)
    cell = result["cells"]["r6/A+"]
    assert cell["wins"] == 4 and cell["invalid"]["gyro"] == 1 and cell["pass"] and result["pass"]
    # a short replay (fewer scans sent than recorded) does not count either
    status = Path(entries[1]["out"]) / "status.json"
    status.write_text(status.read_text().replace('"sent": 10', '"sent": 9'))
    result = gyro_v0.verdict(entries)
    assert result["cells"]["r6/A+"]["wins"] == 3 and not result["pass"]


@pytest.mark.parametrize("change", [{"noise_seed": 7}, {"stamps": (0.1, 0.1, 0.3)}, {"yaw": float("nan")},
                                    {"digest": "e" * 64}, {"vertices": 0}, {"ranges_bin": "live.bin"}])
def test_a_run_that_used_other_settings_or_whose_replies_are_not_its_nodes_does_not_count(tmp_path, small_matrix, change):
    result = gyro_v0.verdict(_matrix(tmp_path, **change))
    assert result["cells"]["r6/A+"]["invalid"]["gyro"] == 2


def test_the_verdict_needs_the_whole_planned_matrix(tmp_path, small_matrix, monkeypatch):
    entries = _matrix(tmp_path)
    assert gyro_v0.verdict(entries[:-1])["matrix_problems"]
    monkeypatch.setattr(gyro_v0, "REQUIRED_RUNS", ("r6", "r7"))  # r7 absent: no pass on r6 alone
    result = gyro_v0.verdict(entries)
    assert not result["pass"] and result["matrix_problems"]


def test_a_record_with_a_gap_or_no_bias_update_is_refused_and_either_wheel_moving_counts():
    model = replace(QUIET, zupt_s=0.5)
    stamps = np.arange(0, 2.0, 1 / 120.0)
    increments = np.zeros((len(stamps), 2))
    vertical = np.zeros(len(stamps) - 1)
    still = np.ones(len(stamps) - 1, bool)
    gap = stamps.copy()
    gap[30:] += 0.2  # a 208 ms step
    with pytest.raises(ValueError, match="gap"):
        gh.gyro_odometry(gap, increments, vertical, still, model, (0.0, 0.0, 0.0), 0.125)
    opposite = increments.copy()
    opposite[10:] = (0.01, -0.01)  # both wheels turn, the mean distance is 0
    with pytest.raises(RuntimeError, match="before the first"):
        gh.gyro_odometry(stamps, opposite, vertical, np.all(opposite[1:] == 0, axis=1), model, (0.0, 0.0, 0.0), 0.125)
    short = stamps[:48]  # 0.4 s standing: never a full block
    with pytest.raises(RuntimeError, match="no standstill bias update"):
        gh.gyro_odometry(short, increments[:48], vertical[:47], still[:47], model, (0.0, 0.0, 0.0), 0.125)


def test_the_replay_refuses_an_odometry_file_made_for_another_record(tmp_path):
    run = tmp_path / "run"
    (run / "spool").mkdir(parents=True)
    (run / "spool" / "manifest.json").write_text("{}")
    records = [{"scan_id": 0, "stamp_s": 0.0}, {"scan_id": 1, "stamp_s": 0.1}]
    good = dict(odom_base=np.zeros((2, 3)), scan_id=np.array([0, 1]), stamp_s=np.array([0.0, 0.1]),
                run=np.asarray(str(run.resolve())), record_sha256=np.asarray("a" * 64))
    np.savez(tmp_path / "ok.npz", **good)
    assert slam_remap_drive.load_odom_file(tmp_path / "ok.npz", run, records, "a" * 64).shape == (2, 3)
    np.savez(tmp_path / "stamps.npz", **{**good, "stamp_s": np.array([0.0, 0.2])})
    with pytest.raises(SystemExit, match="stamps"):
        slam_remap_drive.load_odom_file(tmp_path / "stamps.npz", run, records, "a" * 64)
    with pytest.raises(SystemExit, match="hash"):
        slam_remap_drive.load_odom_file(tmp_path / "ok.npz", run, records, "b" * 64)


def test_the_record_digest_changes_with_the_wheel_data():
    log = {"joint_stamps_s": np.arange(3.0), "wheel_angles_rad": np.zeros((3, 4)), "steering_rad": np.zeros((3, 2)),
           "base_pose_world": np.zeros((3, 7))}
    records = [{"scan_id": 0, "stamp_s": 0.0, "odom_base": [0, 0, 0], "truth_base": [0, 0, 0]}]
    geometry = gh.AckermannOdometryGeometry(0.66, 0.53, 0.125)
    other = {**log, "wheel_angles_rad": np.full((3, 4), 0.1)}
    assert gh.record_digest(log, records, geometry) != gh.record_digest(other, records, geometry)


def test_the_manifest_holds_absolute_paths_as_the_replay_resolves_them(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    entries = gyro_v0.plan({"r6": "rec6", "r7": "rec7"}, Path("results"))
    assert all(Path(e["out"]).is_absolute() and Path(e["run_dir"]).is_absolute() for e in entries)
    assert all(e["noise_seed"] == e["seed"] for e in entries) and not gyro_v0.matrix_problems(entries)
