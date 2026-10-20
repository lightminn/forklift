"""Gyro-heading plan V-G0 tools: the offline gyro model, the standstill bias update and the verdict's counting rules."""

import hashlib
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


def _entry(tmp_path, run, arm, bundle, seed, *, position, yaw=0.01, ok=True, sent=10):
    out = tmp_path / run / f"{arm}_{bundle}_s{seed}"
    out.mkdir(parents=True)
    (out / "status.json").write_text(json.dumps({"driver_exit": 0 if ok else 1, "container_exit": 0, "eval_exit": 0,
                                                 "sent": sent, "serialize_ok": True}))
    (out / "v2.json").write_text(json.dumps({"nodes": 3, "paired": 3, "position_max_m": position, "yaw_max_rad": yaw,
                                             "graph": {"vertices": 3}}))
    (out / "drive.jsonl").write_text("{}\n{}\n{}\n")
    return {"run": run, "run_dir": "R", "arm": arm, "bundle": bundle, "seed": seed, "out": str(out)}


def test_the_verdict_counts_a_failed_or_short_run_as_infinite_and_judges_each_bundle_and_run(tmp_path, monkeypatch):
    monkeypatch.setattr(gyro_v0, "SEEDS", (1, 2, 3, 4, 5))
    monkeypatch.setattr(gyro_v0, "WINS", 4)
    monkeypatch.setattr(gyro_v0, "JUDGED", ("A+",))
    monkeypatch.setattr(gyro_v0, "RECORDED", ())
    monkeypatch.setattr(gyro_v0, "record_scans", lambda run_dir, cache: 10)
    entries = []
    for s in (1, 2, 3, 4, 5):
        entries.append(_entry(tmp_path, "r6", "steering", None, s, position=0.15))
        entries.append(_entry(tmp_path, "r6", "gyro", "A+", s, position=0.05, ok=s != 5))
    result = gyro_v0.verdict(entries)
    cell = result["cells"]["r6/A+"]
    assert cell["wins"] == 4 and cell["invalid"]["gyro"] == 1 and cell["pass"] and result["pass"]
    # a short replay (fewer scans sent than recorded) does not count either
    (Path(entries[1]["out"]) / "status.json").write_text(json.dumps({"driver_exit": 0, "container_exit": 0, "eval_exit": 0,
                                                                     "sent": 9, "serialize_ok": True}))
    result = gyro_v0.verdict(entries)
    assert result["cells"]["r6/A+"]["wins"] == 3 and not result["pass"]


def test_the_replay_refuses_an_odometry_file_made_for_another_record(tmp_path):
    run = tmp_path / "run"
    (run / "spool").mkdir(parents=True)
    (run / "spool" / "manifest.json").write_text("{}")
    records = [{"scan_id": 0, "stamp_s": 0.0}, {"scan_id": 1, "stamp_s": 0.1}]
    good = dict(odom_base=np.zeros((2, 3)), scan_id=np.array([0, 1]), stamp_s=np.array([0.0, 0.1]),
                run=np.asarray(str(run.resolve())), spool_manifest_sha256=np.asarray(hashlib.sha256(b"{}").hexdigest()))
    np.savez(tmp_path / "ok.npz", **good)
    assert slam_remap_drive.load_odom_file(tmp_path / "ok.npz", run, records).shape == (2, 3)
    np.savez(tmp_path / "stamps.npz", **{**good, "stamp_s": np.array([0.0, 0.2])})
    with pytest.raises(SystemExit, match="stamps"):
        slam_remap_drive.load_odom_file(tmp_path / "stamps.npz", run, records)
    np.savez(tmp_path / "hash.npz", **{**good, "spool_manifest_sha256": np.asarray("0" * 64)})
    with pytest.raises(SystemExit, match="hash"):
        slam_remap_drive.load_odom_file(tmp_path / "hash.npz", run, records)
