"""``forklift_core.plant.m_synth`` (docs/design/2026-11-15-m-analyze-entry.md 6, PR 1): one synthetic trial -- the
teleop record (header, ``profile_start``, the runner's own ``profile`` rows, STATE rows by ``state_row``, footer), the
``imu_record/1`` file and the ``truth/1`` file -- from a drive profile and a true plant the analysis must recover."""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from forklift_core.live.synthetic import SyntheticRefused, is_synthetic, refuse_or_mark
from forklift_core.plant import m_analysis as ma
from forklift_core.plant import m_synth as ms
from forklift_core.plant.clock_map import clean_points
from forklift_core.plant.real_vehicle import load_real_vehicle
from forklift_core.plant.record_rows import sample_from_row
from forklift_core.sensors.imu_mount import up_from_gravity
from forklift_core.sensors.imu_record import WRAP_US, imu_clock_fits, imu_stream, unwrap_u32_us

REPO = Path(__file__).resolve().parents[3]
CFG = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
M1 = yaml.safe_load((REPO / "config" / "profiles" / "m1_m3" / "m1_fwd_v010.yaml").read_text(encoding="utf-8"))
M1_REV = yaml.safe_load((REPO / "config" / "profiles" / "m1_m3" / "m1_rev_v010.yaml").read_text(encoding="utf-8"))
ARW = ms.ARW_DESK
SHORT = 5.0  # still_s for the tests that do not need the registered 60 s windows


def _states(rec):
    return [r for r in rec if r.get("t") == "state"]


def _odo(rec, max_gap_s=0.05):
    return ma.odometry_pose([sample_from_row(r).state for r in _states(rec)], CFG, max_gap_s=max_gap_s)


def _truth_points(out):
    rows = [r for r in out["truth"] if r.get("t") == "truth"]
    assert len(rows) == 1
    return rows[0]["points"]


def _pose(p):
    return (p["x_m"], p["y_m"], p["yaw_rad"])


def _counts(r):
    return (r["wheel_count_left"], r["wheel_count_right"])


def _moving_hosts(rec):
    """Host times of the STATEs whose wheel counts differ from the one before."""
    st = _states(rec)
    return [b["host_ns"] for a, b in zip(st, st[1:]) if _counts(a) != _counts(b)]


def _move_start_ns(rec):
    """The first wheel-count change (entry 3.3 3)."""
    hosts = _moving_hosts(rec)
    return hosts[0] if hosts else None


def _gyro_record_order(imu):
    return [r for r in imu if r.get("t") == "imu" and r["s"] == "g"]


@pytest.fixture(scope="module")
def drift():
    return ms.synth_trial(M1, CFG, true_yaw_per_m=0.003, still_s=SHORT, seed=3)


# --- determinism and shape -----------------------------------------------------------------------------------------

def test_same_seed_same_output_other_seed_other_noise():
    a = ms.synth_trial(M1, CFG, still_s=2.0, seed=11)
    b = ms.synth_trial(M1, CFG, still_s=2.0, seed=11)
    c = ms.synth_trial(M1, CFG, still_s=2.0, seed=12)
    assert a == b
    ga = [r["y"] for r in _gyro_record_order(a["imu"])]
    gc = [r["y"] for r in _gyro_record_order(c["imu"])]
    assert ga != gc


def test_the_record_has_the_bridge_rows_and_the_runner_wrote_the_profile_rows(drift):
    rec = drift["record"]
    assert rec[0]["t"] == "header" and rec[0]["schema"] == "teleop_bridge_record/1"
    for key in ("host_boot_id", "device_id", "synthetic", "start_mono_ns"):
        assert key in rec[0]
    assert rec[-1]["t"] == "footer" and rec[-1]["dropped"] == 0 and rec[-1]["write_errors"] == 0
    starts = [r for r in rec if r.get("t") == "profile_start"]
    assert len(starts) == 1
    assert starts[0]["trial"] == 1 and starts[0]["profile"] == "m1_fwd_v010" and starts[0]["refused"] is None
    ends = [r for r in rec if r.get("t") == "profile_end"]
    assert [(e["trial"], e["result"]) for e in ends] == [(1, "done")]
    tr = ma.trials(rec)
    assert len(tr) == 1 and tr[0].valid and set(tr[0].spans) == {"presteer", "still_pre", "straight", "still_post"}
    # every profile row carries the runner's keys (ProfileRunner._row)
    prof = [r for r in rec if r.get("t") == "profile"]
    keys = {"t", "host_ns", "trial", "state", "seg", "label", "s_m", "req_mps", "req_kappa", "arb_mps", "arb_limit",
            "arb_policy", "deadman", "drive_mps", "drive_q", "port", "direction_cut", "meas_q", "result"}
    assert all(set(r) == keys for r in prof)
    drive = [r for r in prof if r["label"] == "straight" and r["state"] == "running"]
    assert max(r["req_mps"] for r in drive) == pytest.approx(0.1)
    # the still segments follow still_s; host times never go back in the record
    lo, hi = tr[0].spans["still_pre"]
    assert (hi - lo) / 1e9 == pytest.approx(SHORT, abs=0.03)
    hosts = [r["host_ns"] for r in rec[1:-1]]
    assert hosts == sorted(hosts)
    # STATE rows read back into samples (state_row's shape) and carry the bridge's read time
    st = _states(rec)
    assert all("read_ns" in r and r["read_ns"] <= r["host_ns"] for r in st)
    assert sample_from_row(st[0]).state.mcu_time_us == st[0]["mcu_time_us"]


# --- the true plant and the truth file -----------------------------------------------------------------------------

def test_truth_end_pose_is_the_integrated_true_plant(drift):
    hdr = drift["truth"][0]
    assert hdr["t"] == "header" and hdr["schema"] == "truth/1" and hdr["method"] == "synthetic"
    assert hdr["body_point"] == "rear_axle_center" and hdr["frame"] == "floor"
    assert set(hdr["sigma"]) == {"pose_yaw_rad", "pose_xy_m", "source"}
    rec_hdr = drift["record"][0]
    assert hdr["record"] == {"host_boot_id": rec_hdr["host_boot_id"], "start_mono_ns": rec_hdr["start_mono_ns"]}
    pts = _truth_points(drift)
    assert set(pts) == {"start", "end"} and all(p["kind"] == "stop_pose" for p in pts.values())
    # the true path length, from the odometry (radius scale 1: the encoders see the true distance)
    odo = _odo(drift["record"])
    s = float(odo["x"][-1])  # odometry runs straight (q = 0)
    assert s == pytest.approx(3.0, abs=0.02)
    rel = ma.relative_pose(_pose(pts["start"]), _pose(pts["end"]))
    k = 0.003
    assert rel["dyaw"] == pytest.approx(k * s, abs=1e-7)
    assert rel["dyaw"] == pytest.approx(0.009, abs=1e-4)
    assert rel["dx_fwd"] == pytest.approx(math.sin(k * s) / k, abs=1e-5)
    assert rel["dy_lat"] == pytest.approx((1 - math.cos(k * s)) / k, abs=1e-5)
    assert rel["dy_lat"] == pytest.approx(k * s * s / 2, rel=1e-3)  # ~13.5 mm
    # the odometry saw none of it
    assert abs(float(odo["yaw"][-1])) < 1e-12 and abs(float(odo["y"][-1])) < 1e-12


def test_truth_yaw_is_wrapped_and_follows_the_start_pose():
    out = ms.synth_trial(M1, CFG, true_yaw_per_m=0.01, still_s=2.0, start_pose=(1.0, -2.0, 3.13))
    pts = _truth_points(out)
    assert pts["start"]["yaw_rad"] == pytest.approx(3.13)
    assert -math.pi < pts["end"]["yaw_rad"] < 0  # 3.13 + 0.03 wraps
    rel = ma.relative_pose(_pose(pts["start"]), _pose(pts["end"]))
    assert rel["dyaw"] == pytest.approx(0.03, abs=3e-4)


@pytest.mark.parametrize("profile", [M1, M1_REV], ids=["fwd", "rev"])
def test_odometry_overestimates_the_true_distance_by_the_radius_scale(profile):
    out = ms.synth_trial(profile, CFG, true_radius_scale=1.02, still_s=2.0, seed=5)
    pts = _truth_points(out)
    s_true = ma.relative_pose(_pose(pts["start"]), _pose(pts["end"]))["dist_m"]
    odo = _odo(out["record"])
    s_odo = math.hypot(float(odo["x"][-1]), float(odo["y"][-1]))
    assert s_odo / s_true == pytest.approx(1.02, abs=1e-5)
    # the runner ended the drive on the encoder distance: the truck went 3 m / 1.02 (plus the stop)
    assert s_true == pytest.approx(3.0 / 1.02, abs=0.02)


# --- the IMU -------------------------------------------------------------------------------------------------------

def _drive_gyro(out, up, bias, pad_s=1.0):
    """The gyro heading from ``pad_s`` before the first wheel motion to ``pad_s`` after the last (both standing)."""
    st = imu_stream(out["imu"], stream="g")
    moving = _moving_hosts(out["record"])
    lo, hi = moving[0] - int(pad_s * 1e9), moving[-1] + int(pad_s * 1e9)
    m = (st.host_ns >= lo) & (st.host_ns <= hi)
    t, g = st.host_ns[m], st.xyz[m]
    return t, ma.imu_heading(t, g, up, bias)


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_gyro_heading_matches_the_true_heading_change(seed):
    up = ms.unit((0.0, -math.cos(0.10), math.sin(0.10)))  # tilted 0.10 rad
    bias = (2e-4, 3e-4, -1e-4)
    out = ms.synth_trial(M1, CFG, true_yaw_per_m=0.05, up=up, bias=bias, still_s=2.0, seed=seed)
    t, h = _drive_gyro(out, up, bias)
    pts = _truth_points(out)
    true_dyaw = ma.relative_pose(_pose(pts["start"]), _pose(pts["end"]))["dyaw"]
    assert true_dyaw == pytest.approx(0.15, abs=2e-3)
    T = (t[-1] - t[0]) / 1e9
    assert abs(h[-1] - true_dyaw) < 5 * ARW * math.sqrt(T)


def test_gyro_noise_has_the_asked_density_and_the_accelerometer_reads_up():
    up = ms.unit((0.05, -0.99, 0.1))
    out = ms.synth_trial(M1, CFG, up=up, still_s=20.0, seed=4)
    st = imu_stream(out["imu"], stream="g")
    acc = imu_stream(out["imu"], stream="a")
    lo = st.host_ns[0]
    standing = st.host_ns < lo + int(15e9)
    rate = st.xyz[standing] @ up
    assert ma.allan_dev_tau(st.host_ns[standing], rate, 1.0) == pytest.approx(ARW, rel=0.15)
    a0 = acc.xyz[acc.host_ns < lo + int(15e9)]
    assert np.allclose(up_from_gravity(a0), up, atol=2e-3)  # the upward reaction (imu_mount's convention)
    assert np.linalg.norm(a0.mean(axis=0)) == pytest.approx(ms.G, rel=1e-3)


def test_bias_step_at_the_drive_start():
    step = (0.0, 4e-4, 0.0)
    out = ms.synth_trial(M1, CFG, bias=(0.0, 1e-4, 0.0), bias_step=step, arw=1e-6, still_s=3.0, seed=2)
    st = imu_stream(out["imu"], stream="g")
    t0 = _move_start_ns(out["record"])
    before = st.xyz[(st.host_ns < t0 - int(0.2e9)) & (st.host_ns > t0 - int(2e9)), 1].mean()
    after = st.xyz[(st.host_ns > t0 + int(0.2e9)) & (st.host_ns < t0 + int(2e9)), 1].mean()
    # white noise of density arw over 1.8 s averages to arw / sqrt(1.8 s) ~ 7.5e-7 rad/s: allow ~5 sigma
    assert before == pytest.approx(1e-4, abs=4e-6)
    assert after - before == pytest.approx(4e-4, abs=6e-6)  # straight, no drift: only the bias moved


# --- headers -------------------------------------------------------------------------------------------------------

def test_headers_are_synthetic_by_default_and_refused_without_the_flag(drift):
    for name in ("record", "imu", "truth"):
        hdr = drift[name][0]
        assert hdr["t"] == "header" and hdr["synthetic"] is True and is_synthetic(hdr)
        with pytest.raises(SyntheticRefused):
            refuse_or_mark(drift[name], allow=False)
    assert drift["imu"][0]["schema"] == "imu_record/1"
    assert drift["imu"][0]["host_boot_id"] == drift["record"][0]["host_boot_id"]


def test_synthetic_false_unmarks_every_header():
    out = ms.synth_trial(M1, CFG, still_s=2.0, synthetic=False)
    for name in ("record", "imu", "truth"):
        hdr = out[name][0]
        assert hdr["synthetic"] is False and not is_synthetic(hdr)
        assert refuse_or_mark(out[name], allow=False) == {}


# --- the IMU file and the adversarial options ----------------------------------------------------------------------

def test_imu_record_rows_are_what_the_recorder_writes(drift):
    imu = drift["imu"]
    prof = [r for r in imu if r.get("t") == "profile"]
    assert len(prof) == 1 and prof[0]["rate_mismatch"] is False
    assert prof[0]["options"]["enable_motion_correction"]["read_back"] == 0
    assert imu[-1]["t"] == "footer" and imu[-1]["stop"] == "seconds" and "error" not in imu[-1]
    assert imu[-1]["dropped"] == 0 and imu[-1]["write_errors"] == 0
    fits = imu_clock_fits(imu, stream="g")
    assert len(fits) == 1 and fits[0].valid
    assert fits[0].dropped == {"device_back": 0, "host_back": 0, "device_repeat": 0}
    # the IMU covers the record
    st = imu_stream(imu, stream="g")
    states = _states(drift["record"])
    assert st.host_ns[0] <= states[0]["read_ns"] and st.host_ns[-1] >= states[-1]["read_ns"] - 20_000_000


def test_dev_us_wrap_is_in_the_record_and_unwraps():
    out = ms.synth_trial(M1, CFG, dev_us_wrap=True, still_s=2.0)
    g = _gyro_record_order(out["imu"])
    dev = [r["dev_us"] for r in g]
    drops = [i for i in range(1, len(dev)) if dev[i] < dev[i - 1] - WRAP_US // 2]
    assert len(drops) == 1
    un = unwrap_u32_us(dev)
    assert all(b > a for a, b in zip(un, un[1:]))
    moving = _moving_hosts(out["record"])
    assert moving[0] < g[drops[0]]["host_ns"] < moving[-1]  # the wrap falls while the truck moves
    fits = imu_clock_fits(out["imu"], stream="g")
    assert len(fits) == 1 and fits[0].valid
    plain = ms.synth_trial(M1, CFG, still_s=2.0)
    dev0 = [r["dev_us"] for r in _gyro_record_order(plain["imu"])]
    assert all(b > a for a, b in zip(dev0, dev0[1:]))


def test_one_late_sample():
    out = ms.synth_trial(M1, CFG, imu_late_sample=True, still_s=2.0)
    g = _gyro_record_order(out["imu"])
    un = unwrap_u32_us([r["dev_us"] for r in g])
    back = [i for i in range(1, len(un)) if un[i] <= max(un[:i])]
    assert len(back) == 1
    hosts = [r["host_ns"] for r in g]
    assert hosts == sorted(hosts)  # the record is in arrival order; the late one arrived after its successors
    _, dropped = clean_points(un, hosts)
    assert dropped["device_back"] == 1
    plain = ms.synth_trial(M1, CFG, still_s=2.0)
    un0 = unwrap_u32_us([r["dev_us"] for r in _gyro_record_order(plain["imu"])])
    assert all(b > a for a, b in zip(un0, un0[1:]))
    assert sorted(un) == un0  # the same samples, the late one in another place


def test_constant_offset_between_mcu_and_imu_host_times():
    off = 0.015
    a = ms.synth_trial(M1, CFG, still_s=2.0, seed=7)
    b = ms.synth_trial(M1, CFG, still_s=2.0, seed=7, mcu_imu_offset_s=off)
    assert a["record"] == b["record"] and a["truth"] == b["truth"]
    ia = [r for r in a["imu"] if r.get("t") == "imu"]
    ib = [r for r in b["imu"] if r.get("t") == "imu"]
    assert len(ia) == len(ib)
    assert all(x["dev_us"] == y["dev_us"] and x["s"] == y["s"] and (x["x"], x["y"], x["z"]) == (y["x"], y["y"], y["z"])
               for x, y in zip(ia, ib))
    assert {y["host_ns"] - x["host_ns"] for x, y in zip(ia, ib)} == {round(off * 1e9)}
    fa, fb = imu_clock_fits(a["imu"])[0], imu_clock_fits(b["imu"])[0]
    assert fb.offset_ns - fa.offset_ns == pytest.approx(off * 1e9, abs=1e3)


def test_deadman_wait_before_motion():
    wait = 3.0
    out = ms.synth_trial(M1, CFG, deadman_wait_s=wait, still_s=2.0)
    rec = out["record"]
    drive = [r for r in rec if r.get("t") == "profile" and r["label"] == "straight" and r["state"] == "running"]
    seg_start = drive[0]["host_ns"]
    assert wait <= (_move_start_ns(rec) - seg_start) / 1e9 <= wait + 0.1
    waiting = [r for r in drive if r["host_ns"] < seg_start + int(wait * 1e9) - 20_000_000]
    assert waiting and all(not r["deadman"] and r["req_mps"] == 0.0 for r in waiting)
    assert ma.trials(rec)[0].valid
    plain = ms.synth_trial(M1, CFG, still_s=2.0)
    d0 = [r for r in plain["record"] if r.get("t") == "profile" and r["label"] == "straight"]
    assert (_move_start_ns(plain["record"]) - d0[0]["host_ns"]) / 1e9 < 0.1


def _flag_count(out, name):
    return sum(name in r["fault_flags"] for r in _states(out["record"]))


def _reasons(odo):
    return [why for _, _, why in odo["reasons"]]


@pytest.mark.parametrize("void, check", [
    ("deadman", lambda out, odo: [e["result"] for e in out["record"] if e.get("t") == "profile_end"]
     == ["void:deadman"]),
    ("ENCODER_JUMP_L", lambda out, odo: _flag_count(out, "ENCODER_JUMP_L") == 1),
    ("STEER_SENSOR_INVALID", lambda out, odo: _flag_count(out, "STEER_SENSOR_INVALID") == 1),
    ("boot_change", lambda out, odo: len({r["boot_id"] for r in _states(out["record"])}) == 2
     and "boot_changed" in _reasons(odo)),
    ("time_not_increasing", lambda out, odo: _reasons(odo).count("time_not_increasing") == 1),
    ("encoder_gap", lambda out, odo: "gap" in _reasons(odo)),
])
def test_void_options_put_their_defect_in_the_drive(void, check):
    out = ms.synth_trial(M1, CFG, void=void, still_s=2.0)
    assert check(out, _odo(out["record"]))


def test_a_clean_trial_has_no_defect():
    out = ms.synth_trial(M1, CFG, still_s=2.0)
    assert _odo(out["record"])["reasons"] == []
    assert all(r["fault_flags"] == [] for r in _states(out["record"]))
    assert len({r["boot_id"] for r in _states(out["record"])}) == 1


def test_unknown_void_and_bad_arguments_are_refused():
    with pytest.raises(ValueError):
        ms.synth_trial(M1, CFG, void="nonsense")
    with pytest.raises(ValueError):
        ms.synth_trial(M1, CFG, true_radius_scale=0.0)
    with pytest.raises(ValueError):
        ms.synth_trial(M1, CFG, up=(0.0, 0.0, 0.0))


def test_registered_m1_with_60_s_stills_runs_and_its_windows_are_long():
    out = ms.synth_trial(M1, CFG, seed=0)
    tr = ma.trials(out["record"])[0]
    assert tr.valid
    for label in ("still_pre", "still_post"):
        lo, hi = tr.spans[label]
        assert (hi - lo) / 1e9 == pytest.approx(60.0, abs=0.05)  # first to last row, rows 20 ms apart
