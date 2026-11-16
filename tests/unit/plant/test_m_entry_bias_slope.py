"""m_entry.analyze, the in-trial bias-slope check judged afterwards over one IMU stretch (registration revision 3,
10.1; entry design 3.10, PR 3b): the other records of the block enter as standing windows only, the fit is per
trial over the last n = min(6, available) intervals, n < 3 is unconfirmed, field rechecks enter as 60 s pieces."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from forklift_core.plant import m_entry as me
from forklift_core.plant import m_synth as ms
from forklift_core.plant.real_vehicle import load_real_vehicle

REPO = Path(__file__).resolve().parents[3]
CFG = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
UP = (0.0, -1.0, 0.0)
SIGMA = {"pose_xy_m": 5e-4, "pose_yaw_rad": 3e-4, "source": "synthetic"}
L09 = "m3_merge_o20_l09_left"
QUIET_ARW = 0.2 * ms.ARW_DESK  # small noise: the fitted slope is then the injected one within ~5e-8


def profile(name):
    return yaml.safe_load((REPO / "config" / "profiles" / "m1_m3" / f"{name}.yaml").read_text(encoding="utf-8"))


def block(items, **kw):
    kw.setdefault("arw", QUIET_ARW)
    out = ms.synth_block(items, CFG, **kw)
    for t in out["truths"]:
        if t is not None:
            t[0]["sigma"] = dict(SIGMA)
    return out


def run_last(out, name, *, field=None):
    """The last record is analysed; the earlier ones are its block records."""
    return me.analyze(out["records"][-1], out["imu"], out["truths"][-1], field, profile(name), CFG, UP,
                      allow_synthetic=True, block_rows=out["records"][:-1])


def trial1(res):
    (t,) = res["trials"]
    assert t["status"] == "valid", t
    return t


def merge_items(n):
    return [{"profile": profile(L09)} for _ in range(n)]


@pytest.mark.parametrize("slope", [0.0, 2e-6])
def test_bias_slope_moving_fit_over_the_block(slope):
    t = trial1(run_last(block(merge_items(3), bias_slope=slope, seed=1), L09))
    b = t["imu"]["bias_slope"]
    assert b["n_windows"] == 6 and b["value"] == pytest.approx(slope, abs=1e-7)
    why = t["values"]["imu_heading"]["merge_end"]["why"]
    if slope == 0.0:
        assert "bias_slope" not in why and "bias_slope_unconfirmed" not in why
        assert b["judgeable"]
    else:
        assert "bias_slope" in why and not b["judgeable"] and b["why"] == ["bias_slope"]


def test_a_trial_alone_is_unconfirmed():
    out = block(merge_items(1), seed=2)
    t = trial1(run_last(out, L09))
    b = t["imu"]["bias_slope"]
    assert b["n_windows"] == 2 and b["value"] is None and b["why"] == ["bias_slope_unconfirmed"]
    assert "bias_slope_unconfirmed" in t["values"]["imu_heading"]["merge_end"]["why"]


def test_standing_records_give_one_window_per_stop():
    # two G3-like stops before the trial: 1 + 1 + the trial's own 2 = 4 intervals, enough for a slope
    out = block([{"stand_s": 65.0}, {"stand_s": 65.0}, {"profile": profile(L09)}], seed=3)
    b = trial1(run_last(out, L09))["imu"]["bias_slope"]
    assert b["n_windows"] == 4 and b["kinds"].count("standing_record") == 2
    assert b["value"] == pytest.approx(0.0, abs=2e-7)


def test_recheck_pieces_enter_and_their_passed_flag_is_not_trusted():
    out = block([{"stand_s": 300.0}, {"profile": profile(L09)}], seed=4)
    still = out["records"][0]
    a = still[1]["host_ns"] + 5_000_000_000
    b = a + 240_000_000_000  # four 60 s pieces; the still record's own window is its last 60 s (kept over a piece)
    field = [{"t": "header", "schema": "field_log/1", "synthetic": True,
              "record": {"host_boot_id": out["records"][-1][0]["host_boot_id"],
                         "start_mono_ns": out["records"][-1][0]["start_mono_ns"]}},
             {"t": "bias_recheck", "block": "day1", "exceeded_host_ns": a - 1, "recheck_start_ns": a,
              "recheck_end_ns": b, "slope_rad_s2": 9.9, "passed": False}]
    res = run_last(out, L09, field=field)
    bs = trial1(res)["imu"]["bias_slope"]
    assert bs["kinds"].count("recheck_piece") >= 3  # passed False and a nonsense slope: the pieces still count
    assert bs["n_windows"] == 6 and bs["value"] == pytest.approx(0.0, abs=2e-7)


def test_block_records_are_checked():
    out = block(merge_items(2), seed=5)
    other = [dict(r) for r in out["records"][0]]
    other[0] = {**other[0], "host_boot_id": "another-boot"}
    with pytest.raises(me.Refused, match="block record"):
        me.analyze(out["records"][-1], out["imu"], out["truths"][-1], None, profile(L09), CFG, UP,
                   allow_synthetic=True, block_rows=[other])
    dirty = [dict(r) for r in out["records"][0]]
    dirty[-1] = {**dirty[-1], "dropped": 3}
    with pytest.raises(me.Refused, match="block record"):
        me.analyze(out["records"][-1], out["imu"], out["truths"][-1], None, profile(L09), CFG, UP,
                   allow_synthetic=True, block_rows=[dirty])
    with pytest.raises(me.Refused, match="overlap"):
        me.analyze(out["records"][-1], out["imu"], out["truths"][-1], None, profile(L09), CFG, UP,
                   allow_synthetic=True, block_rows=[out["records"][-1]])


def test_m1_reports_the_slope_as_information():
    out = block([{"profile": profile("m1_fwd_v010")} for _ in range(3)], seed=6)
    for t in out["truths"]:
        t[0]["sigma"] = dict(SIGMA)
    t = trial1(me.analyze(out["records"][-1], out["imu"], out["truths"][-1], None, profile("m1_fwd_v010"), CFG, UP,
                          allow_synthetic=True, block_rows=out["records"][:-1]))
    assert t["imu"]["report_only"] and t["imu"]["bias_slope"]["n_windows"] == 6


def test_an_imu_clock_break_ends_the_stretch():
    # the device clock restarts before the last trial: the two earlier trials' windows are another stretch
    out = block(merge_items(3), seed=7, imu_restart_before=2)
    b = trial1(run_last(out, L09))["imu"]["bias_slope"]
    assert b["n_windows"] == 2 and b["why"] == ["bias_slope_unconfirmed"]


def test_a_block_record_outside_the_imu_file_is_refused():
    out = block(merge_items(2), seed=8)
    early = block(merge_items(1), seed=9)["records"][0]  # a record of the same boot from before this IMU file
    shifted = [{**r, **({"read_ns": r["read_ns"] - 10**12, "host_ns": r["host_ns"] - 10**12} if "read_ns" in r else {})}
               for r in early]
    with pytest.raises(me.Refused, match="outside the IMU file"):
        me.analyze(out["records"][-1], out["imu"], out["truths"][-1], None, profile(L09), CFG, UP,
                   allow_synthetic=True, block_rows=[shifted])
