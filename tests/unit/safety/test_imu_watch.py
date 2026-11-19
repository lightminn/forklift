"""docs/design/2026-11-19-real-safety-wiring.md 4 (stage 5b): ImuWatch -- distinct valid samples only, a steady
baseline from the current stand frozen before the ARM, stale and gapped samples are a lost watch, tilt held over
consecutive samples."""

import math
from dataclasses import replace

import pytest

from forklift_core.safety.real_watch import ImuConfig, ImuWatch

MS = 1_000_000
G = (0.0, 0.0, 9.81)
CFG = ImuConfig(limit_deg=8.0, hold_s=0.3, max_age_s=0.1, gap_max_s=0.05, baseline_min=10, baseline_spread_deg=2.0,
                baseline_wait_s=0.5, norm_tol_frac=0.30)


def tilted(deg):
    a = math.radians(deg)
    return (9.81 * math.sin(a), 0.0, 9.81 * math.cos(a))


def fed(watch, start_ms, n, accel=G, step_ms=5):
    t = start_ms
    for _ in range(n):
        assert watch.add(t * MS, accel, t * MS) is None
        t += step_ms
    return t


def armed():
    w = ImuWatch(CFG)
    end = fed(w, 1000, 20)
    b, why = w.baseline(1000 * MS)
    assert why is None
    w.arm(b)
    w.settled((end - 5) * MS)  # READY seen at the last baseline sample
    return w, end


def test_invalid_samples_are_not_counted():
    w = ImuWatch(CFG)
    assert w.add(1000 * MS, G, 1000 * MS) is None
    assert w.add(1000 * MS, G, 1001 * MS) == "not_newer"  # the same sample again
    assert w.add(999 * MS, G, 1001 * MS) == "not_newer"  # rewound
    assert w.add(1010 * MS, G, 1005 * MS) == "future"
    assert w.add(1002 * MS, (math.nan, 0.0, 9.81), 1005 * MS) == "not_finite"
    assert w.add(1003 * MS, (0.0, 0.0, 6.8), 1005 * MS) == "norm"  # 31 % under g
    assert w.add(1004 * MS, (0.0, 0.0, 6.9), 1005 * MS) is None  # 29.7 % under: a fraction, not m/s^2
    assert w.counts["valid"] == 2


def test_the_baseline_needs_a_steady_gapless_stand():
    w = ImuWatch(CFG)
    fed(w, 1000, 20)
    assert w.baseline(None) == (None, "imu_not_still")
    assert w.baseline(1090 * MS) == (None, "imu_no_baseline")  # 2 samples since the stand began
    assert w.baseline(1000 * MS)[1] is None
    mixed = ImuWatch(CFG)  # a pose change while the wheels stood: 0 and 14 deg mixed
    t = fed(mixed, 1000, 5)
    fed(mixed, t, 5, tilted(14))
    assert mixed.baseline(1000 * MS) == (None, "imu_baseline_unsteady")
    opposite = ImuWatch(CFG)  # opposite vectors average to nothing
    t = 1000
    for k in range(10):
        opposite.add(t * MS, G if k % 2 else (0.0, 0.0, -9.81), t * MS)
        t += 5
    assert opposite.baseline(1000 * MS) == (None, "imu_baseline_unsteady")
    gapped = ImuWatch(CFG)  # the run after the gap counts alone (5 samples: not enough yet)
    t = fed(gapped, 1000, 8)
    t = fed(gapped, t + 60, 5)
    assert gapped.baseline(1000 * MS) == (None, "imu_no_baseline")
    fed(gapped, t, 5)
    (d, last), why = gapped.baseline(1000 * MS)
    assert why is None and last == (t + 20) * MS


def test_stale_on_the_host_clock_and_held():
    w, end = armed()
    assert w.judge((end + 90) * MS) is None
    assert w.judge((end + 110) * MS) == "watch_imu_stale"
    fed(w, end + 120, 5)
    assert w.judge((end + 145) * MS) == "watch_imu_stale"  # held until the disarm


def test_sparse_samples_are_a_lost_watch():
    """5b round 1 P1-1: tilted samples every 60 ms stay fresh but can never carry a hold."""
    w, end = armed()
    t = end
    for _ in range(3):
        t += 60
        w.add(t * MS, tilted(12), t * MS)
    assert w.judge(t * MS) == "watch_imu_gap"


def test_one_tilted_sample_is_not_a_hold_and_a_held_tilt_trips():
    w, end = armed()
    t = fed(w, end, 1, tilted(12))
    t = fed(w, t, 5)
    assert w.judge(t * MS) is None
    t = fed(w, t, 70, tilted(12))  # 0.345 s of consecutive tilted samples
    assert w.judge(t * MS) == "watch_tilt"


def test_an_upright_sample_restarts_the_hold():
    w, end = armed()
    t = fed(w, end, 40, tilted(12))  # 0.195 s
    t = fed(w, t, 1)
    t = fed(w, t, 40, tilted(12))
    assert w.judge(t * MS) is None


def test_the_baseline_is_frozen_before_the_arm():
    """Parked on a slight slope: the stand's mean is the baseline; it does not follow later samples."""
    w = ImuWatch(CFG)
    t = fed(w, 1000, 20, tilted(3))
    b, why = w.baseline(1000 * MS)
    w.arm(b)
    w.settled((t - 5) * MS)
    t = fed(w, t, 80, tilted(3))
    assert w.judge(t * MS) is None
    t = fed(w, t, 80, tilted(12))  # 9 deg off the frozen baseline
    assert w.judge(t * MS) == "watch_tilt"


def test_disarm_clears_the_trip_and_the_baseline():
    w, end = armed()
    assert w.judge((end + 200) * MS) == "watch_imu_stale"
    w.disarm()
    assert w.judge((end + 200) * MS) is None and not w.armed


@pytest.mark.parametrize("bad", [dict(limit_deg=0.0), dict(limit_deg=181.0), dict(limit_deg=90.0),
                                 dict(baseline_spread_deg=8.0), dict(gap_max_s=0.2), dict(hold_s=0.05),
                                 dict(norm_tol_frac=1.0), dict(baseline_min=1), dict(baseline_min=True),
                                 dict(max_age_s=10 ** 400)])
def test_the_config_refuses_settings_it_cannot_judge_with(bad):
    with pytest.raises(ValueError, match="imu_config:"):
        replace(CFG, **bad)


def test_the_yaml_imu_part_loads():
    from pathlib import Path

    from forklift_core.safety.real_watch import load_imu_config

    root = Path(__file__).resolve().parents[3]
    cfg = load_imu_config(root / "config/real/safety_watch.yaml")
    assert cfg.max_age_s == 0.1 and cfg.gap_max_s == 0.05 and cfg.norm_tol_frac == 0.30


def test_a_long_hold_is_measured_past_the_sample_window():
    """13th draft (round 2 P1-1): a hold of 3 s with samples kept for 2 s still trips."""
    w = ImuWatch(replace(CFG, hold_s=3.0, max_age_s=0.1))
    t = fed(w, 1000, 20)
    w.arm(w.baseline(1000 * MS)[0])
    w.settled((t - 5) * MS)
    for _ in range(80):  # 4 s of 12 deg, judged every 50 ms
        t = fed(w, t, 10, tilted(12))
        if w.judge(t * MS) is not None:
            break
    assert w.judge(t * MS) == "watch_tilt"


def test_the_gap_is_measured_from_the_baselines_last_sample():
    """13th draft (round 2 P2-1): a sample taken just after the baseline's last but received after the freeze."""
    w = ImuWatch(CFG)
    fed(w, 1000, 10)  # 1000..1045 ms
    b, _ = w.baseline(1000 * MS)
    w.arm(b)
    assert w.add(1100 * MS, G, 1101 * MS) is None  # 55 ms after the baseline's last
    assert w.judge(1101 * MS) == "watch_imu_gap"


def test_a_stand_off_the_level_reference_is_refused():
    """5b round 6: where the truck stands, not the stand's history -- a 14 deg pose is not a baseline however it
    came about (a gap, a lost stand)."""
    w = ImuWatch(CFG)
    fed(w, 1000, 20, tilted(14))
    assert w.baseline(1000 * MS) == (None, "imu_not_level")
    w = ImuWatch(CFG)
    fed(w, 1000, 20, tilted(3))
    assert w.baseline(1000 * MS)[1] is None


def test_a_move_between_the_baseline_and_the_arms_answer_is_the_end_at_once():
    """5b round 6 P2-1: while the ARM is unanswered any sample off the baseline by its spread trips -- no hold."""
    w = ImuWatch(CFG)
    t = fed(w, 1000, 20)
    w.arm(w.baseline(1000 * MS)[0])
    t = fed(w, t, 1, tilted(3))
    assert w.judge(t * MS) == "watch_imu_moved"
    w = ImuWatch(CFG)
    t = fed(w, 1000, 20)
    w.arm(w.baseline(1000 * MS)[0])
    w.settled(t * MS)
    t = fed(w, t + 5, 1, tilted(3))
    assert w.judge(t * MS) is None  # armed and answered: the tilt hold rules


def test_one_outlier_ages_out_of_the_window():
    """5b round 6 P2-2: a valid 3 deg outlier makes the stand unsteady only while it is in the window."""
    w = ImuWatch(CFG)
    t = fed(w, 1000, 10)
    t = fed(w, t, 1, tilted(3))
    t = fed(w, t, 20)
    assert w.baseline(1000 * MS) == (None, "imu_baseline_unsteady")
    t = fed(w, t, 420)  # 2.1 s more of level samples
    assert w.baseline(1000 * MS)[1] is None


def test_the_level_reference_must_be_a_direction():
    for bad in ((0.0, 0.0, 0.0), {0: 0.0, 1: 0.0, 2: 1.0}, "001", (0.0, 0.0, True), (0.0, 0.0, float("nan")),
                (0.0, 1.0), (0, 0, 10 ** 400)):
        with pytest.raises(ValueError, match="imu_config:level_ref"):
            replace(CFG, level_ref=bad)
    with pytest.raises(ValueError, match="imu_config:level_tol_deg"):
        replace(CFG, level_tol_deg=9.0)


def test_the_arming_rule_follows_the_acquisition_time_not_the_read():
    """5b round 8 P2: samples taken before the READY boundary meet the ARMING rule even when a later read brings
    them; samples taken after it meet the tilt hold."""
    w = ImuWatch(CFG)
    t = fed(w, 1000, 20)
    w.arm(w.baseline(1000 * MS)[0])
    w.settled((t + 20) * MS)  # READY seen at t + 20 ms, before these samples are read
    for k in range(3):
        w.add((t + 5 * k) * MS, tilted(3), (t + 30) * MS)  # taken before the boundary
    assert w.judge((t + 30) * MS) == "watch_imu_moved"
    w = ImuWatch(CFG)
    t = fed(w, 1000, 20)
    w.arm(w.baseline(1000 * MS)[0])
    w.settled(t * MS)
    t2 = fed(w, t + 5, 3, tilted(3))  # taken after it
    assert w.judge(t2 * MS) is None
