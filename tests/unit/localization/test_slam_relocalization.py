"""docs/design/2026-11-19-real-safety-wiring.md 6 (stage 5c-2): the tracker's relocalization reset and its reads while
the real truck's outputs are closed, and the rear-axle <-> base_link conversion the start comparison uses. Nothing
here changes the tracker's default behaviour (the Isaac path never calls these)."""

import math

import pytest

from forklift_core.localization.slam_pose import LocalizationStale, SlamPoseTracker, compose
from forklift_core.localization.start_reloc import base_from_rear, rear_from_base


def tracking(max_age_s=0.5, hold_limit_m=0.3):
    t = SlamPoseTracker(max_age_s=max_age_s, hold_limit_m=hold_limit_m, bootstrap_scan_id=0)
    t.receive(0, 1.0, "processed", (1.0, 2.0, 0.1))
    assert t.mode == "tracking"
    return t


def test_base_from_rear_inverts_rear_from_base():
    for base in ((0.0, 0.0, 0.0), (1.5, -2.0, 0.7), (-3.0, 4.0, -2.9)):
        for offset in (0.0, 0.21, -0.1):
            back = base_from_rear(rear_from_base(base, offset), offset)
            assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(back, base))


def test_reset_keeps_the_corrections_and_drops_the_hold():
    t = tracking()
    t.hold(odom_from_base=(0.0, 0.0, 0.0))
    t.map_from_base(1.1, (0.1, 0.0, 0.0))
    t.reset_for_relocalization()
    assert t.mode == "warming_up" and not t.may_drive()
    assert t.applied[0] == (1.0, 2.0, 0.1) and t.received[0] == (1.0, 2.0, 0.1)
    assert t.hold_travel_m == 0.0 and t._hold_start is None  # noqa: SLF001


def test_while_relocalizing_processed_replies_apply_but_stay_warming_up():
    t = tracking()
    t.reset_for_relocalization()
    t.receive(50, 2.0, "processed", (3.0, 0.0, 0.0))  # any scan id
    assert t.applied[0] == (3.0, 0.0, 0.0) and t.mode == "warming_up"
    # warming_up reads no age: a long silence is no exception while the generation is open
    assert t.map_from_base(99.0, (0.0, 0.0, 0.0)) == compose((3.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    t.relocalization_done()
    assert t.mode == "tracking"
    with pytest.raises(LocalizationStale):
        t.map_from_base(99.0, (0.0, 0.0, 0.0))  # tracking again: the age counts as before


def test_relocalization_done_needs_a_reset():
    with pytest.raises(ValueError):
        tracking().relocalization_done()


def test_a_closed_read_skips_the_age_and_the_hold_limit_only():
    t = tracking(hold_limit_m=0.3)
    with pytest.raises(LocalizationStale):
        t.map_from_base(5.0, (0.0, 0.0, 0.0))
    assert t.map_from_base(5.0, (0.0, 0.0, 0.0), closed=True) == compose((1.0, 2.0, 0.1), (0.0, 0.0, 0.0))
    t.receive(1, 5.0, "processed", (1.0, 2.0, 0.1))
    t.hold(odom_from_base=(0.0, 0.0, 0.0))
    t.map_from_base(5.1, (0.5, 0.0, 0.0), closed=True)  # 0.5 m over the 0.3 m limit: no exception, still counted
    assert t.hold_travel_m == pytest.approx(0.5)
    with pytest.raises(LocalizationStale):
        t.map_from_base(5.2, (0.5, 0.0, 0.0))  # an open read raises as before


def test_a_closed_read_before_any_correction_is_the_odometry():
    t = SlamPoseTracker(max_age_s=0.5, hold_limit_m=0.3)
    assert t.map_from_base(1.0, (0.4, 0.1, 0.2), closed=True) == (0.4, 0.1, 0.2)
    with pytest.raises(LocalizationStale):
        t.map_from_base(1.0, (0.4, 0.1, 0.2))


def test_the_default_behaviour_is_unchanged():
    t = SlamPoseTracker(max_age_s=0.5, hold_limit_m=0.3, bootstrap_scan_id=4)
    t.receive(2, 1.0, "processed", (0.0, 0.0, 0.0))
    assert t.mode == "warming_up"  # the bootstrap id still ends warming up when not relocalizing
    t.receive(4, 1.1, "processed", (0.0, 0.0, 0.0))
    assert t.mode == "tracking"
