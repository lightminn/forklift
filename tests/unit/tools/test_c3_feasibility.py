import math

import pytest

from tools import c3_feasibility as c3


def test_perturbations_are_fixed_and_bounded():
    a, b = c3.perturbations(), c3.perturbations()
    assert a == b and len(a) == c3.SEEDS
    assert all(abs(lat) <= c3.LATERAL_M and abs(yaw) <= c3.YAW_RAD for lat, yaw in a)


def test_aligned_pallet_sits_gap_plus_half_depth_ahead_of_the_tip():
    x, y, yaw = c3.pallet_pose_in_truck(0.6, 0.95, 0.0, 0.0)
    assert x == pytest.approx(0.95 + 0.6 + c3.HALF_DEPTH)
    assert y == pytest.approx(0.0) and yaw == pytest.approx(0.0)


def test_truck_left_of_the_line_puts_the_pallet_to_its_right():
    _, y, _ = c3.pallet_pose_in_truck(0.6, 0.95, 0.03, 0.0)
    assert y == pytest.approx(-0.03)


def test_redraw_from_the_line_itself_needs_the_remaining_length():
    # On the line, 0.5 m of straight is left at g 0.6 (insertion starts at 0.1): the box passes.
    out = c3.redraw(0.6, 0.0, 0.0)
    assert out["box"] is True
    assert out["offsets"]["length_left_m"] == pytest.approx(0.5, abs=0.05)


def test_redraw_refuses_a_rear_offset_outside_the_box():
    # Rear lateral = tip lateral - 1.29 sin(yaw): 0.04 + 1.29 sin(1.5 deg) > 0.05.
    out = c3.redraw(0.6, 0.04, -math.radians(1.5))
    assert out["box"] is False and out["ok"] is False


def test_a_timed_out_rollout_is_written_as_strict_json(monkeypatch):
    import json
    from types import SimpleNamespace

    monkeypatch.setattr(c3, "bicycle_rollout", lambda *a, **k: SimpleNamespace(
        status="timeout", position_error_m=float("nan"), yaw_error_rad=float("nan"), trajectory=()))
    out = c3.redraw(0.6, 0.0, 0.0)
    assert out["dry"]["position_error_m"] is None and out["dry"]["yaw_error_rad"] is None
    json.dumps(out, allow_nan=False)
