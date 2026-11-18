"""Design 2026-11-19-real-assembly 2, 3, 6: the real options named and checked, the S10-A settings, and readiness."""

from dataclasses import replace
from pathlib import Path

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.plant.kinematics import RealKinematics

from test_teleop_bridge_cycle import VEH

ROOT = Path(__file__).resolve().parents[3]
KIN = RealKinematics(VEH)
LOW, HIGH = KIN.cfg.steering.curvature_range
CAP = min(-LOW, HIGH)


def options(**over):
    o = ra.real_options(kinematics=KIN, planner_config="PC", travel_config="TC", scenario="SC",
                        speeds={"approach": 0.1, "insert": 0.05}, mission_stages=("approach", "insert"))
    for k, v in over.items():
        setattr(o, k, v)
    return o


def refusal(o, curv=None, cap=0.15):
    return ra.real_option_refusal(o, curvatures=curv or {"planner": CAP, "tracker": CAP, "preview": CAP},
                                  max_speed_mps=cap)


def test_the_real_options_pass_and_name_every_switch():
    o = options()
    assert refusal(o) == []
    assert o.dt == 1 / 50 and o.real_vehicle and o.s10_stop_at_lift and o.near_tracking is None
    assert o.bottom_deck_check == "off" and o.observation_waypoints == []


@pytest.mark.parametrize("over, why", [
    ({"dt": 1 / 120}, "dt_not_1_50"),
    ({"real_vehicle": False}, "real_vehicle_not_True"),
    ({"obstacle_act": False}, "obstacle_act_not_True"),
    ({"pocket_check": False}, "pocket_check_not_True"),
    ({"s10_stop_at_lift": False}, "s10_stop_at_lift_not_True"),
    ({"grid_planning": False}, "grid_planning_not_True"),
    ({"near_tracking": object()}, "near_tracking_on"),
    ({"bottom_deck_check": "structure"}, "bottom_deck_check_on"),
    ({"kinematics": None}, "kinematics_missing"),
])
def test_each_required_value_missing_or_wrong_refuses(over, why):
    assert why in refusal(options(**over))


def test_the_curvature_caps_must_agree_and_stay_inside_the_steering_map():
    assert any(r.startswith("curvature_caps_differ") for r in refusal(options(), {"planner": CAP, "tracker": CAP / 2}))
    assert any(r.startswith("curvature_beyond_steering") for r in refusal(options(), {"planner": CAP * 1.1,
                                                                                         "tracker": CAP * 1.1}))


def test_speeds_stay_within_the_s10_cap():
    assert "speed_over_cap:approach" in refusal(options(speeds={"approach": 0.3}))
    assert "speed_over_cap:approach_straight" in refusal(options(approach_straight_speed_mps=0.2))
    assert "max_speed_invalid" in refusal(options(), cap=float("nan"))


def test_the_repository_s10a_template_loads_and_refuses_until_its_field_values_are_set():
    cfg = ra.load_s10a(ROOT / "config/real/s10a.yaml")
    why = ra.s10a_refusal(cfg, loaded_map_id=None)
    assert {"map_id_unset", "pallet_depth_m_unset", "pallet_width_m_unset"} <= set(why)
    good = replace(cfg, map_id="hall-1", pallet_depth_m=0.6, pallet_width_m=0.4)
    assert ra.s10a_refusal(good, loaded_map_id="hall-1") == []
    assert ra.s10a_refusal(good, loaded_map_id="hall-2") == ["map_id_mismatch:hall-2"]
    assert "start_rear_outside_bounds" in ra.s10a_refusal(replace(good, start_rear=(50.0, 0.0, 0.0)), "hall-1")


def evidence(**over):
    ok = dict(worker_ok=lambda n: True, layer_ok=lambda n: True, baseline_ok=lambda n: True,
              link_reason=lambda n: None, measurement_reason=lambda n: None, odometry_continuous=lambda n: True,
              slam_reason=lambda n, stage: None, snapshot_reason=lambda n: None, operator_start_ok=lambda n: True,
              core_refusal=lambda n: None)
    ok.update(over)
    return ra.Evidence(**ok)


@pytest.mark.parametrize("over, why", [
    ({"worker_ok": lambda n: False}, "worker:dead"),
    ({"layer_ok": lambda n: False}, "layer:missing"),
    ({"baseline_ok": lambda n: False}, "baseline:missing"),
    ({"link_reason": lambda n: "clock_rate_unverified"}, "link:clock_rate_unverified"),
    ({"measurement_reason": lambda n: "lift_origin"}, "measurement:lift_origin"),
    ({"odometry_continuous": lambda n: False}, "odometry:cut"),
    ({"slam_reason": lambda n, stage: "stale"}, "slam:stale"),
    ({"snapshot_reason": lambda n: "installing"}, "snapshot:installing"),
    ({"core_refusal": lambda n: "relocalize_required"}, "core:relocalize_required"),
    ({"operator_start_ok": lambda n: False}, "operator_start:unconfirmed"),
])
def test_each_readiness_condition_refuses_a_submit_by_its_reason(over, why):
    assert ra.auto_ready_refusal(1, "submit", evidence(**over)) == why


def test_all_met_is_ready_and_the_operator_check_is_the_submits_only():
    assert ra.auto_ready_refusal(1, "submit", evidence()) is None
    assert ra.auto_ready_refusal(1, "drive", evidence(operator_start_ok=lambda n: False)) is None


def test_the_slam_check_is_told_the_stage():
    seen = []
    ra.auto_ready_refusal(1, "drive", evidence(slam_reason=lambda n, stage: seen.append(stage)))
    assert seen == ["drive"]
    with pytest.raises(ValueError):
        ra.auto_ready_refusal(1, "arm", evidence())
