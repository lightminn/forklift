"""Design 2026-11-19-real-assembly 2, 3, 6: the real options named and checked, the S10-A settings, and readiness."""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.plant.kinematics import RealKinematics

from test_teleop_bridge_cycle import VEH

ROOT = Path(__file__).resolve().parents[3]
KIN = RealKinematics(VEH)
LOW, HIGH = KIN.cfg.steering.curvature_range
CAP = min(-LOW, HIGH)


PLAN = SimpleNamespace(curvature_limit_inv_m=CAP)


def options(**over):
    o = ra.real_options(kinematics=KIN, planner_config=PLAN, travel_config=PLAN, scenario="SC",
                        speeds={name: 0.05 for name in ra.S10A_STAGES}, mission_stages=ra.S10A_STAGES)
    for k, v in over.items():
        setattr(o, k, v)
    return o


def refusal(o, tracker=CAP, preview=CAP, cap=0.15):
    return ra.real_option_refusal(o, tracker_curvature_inv_m=tracker, preview_curvature_inv_m=preview,
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


def test_the_curvature_caps_are_read_from_the_real_configs_and_must_agree_inside_the_steering_map():
    assert any(r.startswith("curvature_caps_differ") for r in refusal(options(), tracker=CAP / 2))
    other = SimpleNamespace(curvature_limit_inv_m=CAP / 2)
    assert any(r.startswith("curvature_caps_differ") for r in refusal(options(travel_config=other)))
    wide = SimpleNamespace(curvature_limit_inv_m=CAP * 1.1)
    assert any(r.startswith("curvature_beyond_steering") for r in refusal(
        options(planner_config=wide, travel_config=wide), tracker=CAP * 1.1, preview=CAP * 1.1))
    assert "curvature_invalid:preview" in refusal(options(), preview=None)
    neg = SimpleNamespace(curvature_limit_inv_m=-CAP)
    assert "curvature_invalid:planner" in refusal(options(planner_config=neg))


def test_speeds_are_finite_and_within_the_s10_cap():
    assert "speed_over_cap:approach" in refusal(options(speeds={**{n: 0.05 for n in ra.S10A_STAGES}, "approach": 0.3}))
    assert "speed_over_cap:approach_straight" in refusal(options(approach_straight_speed_mps=0.2))
    assert "speed_over_cap:approach_straight" in refusal(options(approach_straight_speed_mps=float("nan")))
    assert "max_speed_invalid" in refusal(options(), cap=float("nan"))


def test_the_repository_s10a_template_loads_and_refuses_until_its_field_values_are_set():
    cfg = ra.load_s10a(ROOT / "config/real/s10a.yaml")
    why = ra.s10a_refusal(cfg, loaded_map_id=None)
    assert {"map_id_unset", "pallet_depth_m_unset", "pallet_width_m_unset"} <= set(why)
    good = replace(cfg, map_id="hall-1", pallet_depth_m=0.6, pallet_width_m=0.4)
    assert ra.s10a_refusal(good, loaded_map_id="hall-1") == []
    assert ra.s10a_refusal(good, loaded_map_id="hall-2") == ["map_id_mismatch:hall-2"]
    assert "start_rear_outside_bounds" in ra.s10a_refusal(replace(good, start_rear=(50.0, 0.0, 0.0)), "hall-1")


NOW = 10 * 10**9


def evidence(**over):
    ok = dict(worker_ok=lambda n: True, layer_ok=lambda n: True, baseline_ok=lambda n: True,
              link_reason=lambda n: None,
              measurement=lambda n: ra.MeasurementView(True, True, True, True),
              odometry_continuous=lambda n: True,
              slam=lambda n: ra.SlamView("tracking", n - 10**8),
              snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n - 10**8, False, 4, 4),
              operator_start_ok=lambda n: True, core_refusal=lambda n: None, mission_gen=lambda n: 3)
    ok.update(over)
    return ra.Evidence(**ok)


@pytest.mark.parametrize("over, why", [
    ({"worker_ok": lambda n: False}, "worker:dead"),
    ({"layer_ok": lambda n: False}, "layer:missing"),
    ({"baseline_ok": lambda n: False}, "baseline:missing"),
    ({"core_refusal": lambda n: "relocalize_required"}, "core:relocalize_required"),
    ({"link_reason": lambda n: "clock_rate_unverified"}, "link:clock_rate_unverified"),
    ({"measurement": lambda n: ra.MeasurementView(True, True, False, True)}, "measurement:lift_origin"),
    ({"odometry_continuous": lambda n: False}, "odometry:cut"),
    ({"slam": lambda n: ra.SlamView("holding", n)}, "slam:mode_holding"),
    ({"slam": lambda n: ra.SlamView("tracking", n - 10**9)}, "slam:stale"),
    ({"slam": lambda n: ra.SlamView("tracking", None)}, "slam:stale"),
    ({"snapshot": lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n - 10**9, False, 4, 4)},
     "snapshot:scan_stale"),
    ({"operator_start_ok": lambda n: False}, "operator_start:unconfirmed"),
    ({"snapshot": lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), n, False, 4, 4)},
     "wait:snapshot_installing"),
    ({"snapshot": lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, False, 3, 4)},
     "wait:snapshot_installing"),  # the same values from an older generation (Codex 4b P2)
])
def test_each_readiness_condition_refuses_a_submit_by_its_reason(over, why):
    assert ra.auto_ready_refusal(NOW, "submit", evidence(**over)) == why


def test_a_latch_reason_is_never_hidden_by_an_installing_snapshot():
    """Codex 4b P2: a recovery lock and a snapshot being installed at once -- the lock is what the permit sees."""
    ev = evidence(core_refusal=lambda n: "recovery_required",
                  snapshot=lambda n: ra.SnapshotView(None, (0.1, 0.0, 0.0), n, installing=True))
    why = ra.auto_ready_refusal(NOW, "drive", ev)
    assert why == "core:recovery_required" and ra.latches(why)
    assert not ra.latches("wait:snapshot_installing") and not ra.latches(None)


def test_driving_accepts_the_missions_hold_within_its_travel():
    assert ra.auto_ready_refusal(NOW, "drive", evidence(slam=lambda n: ra.SlamView("holding", n, 0.5, 3))) is None
    assert ra.auto_ready_refusal(NOW, "drive", evidence(
        slam=lambda n: ra.SlamView("holding", n, 2.0, 3))) == "slam:hold_too_long"
    assert ra.auto_ready_refusal(NOW, "drive", evidence(
        slam=lambda n: ra.SlamView("holding", n, 0.5, 2))) == "slam:hold_not_this_missions"


def test_all_met_is_ready_and_the_operator_check_is_the_submits_only():
    assert ra.auto_ready_refusal(NOW, "submit", evidence()) is None
    assert ra.auto_ready_refusal(NOW, "drive", evidence(operator_start_ok=lambda n: False)) is None
    with pytest.raises(ValueError):
        ra.auto_ready_refusal(NOW, "arm", evidence())


def test_the_stages_are_s10as_and_every_one_needs_its_speed():
    assert "speed_missing:insert" in refusal(options(speeds={"approach": 0.1}))
    assert "mission_stages_not_s10a" in refusal(options(mission_stages=("insert",)))
    assert "mission_stages_not_s10a" in refusal(options(mission_stages=()))


BYPASS_CASES = [
    ({"measurement": lambda n: ra.MeasurementView(True, True, False, True)}, "measurement"),
    ({"odometry_continuous": lambda n: False}, "odometry"),
    ({"slam": lambda n: ra.SlamView("lost", None)}, "slam"),
    ({"snapshot": lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n - 10**9, False, 4, 4)},
     "layer_timing"),
    ({"snapshot": lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, False, 4, 4, "layer_overdue")},
     "layer_timing"),
    ({"operator_start_ok": lambda n: False}, "operator_start"),
    ({"snapshot": lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), n, False, 3, 4)}, "snapshot"),
]


@pytest.mark.parametrize("over, check", BYPASS_CASES)
def test_a_bypassed_readiness_check_passes(over, check):
    """Real safety wiring design 5 (5c-1): the operator's session bypass skips that category only."""
    assert ra.auto_ready_refusal(NOW, "submit", evidence(**over)) is not None
    assert ra.auto_ready_refusal(NOW, "submit", evidence(**over), bypass={check}) is None
    others = set(ra.READY_BYPASSABLE) - {check}
    assert ra.auto_ready_refusal(NOW, "submit", evidence(**over), bypass=others) is not None


@pytest.mark.parametrize("over", [{"worker_ok": lambda n: False}, {"layer_ok": lambda n: False},
                                  {"baseline_ok": lambda n: False}, {"link_reason": lambda n: "stale"},
                                  {"core_refusal": lambda n: "relocalize_required"}])
def test_the_rest_of_readiness_is_not_bypassable(over):
    assert ra.auto_ready_refusal(NOW, "submit", evidence(**over), bypass=set(ra.READY_BYPASSABLE)) is not None


def test_the_snapshot_bypass_keeps_an_install_still_waited_for():
    """5c-1 r2 P2-3: another mission's or path's install (the view's installing) is still a wait."""
    ev = evidence(snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), n, True, 3, 4))
    assert ra.auto_ready_refusal(NOW, "drive", ev, bypass={"snapshot"}) == "wait:snapshot_installing"
