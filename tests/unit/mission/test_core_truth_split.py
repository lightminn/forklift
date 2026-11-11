"""Sim-to-real 4d-1: the safety speed and the truth access rules (docs/design/2026-11-11-p05-truth-audit.md 3a, 4)."""

from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.safety.truth_access import SafetySpeed, TruthAccessViolation


def test_the_exact_speed_is_todays_rule():
    for v in (0.3, -0.2, 0.0, -0.0, 1e-9):
        s = SafetySpeed.exact(v)
        assert s.signed_mps == v and s.bound_mps == abs(v) and s.valid
        assert s.directions == ((1,) if v > 0 else (-1,))  # a standing truck reads -1, as before
        assert s.standing(0.01) is (abs(v) < 0.01)


def test_a_safety_speed_must_cover_its_value():
    with pytest.raises(ValueError):
        SafetySpeed(0.3, 0.1, (1,))
    with pytest.raises(ValueError):
        SafetySpeed(0.0, 0.0, (2,))
    assert not SafetySpeed(0.0, 0.0, (1, -1), valid=False).standing(0.01)


def test_the_violation_is_not_absorbed_by_mission_failure_handling():
    assert not issubclass(TruthAccessViolation, Exception)
    with pytest.raises(TruthAccessViolation):
        try:
            raise TruthAccessViolation("truth_velocity")
        except Exception:  # the runner's mission-failure path
            pytest.fail("absorbed")


def core(*, real=False, hooks=None):
    hooks = hooks or SimpleNamespace(truth_speed=lambda: 0.25, truth_velocity=lambda: np.array([0.25, 0.0, 0.0]))
    opts = SimpleNamespace(drive_geometry=None, command_accel_mps2=0.8, obstacle_act=False, real_vehicle=real)
    return MissionCore(MissionState(phase="transport", phase_started=0.0), state={}, obstacle=None, slam=None,
                       pocket={"check": None}, paths={}, trackers={}, geometry=None,
                       settings={"steering_command_rate_rad_s": 1.0}, pallet_geometry=None, hooks=hooks, near=None,
                       options=opts)


def test_the_simulation_falls_back_to_the_true_speed_and_the_real_truck_never_does():
    assert core().safety_speed() == SafetySpeed.exact(0.25)
    with pytest.raises(TruthAccessViolation):
        core(real=True).safety_speed()
    given = SafetySpeed(0.1, 0.15, (1, -1))
    assert core(real=True, hooks=SimpleNamespace(safety_speed=lambda: given)).safety_speed() is given


def test_a_slam_off_truth_read_raises_on_the_real_truck():
    assert np.array_equal(core()._truth("truth_velocity"), [0.25, 0.0, 0.0])  # noqa: SLF001
    with pytest.raises(TruthAccessViolation):
        core(real=True)._truth("truth_velocity")  # noqa: SLF001


def recording_limit(blocked=None):
    calls = []

    def limit(d, cap):
        calls.append((d, cap))
        return (0.0, "occupied") if d == blocked else (min(cap, 0.4), "ok")

    return limit, calls


def test_the_simulation_checks_one_direction_as_before():
    c = core()
    for speed, direction, want in ((SafetySpeed.exact(0.3), None, 1), (SafetySpeed.exact(0.0), None, -1),
                                   (SafetySpeed.exact(-0.3), 1, 1)):
        limit, calls = recording_limit(blocked=-1)
        c.state["command_speed"] = -0.2  # never read in the simulation
        out = c._direction_limits(limit, speed, direction, 0.3)  # noqa: SLF001
        assert calls == [(want, 0.3)] and out[2:] == (False, [want], False)


def test_the_real_truck_checks_the_motion_it_may_have_unasked():
    c = core(real=True)
    # asked forward, rolling back: the request at the cap, the rearward motion at the bound, a hold when blocked
    limit, calls = recording_limit(blocked=-1)
    assert c._direction_limits(limit, SafetySpeed(-0.2, 0.2, (-1,)), 1, 0.4) == (  # noqa: SLF001
        0.0, "occupied", True, [1], True)
    assert calls == [(1, 0.4), (-1, 0.2)]
    # a standing reading still has its residual motion checked (Codex 4d-1 2nd P2)
    limit, calls = recording_limit()
    assert c._direction_limits(limit, SafetySpeed(-0.009, 0.009, (-1,)), 1, 0.4)[2:] == (False, [1], True)  # noqa: SLF001
    assert calls == [(1, 0.4), (-1, 0.009)]
    # a standstill asked forward: one direction, no unasked motion
    limit, calls = recording_limit(blocked=-1)
    assert c._direction_limits(limit, SafetySpeed(0.0, 0.0, (1, -1)), 1, 0.4) == (0.4, "ok", False, [1], False)  # noqa: SLF001
    assert calls == [(1, 0.4)]


def test_the_command_still_in_effect_is_motion_too():
    c = core(real=True)
    c.state["command_speed"] = 0.1
    limit, calls = recording_limit(blocked=1)
    # asked nothing, rolling back, the slew still forward: both driven, the forward one held
    out = c._direction_limits(limit, SafetySpeed(-0.02, 0.02, (-1,)), None, 0.02)  # noqa: SLF001
    assert out == (0.0, "occupied", True, [-1, 1], True)
    assert calls == [(-1, 0.1), (1, 0.1), (-1, 0.02), (1, 0.1)]


def test_a_failed_check_of_a_motion_direction_holds_even_when_it_is_also_driven():
    """Codex 4d-1 6th P1: rolling back with the command still rearward, asked forward -- the rearward motion's
    failed check is a hold although that direction is in the motion the request does not cover."""
    c = core(real=True)
    c.state["command_speed"] = -0.1
    limit, calls = recording_limit(blocked=-1)
    out = c._direction_limits(limit, SafetySpeed(-0.2, 0.2, (-1,)), 1, 0.4)  # noqa: SLF001
    assert out == (0.0, "occupied", True, [1], True) and calls == [(1, 0.4), (-1, 0.2)]


class _Tracker:
    remaining = 5.0

    def remaining_to_goal_m(self):
        return self.remaining

    def progress_m(self):
        return 1.0

    def leg_ahead(self):
        return np.zeros((1, 3)), 1


class _Layer:
    known_enabled = False
    known_config = {}

    def __init__(self, blocked_direction=None):
        self.blocked_direction = blocked_direction
        self.calls = []
        self.permission = SimpleNamespace(path_check=None,
                                          config=SimpleNamespace(stopping=SimpleNamespace(decel_mps2=2.0)))

    def limit(self, t, *, current_pose, curvature_inv_m, direction, loaded, cap_mps):
        self.calls.append(direction)
        return (0.0, "occupied") if direction == self.blocked_direction else (min(cap_mps, 0.5), "ok")


def permission_core(speed, *, phase="transport", blocked=None, pocket_check=None, act=True):
    layer = _Layer(blocked)
    hooks = SimpleNamespace(safety_speed=lambda: speed, truth_rear=lambda: np.zeros(3), require=lambda ok, why: None,
                            save_npz=lambda *a, **k: None, lift_position=lambda: 0.0)
    opts = SimpleNamespace(drive_geometry=None, command_accel_mps2=0.8, obstacle_act=act, real_vehicle=True,
                           pocket_check=pocket_check is not None, grid_planning=False, restart_confirm_scans=0)
    obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "ticks": {}, "min_allowed": {}, "slowed": {},
                "reasons": {}, "version": 0}
    slam = {"tracker": SimpleNamespace(applied=None, mode="tracking"), "stop_now": False, "docking": {}}
    c = MissionCore(MissionState(phase=phase, phase_started=0.0), state={}, obstacle=obstacle, slam=slam,
                    pocket={"check": pocket_check, "frames_read": 0}, paths={}, trackers={phase: _Tracker()},
                    geometry=SimpleNamespace(alignment_straight_m=0.5, delivery_straight_m=0.5),
                    settings={"steering_command_rate_rad_s": 1.0}, pallet_geometry=None, hooks=hooks, near=None,
                    options=opts)
    c.commands.work_elapsed = lambda since, t: t - since
    return c, layer


@pytest.mark.parametrize("act", [True, False])
def test_an_invalid_safety_speed_holds_the_wheels(act):
    c, _ = permission_core(SafetySpeed(0.0, 0.0, (1,), valid=False), act=act)
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.4, False, None)
    assert out.allowed == 0.0 and out.why == "safety_speed_invalid"
    assert out.requested_speed == 0.0 and out.obstacle_hold
    assert c.obstacle["permission_cap"] == (1.0, 0.0)
    w = c.permission_watch(1.0, (0.0, 0.0, 0.0), 0.0, False)
    assert w.allowed == 0.0 and w.why == "safety_speed_invalid" and w.obstacle_hold


def test_a_forward_request_does_not_hide_an_unsure_rearward_motion():
    c, layer = permission_core(SafetySpeed(0.0, 0.05, (1, -1)), blocked=-1)
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.4, False, None)
    assert layer.calls == [1, -1]
    assert out.allowed == 0.0 and out.why == "occupied" and out.requested_speed == 0.0


def test_the_pocket_check_does_not_answer_an_unsure_direction():
    asked = []
    check = SimpleNamespace(limit=lambda *a: asked.append(a[3]) or (0.5, "ok"))
    c, _ = permission_core(SafetySpeed(0.0, 0.05, (1, -1)), phase="insert", pocket_check=check)
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.0, False, None)
    assert out.allowed == 0.0 and out.why == "direction_unsure" and asked == []


def test_a_standstill_asked_forward_lets_the_pocket_check_answer():
    asked = []
    check = SimpleNamespace(limit=lambda *a: asked.append(a[3]) or (0.5, "ok"), summary=lambda: {})
    c, _ = permission_core(SafetySpeed(0.0, 0.0, (1, -1)), phase="insert", pocket_check=check)
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.3, False, None)
    assert out.why != "direction_unsure" and asked == [1]


@pytest.mark.parametrize("act", [True, False])
def test_a_failed_braking_check_holds_the_wheels_where_the_permission_does_not_act(act):
    c, layer = permission_core(SafetySpeed(-0.2, 0.2, (-1,)), blocked=-1, act=act)
    c.trackers["transport"].remaining = 0.2  # the delivery straight: the permission does not act there
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.4, False, None)
    assert not out.acting and layer.calls == [1, -1]
    assert out.allowed == 0.0 and out.requested_speed == 0.0 and out.obstacle_hold
    assert c.obstacle["permission_cap"] == (1.0, 0.0)


@pytest.mark.parametrize("act", [True, False])
def test_the_pocket_phase_stops_on_a_residual_motion_the_other_way(act):
    asked = []
    check = SimpleNamespace(limit=lambda *a: asked.append(a[3]) or (0.5, "ok"), summary=lambda: {})
    c, _ = permission_core(SafetySpeed(-0.02, 0.02, (-1,)), phase="insert", pocket_check=check, act=act)
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.4, False, None)
    assert out.allowed == 0.0 and out.why == "direction_unsure" and asked == []
    assert out.requested_speed == 0.0 and out.obstacle_hold
    assert c.obstacle["permission_cap"] == (1.0, 0.0)


@pytest.mark.parametrize("phase,blocked", [("insert", None), ("transport", 1)])
def test_a_zero_request_with_the_slew_still_forward_checks_forward(phase, blocked):
    """Codex 4d-1 5th P1: asked 0 while rolling back, the slew from +0.1 would still drive forward."""
    asked = []
    check = SimpleNamespace(limit=lambda *a: asked.append(a[3]) or (0.0, "occupied") if a[3] > 0 else (0.5, "ok"),
                            summary=lambda: {})
    c, layer = permission_core(SafetySpeed(-0.02, 0.02, (-1,)), phase=phase, blocked=blocked,
                               pocket_check=check if phase == "insert" else None)
    c.state["command_speed"] = 0.1
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.0, False, None)
    assert out.allowed == 0.0 and c.obstacle["permission_cap"] == (1.0, 0.0)
    # through the command assembly: the slew from +0.1 does not reach the wheels
    from forklift_core.control.path_tracking import AckermannGeometry

    c.options.drive_geometry = AckermannGeometry(wheelbase_m=1.0, track_m=0.6, wheel_radius_m=0.1,
                                                 max_steering_rad=0.8, max_wheel_rate_rad_s=50.0)
    c.hooks.steering_position = lambda: np.zeros(2)
    res = c.assemble_command(1.0, 0.01, out.requested_speed, 0.0, 0.0, np.zeros(2), False, out.obstacle_hold, False,
                             False, False)
    assert c.state["command_speed"] == 0.0 and res.drive.speed_mps == 0.0
    if phase == "insert":
        assert out.why == "direction_unsure" and asked == []
    else:
        assert layer.calls == [-1, 1, -1, 1] and out.why == "occupied"


@pytest.mark.parametrize("act", [True, False])
def test_rolling_back_on_a_rearward_command_into_a_block_holds_through_the_assembly(act):
    """Codex 4d-1 6th P1 end to end: the delivery straight (not acting), asked forward, rolling back with the command
    still rearward, blocked behind -- no wheel speed comes out."""
    from forklift_core.control.path_tracking import AckermannGeometry

    c, layer = permission_core(SafetySpeed(-0.2, 0.2, (-1,)), blocked=-1, act=act)
    c.trackers["transport"].remaining = 0.2
    c.state["command_speed"] = -0.1
    out = c.permission(1.0, (0.0, 0.0, 0.0), 0.0, 0.4, False, None)
    assert not out.acting and out.obstacle_hold and c.obstacle["permission_cap"] == (1.0, 0.0)
    c.options.drive_geometry = AckermannGeometry(wheelbase_m=1.0, track_m=0.6, wheel_radius_m=0.1,
                                                 max_steering_rad=0.8, max_wheel_rate_rad_s=50.0)
    c.hooks.steering_position = lambda: np.zeros(2)
    res = c.assemble_command(1.0, 0.01, out.requested_speed, 0.0, 0.0, np.zeros(2), False, out.obstacle_hold, False,
                             False, False)
    assert res.drive.speed_mps == 0.0  # (without the obstacle switch: no slew, no command_speed record)


# --- 4d-2a: evaluation values and the cusp arc on the real truck ---

def test_unjudgeable_values_are_recorded_as_such():
    assert MissionCore._pose_list(None) is None and MissionCore._pose_list((1, 2.5, 3)) == [1.0, 2.5, 3.0]
    assert MissionCore._npz_value(None).shape == (0,)
    np.testing.assert_array_equal(MissionCore._npz_value((1, 2, 3)), [1.0, 2.0, 3.0])
    assert MissionCore._npz_flag("truth_rear", None) == {"truth_rear_unjudgeable": True}
    assert MissionCore._npz_flag("truth_rear", (1, 2, 3)) == {}


def test_the_real_truck_reads_no_stage_and_no_control_truth_jump():
    read = []
    c = core(real=True, hooks=SimpleNamespace(stage=lambda: read.append("stage")))
    assert c._stage() is None and read == []  # noqa: SLF001
    c.slam = {"control": [[1.0, 0, 0, 0, 0, 0, 0]]}
    c.scene_correction = {"since_s": 0.0}
    assert c._scene_control_jump() is None  # noqa: SLF001  not a zero jump
    s = core(hooks=SimpleNamespace(stage=lambda: "STAGE"))
    assert s._stage() == "STAGE"  # noqa: SLF001


class _ArcLayer:
    def __init__(self, blocked_direction=None):
        self.snapshot = object()
        self.blocked_direction = blocked_direction
        self.arcs = []
        self.permission = SimpleNamespace(config=SimpleNamespace(sensor_timeout_s=0.5))
        self.last_scan_s = {"lidar": 10.0}

    def check_candidate(self, leg, **k):
        return SimpleNamespace(blocked=None, verified_m=1.0)

    def check_arc_candidate(self, *, direction, cap_mps, **k):
        self.arcs.append((direction, cap_mps))
        return SimpleNamespace(blocked="occupied" if direction == self.blocked_direction else None, verified_m=1.0)


def cusp_core(speed, *, real=True, blocked=None, signed=0.3):
    layer = _ArcLayer(blocked)
    hooks = SimpleNamespace(held_kappa=lambda: 0.0, loaded=lambda: False, safety_speed=lambda: speed,
                            signed_speed=lambda: signed)
    c, _ = permission_core(speed)
    c.hooks = hooks
    c.options.real_vehicle = real
    c.obstacle = {"layer": layer}
    return c, layer


def test_the_simulation_cusp_arc_is_unchanged():
    c, layer = cusp_core(SafetySpeed.exact(-0.3), real=False, signed=-0.3)
    owned, record = c.cusp_ownership(10.0, (0.0, 0.0, 0.0), 0.1)
    assert owned and layer.arcs == [(1, 0.3)]  # the leg's direction at max(|command|, |signed|)


def test_the_real_cusp_arc_checks_every_possible_direction_at_the_safety_bound():
    c, layer = cusp_core(SafetySpeed(-0.2, 0.2, (-1,)), blocked=-1, signed=0.0)
    owned, record = c.cusp_ownership(10.0, (0.0, 0.0, 0.0), 0.1)
    assert not owned and record["arc"] == "occupied"
    assert layer.arcs == [(1, 0.2), (-1, 0.2)]  # never the tracker's speed (0.0 here)


def test_the_real_cusp_arc_stops_on_an_invalid_speed():
    c, layer = cusp_core(SafetySpeed(0.0, 0.0, (1,), valid=False))
    owned, record = c.cusp_ownership(10.0, (0.0, 0.0, 0.0), 0.0)
    assert not owned and record["why"] == "safety_speed_invalid" and layer.arcs == []


def test_the_real_obstacle_dump_keeps_the_measured_speed_apart_from_truth():
    """4d-2a (Codex impl review P2-3): the dump's truth_speed is unjudgeable; the safety speed has its own fields."""
    saved = {}
    c, layer = permission_core(SafetySpeed(0.1, 0.12, (1,)), blocked=1)
    layer.snapshot = SimpleNamespace(state=np.zeros((2, 2)), free_stamp=np.zeros((2, 2)), origin_x_m=0.0,
                                     origin_y_m=0.0, resolution_m=0.05, stamp_s=9.9)
    c.hooks.save_npz = lambda name, **arrays: saved.setdefault(name, arrays)
    c.hooks.truth_rear = lambda: None  # the real truck's evaluation hook
    c.permission(10.0, (0.0, 0.0, 0.0), 0.0, 0.4, False, None)
    dump = saved["obstacle_dump_1.npz"]
    assert dump["truth_speed"].shape == (0,) and dump["truth_speed_unjudgeable"] is True
    assert dump["safety_speed_mps"] == 0.1 and dump["safety_speed_bound_mps"] == 0.12
    assert dump["truth_rear"].shape == (0,) and dump["truth_rear_unjudgeable"] is True
