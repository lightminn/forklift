"""The low-clearance guard in the mission core (docs/plans/2026-10-25-low-clearance-guard.md D4, D5): the tick sets
this tick's cap from the tracker being followed, the command assembly applies it after the wheel-speed slew with its
sign kept (a step without the obstacle layer), and off nothing is read or set."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import AckermannGeometry, RearAxlePathTracker, TrackerConfig
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning import Bounds, Footprint, Rectangle
from forklift_core.planning.geometry import Pose2D
from forklift_core.planning.pallet_mission import AssetSpec, PalletSite, PlacedProp, TransportScenario

GEOMETRY = AckermannGeometry(wheelbase_m=1.0, track_m=0.6, wheel_radius_m=0.1, max_steering_rad=0.8,
                             max_wheel_rate_rad_s=50.0)
LOADED = Footprint(1.56, 0.17, 0.4)
UNLOADED = Footprint(1.29, 0.17, 0.36)


def scenario(*rects):
    props = tuple(PlacedProp(AssetSpec("riibotics:block", r.length_m, r.width_m, 0.144), r) for r in rects)
    return TransportScenario(0, Pose2D(0.0, 0.0, 0.0), PalletSite(9.0, 0.0, 0.0), PalletSite(-9.0, 0.0, 0.0), props,
                             Bounds(-20.0, 20.0, -20.0, 20.0))


def tracker(x0=-2.0, x1=4.0):
    xs = np.linspace(x0, x1, 61)
    poses = np.column_stack((xs, np.zeros_like(xs), np.zeros_like(xs)))
    return RearAxlePathTracker(poses, np.ones(len(xs)), np.zeros(len(xs)),
                               TrackerConfig(position_tolerance_m=0.008, overshoot_tolerance_m=0.03))


def make(*, guard="on", obstacle=None, act=True, block_gap=0.08, phase="transport", trackers=None):
    block = Rectangle(1.0, 0.4 + block_gap + 0.5, 3.0, 1.0, 0.0)  # beside the loaded outline's left side
    state = {"transitions": []}
    options = SimpleNamespace(drive_geometry=GEOMETRY, command_accel_mps2=0.8, obstacle_act=act,
                              scenario=scenario(block), low_clearance_margin_m=0.05, low_clearance_deviation_m=0.02,
                              low_clearance_unloaded_shape=UNLOADED)
    if guard is not None:
        options.low_clearance_guard = guard
    geometry = SimpleNamespace(loaded_footprint=LOADED, delivery_straight_m=1.5, lidar_plane_m=1.05)
    hooks = SimpleNamespace(require=lambda ok, reason: None, steering_position=lambda: np.zeros(2))
    core = MissionCore(MissionState(phase=phase, phase_started=0.0), state=state, obstacle=obstacle, slam=None,
                       pocket={"check": None}, paths={}, trackers=trackers if trackers is not None else {phase: tracker()},
                       geometry=geometry, settings={"steering_command_rate_rad_s": 1.2}, pallet_geometry=None,
                       hooks=hooks, near=None, options=options)
    return core, state


def assemble(core, *, speed, t=10.0):
    return core.assemble_command(t, 0.01, speed, 0.0, 0.0, np.zeros(2), False, False, False, False, False)


def slewing_obstacle():
    layer = SimpleNamespace(permission=SimpleNamespace(config=SimpleNamespace(stopping=SimpleNamespace(decel_mps2=2.0))))
    return {"layer": layer, "permission_cap": None}


def test_the_cap_applies_after_the_slew_and_keeps_the_sign():
    core, state = make(obstacle=slewing_obstacle())
    state["command_speed"] = 0.30
    core._low_clearance_cap = (10.0, 0.0)
    assemble(core, speed=0.0)
    assert state["command_speed"] == 0.0  # the slew alone would leave 0.30 - 2.0 * 0.01
    state["command_speed"] = -0.30
    core._low_clearance_cap = (10.0, 0.1)
    assemble(core, speed=-0.3)
    assert state["command_speed"] == pytest.approx(-0.1)
    core._low_clearance_cap = (9.99, 0.0)  # another tick's cap does not apply
    state["command_speed"] = 0.2
    assemble(core, speed=0.2)
    assert state["command_speed"] == pytest.approx(0.2)


def test_without_the_obstacle_layer_the_cap_is_a_step():
    core, state = make(act=False)
    core._low_clearance_cap = (10.0, 0.0)
    out = assemble(core, speed=0.3)
    assert out.drive.speed_mps == 0.0


@pytest.mark.parametrize("guard", [None, "off"])
def test_off_nothing_is_read_or_set(guard):
    core, state = make(guard=guard, obstacle=slewing_obstacle())
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.36, 0.3, True)
    assert "low_clearance" not in state and getattr(core, "_low_clearance_cap", None) is None
    core._low_clearance_cap = (10.0, 0.0)  # even a stale cap is not read off
    state["command_speed"] = 0.2
    assemble(core, speed=0.2)
    assert state["command_speed"] == pytest.approx(0.2)


def test_the_tick_caps_a_drift_toward_a_low_block_and_records_it():
    core, state = make()
    tr = core.trackers["transport"]
    tr.update(np.array([0.0, 0.0, 0.0]), 0.3, 0.01)  # progress 2.0 m along the leg
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.0, 0.3, True)
    assert core._low_clearance_cap is None  # on the path: nothing capped
    core.low_clearance_tick(10.01, (0.0, 0.0, 0.0), 0.36, 0.3, True)  # the wheels turned toward the block
    t_, cap = core._low_clearance_cap
    assert t_ == 10.01 and cap < 0.3
    record = state["low_clearance"]
    assert record["capped_ticks"]["transport"] >= 1 and record["legs"][0]["phase"] == "transport"
    assert record["min_clearance_m"]["transport"] == pytest.approx(0.08, abs=1e-6)
    json.dumps(state)  # the run record stays plain


def test_a_new_tracker_gets_its_own_planned_clearance():
    core, state = make()
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.0, 0.3, True)
    core.trackers["transport"] = tracker(-3.0, 3.0)
    core.low_clearance_tick(10.01, (0.0, 0.0, 0.0), 0.0, 0.3, True)
    assert len(state["low_clearance"]["legs"]) == 2


def test_phases_that_do_not_move_along_a_path_are_not_watched():
    core, state = make(phase="lift", trackers={"lift": tracker()})
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.36, 0.3, True)
    assert core._low_clearance_cap is None and "low_clearance" not in state
