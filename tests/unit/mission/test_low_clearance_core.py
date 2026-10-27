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


def scenario(*rects, uris=None):
    uris = uris or ["riibotics:block"] * len(rects)
    props = tuple(PlacedProp(AssetSpec(u, r.length_m, r.width_m, 0.144), r) for r, u in zip(rects, uris))
    return TransportScenario(0, Pose2D(0.0, 0.0, 0.0), PalletSite(9.0, 0.0, 0.0), PalletSite(-9.0, 0.0, 0.0), props,
                             Bounds(-20.0, 20.0, -20.0, 20.0))


def tracker(x0=-2.0, x1=4.0, direction=1):
    xs = np.linspace(x0, x1, 61)
    poses = np.column_stack((xs, np.zeros_like(xs), np.zeros_like(xs)))
    return RearAxlePathTracker(poses, np.full(len(xs), float(direction)), np.zeros(len(xs)),
                               TrackerConfig(position_tolerance_m=0.008, overshoot_tolerance_m=0.03))


def make(*, guard="on", obstacle=None, act=True, block_gap=0.08, phase="transport", trackers=None, rects=None,
         uris=None, truth_speed=0.0, paths=None):
    block = Rectangle(1.0, 0.4 + block_gap + 0.5, 3.0, 1.0, 0.0)  # beside the loaded outline's left side
    rects = [block] if rects is None else rects
    state = {"transitions": []}
    options = SimpleNamespace(drive_geometry=GEOMETRY, command_accel_mps2=0.8, obstacle_act=act,
                              scenario=scenario(*rects, uris=uris), low_clearance_margin_m=0.05, low_clearance_deviation_m=0.02,
                              low_clearance_unloaded_shape=UNLOADED)
    if guard is not None:
        options.low_clearance_guard = guard
    geometry = SimpleNamespace(loaded_footprint=LOADED, delivery_straight_m=1.5, lidar_plane_m=1.05)
    hooks = SimpleNamespace(require=lambda ok, reason: None, steering_position=lambda: np.zeros(2),
                            truth_speed=lambda: truth_speed)
    core = MissionCore(MissionState(phase=phase, phase_started=0.0), state=state, obstacle=obstacle, slam=None,
                       pocket={"check": None}, paths=paths or {}, trackers=trackers if trackers is not None else {phase: tracker()},
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


AHEAD = Rectangle(1.56 + 0.06 + 0.5, 0.0, 1.0, 1.0, 0.0)  # 6 cm ahead of the loaded front at x = 0
BEHIND = Rectangle(-0.17 - 0.06 - 0.5, 0.0, 1.0, 1.0, 0.0)  # 6 cm behind its rear


def test_the_arc_follows_the_motion_not_the_next_leg():
    # Codex round 4: rolling forward at +0.30 with a reverse leg asked next and nothing requested: the forward arc
    core, state = make(rects=[AHEAD], trackers={"transport": tracker(4.0, -2.0, direction=-1)}, truth_speed=0.3)
    state["command_speed"] = 0.30
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.0, 0.0, True)
    assert core._low_clearance_cap is not None and core._low_clearance_cap[1] < 0.3
    # moving backwards at 0.6 measured with nothing commanded or requested: the reverse arc at 0.6
    core, state = make(rects=[BEHIND], trackers={"transport": tracker(4.0, -2.0, direction=-1)}, truth_speed=-0.6)
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.0, 0.0, True)
    assert core._low_clearance_cap is not None and core._low_clearance_cap[1] < 0.6


def test_a_prefix_tracker_short_of_the_delivery_straight_keeps_the_search_props():
    # Codex round 4: arm_docking's prefix ends before the straight; its end is not the slot stage's
    estimate = Rectangle(1.0, 0.4 - 0.02 + 0.5, 3.0, 1.0, 0.0)  # a stale search-stage estimate over the outline
    full = SimpleNamespace(poses=np.column_stack((np.linspace(-2.0, 4.0, 61), np.zeros(61), np.zeros(61))))
    core, state = make(rects=[estimate], uris=["session:p@search"], trackers={"transport": tracker(-2.0, 2.5)},
                       paths={"transport": full})
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.0, 0.3, True)
    leg_ = state["low_clearance"]["legs"][0]
    assert leg_["slot_from_m"] >= leg_["length_m"] - 1e-9  # the straight starts where the prefix ends
    assert core._low_clearance_cap == (10.0, 0.0)
    core, state = make(rects=[estimate], uris=["session:p@search"], trackers={"transport": tracker(-2.0, 4.0)},
                       paths={"transport": full})
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.0, 0.3, True)
    assert state["low_clearance"]["legs"][0]["slot_from_m"] == pytest.approx(4.5)


def test_a_zero_cap_holds_the_steering():
    core, state = make(obstacle=slewing_obstacle())
    state["command_speed"] = 0.0
    core._low_clearance_cap = (10.0, 0.0)
    steering = np.zeros(2)
    out = core.assemble_command(10.0, 0.01, 0.2, 0.2, 0.0, steering, False, False, False, False, False)
    assert out.drive.speed_mps == 0.0 and np.all(out.steering_command == 0.0)


def test_without_low_props_the_record_stays_strict_json():
    core, state = make(rects=[])
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.0, 0.3, True)
    json.dumps(state, allow_nan=False)


def test_a_backoff_or_escape_keeps_the_search_props_although_it_replaced_the_planned_path():
    # Codex round 5: escapes and backoffs replace paths["transport"] with themselves, so the end comparison holds
    estimate = Rectangle(1.0, 0.4 - 0.02 + 0.5, 3.0, 1.0, 0.0)
    escape = tracker(0.3, 0.0, direction=-1)
    path = SimpleNamespace(poses=escape.path_arrays()[0])
    from forklift_core.control.drive_permission import StoppingModel

    stopping = StoppingModel(0.15, 1.5, 0.05)
    obstacle = {"layer": SimpleNamespace(permission=SimpleNamespace(config=SimpleNamespace(stopping=stopping))),
                "permission_cap": None, "backoff": {"phase": "transport"}}
    core, state = make(rects=[estimate], uris=["session:p@search"], trackers={"transport": escape},
                       paths={"transport": path}, obstacle=obstacle)
    core.low_clearance_tick(10.0, (0.3, 0.0, 0.0), 0.0, -0.15, True)
    assert state["low_clearance"]["legs"][0]["slot_from_m"] is None and core._low_clearance_cap == (10.0, 0.0)


def test_a_docking_reapproach_keeps_the_search_props():
    # Codex round 6: install_reapproach replaces paths["transport"] too; it was planned on the search props
    estimate = Rectangle(1.0, 0.4 - 0.02 + 0.5, 3.0, 1.0, 0.0)
    again = tracker(-1.0, 0.0)
    core, state = make(rects=[estimate], uris=["session:p@search"], trackers={"transport": again},
                       paths={"transport": SimpleNamespace(poses=again.path_arrays()[0])})
    core.slam = {"docking": {"reapproach": True}}
    core.low_clearance_tick(10.0, (-0.5, 0.0, 0.0), 0.0, 0.15, True)
    assert state["low_clearance"]["legs"][0]["slot_from_m"] is None and core._low_clearance_cap == (10.0, 0.0)


def test_the_second_round_prefix_of_the_delivery_straight_is_the_slot_stage():
    # Codex round 7: after the first match, D7 drives the start of the delivery straight again; it lies on the straight
    estimate = Rectangle(-0.5, 0.4 - 0.02 + 0.5, 3.0, 1.0, 0.0)  # a stale search estimate over the straight
    slot_copy = Rectangle(-0.5, 0.4 + 0.10 + 0.5, 3.0, 1.0, 0.0)
    straight_ = tracker(-1.5, 0.0)  # the 1.5 m delivery straight
    prefix = tracker(-1.5, -0.6)  # its start, 0.6 m kept
    core, state = make(rects=[estimate, slot_copy], uris=["session:p@search", "session:p@slot"],
                       trackers={"transport": prefix},
                       paths={"transport": SimpleNamespace(poses=straight_.path_arrays()[0])})
    core.low_clearance_tick(10.0, (-1.2, 0.0, 0.0), 0.0, 0.15, True)
    assert state["low_clearance"]["legs"][0]["slot_from_m"] == pytest.approx(0.0)
    assert core._low_clearance_cap is None


def test_the_d7_delivery_straight_is_the_slot_stage_whatever_its_length():
    # Codex round 8: a corrected goal made the straight 1.65 m; all of it stays the slot stage
    estimate = Rectangle(-0.5, 0.4 - 0.02 + 0.5, 3.0, 1.0, 0.0)
    slot_copy = Rectangle(-0.5, 0.4 + 0.11 + 0.5, 3.0, 1.0, 0.0)
    straight_ = tracker(-1.65, 0.0)
    path = SimpleNamespace(poses=straight_.path_arrays()[0])
    core, state = make(rects=[estimate, slot_copy], uris=["session:p@search", "session:p@slot"],
                       trackers={"transport": straight_}, paths={"transport": path})
    core._delivery_straight_path = path
    core.low_clearance_tick(10.0, (-1.6, 0.0, 0.0), 0.0, 0.15, True)
    assert state["low_clearance"]["legs"][0]["slot_from_m"] == 0.0 and core._low_clearance_cap is None


def _return_start_backoff_core(tracker_):
    """A return whose start lies inside a neighbour's estimate (plan 2026-10-27-return-start-withdraw-extension R5, the
    Codex round-3 counterexample's shape): the estimate ahead overlaps the unloaded front, its slot copy is far away;
    paths["return_home"] is a 2 m reverse whose first 0.30 m is the start backoff."""
    from forklift_core.control.drive_permission import StoppingModel

    estimate = Rectangle(1.29 + 0.3, 0.0, 0.8, 1.0, 0.0)  # front edge 1.19: 0.10 into the outline
    slot = Rectangle(1.29 + 0.3, 5.0, 0.8, 1.0, 0.0)
    planned = tracker(0.0, -2.0, direction=-1)
    path = SimpleNamespace(poses=planned.path_arrays()[0], start_backoff={"status": "prefix", "length_m": 0.30})
    stopping = StoppingModel(0.15, 1.5, 0.05)
    obstacle = {"layer": SimpleNamespace(permission=SimpleNamespace(config=SimpleNamespace(stopping=stopping))),
                "permission_cap": None}
    return make(rects=[estimate, slot], uris=["session:pallet:P-5@search", "session:pallet:P-5@slot"],
                phase="return_home", trackers={"return_home": tracker_(planned)},
                paths={"return_home": path}, obstacle=obstacle)


def test_a_return_start_backoff_is_the_slot_stage_on_its_own_tracker():
    core, state = _return_start_backoff_core(lambda planned: planned)
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.0, -0.15, False)
    assert state["low_clearance"]["legs"][0]["slot_to_m"] == 0.30
    assert core._low_clearance_cap is None or core._low_clearance_cap[1] != 0.0  # backing away is not stopped


def test_another_tracker_on_the_same_return_keeps_the_search_props():
    # Codex round 4 P3: an escape keeps paths["return_home"] and installs its own tracker
    core, state = _return_start_backoff_core(lambda planned: tracker(0.0, -1.0, direction=-1))
    core.low_clearance_tick(10.0, (0.0, 0.0, 0.0), 0.0, -0.15, False)
    assert "slot_to_m" not in state["low_clearance"]["legs"][0] and core._low_clearance_cap == (10.0, 0.0)
