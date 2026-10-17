"""M-a1 slice 1: the phase transition moved from the runner (behaviour, state, call order, effects up to a
refusal -- the plan's M-a1 criterion)."""

import math
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState


class Log(list):
    def hook(self, name, result=None):
        def call(*args, **kwargs):
            self.append((name, args, kwargs))
            return result
        return call


def make(phase="observe", *, slam_mode="tracking", with_slam=True, check=None, layer=None, known=False):
    log = Log()

    def require(ok, reason):
        log.append(("require", (bool(ok), reason), {}))
        if not ok:
            raise RuntimeError(reason)

    hooks = SimpleNamespace(
        echo=log.hook("echo"), record_json=lambda v: f"{v['from']}->{v['to']}@{v['time_s']}", snapshot=log.hook("snapshot"),
        require=require, control_correction=log.hook("control_correction", (0.0, 0.0, 0.0)),
        known_pallet=log.hook("known_pallet"), slam_odom_base=log.hook("slam_odom_base", (1.0, 2.0, 0.5)),
        slam_error=log.hook("slam_error", (0.01, 0.0, 0.0)), fork_blades=log.hook("fork_blades", []),
    )
    tracker = SimpleNamespace(mode=slam_mode, hold=log.hook("tracker.hold"))
    slam = {"tracker": tracker, "holds": [], "pending_release": None} if with_slam else None
    if layer is None:
        layer = SimpleNamespace(known_enabled=known, memory=None, exempt="E", depth_support="D", noise_cut_m=0.01)
    obstacle = {"layer": layer}
    pocket = {"check": check, "frames_read": 7}
    ms = MissionState(phase=phase, phase_started=1.0)
    state = {"transitions": []}
    geometry = SimpleNamespace(pallet_depth_m=0.6, pallet_width_m=0.8, axle_to_fork_tip_m=1.29, inserted_offset_m=0.3)
    core = MissionCore(ms, state=state, obstacle=obstacle, slam=slam, pocket=pocket, paths={}, trackers={},
                       geometry=geometry, settings={"withdraw_speed_mps": 0.12}, pallet_geometry=None, hooks=hooks)
    # Moved into MissionCore (M-a1 slice 6e): this test's stand-ins go on the instance.
    for name_ in ['control_correction', 'known_pallet', 'return_turnaround_kwargs', 'realign_active', 'realign_install', 'start_near_realign', 'apply_tracker_profile', 'record_tracker_configs', 'build_near_tracker']:
        if hasattr(hooks, name_):
            setattr(core, name_, getattr(hooks, name_))
    return core, ms, state, obstacle, pocket, slam, log


def test_a_transition_records_echoes_snapshots_then_moves_the_phase_and_holds_slam():
    core, ms, state, _, _, slam, log = make("observe")
    core.transition("approach", 12.5)
    assert state["transitions"] == [{"from": "observe", "to": "approach", "time_s": 12.5}]
    assert (ms.phase, ms.phase_started, state["phase"]) == ("approach", 12.5, "approach")
    assert [name for name, *_ in log] == ["echo", "snapshot", "slam_odom_base", "tracker.hold", "slam_error"]
    assert log[0][1] == ("TRANSITION observe->approach@12.5",)
    assert log[1][1] == ("observe",)  # the snapshot is labelled with the phase being left
    assert log[3][2] == {"odom_from_base": (1.0, 2.0, 0.5)}
    assert slam["holds"] == [{"phase": "approach", "time_s": 12.5, "event": "hold", "error": (0.01, 0.0, 0.0)}]


def test_no_hold_unless_tracking_and_a_release_is_scheduled_for_transport_and_settle():
    core, ms, _, _, _, slam, log = make("approach", slam_mode="holding")
    core.transition("insert", 20.0)
    assert slam["holds"] == [] and "tracker.hold" not in [n for n, *_ in log]
    for phase in ("transport", "settle"):
        core.transition(phase, 30.0)
        assert (slam["pending_release"], slam["release_wait_from"]) == (phase, 30.0)
    core, ms, state, *_ = make("observe", with_slam=False)
    core.transition("approach", 1.0)  # no SLAM: the phase moves and nothing else
    assert ms.phase == "approach" and state["phase"] == "approach"


def test_the_pocket_check_waits_for_a_still_frame_into_insert_and_is_closed_after_it():
    check = SimpleNamespace(summary=lambda: {"frames": 3})
    core, _, state, obstacle, pocket, _, _ = make("approach", check=check)
    core.transition("insert", 5.0)
    assert pocket["await_still_frame"] is True
    core.transition("lift", 9.0)
    assert pocket["check"] is None
    assert (obstacle["layer"].exempt, obstacle["layer"].depth_support) == (None, None)
    assert state["pocket_check"] == {"frames": 3, "frames_read": 7}


def test_extract_retracts_the_pickup_from_the_memory_by_the_estimate():
    calls = []
    memory = SimpleNamespace(retract_endpoints=lambda *a: calls.append(a) or 5)
    layer = SimpleNamespace(known_enabled=False, memory=memory, exempt=None, depth_support=None, noise_cut_m=0.01)
    core, _, _, obstacle, _, _, _ = make("lift", layer=layer)
    obstacle["pickup_estimate"] = SimpleNamespace(x_m=3.0, y_m=1.0, yaw_rad=0.2)
    core.transition("extract", 40.0)
    assert calls == [(3.0, 1.0, 0.6, 0.8, 0.2, 0.05 + 0.01)]
    assert obstacle["memory_events"] == [{"time_s": 40.0, "event": "retract_pickup", "cells": 5, "grow_m": 0.05 + 0.01}]


def test_a_refused_certified_withdrawal_keeps_what_was_done_before_the_refusal():
    layer = SimpleNamespace(known_enabled=True, memory=None, exempt=None, depth_support=None, noise_cut_m=0.01,
                            known_config={"withdraw_mode": "certified"}, known=[])
    core, ms, state, obstacle, _, slam, log = make("lower", layer=layer)
    obstacle["control_rear"] = (1.0, 0.0, 0.0)  # no pallet_rel: nothing to certify against
    with pytest.raises(RuntimeError, match="withdraw_not_certified:no_pallet_estimate"):
        core.transition("withdraw", 50.0)
    # Recorded before the refusal, as in the runner: the transition entry; the phase has not moved.
    assert state["transitions"] == [{"from": "lower", "to": "withdraw", "time_s": 50.0}]
    assert ms.phase == "lower" and "phase" not in state
    assert [n for n, *_ in log] == ["require"] and slam["holds"] == []


def test_lift_keeps_the_pallet_relative_to_the_truck_and_clears_the_known_pallet():
    layer = SimpleNamespace(known_enabled=True, memory=None, exempt=None, depth_support=None, noise_cut_m=0.01,
                            known_config={}, known=["pickup"])
    core, _, state, obstacle, _, _, _ = make("insert", layer=layer)
    obstacle["control_rear"] = (1.0, 0.0, 0.0)
    obstacle["pickup_estimate"] = SimpleNamespace(x_m=2.0, y_m=0.0, yaw_rad=0.0)
    core.transition("lift", 60.0)
    assert layer.known == []
    assert tuple(round(v, 9) for v in obstacle["pallet_rel"]) == (1.0, 0.0, 0.0)
    assert state["known_pallets"] == [{"event": "lifted", "time_s": 60.0}]


@dataclass(frozen=True)
class Leg:
    poses: np.ndarray
    directions: np.ndarray
    curvatures_inv_m: np.ndarray
    length_m: float


def withdraw_setup(*, mode="certified", certified=True, known_config=None):
    certify_calls = []

    def certify(poses, shape, until, gaps):
        certify_calls.append((np.array(poses), shape, until, gaps))
        return certified

    layer = SimpleNamespace(
        known_enabled=True, memory=None, exempt=None, depth_support=None, noise_cut_m=0.01, known=["pickup"],
        known_config={"withdraw_mode": mode, "b_w_m": 0.0129, "e_w_m": 0.0084, **(known_config or {})},
        permission=SimpleNamespace(config=SimpleNamespace(stopping=SimpleNamespace(distance_m=lambda v: 4 * v))),
        parts_shape="SHAPE", certify_withdrawal=certify, end_withdrawal=None,
    )
    core, ms, state, obstacle, _, slam, log = make("lower", layer=layer)
    core.hooks.fork_blades = log.hook("fork_blades", [(0.0, 1.0, 0.10, 0.20), (0.0, 1.0, -0.20, -0.10)])
    core.pallet_geometry = SimpleNamespace(block_widths_m=(0.1, 0.2), overall_width_m=0.8)
    planned = Leg(np.array([[1.0, 0.0, 0.0], [0.5, 0.0, 0.0]]), np.array([-1, -1], dtype=np.int8), np.zeros(2), 0.5)
    core.paths["withdraw"] = planned
    old_tracker = SimpleNamespace(config=TrackerConfig())
    core.trackers["withdraw"] = old_tracker
    obstacle["control_rear"] = (2.0, 1.0, math.pi / 2)
    obstacle["pallet_rel"] = (1.5, 0.01, 0.02)
    obstacle["last_stamp"] = 49.5
    return core, ms, state, obstacle, slam, log, layer, certify_calls, old_tracker


def test_a_certified_withdrawal_fixes_the_delivered_pallet_and_backs_out_along_the_truck_heading():
    core, ms, state, obstacle, _, log, layer, certify_calls, old = withdraw_setup()
    core.transition("withdraw", 50.0)
    names = [n for n, *_ in log]
    assert names[:4] == ["require", "known_pallet", "fork_blades", "require"]
    rect, r_fix, event = log[1][1]
    assert np.allclose(rect, (2.0 - 0.01, 1.0 + 1.5, math.pi / 2 + 0.02)) and (r_fix, event) == (0.0129, "delivered")
    leg = core.paths["withdraw"]  # a straight from where the truck stands, back along its heading
    assert len(leg.poses) == 26 and leg.length_m == 0.5
    assert np.allclose(leg.poses[0], (2.0, 1.0, math.pi / 2)) and np.allclose(leg.poses[-1], (2.0, 0.5, math.pi / 2))
    assert (leg.directions == -1).all() and not leg.curvatures_inv_m.any()
    tracker = core.trackers["withdraw"]
    assert tracker is not old and tracker.config is old.config
    assert obstacle["withdraw_line"] == ((2.0, 1.0, math.pi / 2), 0.5)
    depth = 1.29 + 0.6 / 2 - 0.3
    until = depth + 0.10 + 4 * 0.12
    (poses, shape, until_arg, gaps), = certify_calls
    assert np.array_equal(poses, leg.poses) and shape == "SHAPE" and until_arg == pytest.approx(until)
    assert len(gaps) == 4
    assert state["withdraw_certificate"] == {"time_s": 50.0, "gaps_m": [float(g) for g in gaps], "b_w_m": 0.0129,
                                             "e_w_m": 0.0084, "corridor_until_m": until_arg, "certified": True}
    assert ms.phase == "withdraw"


def test_an_uncertified_corridor_refuses_after_the_straight_and_the_certificate_are_in_place():
    core, ms, state, obstacle, slam, log, layer, _, old = withdraw_setup(certified=False)
    with pytest.raises(RuntimeError, match="withdraw_not_certified:{'time_s': 50.0"):
        core.transition("withdraw", 50.0)
    # Everything the runner did before the refusal is kept: the delivered pallet, the straight, the certificate.
    assert [n for n, *_ in log] == ["require", "known_pallet", "fork_blades", "require"]
    assert core.trackers["withdraw"] is not old and obstacle["withdraw_line"][1] == 0.5
    assert state["withdraw_certificate"]["certified"] is False
    assert ms.phase == "lower" and "phase" not in state and slam["holds"] == []


def test_the_interim_uncertified_withdrawal_leaves_the_delivered_pallet_pending_and_is_refixed_where_it_stops():
    core, ms, state, obstacle, _, log, layer, certify_calls, old = withdraw_setup(mode="uncertified",
                                                                                 known_config={"e_w_full_m": 0.02})
    core.control_correction = log.hook("control_correction", (0.1, 0.0, 0.0))
    core.transition("withdraw", 50.0)
    assert certify_calls == [] and core.trackers["withdraw"] is old  # the planned straight, no permission
    pose, corr, released = obstacle["delivered_pending"]
    assert np.allclose(pose, (2.0 - 0.01, 2.5, math.pi / 2 + 0.02)) and corr == (0.1, 0.0, 0.0) and released == 49.5
    assert state["known_pallets"][-1] == {"event": "withdraw_uncertified", "time_s": 50.0}
    ends = []
    layer.end_withdrawal = lambda now_s: ends.append(now_s)
    obstacle["last_stamp"] = 55.0
    core.transition("settle", 56.0)
    assert ends == [55.0] and "delivered_pending" not in obstacle
    (known,) = layer.known  # re-fixed at the stop: release error + measured four-corner drift
    assert known.fix_stamp_s == 55.0 and known.r_fix_m == pytest.approx(0.0129 + 0.02)
    assert np.allclose(known.rect, (2.0 - 0.01, 2.5, 0.6, 0.8, math.pi / 2 + 0.02))
    assert known.frame_correction == (0.1, 0.0, 0.0) and known.rho_m is None
    assert state["known_pallets"][-1] == {"event": "delivered_after_withdraw", "time_s": 55.0,
                                          "rect": [known.rect[0], known.rect[1], known.rect[4]],
                                          "r_fix_m": known.r_fix_m}


def test_without_the_full_drift_bound_the_pending_pallet_keeps_ageing_from_its_release():
    core, _, state, obstacle, _, _, layer, _, _ = withdraw_setup(mode="uncertified")
    core.transition("withdraw", 50.0)
    layer.end_withdrawal = lambda now_s: None
    obstacle["last_stamp"] = 55.0
    core.transition("settle", 56.0)
    (known,) = layer.known
    assert known.fix_stamp_s == 49.5 and known.r_fix_m == 0.0129  # the release error alone
    core, _, _, obstacle, _, _, layer, _, _ = withdraw_setup()
    core.transition("withdraw", 50.0)  # certified: nothing pending, the known pallets stay as fixed
    layer.end_withdrawal = lambda now_s: None
    before = list(layer.known)
    core.transition("settle", 56.0)
    assert layer.known == before and "delivered_pending" not in obstacle
