"""The C2 injection's detour (docs/plans/2026-11-03-near-end-stand-c2-injection.md 1.2, 2, 4): the argument and its
contract, the final leg and sign rule (single-leg paths included), the decision once, the watch's events (the C2
install after the judgement is not a supersede) and the pallet-shift windows as maxima."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from test_riibotics_isaac_scenes import _runner_args  # noqa: E402

import near_end_detour as nd  # noqa: E402
from forklift_core.control import RearAxlePathTracker, TrackerConfig  # noqa: E402


def path(segments):
    """Samples along (length, gear, curvature) pieces from the origin, heading 0 (gear-signed motion)."""
    pose = np.zeros(3)
    poses, gears, curvatures = [pose.copy()], [segments[0][1]], [segments[0][2]]
    for length, gear, kappa in segments:
        for _ in range(int(round(length / 0.05))):
            ds = gear * 0.05
            pose = pose + [ds * math.cos(pose[2] + kappa * ds / 2), ds * math.sin(pose[2] + kappa * ds / 2), kappa * ds]
            poses.append(pose.copy()); gears.append(gear); curvatures.append(kappa)
    return np.array(poses), np.array(gears), np.array(curvatures)


def tracker_of(segments):
    poses, gears, curvatures = path(segments)
    return RearAxlePathTracker(poses, gears, curvatures, TrackerConfig(max_curvature_inv_m=0.36))


def at_remaining(tracker, remaining):
    tracker._leg = len(tracker._leg_ends) - 1
    tracker._progress = float(tracker._distance[-1] - remaining)
    tracker._segment = int(np.searchsorted(tracker._distance, tracker._progress, side="right") - 1)
    return tracker


def test_the_argument_is_off_by_default_and_needs_c2():
    assert _runner_args([]).inject_near_end_detour is False
    with pytest.raises(SystemExit):
        _runner_args(["--inject-near-end-detour"])
    args = _runner_args(["--inject-near-end-detour", "--near-end-stand", "shadow"])
    assert args.inject_near_end_detour is True and args.near_end_stand == "shadow"


def test_the_contract_and_the_mission_key():
    import near_tracking
    from forklift_core.mission.baseline import MISSION_KEYS

    assert "inject_near_end_detour" in near_tracking.RUN_SPECIFIC_ARGUMENTS
    assert "near_end_injections" in MISSION_KEYS["state"]
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    # read with getattr: a recorded run's arguments (the replay gate's) predate it
    assert 'if getattr(args, "inject_near_end_detour", False):' in source
    assert "args.inject_near_end_detour" not in source.replace("if args.inject_near_end_detour and", "")


def test_the_final_leg_of_a_single_leg_path_starts_at_zero():
    # Codex third review: 12 of the 40 near legs had no cusp, _leg_ends holding the goal alone
    single = tracker_of([(1.5, -1, 0.29)])
    assert single._leg_ends == [len(single._poses) - 1]
    start, length, gear = nd.final_leg(single)
    assert start == 0 and math.isclose(length, single._distance[-1]) and gear == -1
    cusp = tracker_of([(1.0, 1, 0.0), (1.2, -1, -0.29)])
    start, length, gear = nd.final_leg(cusp)
    assert start == cusp._leg_ends[-2] and math.isclose(length, cusp._distance[-1] - cusp._distance[start])


@pytest.mark.parametrize("kappa, s", [(0.29, 1.0), (-0.29, -1.0), (0.0, 1.0)])
def test_the_sign_rule(kappa, s):
    decision = nd.decide(tracker_of([(1.5, -1, kappa)]))
    assert decision["event"] == "armed" and decision["s"] == s
    assert decision["dx_left_m"] == -nd.D_M * s and decision["dpsi_rad"] == nd.PSI_RAD * s


def test_forward_and_short_final_legs_are_not_armed():
    assert nd.decide(tracker_of([(1.5, 1, 0.2)]))["reason"] == "forward_final_leg"
    assert nd.decide(tracker_of([(1.0, 1, 0.0), (0.5, -1, 0.2)]))["reason"] == "final_leg_short"


def stand(t, outcome):
    return {"kind": "near", "time_s": t, "outcome": outcome}


def test_the_watch_arms_once_restores_and_closes_at_the_judgement_not_at_c2s_install():
    tracker = at_remaining(tracker_of([(1.5, -1, 0.29)]), 0.79)
    detour, state = nd.NearEndDetour(), {"observe_end_stands": [{"kind": "o1", "time_s": 0.0}]}
    detour.tick(1.0, "observe", tracker, True, state, 0.0, tracker._poses[20])
    record = state["near_end_injections"]
    assert record["events"][0]["event"] == "armed" and tracker.input_offset_active()
    detour.tick(1.1, "observe", tracker, True, state, 0.0, tracker._poses[21])  # once: no second decision
    assert [e["event"] for e in record["events"]] == ["armed"]
    tracker.input_offset_events.append({"event": "cleared", "progress_m": 1.4, "remaining_m": 0.1})
    tracker._input_offset = None
    detour.tick(2.0, "observe", tracker, True, state, 0.0, tracker._poses[-1])
    state["observe_end_stands"].append(stand(5.0, "installed"))
    c2_tracker = tracker_of([(1.0, -1, 0.0)])  # C2's install replaces the tracker after its judgement
    detour.tick(5.1, "observe", c2_tracker, True, state, 0.0, tracker._poses[-1])
    events = [e["event"] for e in record["events"]]
    assert events == ["armed", "restored", "judged"]
    assert record["events"][-1]["restored_before"] is True and record["events"][-1]["outcome"] == "installed"
    assert record["windows"]["injection"] == [1.0, 5.0]


@pytest.mark.parametrize("restored, event", [(False, "superseded"), (True, "superseded_after_restore")])
def test_a_tracker_change_before_the_judgement_is_a_supersede(restored, event):
    tracker = at_remaining(tracker_of([(1.5, -1, 0.29)]), 0.79)
    detour, state = nd.NearEndDetour(), {}
    detour.tick(1.0, "observe", tracker, True, state, 0.0, tracker._poses[20])
    if restored:
        tracker.input_offset_events.append({"event": "cleared"})
        tracker._input_offset = None
        detour.tick(1.5, "observe", tracker, True, state, 0.0, tracker._poses[-1])
    detour.tick(2.0, "observe", tracker_of([(1.0, -1, 0.0)]), True, state, 0.0, tracker._poses[-1])
    assert state["near_end_injections"]["events"][-1]["event"] == event


def test_the_pallet_windows_keep_their_maxima():
    tracker = at_remaining(tracker_of([(1.5, -1, 0.29)]), 0.79)
    detour, state = nd.NearEndDetour(), {}
    for t, shift in ((1.0, 0.0), (1.2, 0.025), (1.4, 0.0)):  # 0 -> 25 -> 0 mm: the rise stays
        detour.tick(t, "observe", tracker, True, state, shift, tracker._poses[20])
    record = state["near_end_injections"]
    assert record["pallet_shift_max_m"]["injection"] == 0.025
    tracker._input_offset = None
    state["observe_end_stands"] = [stand(2.0, "installed")]
    for t, phase, shift in ((2.1, "observe", 0.004), (2.2, "observe", 0.019), (2.3, "approach", 0.002),
                            (2.4, "approach", 0.5)):
        detour.tick(t, phase, tracker_of([(1.0, -1, 0.0)]), True, state, shift, tracker._poses[-1])
    assert record["pallet_shift_max_m"]["install"] == 0.019  # closed at the approach transition
    assert record["windows"]["install"] == [2.0, 2.3]


def test_an_unarmed_run_still_keeps_the_install_window():
    detour, state = nd.NearEndDetour(), {"observe_end_stands": [stand(3.0, "installed")]}
    detour.decided = True  # decided not to arm (or never reached)
    detour.tick(3.1, "observe", None, False, state, 0.03, np.zeros(3))
    assert state["near_end_injections"]["pallet_shift_max_m"]["install"] == 0.03
