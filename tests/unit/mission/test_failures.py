"""The live failure classes (persistent console plan D6): every require reason the runner and the core can raise is
classified; unknown reasons end the session."""

import ast
from pathlib import Path

import pytest

from forklift_core.mission.failures import DYNAMIC_SITES, classify

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize("reason, cls", [
    ("Tracking failed in observe", "mission"),
    ("approach:no_path", "mission"),
    ("near_field_mismatch", "mission"),
    ("near_field_contract: x", "session"),
    ("near_field_frame_released", "session"),
    ("Excessive body tilt in lift: 0.2", "scene"),
    ("slam_link_failed: reply late", "session"),
    ("something nobody wrote down", "session"),
    ("", "session"),
])
def test_reasons_are_classified_by_their_longest_prefix(reason, cls):
    assert classify(reason) == cls


def _require_reasons(path):
    tree = ast.parse(path.read_text())
    out = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        name = func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", None)
        index = {"require": 1, "scene_require": 2}.get(name)  # scene_require(t, ok, reason): the truth checks (D6)
        if index is None or len(node.args) <= index:
            continue
        arg = node.args[index]
        if isinstance(arg, ast.Constant):
            out.append((node.lineno, arg.value, None))
        elif isinstance(arg, ast.JoinedStr):
            first = arg.values[0]
            out.append((node.lineno, first.value if isinstance(first, ast.Constant) else "", "f"))
        else:
            out.append((node.lineno, None, ast.unparse(arg)))
    return out


@pytest.mark.parametrize("rel", ["sim/isaac/run_transport.py", "src/forklift_core/mission/core.py"])
def test_every_require_reason_has_a_class_of_its_own(rel):
    for line, text, kind in _require_reasons(ROOT / rel):
        if text is None:  # a reason made at run time: its site is listed with the class its values take
            assert (rel, kind) in DYNAMIC_SITES, f"{rel}:{line} dynamic reason {kind!r} not listed"
            continue
        if kind == "f" and text == "":
            continue  # an f-string with no fixed head: its sites are checked through DYNAMIC_SITES below
        assert classify(text, strict=True) is not None, f"{rel}:{line} {text!r} unclassified"


# Every reason the two files raise today, by its text (a constant, or an f-string's fixed head): a reason added later
# fails this test until it is read and classified here -- a broad prefix such as near_field_ must not take it
# silently (Codex D6 round 1).
KNOWN = {
    "sim/isaac/run_transport.py": {
        "Cannot open base scene", "Invalid pallet spawn after settling", "Truck spawn displaced",
        "recognition label target ",  # main's far recognition (merged 2026-10-19): a setup check
        "Tracking failed in ",  # persistent console plan section 4 (U10b): --live-test-fail-at, a mission failure
        "This adapter requires 120Hz physics (240 Hz only for a --steering-probe convergence check)",
        "Settings must be positive finite values", "Camera render product prim is missing",
        "Perception camera intrinsics mismatch (G1a)", "base scene built from another forklift URDF",
        "pocket_check_without_capture", "pocket_check_refused:", "Mission planning failed: ",
        "Mission exceeded simulation time budget", "--odometry-wheel-radius-m ",
        "Perception camera intrinsics check failed: ", "pocket camera intrinsics differ from the perception camera's",
        "live video camera intrinsics differ from the scaled perception camera's (the page's camera_model)",
        "Perception display clipping range did not preserve the far plane",
        "Perception camera local pose differs from the planned mount", "observe:all_candidates_failed:",
        "Camera did not produce RGB", "Nonfinite body state", "Excessive body tilt in ",
        "Actual truck/load footprint overlap in ", "Measured pallet footprint overlap in ", "mission_cancelled",
        "Pallet dropped during transport", "Pallet slipped off forks", "Pallet displaced before pickup",
        # "slam_link_failed: " goes through slam_reason_ now (a DYNAMIC_SITE: an L4 reason or the link's own)
        "Forbidden fork/pallet contact in ", "localization_stale: ",
        "Invalid pallet spawn after settling: ",  # a session pallet's settle check (setup; plan 2026-10-22 rigid 2.1)
        "Session pallet displaced: ",  # a session pallet nobody handles moved (scene damage; rigid plan 2.4)
        "withdraw_tracking_exceeded:", "Perception camera did not produce depth", "Measured pallet solid overlap in ",
        "",
    },
    "src/forklift_core/mission/core.py": {
        "not_pickable:fork_spacing_adjust",  # the forks do not fit the far reading at this spacing (2026-10-21)
        "obstacle_no_progress in ", "near_capture_realign_held_recovery", "Tracking timeout in ", "Tracking failed in ",
        "perception_capture_failed:", "observation_discarded:", "docking_retry_no_plan:", "docking_unaligned:",
        "slam_release_replan_failed: ", "async_plan_failed:", "near_field_frame_released",
        "near_capture_realign_too_long:", "near_field_without_capture", "near_field_without_hold",
        "Pallet was not lifted", "Pallet did not settle onto destination floor", "Fork did not lower",
        "sensor_silence_stopped", "near_capture_failed:", "observe:all_candidates_exhausted:",
        "near_capture_no_final_straight", "slam_release_no_stop after ", "docking_retry_match_refused:",
        "obstacle_live_plan_failed in ", "planning_wait_moving", "async_plan_discarded:", "near_field_",
        "withdraw_not_certified:no_pallet_estimate", "withdraw_not_certified:", "pocket_stop in ",
        "obstacle_replan_failed in ", "transport_recovery_after_docking", "stall_replan_failed:",
        "observe_replan_failed:", "obstacle_blocked in ", "obstacle_replan_limit in ", "async_near_no_fresh_depth",
        "plan_worker_dead: ",
        "session_anchor_unaligned:",  # an unaligned delivery straight the anchors allowed (plan 2026-10-22-slot 2.3)
    },
}


@pytest.mark.parametrize("rel", sorted(KNOWN))
def test_the_reasons_raised_are_the_ones_read_and_classified(rel):
    found = {text for _, text, _ in _require_reasons(ROOT / rel) if text is not None}
    assert found == KNOWN[rel], (sorted(found - KNOWN[rel]), sorted(KNOWN[rel] - found))


CAPTURE = {"sensor_metadata_unavailable": "session", "invalid_capture_pose": "session", "pose_changed": "mission",
           "sensor_frame_changed_during_read": "mission", "sensor_frame_not_ready": "mission",
           "acquisition_outside_capture": "mission", "lift_not_zero": "mission"}


def test_every_capture_failure_reason_is_classified_through_the_core_s_prefix():
    import re

    found = set()
    for rel in ("sim/isaac/perception_adapter.py", "sim/isaac/run_transport.py"):
        found |= set(re.findall(r'CaptureFailure\("([a-z_]+)"', (ROOT / rel).read_text()))
    assert found == set(CAPTURE)
    for reason, cls in CAPTURE.items():
        assert classify(f"perception_capture_failed:{reason}") == cls, reason


def test_the_scene_checks_are_scene_class_and_go_through_the_latch():
    tree = ast.parse((ROOT / "sim/isaac/run_transport.py").read_text())
    scene = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "scene_require":
            arg = node.args[2]
            scene.add(arg.value if isinstance(arg, ast.Constant) else arg.values[0].value)
    assert scene == {"Excessive body tilt in ", "Actual truck/load footprint overlap in ",
                     "Measured pallet footprint overlap in ", "Measured pallet solid overlap in ",
                     "Pallet dropped during transport", "Pallet slipped off forks", "Pallet displaced before pickup",
                     "Forbidden fork/pallet contact in ", "Session pallet displaced: "}
    assert all(classify(r) == "scene" for r in scene)


def test_no_live_decision_rests_on_pocket_phase_names_any_more():
    """Plan D5a: what a live mission's end left the forks in is the session's cargo state (the runner and the core
    named the pocket phases twice to decide it, plan D2/D6)."""
    import forklift_core.mission.core as core_module

    assert not hasattr(core_module, "POCKET_PHASES")
    assert "LIVE_POCKET_PHASES" not in (ROOT / "sim/isaac/run_transport.py").read_text()
