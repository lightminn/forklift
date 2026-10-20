"""What a live run does with a failure (persistent console plan D6): a mission's own failure ends that mission and
the session goes on; damage to the scene is latched and ends the mission; anything else ends the session.

``classify(reason)`` picks the class by the reason's longest known prefix; a reason no prefix covers is a session
failure, so a new, unclassified reason can never be swallowed as a mission's.
"""

from __future__ import annotations

MISSION, SCENE, SESSION = "mission", "scene", "session"

PREFIXES = {
    # the truth checks (simulation only): the scene is damaged, latched
    "Excessive body tilt": SCENE, "Actual truck/load footprint overlap": SCENE,
    "Measured pallet footprint overlap": SCENE, "Measured pallet solid overlap": SCENE,
    "Pallet dropped during transport": SCENE, "Pallet slipped off forks": SCENE,
    "Pallet displaced before pickup": SCENE, "Forbidden fork/pallet contact": SCENE,
    # contracts, configuration, localisation, sensors, the body: the session ends
    "near_field_contract": SESSION, "near_field_without_capture": SESSION, "near_field_without_hold": SESSION,
    "near_field_frame_released": SESSION, "pocket_check_without_capture": SESSION,
    # the capture's own contract (a broken sensor or pose, not a scene that could not be seen; Codex D6 P1-2)
    "perception_capture_failed:invalid_capture_pose": SESSION,
    "perception_capture_failed:sensor_metadata_unavailable": SESSION,
    "plan_worker_dead": SESSION,  # the planning worker could not be started again (Codex D6 P2-4)
    "localization_stale": SESSION, "slam_link_failed": SESSION, "slam_prewarm_timeout": SESSION,
    # SLAM localization plan L4: the survey map bundle at session start (slam_map_mismatch stops the supervisor at once)
    "slam_map_mismatch": SESSION, "slam_map_not_loaded": SESSION, "slam_not_localized": SESSION,
    "sensor_silence_stopped": SESSION, "Camera did not produce RGB": SESSION,
    "Perception camera did not produce depth": SESSION, "Nonfinite body state": SESSION,
    "Mission exceeded simulation time budget": SESSION, "mission_cancelled": SESSION,
    # the setup's own (never inside a live mission)
    "Cannot open base scene": SESSION, "Invalid pallet spawn after settling": SESSION, "Truck spawn displaced": SESSION,
    "This adapter requires 120Hz physics": SESSION, "Settings must be positive finite values": SESSION,
    "Camera render product prim is missing": SESSION, "Perception camera intrinsics": SESSION,
    "base scene built from another forklift URDF": SESSION, "pocket camera intrinsics differ": SESSION,
    "live video camera intrinsics differ": SESSION, "Perception display clipping range": SESSION,
    "Perception camera local pose differs": SESSION, "--odometry-wheel-radius-m": SESSION,
    "Pallet missed the green destination center": SESSION, "Delivered pallet is not grounded": SESSION,
    "Fork still inside delivered pallet": SESSION,
    "recognition label target": SESSION,  # main's far recognition (2026-10-18): the scene's label prim is missing
    # the mission's own work: that mission fails
    "Tracking timeout": MISSION, "Tracking failed": MISSION, "obstacle_": MISSION,
    "perception_capture_failed": MISSION, "observation_discarded": MISSION, "near_capture_": MISSION,
    "pocket_check_refused": MISSION,  # the insertion refused (it turns on the observed yaw too; Codex D6 P2-5)
    "near_field_": MISSION, "observe:": MISSION, "observe_replan_failed": MISSION, "docking_": MISSION,
    "slam_release_": MISSION, "async_plan_failed": MISSION, "async_plan_discarded": MISSION,
    "async_near_no_fresh_depth": MISSION, "planning_wait_moving": MISSION, "Pallet was not lifted": MISSION,
    "Pallet did not settle onto destination floor": MISSION, "Fork did not lower": MISSION,
    "withdraw_not_certified": MISSION, "withdraw_tracking_exceeded": MISSION, "pocket_stop": MISSION,
    "transport_recovery_after_docking": MISSION, "stall_replan_failed": MISSION, "Mission planning failed": MISSION,
    "approach:": MISSION, "insert:": MISSION, "extract:": MISSION, "transport:": MISSION, "withdraw:": MISSION,
    "return_home:": MISSION, "cusp_position_stall": MISSION,
}

# The require sites whose reason is made at run time, by (file, expression), with what their values are.
DYNAMIC_SITES = {
    ("sim/isaac/run_transport.py", "str(table_problem_)"): SESSION,  # the setup's age table check
    ("sim/isaac/run_transport.py", "'near_field_contract:' + '; '.join(problems_)"): SESSION,
    ("sim/isaac/run_transport.py", "verdict_.reason or ''"): SESSION,  # the batch run's delivery check
    ("sim/isaac/run_transport.py", "f"): SESSION,  # f-strings with no fixed head (setup messages)
    ("sim/isaac/run_transport.py", "slam_reason_"): SESSION,  # an L4 reason (slam_map_*, slam_not_localized) or the link's
    ("sim/isaac/run_transport.py", "reason_"): SCENE,  # scene_require's own raise: the truth checks' reasons
    ("src/forklift_core/mission/core.py", "plans.status"): MISSION,  # "<phase>:<status>"
    ("src/forklift_core/mission/core.py", "failure"): MISSION,  # a docking retry's "docking_*"
    ("src/forklift_core/mission/core.py", "result_.end"): MISSION,  # "cusp_position_stall*"
    ("src/forklift_core/mission/core.py", "reason_"): MISSION,  # "near_field_retry_no_detection", "near_capture_*"
    ("src/forklift_core/mission/core.py", "refusal"): MISSION,  # "near_capture_realign_short:*" and the like
    ("src/forklift_core/mission/core.py", "'near_field_contract:' + '; '.join(tracker_problems)"): SESSION,
}


def classify(reason: str, *, strict: bool = False) -> str | None:
    """The class of ``reason`` by its longest known prefix; unknown: session (None with ``strict``)."""
    best = None
    for prefix, cls in PREFIXES.items():
        if reason.startswith(prefix) and (best is None or len(prefix) > len(best[0])):
            best = (prefix, cls)
    if best is None:
        return None if strict else SESSION
    return best[1]
