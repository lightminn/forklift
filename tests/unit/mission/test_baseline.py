"""The mission baseline (persistent console plan D2, D0 table): what a live mission owns is put back, at each
adoption, to what it was before the session's first mission; the last mission's records are handed over."""

from types import SimpleNamespace

import pytest

from forklift_core.mission.baseline import MissionBaseline
from forklift_core.mission.state import MissionState


def fake_core():
    ms = MissionState(phase="observe", phase_started=0.0)
    return SimpleNamespace(
        ms=ms, state={"samples": [1], "near_tracking": {"enabled": True, "realigns": 0}},
        obstacle={"version": 3, "replans": {}}, slam={"docking": {"status": "pending"}, "holds": []},
        near={"episode": 0, "tracker": None}, pocket={"check": None, "planning_pickup": None, "frames_read": 0},
        release_replan_needed=None, fresh_depth_wait=None, last_permission=None)


def test_a_restore_puts_every_mission_key_back_and_drops_the_ones_made_since():
    core = fake_core()
    base = MissionBaseline.capture(core)
    core.obstacle["replans"]["transport"] = 4
    core.obstacle["live_replanned"] = {"transport": True}  # made during the mission
    core.slam["docking"] = {"status": "done"}
    core.slam["transport_scenario"] = "SC"
    core.near["tracker"] = object()
    core.pocket["planning_pickup"] = (1, 2, 3)
    core.state["near_tracking"]["realigns"] = 2
    core.state["paths_replaced"] = [1]
    core.ms.observe_stall_ticks, core.ms.last_tracking = 7, "T"
    core.last_permission = (0.3, "ok")
    base.restore(core)
    assert core.obstacle["replans"] == {} and "live_replanned" not in core.obstacle
    assert core.slam["docking"] == {"status": "pending"} and "transport_scenario" not in core.slam
    assert core.near["tracker"] is None and core.pocket["planning_pickup"] is None
    assert core.state["near_tracking"] == {"enabled": True, "realigns": 0}
    assert "paths_replaced" not in core.state
    assert core.ms.observe_stall_ticks == 0 and core.ms.last_tracking is None and core.last_permission is None


def test_session_keys_are_kept():
    core = fake_core()
    base = MissionBaseline.capture(core)
    core.state["samples"].append(2)
    core.obstacle["version"] = 9
    core.slam["holds"].append({"event": "hold"})
    core.near["episode"] = 3
    base.restore(core)
    assert core.state["samples"] == [1, 2] and core.obstacle["version"] == 9
    assert core.slam["holds"] == [{"event": "hold"}] and core.near["episode"] == 3


def test_each_restore_is_a_fresh_copy_and_the_objects_stay_the_same():
    core = fake_core()
    obstacle, slam = core.obstacle, core.slam
    base = MissionBaseline.capture(core)
    base.restore(core)
    core.obstacle["replans"]["x"] = 1
    base.restore(core)
    assert core.obstacle["replans"] == {} and core.obstacle is obstacle and core.slam is slam


def test_the_last_missions_records_are_handed_over():
    core = fake_core()
    base = MissionBaseline.capture(core)
    core.state["async_plans"] = [{"site": "leg"}]
    core.obstacle["dumps"] = 2
    core.pocket["frames_read"] = 40
    records = base.restore(core)
    assert records["state"]["async_plans"] == [{"site": "leg"}] and records["obstacle"]["dumps"] == 2
    assert records["pocket"]["frames_read"] == 40 and core.pocket["frames_read"] == 0
    assert "async_plans" not in core.state


def test_a_mission_value_that_is_not_plain_data_is_refused_at_capture():
    core = fake_core()
    core.pocket["planning_pickup"] = object()
    with pytest.raises(TypeError):
        MissionBaseline.capture(core)


def test_the_baseline_owns_no_key_the_ownership_table_calls_the_sessions():
    from forklift_core.mission import baseline

    for owner, keys in baseline.MISSION_KEYS.items():
        assert not set(keys) & set(baseline.SESSION_KEYS.get(owner, ())), owner


def test_a_restore_keeps_the_lists_and_dicts_other_objects_hold():
    """Codex stage 2 impl P2-4: CuspStallMonitor holds state["cusp_stalls"]; a new list cut it off from the state."""
    core = fake_core()
    core.state["cusp_stalls"] = []
    held = core.state["cusp_stalls"]
    replans = core.obstacle["replans"]
    base = MissionBaseline.capture(core)
    held.append({"stall": 1})
    replans["x"] = 1
    records = base.restore(core)
    assert core.state["cusp_stalls"] is held and held == [] and core.obstacle["replans"] is replans and replans == {}
    assert records["state"]["cusp_stalls"] == [{"stall": 1}] and records["obstacle"]["replans"] == {"x": 1}
    assert records["state"]["cusp_stalls"] is not held


def test_the_near_fields_record_goes_with_its_mission_and_the_next_starts_empty():
    """Plan D8 (Codex D8 design review P2-3): the near-field ticks (one per control tick while tracking, ~6 MB a
    minute) and its frames, events, corrections and revisions are the mission's; a session left up does not keep
    every mission's."""
    core = fake_core()
    core.near.update(ticks=[], frames=[], events=[], corrections=[], path_revisions=[])
    ticks = core.near["ticks"]
    base = MissionBaseline.capture(core)
    core.near["ticks"].extend({"face_distance_m": d, "episode": 1} for d in (0.5, 0.4))
    core.near["events"].append({"event": "armed"})
    core.near["episode"] = 1
    records = base.restore(core)
    assert records["near"]["ticks"] == [{"face_distance_m": 0.5, "episode": 1}, {"face_distance_m": 0.4, "episode": 1}]
    assert records["near"]["events"] == [{"event": "armed"}]
    assert core.near["ticks"] == [] and core.near["ticks"] is ticks and core.near["episode"] == 1


def test_a_run_without_near_tracking_hands_over_no_near_record():
    core = fake_core()
    core.near = None
    records = MissionBaseline.capture(core).restore(core)
    assert "near" not in records  # start_mission puts each owner's records back: there is no near to put them in


def test_the_start_digest_names_every_mission_slot_and_follows_only_mission_values():
    """Use case U21 on Isaac: each mission-owned slot at a mission's start as a short hash, so the second mission's
    start can be compared with the first's; session keys are not in it, and an object's hash does not carry its
    address (two equal starts in two processes hash alike)."""
    from forklift_core.mission.baseline import start_digest

    core = fake_core()
    first = start_digest(core)
    assert first["obstacle.replans"] != "absent" and first["obstacle.live_replanned"] == "absent"
    assert first["ms.next_candidate_index"] != "absent" and first["state.near_tracking.realigns"] != "absent"
    assert not any(k.startswith(("state.samples", "slam.holds", "obstacle.version")) for k in first)
    core.state["samples"].append(2)  # a session key
    core.near["tracker"] = object()  # an object: hashed by its type, not its address
    again = start_digest(core)
    assert {k for k in first if first[k] != again[k]} == {"near.tracker"}
    core.near["tracker"] = object()
    assert start_digest(core) == again
    core.obstacle["replans"]["transport"] = 1
    assert {k for k, v in start_digest(core).items() if v != again[k]} == {"obstacle.replans"}


def test_the_start_digest_takes_dicts_with_keys_json_cannot():
    from forklift_core.mission.baseline import start_digest

    core = fake_core()
    core.obstacle["replans"] = {("transport", 1): 2, "leg": 1}
    assert start_digest(core)["obstacle.replans"] != "absent"
