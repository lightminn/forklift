"""M-a4: the targets and destinations list and its revision (plan D7, contract section 5.1)."""

import pytest

from forklift_core.mission.targets import Pose, TargetsBook


def book():
    b = TargetsBook()
    b.add_pallet("P-1", "epal6", Pose(5.0, 0.0, 0.0))
    b.add_destination("D-1", Pose(9.0, 0.0, 0.0))
    return b


def test_registration_raises_the_revision_and_ids_are_never_reused():
    b = book()
    assert b.revision == 2
    with pytest.raises(ValueError):
        b.add_pallet("P-1", "epal6", Pose(1.0, 1.0, 0.0))
    with pytest.raises(ValueError):
        b.add_destination("P-1", Pose(1.0, 1.0, 0.0))


def test_a_pose_or_state_change_raises_the_revision_and_no_change_does_not():
    b = book()
    assert b.update_pose("P-1", Pose(5.0, 0.0, 0.0)) is False and b.revision == 2
    assert b.update_pose("P-1", Pose(5.02, 0.0, 0.0)) is True and b.revision == 3


def test_a_carried_pallet_has_no_pose_and_moving_it_does_not_raise_the_revision():
    b = book()
    b.carried("P-1")
    assert b.pallets["P-1"].pose is None and b.pallets["P-1"].state == "carried" and b.revision == 3
    for x in (6.0, 7.0, 8.0):
        assert b.update_pose("P-1", Pose(x, 0.0, 0.0)) is False
    assert b.revision == 3
    assert b.message()["pallets"] == [{"pallet_id": "P-1", "kind": "epal6", "pose": None, "state": "carried"}]


def test_put_down_registers_it_again_where_it_stands_under_the_same_id():
    b = book()
    b.carried("P-1")
    b.put_down("P-1", Pose(9.01, 0.02, 0.0))
    p = b.pallets["P-1"]
    assert (p.pallet_id, p.state, p.pose) == ("P-1", "delivered", Pose(9.01, 0.02, 0.0)) and b.revision == 4


def test_a_correction_change_reexpresses_every_pose_and_raises_the_revision():
    b = book()
    b.set_correction_version(1, reexpress=lambda p: Pose(p.x_m + 0.1, p.y_m, p.yaw_rad))
    assert b.pallets["P-1"].pose == Pose(5.1, 0.0, 0.0) and b.destinations["D-1"] == Pose(9.1, 0.0, 0.0)
    assert b.revision == 3 and b.correction_version == 1
    assert b.set_correction_version(1) is False and b.revision == 3


def test_a_submit_must_name_the_current_list():
    b = book()
    assert b.check_submit("P-1", "D-1", 2) is None
    assert b.check_submit("P-1", "D-1", 1) == "stale_targets"
    assert b.check_submit("P-9", "D-1", 2) == "invalid" and b.check_submit("P-1", "D-9", 2) == "invalid"
    b.carried("P-1")
    assert b.check_submit("P-1", "D-1", b.revision) == "invalid"  # already on the forks


def test_a_submit_with_an_older_list_is_refused_by_the_lifecycle():
    from forklift_core.mission.commands import MissionCommands, Submit

    b = book()
    m = MissionCommands(b)
    m.queue(Submit("P-1", "D-1", 1))
    assert m.apply(0.0)[0]["reason"] == "stale_targets" and m.status == "idle"
    m.queue(Submit("P-1", "D-1", b.revision))
    assert m.apply(0.1)[0]["accepted"]


def core_with(slam=None):
    from types import SimpleNamespace

    from forklift_core.mission.commands import Submit
    from forklift_core.mission.core import MissionCore
    from forklift_core.mission.state import MissionState

    pocket = {"planning_pickup": None}
    scenario = SimpleNamespace(destination=SimpleNamespace(x_m=9.0, y_m=0.5, yaw_rad=0.1))
    core = MissionCore(MissionState(phase="observe", phase_started=0.0), state={}, obstacle={"version": 3}, slam=slam,
                       pocket=pocket, paths={}, trackers={}, geometry=None, settings={}, pallet_geometry=None,
                       hooks=SimpleNamespace(), options=SimpleNamespace(scenario=scenario))
    core.targets.add_pallet("P-1", "epal6", Pose(5.0, 0.0, 0.0))
    core.targets.add_destination("D-1", Pose(9.0, 0.5, 0.1))
    core.commands.queue(Submit("P-1", "D-1", 2))
    core.commands.apply(0.0)
    core.commands.start()  # the batch run's mission runs at once; a live one only once adopted (plan D2)
    return core, pocket


def test_the_list_is_not_refreshed_for_a_mission_not_yet_adopted():
    """Persistent console stage 2 (Codex design P1): preparing P-1 -> D-0 wrote the last scenario's destination over
    D-0 and raised the revision, so the adoption check saw its own change."""
    from types import SimpleNamespace

    from forklift_core.mission.commands import MissionCommands, Submit

    core, pocket = core_with()
    core.commands = MissionCommands(core.targets)
    core.targets.add_destination("D-0", Pose(5.0, 0.0, 0.0))
    revision = core.targets.revision
    core.commands.queue(Submit("P-1", "D-0", revision))
    core.commands.apply(1.0)
    pocket["planning_pickup"] = SimpleNamespace(x_m=5.05, y_m=0.01, yaw_rad=0.0)
    core.commands.begin_prepare()
    core.refresh_targets()
    assert core.targets.destinations["D-0"] == Pose(5.0, 0.0, 0.0) and core.targets.revision == revision
    core.commands.terminate("failed", 2.0, "prepare:timeout")  # the ending tick: FINAL, never adopted
    core.refresh_targets()
    assert core.targets.destinations["D-0"] == Pose(5.0, 0.0, 0.0) and core.targets.revision == revision


def test_the_core_carries_the_list_through_the_mission():
    from types import SimpleNamespace

    core, pocket = core_with()
    core.refresh_targets()
    assert core.targets.correction_version == 0 and core.targets.revision == 2  # the obstacle version is not it
    pocket["planning_pickup"] = SimpleNamespace(x_m=5.05, y_m=0.01, yaw_rad=0.0)  # recognised
    core.refresh_targets()
    assert core.targets.pallets["P-1"].pose == Pose(5.05, 0.01, 0.0) and core.targets.revision == 3
    for phase in ("lift", "transport", "lower"):
        core.ms.phase = phase
        core.refresh_targets()
    assert core.targets.pallets["P-1"].state == "carried" and core.targets.revision == 4
    core.ms.phase = "withdraw"
    core.refresh_targets()
    assert core.targets.pallets["P-1"].pose == Pose(9.0, 0.5, 0.1) and core.targets.pallets["P-1"].state == "delivered"


def test_a_slam_release_is_a_new_correction_version_without_an_obstacle_layer():
    """Codex M-a4 review P2: the frame the poses are in changes at a release, obstacle layer or not."""
    slam = {"holds": []}
    core, _ = core_with(slam)
    core.obstacle = None
    core.refresh_targets()
    revision = core.targets.revision
    slam["holds"].append({"event": "release_after_backoff"})  # the event's name is not what counts
    core.refresh_targets()
    assert core.targets.correction_version == 0 and core.targets.revision == revision
    core.slam_releases += 1  # what every tracker.release in the core does (Codex M-a4 round 2)
    core.refresh_targets()
    assert core.targets.correction_version == 1 and core.targets.revision == revision + 1
    assert core.commands.targets.check_submit("P-1", "D-1", revision) == "stale_targets"


def test_the_destination_and_the_delivered_pallet_follow_the_cores_estimate():
    """Codex M-a4 review P2: a docking match moves the destination; a release carries the delivered pallet."""
    from types import SimpleNamespace

    slam = {"holds": []}
    core, _ = core_with(slam)
    slam["transport_scenario"] = SimpleNamespace(destination=SimpleNamespace(x_m=9.05, y_m=0.02, yaw_rad=0.01))
    core.refresh_targets()
    assert core.targets.destinations["D-1"] == Pose(9.05, 0.02, 0.01) and core.targets.revision == 3
    for phase in ("lift", "withdraw"):
        core.ms.phase = phase
        core.refresh_targets()
    assert core.targets.pallets["P-1"].pose == Pose(9.05, 0.02, 0.01)
    core.slam_releases += 1
    slam["return_scenario"] = SimpleNamespace(destination=SimpleNamespace(x_m=9.1, y_m=0.0, yaw_rad=0.0))
    core.ms.phase = "return_home"
    core.refresh_targets()
    assert core.targets.pallets["P-1"].pose == Pose(9.1, 0.0, 0.0) == core.targets.destinations["D-1"]
    assert core.targets.pallets["P-1"].state == "delivered"


def test_every_slam_release_goes_through_the_one_counting_method():
    """Codex M-a4 rounds 2-3: six release sites, one of them spread over three lines; each must count. The tracker's
    release is called in one place only, which counts it."""
    import pathlib
    import re

    import forklift_core

    root = pathlib.Path(forklift_core.__file__).parent
    sites = [(path.name, m.start()) for path in sorted(root.rglob("*.py")) if path.name != "slam_pose.py"
             for m in re.finditer(r"\.release\s*\(", path.read_text())]
    assert [name for name, _ in sites] == ["core.py"]
    source = (root / "mission" / "core.py").read_text()
    body = source[source.index("    def release_slam("):source.index("    def refresh_targets(")]
    assert '.release(odom_from_base=odom_from_base)' in body and "self.slam_releases += 1" in body


def test_a_pallet_registered_with_its_size_carries_it_in_the_message():
    # the console draws the pallet from these (C3); a pallet registered without them has none
    b = TargetsBook()
    b.add_pallet("P-1", "epal6", Pose(5.0, 0.0, 0.0), depth_m=0.8, width_m=0.6)
    b.add_pallet("P-2", "t11", Pose(6.0, 0.0, 0.0))
    p1, p2 = b.message()["pallets"]
    assert (p1["depth_m"], p1["width_m"]) == (0.8, 0.6)
    assert "depth_m" not in p2 and "width_m" not in p2
    b.carried("P-1")
    assert b.message()["pallets"][0]["depth_m"] == 0.8  # still known while carried
