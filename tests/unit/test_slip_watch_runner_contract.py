"""The runner's half of the slip watch (docs/plans/2026-10-25-slip-check-relative.md): the watched phases, the
baseline at the tick's end with the phase the tick ended in, the mission's record, the push injection in the truck's
frame and the diagnostic pallet-only friction. Source checks: the runner needs Isaac to run."""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SOURCE = (ROOT / "sim/isaac/run_transport.py").read_text()
SCENE = (ROOT / "sim/isaac/scene.py").read_text()


def _block(start, end):
    a = SOURCE.index(start)
    return SOURCE[a:SOURCE.index(end, a)]


def test_the_slip_watch_judges_lift_extract_transport_and_live_carrying():
    at = SOURCE.index("slip_watch.check(")
    head = SOURCE[at - 400:at]
    assert '"lift", "extract", "transport"' in head
    assert 'core.cargo.relation == "carrying"' in head
    call = SOURCE[at:SOURCE.index("cargo=True", at)]
    assert "insertion_geometry.pocket_side_gaps(" in call
    assert 'f"Pallet slipped off forks: {slip_}"' in call


def test_the_lift_comes_from_the_steering_reads_joints_with_no_new_recorded_call():
    """The replay compares the recorded calls in order: the watch reads the lift from the joints the command's
    steering read took this tick, so it runs after the command assembly and before the physics step."""
    at = SOURCE.index("slip_watch.check(")
    assert SOURCE.index("command_ = core.assemble_command(") < at
    assert at < SOURCE.index('stepper["fn"]((pocket_frames and step % 12 == 0) or capturing_)')
    assert 'joints_tick_["q"][lift_index[0]]' in SOURCE[at:at + 700]
    assert "steering_position=steering_position_," in SOURCE
    hook = _block("def steering_position_():", "return")
    assert 'joints_tick_["q"] = robot.get_joint_positions()' in hook
    assert "steering_position=lambda: robot.get_joint_positions()[steers]" not in SOURCE


def test_the_watch_record_is_left_out_of_the_replay_end_state():
    """A record made before the watch has no such key (as the applied wheel speed, goal creep plan)."""
    assert '"state": ("slip_watch",)' in SOURCE[SOURCE.index("PROJECTION_NOT_COMPARED = "):][:400]


def test_the_old_absolute_lateral_is_no_longer_judged_and_the_drop_check_keeps_its_phases():
    assert "< 0.04,\n" not in SOURCE[SOURCE.index("Pallet dropped during transport") - 400:
                                      SOURCE.index("Pallet dropped during transport") + 900]
    drop = SOURCE[SOURCE.index("Pallet dropped during transport") - 400:SOURCE.index("Pallet dropped during transport")]
    assert 'ms.phase in ["extract", "transport"]' in drop  # not in the lift: the pallet starts it on the floor


def test_the_baseline_is_taken_at_the_tick_end_with_the_phase_the_tick_ended_in():
    end = SOURCE.index("slip_watch.end_tick(")
    assert SOURCE.index('core.cargo.observe(ms.phase, rear)') < end  # after the tick's phase changes
    assert end < SOURCE.index('stepper["fn"]((pocket_frames and step % 12 == 0) or capturing_)')
    call = SOURCE[end:end + 400]
    assert 'ms.phase if live is None or core.commands.adopted_id is not None else "idle"' in call
    # not inside the live-only block: a batch run takes it too
    line_start = SOURCE.rindex("\n", 0, end) + 1
    assert SOURCE[line_start:end] == " " * 16


def test_the_watch_is_made_from_the_runners_geometry():
    make = _block("slip_watch = SlipWatch(", "\n\n")
    assert "geometry.inserted_offset_m - abs(args.rear_axle_offset_m)" in make
    assert "geometry.pallet_depth_m / 2, geometry.pallet_width_m / 2" in make


def test_the_mission_record_carries_the_summary():
    from forklift_core.mission.baseline import MISSION_KEYS

    assert "slip_watch" in MISSION_KEYS["state"]


def test_the_push_injection_moves_the_pallet_to_the_trucks_left():
    push = _block('live_injected_["push"] = True', "ppos, pq = pallet.get_world_pose()")
    assert "np.array([0.0, 0.05, 0.0])" not in push
    assert "-math.sin(yaw_push_)" in push and "math.cos(yaw_push_)" in push


def test_the_diagnostic_pallet_friction_binds_a_min_combined_material_to_the_pallets_only():
    assert 'settings.get("pallet_contact_friction")' in SCENE
    block = SCENE[SCENE.index('"/World/PalletContactMaterial"') - 300:SCENE.index('"/World/PalletContactMaterial"') + 900]
    assert 'CreateFrictionCombineModeAttr("min")' in block
    bind = SCENE[SCENE.index("UsdShade.MaterialBindingAPI.Apply(prim).Bind("):][:300]
    assert "pallet_material" in bind
    assert "root_path in pallet_roots" in SCENE
