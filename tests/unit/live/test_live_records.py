"""The live session's spooled records (persistent console plan D8): which lists go to the spool, and the end-of-run
summaries a live run makes from the chunks one at a time -- the same numbers a batch run makes from the whole list."""

from pathlib import Path

import numpy as np
import pytest

from forklift_core.live.spool import RecordSpool, iter_chunks
from sim.isaac import live_records


def rows(n, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n):
        est = rng.normal(size=3)
        truth = est + rng.normal(scale=0.05, size=3)
        out.append([i / 120, *est.tolist(), *truth.tolist()])
    return out


def batch_summary(control_rows, statuses):
    control = np.asarray(control_rows, dtype=float).reshape(-1, 7)
    error = np.hypot(control[:, 1] - control[:, 4], control[:, 2] - control[:, 5])
    yaw_error = np.abs(np.angle(np.exp(1j * (control[:, 3] - control[:, 6]))))
    return {"control_samples": len(control), "raw_position_rmse_m": float(np.sqrt(np.mean(error**2))),
            "raw_position_max_m": float(error.max()), "raw_yaw_rmse_rad": float(np.sqrt(np.mean(yaw_error**2))),
            "raw_yaw_max_rad": float(yaw_error.max()), "scans": len(statuses),
            "processed": statuses.count("processed"), "skipped": statuses.count("skipped"),
            "link_failures": statuses.count("link_failure")}


def test_the_streamed_slam_summary_is_the_whole_lists_summary(tmp_path):
    control, records = rows(1000), [{"status": s} for s in ["processed"] * 70 + ["skipped"] * 25 + ["link_failure"] * 5]
    spool = RecordSpool(tmp_path, interval_s=0.0, queue_cuts=100)  # cut back to back: no backlog drop
    live_control, live_records_ = [], []
    spool.register("slam.control", live_control, keep=1)
    spool.register("slam.records", live_records_, keep=1)
    for i, row in enumerate(control):
        live_control.append(list(row))
        if i % 10 == 0 and i // 10 < len(records):
            live_records_.append(records[i // 10])
        if i % 137 == 0:
            spool.tick(float(i))
    spool.close()
    streamed = live_records.slam_summary(iter_chunks(tmp_path, "slam.control"), iter_chunks(tmp_path, "slam.records"))
    expected = batch_summary(control, [r["status"] for r in records])
    assert streamed.keys() == expected.keys()
    for key, value in expected.items():
        assert streamed[key] == pytest.approx(value, rel=1e-12), key


def test_an_empty_slam_record_summarises_as_nothing(tmp_path):
    s = live_records.slam_summary(iter([]), iter([]))
    assert s["control_samples"] == 0 and s["raw_position_rmse_m"] is None and s["scans"] == 0


def test_the_streamed_scan_wall_is_the_whole_lists(tmp_path):
    walls = [{"total_wall_s": w} for w in np.linspace(0.001, 0.02, 57)]
    stats = live_records.scan_wall_stats([walls[:20], walls[20:]])
    assert stats["scans"] == 57
    assert stats["wall_s_per_scan"]["mean"] == pytest.approx(float(np.mean([w["total_wall_s"] for w in walls])))
    assert stats["wall_s_per_scan"]["max"] == pytest.approx(0.02)
    assert live_records.scan_wall_stats([])["wall_s_per_scan"] == {"mean": 0.0, "max": 0.0}


def test_registration_takes_the_growing_session_lists_with_their_last_item(tmp_path):
    slam_log = {k: [] for k in ("silenced_scan_stamps_s", "joint_stamps_s", "wheel_rates_rad_s", "steering_rad",
                                "base_pose_world", "scan_stamps_s", "scan_ranges_m", "laser_pose_world",
                                "wheel_angles_rad", "wheel_counts", "wheel_increments_rad", "steering_measured_rad")}
    slam = {"control": [], "records": []}
    obstacle = {"scans": [], "plans": []}
    state = {"samples": [], "tyre_contact_rows": []}
    video = {"frame_audit": [], "video_frames": [], "overlay": {"grids": [], "plan_history": []}}
    spool = RecordSpool(tmp_path, interval_s=0.0)
    names = live_records.register(spool, slam_log=slam_log, slam=slam, obstacle=obstacle, state=state, video=video)
    assert names == ["slam_log.joint", "slam_log.scan", "slam_log.silenced", "slam.control", "slam.records",
                     "obstacle.scans", "state.samples", "state.tyre_contact_rows", "video.frame_audit",
                     "video.frames", "video.grids", "video.plan_history"]
    joint = spool._groups["slam_log.joint"][0]
    assert set(joint) == {"joint_stamps_s", "wheel_rates_rad_s", "steering_rad", "base_pose_world", "wheel_angles_rad",
                          "wheel_counts", "wheel_increments_rad", "steering_measured_rad"}
    assert set(spool._groups["slam_log.scan"][0]) == {"scan_stamps_s", "scan_ranges_m", "laser_pose_world"}
    assert all(keep == 1 for _, keep in [*spool._lists.values(), *spool._groups.values()])
    assert "obstacle.plans" not in names  # its last entry is written right after it is added, in the same call
    spool.close()


def test_registration_skips_what_the_run_does_not_have(tmp_path):
    spool = RecordSpool(tmp_path, interval_s=0.0)
    names = live_records.register(spool, slam_log=None, slam=None, obstacle=None, state={"samples": []}, video=None)
    assert names == ["state.samples"]
    spool.close()


def _runner() -> str:
    from pathlib import Path

    return (Path(__file__).resolve().parents[3] / "sim" / "isaac" / "run_transport.py").read_text()


def test_the_runner_spools_only_a_live_run_and_cuts_at_the_tick_start():
    src = _runner()
    made = src.index('live_spool = RecordSpool(args.output / "spool", interval_s=10.0)')
    assert src.index("core.baseline = MissionBaseline.capture(core)") < made < src.index("            live.start()")
    assert src.count("live_spool = RecordSpool(") == 1 and "    live_spool = None" in src
    cut = src.index("live_spool.tick(time.monotonic())")
    start = src.index("stop_kind_, applied_commands_ = live.tick_start(")
    assert cut < start and src.rfind("if live is not None:", 0, cut) > src.rfind("try:", 0, cut)


def test_the_runner_closes_the_spool_before_the_records_and_takes_them_from_it():
    src = _runner()
    closed = src.index('state["spool"] = {"root": "spool", **live_spool.close()}')
    for later in ('if state.get("obstacle_layer") is not None and obstacle is not None:',
                  'if slam is not None and "slam.control" in spooled_:', 'if "video.frame_audit" not in spooled_:',
                  'if slam_log is not None and "slam_log.joint" in spooled_:'):
        assert closed < src.index(later), later
    assert 'if "obstacle.scans" not in spooled_:' in src  # a batch run (nothing spooled) writes as before
    assert "elif slam_log is not None and slam_log[\"scan_stamps_s\"]:" in src


def test_a_near_frame_record_stops_at_its_bound_and_counts_the_rest():
    """Codex D8 design round 2 P2-1: a STOP while tracking pauses the mission without ending it, and frames are
    still read (10 Hz): a mission's frame record is bounded on its own, the overflow counted."""
    near = {"frames": []}
    for i in range(7):
        live_records.keep_near_frame(near, {"i": i}, kept=5)
    assert [f["i"] for f in near["frames"]] == [0, 1, 2, 3, 4] and near["frames_overflow"] == 2
    batch = {"frames": []}
    for i in range(7):
        live_records.keep_near_frame(batch, {"i": i}, kept=None)  # a batch run keeps every frame, as before
    assert len(batch["frames"]) == 7 and "frames_overflow" not in batch


def test_the_runner_keeps_near_frames_through_the_bound():
    src = _runner()
    assert 'near["frames"].append(' not in src
    assert src.count('keep_near_frame(near, row, NEAR_FRAMES_KEPT if getattr(args, "live", False) else None)') == 5


def test_video_recording_without_the_robot_camera_spools_no_frame_records(tmp_path):
    """Codex D8 design round 2 P2-4: the robot camera's frame records exist only with --robot-camera; registering
    them otherwise sent the end into the robot camera's metadata (perception_calibration unset)."""
    spool = RecordSpool(tmp_path, interval_s=0.0)
    names = live_records.register(spool, slam_log=None, slam=None, obstacle=None, state={"samples": []},
                                  video={"frame_audit": [], "video_frames": None,
                                         "overlay": {"grids": [], "plan_history": []}})
    assert "video.frames" not in names and "video.frame_audit" in names
    spool.close()
    assert '"video_frames": video_frames if args.robot_camera else None' in _runner()


def test_a_live_runs_tyre_contacts_stay_in_the_spool():
    src = _runner()
    assert 'if "state.tyre_contact_rows" in (state.get("spool_registered") or ()):' in src
    assert src.index('if "state.tyre_contact_rows" in') < src.index('np.save(args.output / "tyre_contacts.npy"')


def test_the_live_teleop_permit_takes_the_pallet_zones_and_advances_them():
    """Plan D10: the target pallets on the same arcs and outline as the grid permit; the slower wins; the installed
    distances are advanced by the same driving; the U11 control flag turns it off (live only)."""
    src = _runner()
    assert "zones_ = live_zones_from(core.targets, pickup_zone)" in src and "live_pallet_perm.limit(zones_, core.targets.revision" in src
    assert "if pallet_[0] < result_[0]:" in src
    assert src.index("live_perm.advance(driven_)") < src.index("live_pallet_perm.advance(driven_)")
    assert "if obstacle is not None and not args.live_test_no_pallet_permit else None" in src
    from pathlib import Path

    near = (Path(__file__).resolve().parents[3] / "sim" / "isaac" / "near_tracking.py").read_text()
    assert '"live_test_no_pallet_permit": False' in near


def test_the_trace_evidence_follows_where_the_pallet_is():
    """Plan D4 (6th ed.): on the floor its map pose against the insert start / the lower verdict; riding the forks
    its pose relative to them against the insert's end; a normal withdraw is no departure."""
    ev = live_records.TraceEvidence()
    pallet = (5.0, 0.0, 0.0)
    assert ev.update("insert", (3.6, 0.0, 0.0), pallet)  # the insert start: the floor reference
    assert ev.update("insert", (3.9, 0.0, 0.0), pallet)
    assert ev.update("lift", (3.9, 0.0, 0.0), (5.0, 0.0, 0.0))  # the insert's end: the riding reference
    assert ev.update("transport", (8.9, 2.0, 0.0), (10.0, 2.0, 0.0))  # carried along: same relative pose
    assert ev.update("lower", (8.9, 2.0, 0.0), (10.0, 2.0, 0.0))
    assert ev.update("withdraw", (8.9, 2.0, 0.0), (10.0, 2.0, 0.0))  # the lower verdict: the floor reference again
    assert ev.update("withdraw", (8.6, 2.0, 0.0), (10.0, 2.0, 0.0))  # backed out, the pallet still: valid


def test_a_pallet_turned_on_the_forks_before_the_lift_verdict_is_caught():
    """Codex 6b round 4 P2-1: the riding reference is the insert's end, so a turn during the lift shows."""
    ev = live_records.TraceEvidence()
    ev.update("insert", (3.6, 0.0, 0.0), (5.0, 0.0, 0.0))
    ev.update("lift", (3.9, 0.0, 0.0), (5.0, 0.0, 0.0))
    assert not ev.update("lift", (3.9, 0.0, 0.0), (5.0, 0.0, 0.02))  # 0.02 rad past 0.01
    assert not ev.update("transport", (8.9, 2.0, 0.0), (10.0, 2.0, 0.0))  # once lost, stays lost


def test_a_floor_pallet_pushed_during_the_insert_is_caught():
    ev = live_records.TraceEvidence()
    ev.update("insert", (3.6, 0.0, 0.0), (5.0, 0.0, 0.0))
    assert not ev.update("insert", (3.9, 0.0, 0.0), (5.03, 0.0, 0.0))


def test_the_runner_feeds_the_recovery_its_evidence_checks_and_outcome():
    """Plan D4: the cargo observes each live tick's end pose; the truth evidence clears trace_valid; the retrace has
    the forbidden-contact check and the lateral gaps; a failure while recovering ends recovery_required."""
    src = _runner()
    assert "core.cargo.observe(ms.phase, rear)" in src
    assert src.index("core.cargo.observe(ms.phase, rear)") < src.index("live_safety_ = core.safety_state(t)")
    assert "core.cargo.trace_valid = False" in src and "trace_evidence = live_records.TraceEvidence()" in src
    # plan D4b: the back-out too, with the whole truck's contacts as when free
    assert 'if checks_phase_ in ["insert", "withdraw", "recover_withdraw"] or live_free_ or clear_out_ or withdraw_watch_:' in src
    assert "if live_free_ or clear_out_ or withdraw_watch_:" in src and 'clear_out_ = ms.phase == "clear_out"' in src
    assert 'core.observe_motion(slam["odom_rear"] if slam is not None else rear)' in src
    assert 'if checks_phase_ in ["insert", "extract", "withdraw", "recover_withdraw"]:' in src
    assert '"recovery_required" if core.commands.recovery else "failed"' in src


def test_the_evidence_reference_is_the_transition_ticks_truth():
    """Codex 6b impl round 1 P2-5: the scene check runs before the tick's transition; the runner feeds the same
    truth again at the tick's end with the phase it ended in, so the reference is that instant's."""
    ev = live_records.TraceEvidence()
    ev.update("insert", (3.6, 0.0, 0.0), (5.0, 0.0, 0.0))
    ev.update("insert", (3.9, 0.0, 0.0), (5.0, 0.0, 0.0))  # the scene check of the insert's last tick
    ev.update("lift", (3.9, 0.0, 0.0), (5.0, 0.0, 0.0))  # the same tick's end: insert -> lift happened
    assert not ev.update("lift", (3.9, 0.0, 0.0), (5.0, 0.0, 0.02))  # one physics step later: turned 0.02 rad


def test_the_runner_publishes_the_retrace_and_keeps_a_recovery_refusals_reason():
    src = _runner()
    assert '"running", "paused", "held", "resuming", "recovering") else None' in src
    assert 'live.end_mission(t, "recovery_required", core.commands.end_reason)' in src
    end_ = src.index("core.cargo.observe(ms.phase, rear)")
    assert src.index("trace_evidence.update(ms.phase", end_) < src.index("live_safety_ = core.safety_state(t)")


def test_the_manual_ground_verdict_moves_the_evidence_to_the_floor_and_idle_keeps_it():
    """Plan D14: lowered by hand, the pallet's floor pose is the reference; idle ticks (an ended mission's phase is
    not the truck's) keep the reference they have -- backing out with the pallet still is valid, dragging it is not."""
    ev = live_records.TraceEvidence()
    ev.update("insert", (3.6, 0.0, 0.0), (5.0, 0.0, 0.0))
    ev.update("lift", (3.9, 0.0, 0.0), (5.0, 0.0, 0.0))
    assert ev.update("idle", (3.9, 0.0, 0.0), (5.0, 0.0, 0.0))  # riding, ended: still the riding reference
    ev.ground((5.0, 0.0, 0.0))
    assert ev.update("idle", (3.5, 0.0, 0.0), (5.0, 0.0, 0.0))  # backed out 0.4 m, the pallet on the floor
    assert not ev.update("idle", (3.3, 0.0, 0.0), (4.8, 0.0, 0.0))  # dragged with the forks: lost
    ev.ground((4.8, 0.0, 0.0))
    assert not ev.update("idle", (3.3, 0.0, 0.0), (4.8, 0.0, 0.0))  # a lost evidence is not revived


def test_the_runner_wires_the_manual_recovery():
    src = _runner()
    assert "core.manual_recovery_tick(t, rear, standing=bool(live.arbiter.stop_confirmed))" in src
    assert src.index("core.manual_recovery_tick(") < src.index("live_safety_ = core.safety_state(t)")
    assert '"lift_command_m": float(ms.lift_command)' in src and "ms.lift_command = float(out_.lift_target_m)" in src
    assert "ms.lift_command = float(live._lift_latch)" in src
    assert "core.hooks.trace_ground = lambda: trace_evidence.ground(" in src
    assert 'trace_evidence.update(ms.phase if core.commands.adopted_id is not None else "idle"' in src
    assert "core.manual_clear_done(now_)" in src and 'z.zone_id != ctx_["pallet_id"]' in src
    assert 'state.setdefault("uncertified_teleop_withdraw", [])' in src
    assert 'core.cargo.invalidate("teleop_withdraw_contact")' in src
    near = (Path(__file__).resolve().parents[3] / "sim" / "isaac" / "near_tracking.py").read_text()
    assert '"live_test_recovery_blocked": None' in near


def test_the_urdf_lift_limit_is_the_models_or_the_documented_default(tmp_path):
    import ast
    import math as _math

    src = _runner()
    fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == "urdf_lift_limit")
    scope = {"math": _math}
    exec(compile(ast.Module([fn], []), "run_transport.py", "exec"), scope)
    root = Path(__file__).resolve().parents[3]
    assert scope["urdf_lift_limit"](root / "sim/models/dls08_measured/forklift.urdf") == (0.28, "urdf")
    (tmp_path / "x.urdf").write_text("<robot name='x'/>")
    assert scope["urdf_lift_limit"](tmp_path / "x.urdf") == (0.20, "default")
    assert scope["urdf_lift_limit"](tmp_path / "missing.urdf") == (0.20, "default")


def test_the_runner_closes_the_review_gaps_of_d14():
    """Codex D14 impl round 1: the gate's latch synced into the manual lift, the released pose to the permit, the
    full permit advanced, the tick-start evidence idle too, the lift capability on, the episode kept to standing."""
    src = _runner()
    assert "live.arbiter.lift.sync(float(live._lift_latch))" in src
    assert "if core.manual_recovery_tick(t, rear, standing=bool(live.arbiter.stop_confirmed)):" in src
    assert "live_pallet_full.advance(driven_)" in src
    assert src.count('ms.phase if core.commands.adopted_id is not None else "idle"') >= 2
    rt = (Path(__file__).resolve().parents[3] / "sim" / "isaac" / "live_runtime.py").read_text()
    assert '"lift": True' in rt


def test_the_models_lift_sag_bounds_the_hold_tolerance(tmp_path):
    """Main 2026-10-19: a hold tolerance below the lift's gravity sag would ratchet a standing lift down at every
    event -- the runner refuses it. The sag is the model's carriage weight over its lift position gain."""
    import ast
    import math as _math

    src = _runner()
    fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == "model_lift_sag_m")
    scope = {"math": _math, "Path": Path}
    exec(compile(ast.Module([fn], []), "run_transport.py", "exec"), scope)
    root = Path(__file__).resolve().parents[3]
    sag = scope["model_lift_sag_m"](root / "sim/models/dls08_measured/forklift.urdf")
    assert sag == pytest.approx(1.6 * 9.81 / 1800.0)
    assert scope["model_lift_sag_m"](tmp_path / "none.urdf") is None
    assert "if live_sag_ is not None and LiveTeleopLimits().lift_hold_tolerance_m < live_sag_:" in src
