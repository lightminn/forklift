"""M-c4: the live runtime with a fake core and fake Isaac hooks (plan C2, C4)."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("websockets")

from forklift_core.mission.commands import MissionCommands  # noqa: E402
from forklift_core.mission.targets import Pose, TargetsBook  # noqa: E402
from sim.isaac.live_runtime import Hooks, LiveRuntime  # noqa: E402

DT = 1 / 120


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t

    def sleep(self, s):
        self.t += max(0.0, s)


def _squash(text: str) -> str:
    """Runs of whitespace as one space: a source test that does not hang on the runner's indentation (the tick body
    moved one level in for the persistent console plan D6's catch)."""
    import re

    return re.sub(r"\s+", " ", text)


_MADE: list = []  # every runtime a test made, closed after it (its live_log file; -W error, ws1 full suite)


@pytest.fixture(autouse=True)
def _close_runtimes():
    yield
    while _MADE:
        rt = _MADE.pop()
        if not rt._log.closed:
            rt.close()


def runtime(tmp_path, **k):
    targets = TargetsBook()
    targets.add_pallet("P-1", "epal6", Pose(5.0, 0.0, 0.0))
    targets.add_destination("D-1", Pose(9.0, 0.0, 0.0))
    core = SimpleNamespace(commands=MissionCommands(targets), targets=targets)
    did = {"zero": 0, "lift": [], "lift_pos": 0.10, "render": None}
    hooks = Hooks(zero_wheels=lambda: did.__setitem__("zero", did["zero"] + 1),
                  hold_lift=lambda p: did["lift"].append(p), lift_position=lambda: did["lift_pos"],
                  camera=lambda: (np.zeros((4, 4, 3), np.uint8), did["render"]), grid=lambda: None, scan=lambda: None)
    clock = Clock()
    rt = LiveRuntime(core, hooks, output=tmp_path, port_file=tmp_path / "port.json", dt=DT, clock=clock,
                     sleep=clock.sleep, session_epoch=9, **k)
    _MADE.append(rt)
    return rt, core, did, clock


def step(rt, clock, sim_t, *, speed=0.0, rendered=False, cost=0.001):
    zero = rt.before_step(sim_t)
    clock.t += cost
    rt.after_step(sim_t + DT, odom_speed=speed, yaw_rate=0.0, rendered=rendered)
    return zero


def test_a_stop_zeroes_the_wheels_at_the_next_step_and_latches_the_lift_once(tmp_path):
    rt, core, did, clock = runtime(tmp_path)
    rt.start()
    try:
        assert step(rt, clock, 0.0) is False
        rt.arbiter.stop(clock(), "stop:test")
        did["lift_pos"] = 0.10
        assert step(rt, clock, DT) is True and rt.stopped_in_tick
        did["lift_pos"] = 0.0972  # the load sags: the latch does not follow it
        step(rt, clock, 2 * DT)
        assert did["zero"] == 2 and did["lift"] == [0.10, 0.10]
    finally:
        rt.close()
    assert json.loads((tmp_path / "port.json").read_text())["state"] == "ended"


def test_a_frame_is_one_per_render_time_and_its_latency_counts_from_the_step_start(tmp_path):
    rt, core, did, clock = runtime(tmp_path)
    rt.start()
    try:
        did["render"] = 0.5
        rt.step_start.append((0.5, clock.t - 0.03))  # the step that drew sim 0.5 started 30 ms ago
        step(rt, clock, 0.5 + DT, rendered=True)
        first = rt.store.get("camera_rgb")
        step(rt, clock, 0.5 + 2 * DT, rendered=True)  # the same render time again: no new frame
        assert rt.store.get("camera_rgb")[0] == first[0] == 1 and rt.frame_id == 1
        assert rt.camera_latency[0] == pytest.approx(0.03 + 0.001, abs=1e-6)
    finally:
        rt.close()


def test_feedback_is_stamped_at_the_step_that_made_it(tmp_path):
    rt, core, did, clock = runtime(tmp_path)
    rt.start()
    try:
        step(rt, clock, 0.0, speed=0.2)
        stamp = rt.feedback.stamp_mono
        clock.t += 0.5  # a synchronous call: no step
        rt.tick_start(0.01, False)
        assert rt.feedback.stamp_mono == stamp and not rt.arbiter.feedback_ok  # stale, not re-stamped
    finally:
        rt.close()


def test_the_pacer_holds_the_declared_factor_through_the_runtime(tmp_path):
    rt, core, did, clock = runtime(tmp_path, factor=0.5)
    rt.start()
    try:
        start = clock.t
        for k in range(120):
            step(rt, clock, k * DT)
        assert clock.t - start == pytest.approx(119 * 2 * DT + 0.001, rel=0.01)
    finally:
        rt.close()



def test_a_frame_is_sent_with_its_arrival_and_registered_with_its_pixels_time(tmp_path):
    """Codex M-c round 20: the frame keeps its real arrival time (contract 4.2); the arbiter also gets the start of
    the step its pixels show (a pacing stall in between makes that old), and -inf when the table does not have it."""
    import math
    rt, core, did, clock = runtime(tmp_path, camera_pixel_lag_s=24 * DT)
    rt.start()
    try:
        pixel_start = clock.t - 2.0  # a stall since
        rt.step_start.append((1.0 - 24 * DT, pixel_start))
        rt.step_start.append((1.0, clock.t - 0.05))
        did["render"] = 1.0
        step(rt, clock, 1.0 + DT, rendered=True)
        assert rt.arbiter.frames.acquired_mono(1) == pytest.approx(clock.t)
        assert rt.arbiter.frames.pixel_mono(1) == pytest.approx(pixel_start)
        assert rt.store.get("camera_rgb")[0] == 1
        did["render"] = 50.0  # a render time whose pixels' step is not in the table
        rt.step_start.append((50.0, clock.t - 0.05))
        step(rt, clock, 50.0 + DT, rendered=True)
        assert rt.arbiter.frames.pixel_mono(2) == -math.inf and rt.camera_pixel_unknown == 1
    finally:
        rt.close()


def test_a_capture_seen_before_is_not_given_a_new_id(tmp_path):
    """Codex M-c round 5 P1: render times 1, 2, 1 -- the old capture is not made young."""
    rt, core, did, clock = runtime(tmp_path)
    rt.start()
    try:
        for k, render in enumerate((1.0, 2.0, 1.0)):
            did["render"] = render
            step(rt, clock, 2.0 + k * DT, rendered=True)
        assert rt.frame_id == 2
    finally:
        rt.close()


def test_feedback_keeps_the_time_its_sample_was_made(tmp_path):
    """Codex M-c round 5 P1: the odometry was made before a 0.5 s SLAM wait; its stamp is the earlier time."""
    rt, core, did, clock = runtime(tmp_path)
    rt.start()
    try:
        rt.before_step(0.0)
        made = clock.t
        clock.t += 0.5  # the exchange waits
        rt.after_step(DT, odom_speed=0.1, yaw_rate=0.0, odom_mono=made, rendered=False)
        assert rt.feedback.stamp_mono == made
    finally:
        rt.close()


def test_the_welcome_carries_the_chassis_outline_the_console_draws(tmp_path):
    # without it the page cannot draw the truck or its teleop sweep (C3)
    rt, *_ = runtime(tmp_path)
    try:
        ch = rt.welcome["capabilities"]["chassis"]
        assert ch["nose_from_rear_m"] == pytest.approx(1.29) and ch["rear_overhang_m"] == pytest.approx(0.17)
        assert ch["half_width_m"] == pytest.approx(0.315)
    finally:
        rt.close()


def test_the_camera_model_is_the_isaac_mount_in_the_pages_frame():
    # The page draws the teleop sweep over the picture from camera_model: camera_link (x forward, z up) placed from
    # the rear axle on the floor, intrinsics of the picture as sent (C3 b). The carriage_low_measured mount:
    # base_link (0.619, 0, 0.27), tilted 0.10 rad down, rear axle 0.34 m behind base_link.
    import math

    from sim.isaac.live_runtime import live_camera_model
    from tools.scene_rig import Camera as RigCamera

    mount = RigCamera((0.619, 0.0, 0.27), 0.10).base_from_optical()
    k = SimpleNamespace(width=1280, height=720, fx=640.0, fy=640.0, cx=641.0, cy=361.0)
    m = live_camera_model(k, mount, rear_axle_offset_m=-0.34, sent_width=640, rides_lift=True)
    assert m["width"] == 640 and m["height"] == 360
    assert m["fx"] == pytest.approx(320.0) and m["cy"] == pytest.approx(180.5)
    cam = m["base_from_camera"]
    assert cam["xyz_m"] == pytest.approx([0.959, 0.0, 0.27])
    qx, qy, qz, qw = cam["quaternion_xyzw"]
    assert qx == pytest.approx(0.0, abs=1e-9) and qz == pytest.approx(0.0, abs=1e-9)
    assert 2 * math.atan2(qy, qw) == pytest.approx(0.10)  # positive = looking down, as the page reads it
    assert "lift 0" in cam["frame"]


def test_the_welcome_carries_a_camera_model_when_the_runner_gives_one(tmp_path):
    model = {"width": 640, "height": 360, "fx": 320.0}
    rt, *_ = runtime(tmp_path, camera_model=model)
    try:
        assert rt.welcome["capabilities"]["camera_model"] == model
    finally:
        rt.close()
    rt, *_ = runtime(tmp_path)
    try:
        assert "camera_model" not in rt.welcome["capabilities"]  # no perception camera: none, not a guess
    finally:
        rt.close()


def test_a_carriage_camera_says_it_rides_the_lift():
    # the page adds the lift height to a carriage camera's placement (Codex C3 b review P2)
    from sim.isaac.live_runtime import live_camera_model
    from tools.scene_rig import Camera as RigCamera

    k = SimpleNamespace(width=640, height=480, fx=465.7, fy=465.7, cx=320.0, cy=240.0)
    mount = RigCamera((0.619, 0.0, 0.27), 0.10).base_from_optical()
    assert live_camera_model(k, mount, rear_axle_offset_m=-0.34, sent_width=640, rides_lift=True)["rides_lift"]
    assert not live_camera_model(k, mount, rear_axle_offset_m=-0.34, sent_width=640, rides_lift=False)["rides_lift"]


def test_the_live_video_camera_renders_the_picture_as_sent():
    # The video comes from its own camera (near plane 0.05 m; the perception camera's 1.0 m left the lower third of
    # the picture black, 2026-10-18 demo) rendered at the sent size, so the page's camera_model is that camera's.
    from sim.isaac.live_runtime import live_camera_model, live_video_size
    from tools.scene_rig import Camera as RigCamera

    mount = RigCamera((0.619, 0.0, 0.27), 0.10).base_from_optical()
    for k in (SimpleNamespace(width=1280, height=720, fx=640.0, fy=640.0, cx=640.0, cy=360.0),
              SimpleNamespace(width=640, height=480, fx=465.7, fy=465.7, cx=320.0, cy=240.0)):
        w, h = live_video_size(k, 640)
        m = live_camera_model(k, mount, rear_axle_offset_m=-0.34, sent_width=640, rides_lift=True)
        assert (m["width"], m["height"]) == (w, h) and w <= 640
        assert m["fx"] / m["width"] == pytest.approx(k.fx / k.width)  # the same field of view


def test_the_runner_streams_the_live_video_camera_with_a_near_plane_for_the_forks():
    from pathlib import Path

    source = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()
    head = source[:source.index("from live_runtime import live_video_size")][-400:]
    assert "sys.path.insert(0, repo_root_)" in head  # live_runtime imports console/: the root first (Isaac run 4115)
    setup = source[source.index("live_video_size(perception_calibration, 640)"):][:2500]
    assert '"/LiveVideoCamera"' in setup and "resolution=(video_w_, video_h_)" in setup
    assert "set_clipping_range(near_distance=0.05" in setup
    assert "live_video_camera" in source[source.index("live_cam_ = "):][:200]


def test_the_plan_rides_the_state_when_it_changes_and_again_every_two_seconds():
    # The page draws the plan as a road (contract 6: on change only); a page that connects later gets it at the next
    # refresh. Isaac live sent none at all before (2026-10-18 demo: the plan layer stayed empty in AUTO).
    from sim.isaac.live_runtime import OMIT, PlanFeed

    xs = np.linspace(0.0, 9.0, 1000)
    poses = np.column_stack((xs, np.zeros_like(xs), np.zeros_like(xs)))
    dirs = np.ones(1000)
    dirs[900:] = -1  # a cusp: the page marks it with a node
    feed = PlanFeed(refresh_s=2.0)
    first = feed.update(7, lambda: (poses, dirs), now=0.0)
    assert first["revision"] == 1 and 2 <= len(first["poses"]) <= 202
    assert first["poses"][0] == [0.0, 0.0, 1] and first["poses"][-1] == [9.0, 0.0, -1]
    signs = [p[2] for p in first["poses"]]
    k = signs.index(-1)
    assert first["poses"][k - 1][0] == pytest.approx(xs[899], abs=1e-3) and first["poses"][k][0] == pytest.approx(xs[900], abs=1e-3)
    assert feed.update(7, lambda: (poses, dirs), now=1.0) is OMIT
    assert feed.update(7, lambda: (poses, dirs), now=2.1)["revision"] == 1  # the refresh, same plan
    assert feed.update(8, lambda: (poses[:10], dirs[:10]), now=2.2)["revision"] == 2
    assert feed.update(None, None, now=2.3) is None  # no plan any more: said once
    assert feed.update(None, None, now=2.4) is OMIT


def test_the_runner_hands_the_followed_path_to_the_live_state():
    from pathlib import Path

    source = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()
    call = source[source.index("live.publish(\n"):][:1600]  # the tick's own call (live_failed's is on one line)
    assert "plan_key=" in call and "plan_poses=" in call


def test_a_cancelled_mission_ends_the_mission_but_not_a_live_run():
    # 2026-10-18 demo (ws1 run 4129): the operator cancelled 1.7 s after a submit and the run ended "mission_cancelled";
    # live, a cancel frees the slot (commands: cancelled -> idle at the next apply) and the session goes on.
    from pathlib import Path

    source = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()
    flat = _squash(source)
    assert "if cancel_stop_ and live is None: break" in flat
    # persistent console plan D1: live, the confirmed cancel goes through the mission end (recorded)
    assert "if cancel_stop_ and live is not None:" in flat and 'else: live.end_mission(t, "cancelled", "operator")' in flat
    # plan D4: a cancel that could not recover ends with its own reason
    assert 'if core.commands.status == "recovery_required": live.end_mission(t, "recovery_required"' in flat


def test_the_withdraw_tracking_certificate_is_checked_under_the_obstacle_layer_every_tick():
    # 7610ed5 put the live first-leg replan between `if obstacle is not None:` and the withdraw tracking check, which
    # then ran only inside the replan's branch (found by the persistent-console survey, 2026-10-18).
    import ast
    from pathlib import Path

    tree = ast.parse((Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text())
    parents = {}
    for node in ast.walk(tree):
        for child in ast.iter_child_nodes(node):
            parents[child] = node
    (call,) = [n for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)
               and n.value.startswith("withdraw_tracking_exceeded")] or [
        n for n in ast.walk(tree) if isinstance(n, ast.JoinedStr)
        and any(isinstance(v, ast.Constant) and "withdraw_tracking_exceeded" in str(v.value) for v in n.values)]
    ifs = []
    node = call
    while node in parents:
        node = parents[node]
        if isinstance(node, ast.If):
            ifs.append(ast.unparse(node.test))
    assert ifs[1] == "obstacle is not None", ifs[:3]


# --- the persistent console's mission end (docs/plans/2026-10-18-persistent-live-console.md D1) ---------------------
def ending(tmp_path, verdict=None):
    """A runtime whose mission runs, with the end's hooks recorded in order."""
    order = []
    from types import SimpleNamespace as NS

    from sim.isaac.live_runtime import EndHooks

    end = EndHooks(stop=lambda: order.append("stop"), unlock_wheels=lambda: order.append("unlock"),
                   drain_trackers=lambda: order.append("drain"),
                   judge_delivery=lambda: order.append("judge") or (verdict or NS(ok=True, reason=None,
                                                                                   delivery_error_m=0.02)))
    rt, core, did, clock = runtime(tmp_path, end=end)
    core.abandon_jobs = lambda t: order.append(("abandon", t))
    core.end_reset = lambda t: order.append(("reset", t)) or {"state": {"async_plans": [{"site": "leg"}]}}
    from forklift_core.mission.commands import Submit

    core.commands.queue(Submit("P-1", "D-1", core.targets.revision))
    core.commands.apply(10.0)  # the fixture's arbiter has no feedback (stopping): the submit goes in directly
    core.commands.start()
    return rt, core, order


def test_a_completed_mission_is_judged_on_the_truth_recorded_and_published(tmp_path):
    rt, core, order = ending(tmp_path)
    core.commands.begin_wait(11.0)
    entry = rt.end_mission(20.0, "complete", None)
    assert order == ["stop", ("abandon", 20.0), "unlock", "drain", "judge", ("reset", 20.0)]
    assert entry == {"seq": 1, "mission_id": "m1", "outcome": "done", "reason": None, "started_s": 10.0,
                     "ended_s": 20.0, "pallet_id": "P-1", "destination_id": "D-1"}
    assert core.commands.status == "done" and core.commands.waits == [] and rt.results.last == entry
    seq, message = rt.store.get("missions")
    assert seq == 1 and message == {"t": "missions", "missions_seq": 1, "results": [entry]}
    record = json.loads((tmp_path / "missions" / "m1" / "result.json").read_text())
    assert record["result"] == entry and record["delivery_error_m"] == 0.02
    assert record["clock"] == {"pauses": [], "waits": [[11.0, 20.0]]}
    assert rt.end_mission(21.0, "failed", "late") is None  # ended once
    rt.tick_start(21.0, False)
    assert core.commands.status == "idle" and rt.results.last == entry


def test_a_completed_mission_that_misses_the_destination_fails_with_the_checks_message(tmp_path):
    from types import SimpleNamespace as NS

    rt, core, _ = ending(tmp_path, NS(ok=False, reason="Pallet missed the green destination center",
                                      delivery_error_m=0.11))
    entry = rt.end_mission(20.0, "complete", None)
    assert entry["outcome"] == "failed" and entry["reason"] == "Pallet missed the green destination center"
    assert core.commands.status == "failed" and core.commands.end_reason == entry["reason"]


def test_a_failed_mission_is_not_judged_and_keeps_its_reason(tmp_path):
    rt, core, order = ending(tmp_path)
    entry = rt.end_mission(15.0, "failed", "Pallet was not lifted")
    assert "judge" not in order and entry["outcome"] == "failed" and entry["reason"] == "Pallet was not lifted"


def test_a_cancel_confirmed_standing_is_recorded_as_cancelled(tmp_path):
    from forklift_core.mission.commands import Cancel

    rt, core, order = ending(tmp_path)
    core.commands.queue(Cancel())
    rt.tick_start(12.0, False)
    assert core.commands.status == "cancelling" and rt.results.last is None
    core.commands.cancelled(13.0)  # the core's paused tick: standing confirmed
    entry = rt.end_mission(13.0, "cancelled", "operator")
    assert entry["outcome"] == "cancelled" and entry["reason"] == "operator" and "judge" not in order
    assert core.commands.pauses == []  # handed to the record


def test_a_mission_cancelled_before_it_ran_is_recorded_at_the_tick_start(tmp_path):
    from forklift_core.mission.commands import Cancel, Submit

    rt, core, order = ending(tmp_path)
    rt.end_mission(11.0, "failed", "x")
    rt.tick_start(12.0, False)
    core.commands.queue(Submit("P-1", "D-1", core.targets.revision))
    core.commands.apply(13.0)
    core.commands.queue(Cancel())
    rt.tick_start(14.0, False)  # accepted, nothing moved: cancelled at once
    assert rt.results.last["mission_id"] == "m2" and rt.results.last["outcome"] == "cancelled"
    assert rt.results.last["started_s"] == 13.0 and rt.results.last["ended_s"] == 14.0


def test_no_mission_no_result(tmp_path):
    rt, core, _, _ = runtime(tmp_path)
    assert rt.end_mission(1.0, "failed", "x") is None and rt.results.last is None


def test_the_state_carries_the_last_result_from_the_runtime():
    import inspect

    from sim.isaac import live_runtime

    source = inspect.getsource(live_runtime.LiveRuntime.publish)
    assert 'msg["last_result"] = self.results.last' in source


def test_a_record_that_cannot_be_written_is_marked_in_the_result(tmp_path):
    rt, core, _ = ending(tmp_path)
    (tmp_path / "missions").write_text("not a folder")
    entry = rt.end_mission(20.0, "failed", "x")
    assert entry["record_error"].startswith(("FileExistsError", "NotADirectoryError"))
    assert rt.results.last["record_error"] == entry["record_error"]
    assert rt.store.get("missions")[1]["results"][0]["record_error"] == entry["record_error"]


def test_a_completed_mission_ends_the_mission_but_not_a_live_run():
    """2026-10-18 demo: "한 운반 사이클 끝나니까 잡이 걍 죽는데" -- live, the end of the timers goes through end_mission."""
    from pathlib import Path

    source = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()
    block = _squash(source[source.index("core.lift_and_timers(t, dt):"):][:500])
    assert "if live is None: break" in block
    assert 'live.end_mission(t, "complete", None)' in block and "paused_, requested_speed, curvature = True" in block
    assert "end=LiveEndHooks(stop=live_zero_wheels, unlock_wheels=live_unlock_wheels," in source


def test_a_cancel_confirmed_standing_logs_its_end_once(tmp_path):
    from forklift_core.mission.commands import Cancel

    rt, core, _ = ending(tmp_path)
    core.commands.queue(Cancel())
    rt.tick_start(12.0, False)
    core.commands.cancelled(13.0)
    rt.end_mission(13.0, "cancelled", "operator")
    ends = [r for r in core.commands.log if r["command"] == "end"]
    assert ends == [{"t": 13.0, "command": "end", "accepted": True, "reason": "operator", "mission_id": "m1",
                     "status": "cancelled"}]


def test_a_preparation_ended_by_a_stop_is_collected_at_the_tick_start_once(tmp_path):
    """Persistent console plan D2: the fixture's arbiter has no feedback, so its stop kind is not none."""
    rt, core, order = ending(tmp_path)
    core.commands.status = "accepted"
    core.commands.begin_prepare()
    rt.tick_start(12.0, False)
    last = rt.results.last
    assert last["mission_id"] == "m1" and last["outcome"] == "failed" and last["reason"].startswith("prepare:")
    assert [r["command"] for r in core.commands.log].count("end") == 1
    assert ("abandon", 12.0) in order


def test_a_submit_is_refused_while_the_forks_may_be_in_a_pocket(tmp_path):
    """Codex stage 2 design P1: a cancel during the insertion, then the same pallet submitted again (until D4/D5)."""
    from forklift_core.mission.commands import Submit

    rt, core, did, clock = runtime(tmp_path)
    captured = []
    real_apply = core.commands.apply
    core.commands.apply = lambda t, context: captured.append(context) or real_apply(t, context)
    core.commands.queue(Submit("P-1", "D-1", core.targets.revision))
    rt.tick_start(1.0, False, clear_for_submit=False)
    assert captured[-1].clear_for_submit is False
    rt.tick_start(2.0, False)
    assert captured[-1].clear_for_submit is True


def test_a_preparation_ended_after_the_apply_is_collected_before_the_next_one(tmp_path):
    """Codex stage 2 design round 2 P2: a job_hold failure (timeout, no candidate) comes after the tick's apply; the
    next apply frees the slot first, so it is collected before it -- exactly once, at the end log's time."""
    rt, core, order = ending(tmp_path)
    core.commands.status = "accepted"
    core.commands.begin_prepare()
    core.commands.terminate("failed", 11.5, "prepare:timeout")  # job_hold, late in tick 11.5
    rt.tick_start(12.0, False)
    last = rt.results.last
    assert last["outcome"] == "failed" and last["reason"] == "prepare:timeout" and last["ended_s"] == 11.5
    rt.tick_start(13.0, False)
    assert rt.results.seq == 1 and core.commands.status == "idle"


def test_the_last_missions_records_are_written_off_the_control_thread(tmp_path):
    """Persistent console plan D2: the adoption hands the last mission's records over (MissionBaseline.restore)."""
    rt, core, did, clock = runtime(tmp_path)
    records = {"state": {"async_plans": [{"site": "leg", "wall_s": np.float64(0.25)}]}, "obstacle": {}, "pocket": {}}
    rt.archive("m1", records)
    rt.close()  # waits for the writes
    written = json.loads((tmp_path / "missions" / "m1" / "records.json").read_text())
    assert written["state"]["async_plans"] == [{"site": "leg", "wall_s": 0.25}]


def test_the_runner_prepares_each_live_mission_from_where_the_truck_stands():
    """Persistent console plan D2 (stage 2 design, 3rd edition): the runner's side of the preparation."""
    from pathlib import Path

    source = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()
    # no setup ladder live (a live session starts without a mission), and the worker is required
    assert 'if getattr(args, "live", False):\n' in source and "initial_candidates = []" in source
    assert 'raise ValueError("a live session needs the planning worker' in source
    # each submit is prepared once the control pose is this tick's, and stands while it is
    flat = _squash(source)
    assert ('if live is not None and core.commands.status in ("accepted", "preparing") and not core.job_pending(): '
            'live_prepare(t, rear)') in flat
    assert source.index("live_prepare(t, rear)  #") < source.index("plan_wait_ = core.jobs is not None and core.job_hold(")
    assert 'core.commands.status in ("idle", "done", "failed", "cancelled", "recovery_required"):' in source  # accepted is not paused
    assert 'if live is not None and core.commands.status in ("accepted", "preparing"): plan_wait_ = True' in flat
    assert "submit_refusal=core.cargo.submit_refusal())" in source  # plan D5a: the forks, at the last tick's end
    # the adoption's runner part and the session's baseline, before the console starts
    assert "nonlocal scenario, truth_destination, initial_pallet, pickup_obstacle, pickup_zone, pickup_bounds" in source
    start = source.index("            live.start()  # after the SLAM warm-up")
    before = source[start - 1600:start]  # (the D8 record spool is made between them)
    assert "core.options.observe_bounds = None" in before and "core.baseline = MissionBaseline.capture(core)" in before
    # D-0 live only; the session's end ends a mission still on and skips the batch judgement
    assert source.index('core.targets.add_destination("D-0"') > source.index('    if getattr(args, "live", False):\n        # Persistent console plan D2/D3')
    assert 'live.end_mission(book_clock["t"] if book_clock["t"] is not None else 0.0, "failed", "session_end")' in source
    # the temporary handling D2 replaces is gone
    for gone in ("live_first_replan_", "live_last_rear_", "at_start", "start_submitted", "hall=True"):
        assert gone not in source, gone


def test_the_end_archives_the_missions_records_under_its_own_id(tmp_path):
    """Codex stage 2 impl P2-5: the records moved at the next adoption went under whatever result was last."""
    rt, core, order = ending(tmp_path)
    rt.end_mission(15.0, "failed", "x")
    rt.close()
    written = json.loads((tmp_path / "missions" / "m1" / "records.json").read_text())
    assert written == {"state": {"async_plans": [{"site": "leg"}]}}


def test_a_mission_end_holds_the_lift_where_it_is(tmp_path):
    """Codex D6 design P2-6: the end (also after the physics step) fixes the lift target at the measured height."""
    rt, core, order = ending(tmp_path)
    core.ms = SimpleNamespace(lift_command=0.08)
    rt.hooks.lift_position = lambda: 0.031
    held = []
    rt.hooks.hold_lift = held.append
    rt._lift_moving = True  # the ramp was moving in the last tick
    rt.end_mission(15.0, "failed", "x")
    # within the hold tolerance (0.02 m) of the measured lift: a target that ran ahead stops near it (main
    # 2026-10-19, plan D14 condition 3 amended)
    assert held == [pytest.approx(0.051)] and core.ms.lift_command == pytest.approx(0.051)


def test_a_standing_target_is_kept_at_a_mission_end(tmp_path):
    """Main 2026-10-19 (option 1): a target not moving in the last tick is kept, however far the lift sags below."""
    rt, core, order = ending(tmp_path)
    core.ms = SimpleNamespace(lift_command=0.08)
    rt.hooks.lift_position = lambda: 0.031
    held = []
    rt.hooks.hold_lift = held.append
    rt._lift_moving = False
    rt.end_mission(15.0, "failed", "x")
    assert held == [pytest.approx(0.08)] and core.ms.lift_command == pytest.approx(0.08)


def test_a_loaded_standing_lift_keeps_its_target_through_three_gate_events(tmp_path):
    """Main 2026-10-19: a pallet on the forks sags the lift 36 mm (above a = 0.02 m); three STOPs in a row while it
    stands must not lower it."""
    rt, core, did, clock = runtime(tmp_path)
    core.ms = SimpleNamespace(lift_command=0.20)
    did["lift_pos"] = 0.164
    rt.start()
    try:
        t = 0.0
        for _ in range(3):
            rt.arbiter.stop(clock(), "stop:test")
            step(rt, clock, t)
            assert did["lift"][-1] == pytest.approx(0.20)
            rt._lift_latch = None  # the gate lifted between the events
            t += DT
        assert core.ms.lift_command == pytest.approx(0.20)
    finally:
        rt.close()


def test_a_standing_lifts_sag_is_not_taken_for_a_new_target_at_a_gate_or_an_end(tmp_path):
    """Main 2026-10-19: a loaded lift standing 9 mm below its target (the drive's gravity sag) keeps the target at
    the gate's latch and at a mission's end -- the measured height would lower it by the sag at every event."""
    rt, core, did, clock = runtime(tmp_path)
    core.ms = SimpleNamespace(lift_command=0.20)
    did["lift_pos"] = 0.191
    rt.start()
    try:
        rt.arbiter.stop(clock(), "stop:test")
        step(rt, clock, 0.0)
        assert did["lift"][-1] == pytest.approx(0.20)
    finally:
        rt.close()
    (tmp_path / "end").mkdir()
    rt2, core2, order = ending(tmp_path / "end")
    core2.ms = SimpleNamespace(lift_command=0.20)
    rt2.hooks.lift_position = lambda: 0.191
    held = []
    rt2.hooks.hold_lift = held.append
    rt2.end_mission(15.0, "failed", "x")
    assert held == [pytest.approx(0.20)] and core2.ms.lift_command == pytest.approx(0.20)


def test_the_state_carries_the_scene_and_a_damaged_scene_refuses_submits(tmp_path):
    from forklift_core.mission.commands import Submit

    rt, core, did, clock = runtime(tmp_path)
    rt.scene = {"damaged": {"reason": "Pallet displaced before pickup", "t": 3.0}}
    captured = []
    real_apply = core.commands.apply
    core.commands.apply = lambda t, context: captured.append(context) or real_apply(t, context)
    core.commands.queue(Submit("P-1", "D-1", core.targets.revision))
    rt.tick_start(4.0, False)
    assert captured[-1].scene_damaged is True
    import inspect

    from sim.isaac import live_runtime

    assert 'msg["scene"] = self.scene' in inspect.getsource(live_runtime.LiveRuntime.publish)


def test_a_live_missions_failure_in_its_tick_ends_the_mission_and_the_tick_finishes_once():
    """Persistent console plan D6 (stage 3 design): one catch around the tick body; only live mission and scene
    reasons are handled, the output and the physics step run once."""
    import ast
    from pathlib import Path

    source = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()
    tree = ast.parse(source)
    (loop,) = [n for n in ast.walk(tree) if isinstance(n, ast.For)
               and "range(int(120 * args.max_sim_seconds))" in ast.unparse(n.iter)]
    assert ast.unparse(loop.body[0]) == "live_tick_out_ = live_stepped_ = False"
    (body,) = [n for n in loop.body if isinstance(n, ast.Try)]
    (handler,) = body.handlers
    assert ast.unparse(handler.type) == "RuntimeError"
    text = ast.unparse(handler)
    assert ("if live is None or isinstance(exc_, SessionFailure) or classify_failure(str(exc_)) == 'session':\n"
            "        raise") in text
    assert "live_failed(t, str(exc_), live_tick_out_, live_stepped_" in text
    # the flags follow the output and the physics step, and the step's flag comes before capture_stepped
    inside = source[source.index("            try:\n                if early_ports:"):]
    assert inside.index("live_tick_out_ = True") > inside.index("out_ = live.tick_end(")
    step_at = inside.index('stepper["fn"]((pocket_frames and step % 12 == 0) or capturing_)')
    assert step_at < inside.index("live_stepped_ = True") < inside.index("core.capture_stepped(")
    failed = source[source.index("        def live_failed("):source.index("        for step in (")]
    assert failed.index("live.end_mission(t_") < failed.index("if not ticked_:") < failed.index("if not stepped__:")


def test_a_supervised_runtime_offers_the_scene_reset_and_binds_the_port_it_is_given(tmp_path):
    """Persistent console plan D7/D9: a fixed port across epochs; the reset only where a supervisor restarts it."""
    import socket

    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        free = probe.getsockname()[1]
    rt, core, did, clock = runtime(tmp_path, port=free, supervised=True)
    assert rt.welcome["capabilities"]["sim_reset"] is True and rt.arbiter.sim_reset_allowed
    assert rt.start() == free
    (tmp_path / "plain").mkdir()
    plain, *_ = runtime(tmp_path / "plain")
    assert plain.welcome["capabilities"]["sim_reset"] is False and not plain.arbiter.sim_reset_allowed


def test_the_runner_leaves_its_loop_on_a_session_request_with_its_exit_code():
    from pathlib import Path

    source = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()
    flat = _squash(source)
    assert "for step in (itertools.count() if live is not None else range(int(120 * args.max_sim_seconds))):" in flat
    head = flat[flat.index("if live is not None and live.arbiter.session_request is not None:"):][:400]
    assert 'state["live_session_request"] = live.arbiter.session_request break' in head
    assert '{"end": 0, "reset": 3}' in flat and 'raise SystemExit(state.get("live_exit_code", 0) if state["success"] else 1)' in flat


def test_a_live_runtime_judges_requests_by_the_ledger(tmp_path):
    rt, core, did, clock = runtime(tmp_path)
    assert rt.arbiter.config.request_tokens is True


def test_the_camera_latency_keeps_a_window_and_counts_every_frame(tmp_path):
    """Persistent console plan D8: a session left up for hours keeps the last hour of latencies, not every one."""
    from sim.isaac.live_runtime import LatencyWindow

    window = LatencyWindow(maxlen=3)
    for v in (0.05, 0.40, 0.01, 0.02, 0.03):
        window.append(v)
    assert list(window) == [0.01, 0.02, 0.03]
    report = window.report()
    assert report["n"] == 5 and report["max"] == 0.40  # the whole session
    assert report["window_n"] == 3 and report["window_median"] == 0.02
    rt, *_ = runtime(tmp_path)
    assert isinstance(rt.camera_latency, LatencyWindow) and isinstance(rt.camera_pixel_latency, LatencyWindow)
    assert rt.camera_latency.maxlen == 36_000


def test_the_arbiters_events_are_logged_once_and_then_let_go(tmp_path):
    rt, core, did, clock = runtime(tmp_path)
    rt.start()
    try:
        rt.arbiter.stop(clock(), "stop:test")
        step(rt, clock, 0.0)
        rt.tick_start(0.0, near_active=False)
        assert rt.arbiter.events == []
        step(rt, clock, DT)
        rt.tick_start(DT, near_active=False)
    finally:
        rt.close()
    logged = [json.loads(line) for line in (tmp_path / "live_log.jsonl").read_text().splitlines()]
    stops = [(e["t"], e["event"]) for e in logged if e.get("arbiter") and e.get("reason") == "stop:test"]
    assert stops and len(stops) == len(set(stops))  # each boundary event logged, and only once


def test_the_live_log_rolls_over_and_keeps_the_newest_files(tmp_path):
    rt, *_ = runtime(tmp_path, log_rotate_bytes=200, log_keep=2)
    for i in range(40):
        rt.log({"event": "x", "i": i})
    rt.close()
    names = sorted(p.name for p in tmp_path.glob("live_log.jsonl*"))
    assert names == ["live_log.jsonl", "live_log.jsonl.1", "live_log.jsonl.2"]
    kept = [json.loads(line) for name in names[::-1][:-1] + ["live_log.jsonl"]  # oldest first
            for line in (tmp_path / name).read_text().splitlines()]
    xs = [e["i"] for e in kept if e.get("event") == "x"]
    assert xs == list(range(xs[0], 40)) and xs[0] > 0  # the newest entries, in order, the oldest gone
    assert all(p.stat().st_size < 200 + 200 for p in tmp_path.glob("live_log.jsonl.*"))


def test_the_log_rolls_over_safely_while_a_record_writer_also_logs(tmp_path):
    """The mission records' writer thread logs its errors (plan D2): the rollover is one owner's, so no line is lost
    or cut while another thread writes."""
    import threading

    rt, *_ = runtime(tmp_path, log_rotate_bytes=500, log_keep=1000)

    def writer(tag):
        for i in range(300):
            rt.log({"event": "w", "tag": tag, "i": i})

    threads = [threading.Thread(target=writer, args=(k,)) for k in range(3)]
    for th in threads:
        th.start()
    writer(9)
    for th in threads:
        th.join()
    rt.close()
    lines = [json.loads(line) for p in tmp_path.glob("live_log.jsonl*") for line in p.read_text().splitlines()]
    seen = {(e["tag"], e["i"]) for e in lines if e.get("event") == "w"}
    assert len(seen) == 4 * 300


def test_the_rolled_log_says_how_often_it_rolled_so_a_reader_knows_what_is_gone(tmp_path):
    from forklift_core.live.spool import read_live_log

    rt, *_ = runtime(tmp_path, log_rotate_bytes=200, log_keep=50)
    for i in range(10):
        rt.log({"event": "x", "i": i})
    rt.close()
    index = json.loads((tmp_path / "live_log.index.json").read_text())
    assert index["keep"] == 50 and index["rotations"] >= 1
    assert [e["i"] for e in read_live_log(tmp_path) if e.get("event") == "x"] == list(range(10))
    rt2, *_ = runtime(tmp_path / "x" if (tmp_path / "x").mkdir() is None else None, log_rotate_bytes=200, log_keep=1)
    for i in range(20):
        rt2.log({"event": "x", "i": i})
    rt2.close()
    with pytest.raises(ValueError, match="rolled"):
        read_live_log(tmp_path / "x")
