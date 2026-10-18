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
    call = source[source.index("live.publish("):][:1600]
    assert "plan_key=" in call and "plan_poses=" in call


def test_the_tick_hands_the_start_check_to_the_mission_commands(tmp_path):
    # 2026-10-18 demo: driven 7 m by hand, a submit ran the observe leg planned from the start and ended the run with
    # "Tracking failed in observe". Without grid planning (nothing to replan with) the runner says the truck is off
    # the start and the submit is refused (commands: not_at_start).
    rt, core, did, clock = runtime(tmp_path)
    seen = []
    apply_ = core.commands.apply
    core.commands.apply = lambda t, context: seen.append(context) or apply_(t, context)
    rt.start()
    try:
        rt.tick_start(0.0, False, at_start=False)
        rt.tick_start(0.1, False)
        assert [c.at_start for c in seen] == [False, True]
    finally:
        rt.close()


def test_the_runner_replans_the_first_leg_from_where_a_submit_finds_the_truck():
    # With grid planning a submit off the start is accepted and the first leg replanned from the truck (the
    # obstacle recovery's own replan); a replan that installs no path from the truck fails the mission, not the run.
    from pathlib import Path

    source = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()
    head = source[:source.index("live.tick_start(t, core.near_active()")][-1800:]
    assert "_poses[0]" in head and "live_last_rear_" in head
    assert "at_start=live_at_start_" in source[source.index("live.tick_start(t, core.near_active()"):][:120]
    replan = source[source.index('live_first_replan_["due"] and'):][:1600]
    assert "core.blocked_replan(t, rear, 0.0, 0.0, hall=True)" in replan
    assert 'core.commands.finish("failed", t)' in replan
