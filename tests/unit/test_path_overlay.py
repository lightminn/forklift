"""The planned-path and destination display composited into video frames instead of drawn into the scene
(docs/plans/2026-11-14-path-display-perception-leak.md 2): projection, near-plane clipping, copies only, the mode
rules and the moving cameras' pose history."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))

import path_overlay as po  # noqa: E402


class FakeCamera:
    """A camera looking along world +x from (0, 0, h): ROS optical axes x right (world -y), y down (world -z),
    z forward (world +x)."""

    def __init__(self, height=1.0, width=640, rows=480, f=320.0, prim_path="/World/Cam", x=0.0):
        self.prim_path = prim_path
        self._res = (width, rows)
        self._k = np.array([[f, 0, width / 2], [0, f, rows / 2], [0, 0, 1.0]])
        self._pos = np.array([x, 0.0, height])
        # rotation camera->world, columns = camera axes in world: x_c=-y_w, y_c=-z_w, z_c=+x_w
        r = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
        self._q = po.quaternion_wxyz_from_matrix(r)
        self.pose_calls = 0

    def get_world_pose(self, camera_axes="world"):
        assert camera_axes == "ros"
        self.pose_calls += 1
        return self._pos.copy(), self._q.copy()

    def get_intrinsics_matrix(self):
        return self._k.copy()

    def get_resolution(self):
        return self._res


@pytest.fixture(autouse=True)
def _clean():
    po.reset()
    yield
    po.reset()


def test_quaternion_round_trip():
    r = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=float)
    q = po.quaternion_wxyz_from_matrix(r)
    assert np.allclose(po.matrix_from_quaternion_wxyz(q), r)


def test_a_floor_point_ahead_projects_below_the_centre():
    cam = FakeCamera(height=1.0)
    pos, q = cam.get_world_pose("ros")
    k = po.Intrinsics(320.0, 320.0, 320.0, 240.0)
    runs = po.project_polyline(np.array([[2.0, 0.0, 0.0], [2.0, -1.0, 0.0]]), pos, q, k)
    assert len(runs) == 1
    (u0, v0), (u1, v1) = runs[0]
    assert (u0, v0) == pytest.approx((320.0, 240.0 + 320.0 * 1.0 / 2.0))  # 1 m below at 2 m ahead
    assert u1 == pytest.approx(320.0 + 320.0 * 1.0 / 2.0)  # world -y is image right


def test_points_behind_or_at_the_camera_are_clipped_at_the_near_plane():
    cam = FakeCamera(height=1.0)
    pos, q = cam.get_world_pose("ros")
    k = po.Intrinsics(320.0, 320.0, 320.0, 240.0)
    # from 1 m behind to 2 m ahead: the part behind z = 0.05 m is cut, the rest kept
    runs = po.project_polyline(np.array([[-1.0, 0.0, 1.0], [2.0, 0.0, 1.0]]), pos, q, k)
    assert len(runs) == 1
    (u0, v0), (u1, v1) = runs[0]
    assert np.isfinite([u0, v0, u1, v1]).all()
    assert abs(u0) < 1e5 and abs(v0) < 1e5  # no blow-up near z = 0
    # a point exactly at the camera plane and one just in front: no division by ~0
    runs = po.project_polyline(np.array([[0.0, 0.0, 0.0], [1e-6, 0.0, 0.0]]), pos, q, k)
    assert runs == []
    # a polyline that leaves and comes back splits into two runs
    runs = po.project_polyline(np.array([[2.0, 0.0, 0.0], [-1.0, 0.0, 0.0], [2.0, 1.0, 0.0]]), pos, q, k)
    assert len(runs) == 2


def test_thin_keeps_the_ends_and_caps_the_count():
    pts = np.column_stack([np.linspace(0, 10, 1001), np.zeros(1001), np.zeros(1001)])
    out = po.thin(pts, 200)
    assert len(out) <= 200 and np.allclose(out[0], pts[0]) and np.allclose(out[-1], pts[-1])
    assert len(po.thin(pts[:5], 200)) == 5


def test_compose_draws_on_a_copy_and_never_on_the_frame():
    cam = FakeCamera()
    po.add_path("Transport", [(1.0, 0.0, 0.0), (3.0, 0.0, 0.0)], (1.0, 0.65, 0.04))
    frame = np.zeros((480, 640, 4), dtype=np.uint8)
    view = frame[:, :, :3]
    out = po.compose(view, cam)
    assert out is not view and not np.shares_memory(out, frame)
    assert frame.sum() == 0  # the sensor buffer untouched
    assert (out == np.array([255, 166, 10], dtype=np.uint8)).all(axis=2).sum() > 50


def test_compose_reads_nothing_while_the_list_is_empty_or_the_frame_missing():
    cam = FakeCamera()
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert po.compose(frame, cam) is frame and cam.pose_calls == 0
    po.add_path("Approach", [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], (0.05, 0.45, 1.0))
    assert po.compose(None, cam) is None and cam.pose_calls == 0


def test_a_name_replaces_its_earlier_line():
    po.add_path("Transport", [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], (1.0, 0.65, 0.04))
    po.add_path("Transport", [(1.0, 1.0, 0.0), (2.0, 1.0, 0.0)], (1.0, 0.65, 0.04))
    items = po.items()
    assert list(items) == ["Transport"] and items["Transport"]["points"][0][1] == pytest.approx(1.0)
    assert items["Transport"]["points"][0][2] == pytest.approx(po.LINE_Z_M)


def test_a_smaller_image_scales_the_intrinsics():
    cam = FakeCamera(width=1280, rows=960, f=640.0)
    po.add_path("Approach", [(2.0, -0.2, 0.0), (2.0, 0.2, 0.0)], (0.0, 0.0, 1.0))
    full = po.compose(np.zeros((960, 1280, 3), dtype=np.uint8), cam)
    half = po.compose(np.zeros((480, 640, 3), dtype=np.uint8), cam)
    rows_full = np.nonzero(full[:, :, 2].any(axis=1))[0]
    rows_half = np.nonzero(half[:, :, 2].any(axis=1))[0]
    assert rows_half.mean() == pytest.approx(rows_full.mean() / 2, abs=2.0)


def test_line_width_follows_the_image_width():
    assert po.line_width(1280) == 3 and po.line_width(640) == 2 and po.line_width(100) == 1


def test_the_ring_is_a_closed_circle_at_the_ring_height():
    po.add_ring("Destination", 2.0, 1.0, (0.01, 0.9, 0.04))
    item = po.items()["Destination"]
    assert item["closed"] and item["points"][0][2] == pytest.approx(po.RING_Z_M)
    r = np.hypot(item["points"][:, 0] - 2.0, item["points"][:, 1] - 1.0)
    assert np.allclose(r, po.RING_RADIUS_M)


def test_modes_and_the_diagnostic_variable():
    assert po.effective_mode("overlay", {}) == "overlay"
    assert po.effective_mode("off", {}) == "off"
    with pytest.raises(ValueError, match="diagnostic"):
        po.effective_mode("stage", {})
    assert po.effective_mode("stage", {po.DIAGNOSTIC_ENV: "stage"}) == "stage"
    assert po.effective_mode("overlay", {po.DIAGNOSTIC_ENV: "stage"}) == "stage"
    assert po.effective_mode("overlay", {po.DIAGNOSTIC_ENV: "both"}) == "both"
    with pytest.raises(ValueError):
        po.effective_mode("overlay", {po.DIAGNOSTIC_ENV: "yes"})
    assert po.draws_stage("stage") and po.draws_stage("both") and not po.draws_stage("overlay")
    assert po.draws_overlay("overlay") and po.draws_overlay("both") and not po.draws_overlay("stage")
    # what a record says: stage as before the fix (nothing), the rest by name
    assert po.recorded_value("stage") is None and po.recorded_value("overlay") == "overlay"
    assert po.recorded_value("both") == "both" and po.recorded_value("off") == "off"


def test_a_moving_camera_is_drawn_from_its_pose_at_the_frame_time():
    cam = FakeCamera(prim_path="/World/Chase")
    po.add_path("Approach", [(3.0, -0.5, 0.0), (3.0, 0.5, 0.0)], (0.0, 0.0, 1.0))
    po.register_moving(cam)
    po.note_moving(10.0)
    cam._pos = np.array([1.0, 0.0, 1.0])  # 1 m closer by the time the frame is written
    po.note_moving(10.2)
    late = po.compose(np.zeros((480, 640, 3), dtype=np.uint8), cam, frame_time_s=10.0)
    now = po.compose(np.zeros((480, 640, 3), dtype=np.uint8), cam)
    row_late = np.nonzero(late[:, :, 2].any(axis=1))[0].mean()
    row_now = np.nonzero(now[:, :, 2].any(axis=1))[0].mean()
    assert row_late == pytest.approx(240 + 320 * 1.0 / 3.0, abs=3)  # drawn from 3 m away, as rendered
    assert row_now == pytest.approx(240 + 320 * 1.0 / 2.0, abs=3)
    assert math.isclose(po.pose_history_span_s(cam), 0.2)


def test_note_moving_does_nothing_without_lines():
    cam = FakeCamera(prim_path="/World/Chase")
    po.register_moving(cam)
    po.note_moving(1.0)
    assert cam.pose_calls == 0


def test_note_moving_stamps_with_the_clock_when_no_time_is_given():
    cam = FakeCamera(prim_path="/World/Robot")
    po.add_path("Approach", [(3.0, -0.5, 0.0), (3.0, 0.5, 0.0)], (0.0, 0.0, 1.0))
    po.register_moving(cam)
    po.note_moving()  # no clock: nothing kept
    assert cam.pose_calls == 0
    now = iter([5.0, 5.5])
    po.set_clock(lambda: next(now))
    po.note_moving()
    po.note_moving()
    assert math.isclose(po.pose_history_span_s(cam), 0.5)


def test_the_isaac_clock_is_core_nodes_sim_time(monkeypatch):
    # never import isaacsim here: outside a running app it bootstraps Kit (the runner calls this after SimulationApp)
    import types

    interface = types.SimpleNamespace(get_sim_time=lambda: 12.5)
    bindings = types.ModuleType("isaacsim.core.nodes.bindings")
    bindings._isaacsim_core_nodes = types.SimpleNamespace(acquire_interface=lambda: interface)
    for name in ("isaacsim", "isaacsim.core", "isaacsim.core.nodes"):
        monkeypatch.setitem(sys.modules, name, types.ModuleType(name))
    monkeypatch.setitem(sys.modules, "isaacsim.core.nodes.bindings", bindings)
    assert po.isaac_sim_clock()() == 12.5


def test_contrast_draws_one_pixel_magenta_and_reset_clears_it():
    cam = FakeCamera()
    po.add_path("Transport", [(1.0, 0.0, 0.0), (3.0, 0.0, 0.0)], (1.0, 0.65, 0.04))
    po.set_contrast(True)
    out = po.compose(np.zeros((480, 640, 3), dtype=np.uint8), cam)
    magenta = (out == np.array(po.CONTRAST_COLOUR, dtype=np.uint8)).all(axis=2)
    assert magenta.sum() > 50 and magenta.any(axis=1).sum() == magenta.sum()  # one pixel per row on a vertical line
    po.reset()
    assert not po._contrast["on"]


def test_a_clock_that_goes_back_restarts_the_history():
    cam = FakeCamera(prim_path="/World/Robot")
    po.add_path("Approach", [(3.0, -0.5, 0.0), (3.0, 0.5, 0.0)], (0.0, 0.0, 1.0))
    po.register_moving(cam)
    for t in (10.0, 10.1, 10.2):
        po.note_moving(t)
    po.note_moving(0.05)  # a reset
    assert po.pose_history_span_s(cam) == 0.0
    po.note_moving(0.1)
    assert math.isclose(po.pose_history_span_s(cam), 0.05)


class BrokenCamera(FakeCamera):
    def get_intrinsics_matrix(self):
        raise RuntimeError("no render product yet")


def test_a_compose_failure_skips_the_lines_and_never_raises(capsys):
    cam = BrokenCamera(prim_path="/World/Live")
    po.add_path("Transport", [(1.0, 0.0, 0.0), (3.0, 0.0, 0.0)], (1.0, 0.65, 0.04))
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    for _ in range(3):
        assert po.compose(frame, cam) is frame  # the frame as it was: no lines, no exception
    st = po.stats()
    assert st["compose_failures"] == 3 and "no render product yet" in st["last_error"]
    err = capsys.readouterr().err
    assert err.count("PATH_OVERLAY_WARN compose_failures") == 1  # the first time, then every WARN_EVERY-th


def test_a_note_failure_is_counted_not_raised():
    class Bad(FakeCamera):
        def get_world_pose(self, camera_axes="world"):
            raise RuntimeError("prim gone")

    po.add_path("Approach", [(3.0, -0.5, 0.0), (3.0, 0.5, 0.0)], (0.0, 0.0, 1.0))
    po.register_moving(Bad(prim_path="/World/Chase"))
    po.note_moving(1.0)
    assert po.stats()["note_failures"] == 1


def test_the_pixel_lag_draws_from_the_earlier_pose():
    cam = FakeCamera(prim_path="/World/Robot")
    po.add_path("Approach", [(3.0, -0.5, 0.0), (3.0, 0.5, 0.0)], (0.0, 0.0, 1.0))
    po.register_moving(cam)
    po.note_moving(10.0)
    cam._pos = np.array([1.0, 0.0, 1.0])
    po.note_moving(10.2)
    po.set_pixel_lag(0.2)
    lagged = po.compose(np.zeros((480, 640, 3), dtype=np.uint8), cam, frame_time_s=10.2)
    row = np.nonzero(lagged[:, :, 2].any(axis=1))[0].mean()
    assert row == pytest.approx(240 + 320 * 1.0 / 3.0, abs=3)  # the pose at 10.0 s, 3 m from the line
    assert po.stats()["pixel_lag_s"] == 0.2
    with pytest.raises(ValueError):
        po.set_pixel_lag(-0.1)


def test_a_frame_time_on_another_clock_is_warned_not_matched_to_an_end(capsys):
    cam = FakeCamera(prim_path="/World/Robot")
    po.add_path("Approach", [(3.0, -0.5, 0.0), (3.0, 0.5, 0.0)], (0.0, 0.0, 1.0))
    po.register_moving(cam)
    po.note_moving(10.0)
    cam._pos = np.array([1.0, 0.0, 1.0])
    po.note_moving(10.2)
    # a frame stamped on a wall-clock-like axis: far outside the kept 10.0-10.2 s
    out = po.compose(np.zeros((480, 640, 3), dtype=np.uint8), cam, frame_time_s=1794630000.0)
    row = np.nonzero(out[:, :, 2].any(axis=1))[0].mean()
    assert row == pytest.approx(240 + 320 * 1.0 / 2.0, abs=3)  # the current pose, not the history's first
    assert po.stats()["clock_mismatches"] == 1
    assert "PATH_OVERLAY_WARN clock_mismatches" in capsys.readouterr().err
    # within the slack it is matched as before
    po.compose(np.zeros((480, 640, 3), dtype=np.uint8), cam, frame_time_s=10.2 + po.CLOCK_SLACK_S / 2)
    assert po.stats()["clock_mismatches"] == 1


def test_compose_and_note_are_timed_when_the_live_timers_are_on():
    from forklift_core.perf import TIMERS

    cam = FakeCamera(prim_path="/World/Robot")
    po.add_path("Approach", [(3.0, -0.5, 0.0), (3.0, 0.5, 0.0)], (0.0, 0.0, 1.0))
    po.register_moving(cam)
    was = TIMERS.enabled
    TIMERS.enabled = True
    try:
        before = dict(TIMERS._count)
        po.note_moving(1.0)
        po.compose(np.zeros((480, 640, 3), dtype=np.uint8), cam)
        assert TIMERS._count.get("overlay.note_moving", 0) == before.get("overlay.note_moving", 0) + 1
        assert TIMERS._count.get("overlay.compose", 0) == before.get("overlay.compose", 0) + 1
    finally:
        TIMERS.enabled = was


def test_a_fixed_camera_never_reads_a_history_and_re_registering_starts_fresh():
    cam = FakeCamera(prim_path="/World/Cam")
    po.add_path("Approach", [(3.0, -0.5, 0.0), (3.0, 0.5, 0.0)], (0.0, 0.0, 1.0))
    po.register_moving(cam)
    po.note_moving(10.0)
    po._moving.clear()  # the same prim path, no longer a moving camera
    po.set_pixel_lag(5.0)
    po.compose(np.zeros((480, 640, 3), dtype=np.uint8), cam, frame_time_s=10.0)
    assert po.stats()["clock_mismatches"] == 0  # no history, no lag, no mismatch for a fixed camera
    po.register_moving(cam)
    assert po.pose_history_span_s(cam) == 0.0 and "/World/Cam" not in po._history


def test_a_failing_timer_is_caught_too(monkeypatch):
    def broken(name):
        raise RuntimeError("timer broken")

    monkeypatch.setattr(po, "_timer", broken)
    cam = FakeCamera(prim_path="/World/Robot")
    po.add_path("Approach", [(3.0, -0.5, 0.0), (3.0, 0.5, 0.0)], (0.0, 0.0, 1.0))
    po.register_moving(cam)
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    assert po.compose(frame, cam) is frame
    po.note_moving(1.0)
    st = po.stats()
    assert st["compose_failures"] == 1 and st["note_failures"] == 1


def _rows(path):
    import json

    return [json.loads(line) for line in path.read_text().splitlines()]


def test_the_display_log_times_each_line_on_the_live_frames_axis(tmp_path):
    log = tmp_path / "path_display.jsonl"
    now = {"t": 100.0}
    po.set_clock(lambda: now["t"])
    po.start_event_log(log, "overlay", 0.0917)
    po.add_path("Approach", [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], (0.05, 0.45, 1.0))  # before drive's origin: waits
    assert len(_rows(log)) == 1
    po.set_time_origin(99.5)
    now["t"] = 103.25
    po.add_ring("/World/Destination", 2.0, 1.0, (0.01, 0.9, 0.04))
    po.add_path("Transport", [(2.0, 0.0, 0.0), (4.0, 0.0, 0.0)], (1.0, 0.65, 0.04))
    header, approach, ring, transport = _rows(log)
    assert header == {"t": "header", "schema": "path_display/1", "mode": "overlay", "pixel_lag_s": 0.0917,
                      "time_axis": "world.current_time - initial_time", "line_width_px_at_1280": 3}
    assert approach["t"] == "display" and approach["time_s"] == pytest.approx(0.5) and approach["kind"] == "path"
    assert approach["points"][0] == [1.0, 0.0, po.LINE_Z_M] and approach["colour"] == pytest.approx([0.05, 0.45, 1.0], abs=0.003)
    assert ring["kind"] == "ring" and ring["closed"] and ring["width_scale"] == 2.0 and ring["time_s"] == pytest.approx(3.75)
    assert len(ring["points"]) == po.RING_SEGMENTS and ring["points"][0][2] == po.RING_Z_M
    assert transport["name"] == "Transport" and "clock_s" not in transport


def test_rows_never_given_an_origin_are_written_with_a_null_time(tmp_path):
    log = tmp_path / "path_display.jsonl"
    po.set_clock(lambda: 5.0)
    po.start_event_log(log, "both", 0.0)
    po.add_path("Approach", [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], (0.05, 0.45, 1.0))
    assert po.stats()["event_rows_pending"] == 1
    po.close_event_log()
    rows = _rows(log)
    assert rows[1]["time_s"] is None and rows[1]["name"] == "Approach" and po.stats()["event_rows_pending"] == 0


def test_without_a_log_nothing_is_written_and_a_bad_path_never_raises(tmp_path):
    po.add_path("Approach", [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], (0.05, 0.45, 1.0))  # no log started (a replay)
    po.reset()
    po.start_event_log(tmp_path / "missing_dir" / "path_display.jsonl", "overlay", 0.0)
    po.set_time_origin(0.0)
    po.add_path("Approach", [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)], (0.05, 0.45, 1.0))
    assert po.stats()["event_failures"] >= 1
