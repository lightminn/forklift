"""CPU-only Isaac adapter contracts; no simulator SDK or rendered evidence."""

import importlib.util
import math
from dataclasses import asdict
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.pocket_observation import Pocket, PocketObservation
from forklift_core.planning.pallet_mission import PalletSite

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "perception_adapter", ROOT / "sim/isaac/perception_adapter.py"
)
MODULE = importlib.util.module_from_spec(SPEC)


@pytest.fixture(scope="module", autouse=True)
def load_adapter():
    SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize(
    "base_yaw, expected",
    [
        (0, (4, 4, 0.25)),
        (math.pi / 2, (3, 5, math.pi / 2 + 0.25)),
    ],
)
def test_estimated_world_site_rotates_then_translates(base_yaw, expected):
    site = MODULE.estimate_world_pallet_site((1, 0), 0.25, (3, 4), base_yaw)
    assert isinstance(site, PalletSite)
    assert (site.x_m, site.y_m, site.yaw_rad) == pytest.approx(expected)


@pytest.mark.parametrize(
    "base_yaw, insertion_yaw, expected",
    [
        (math.pi, 0.1, -math.pi + 0.1),
        (-math.pi, -0.1, math.pi - 0.1),
        (-math.pi, 0, math.pi),
        (math.pi, 0, math.pi),
    ],
)
def test_estimated_world_site_wraps_yaw(base_yaw, insertion_yaw, expected):
    site = MODULE.estimate_world_pallet_site((0, 0), insertion_yaw, (0, 0), base_yaw)
    assert -math.pi < site.yaw_rad <= math.pi
    assert site.yaw_rad == pytest.approx(expected)


class FakeCamera:
    def __init__(self):
        self.rgba = np.zeros((480, 640, 4), dtype=np.uint8)
        self.rgba[:, ::2, :3] = 200
        self.depth = np.full((480, 640), 2.0, dtype=np.float32)
        self.reads = 0

    def get_rgba(self):
        return self.rgba

    def get_current_frame(self):
        raise AssertionError("Capture must read depth directly, not the frame cache")

    def get_depth(self):
        self.reads += 1
        return self.depth


def capture(camera, **kwargs):
    return MODULE.capture_scene_input(
        camera, MODULE.default_base_from_optical(), 123, **kwargs
    )


def test_capture_reads_direct_depth_instead_of_stale_frame_cache():
    class CachedCamera(FakeCamera):
        def get_current_frame(self):
            return {"distance_to_image_plane": np.full((480, 640), 9.0)}

    camera = CachedCamera()
    scene, _, attempts = capture(camera, max_attempts=1)
    np.testing.assert_array_equal(scene.depth_m, camera.depth)
    assert attempts == camera.reads == 1


def test_capture_squeezes_trailing_singleton_depth_axis_without_reordering():
    camera = FakeCamera()
    expected = np.arange(480 * 640, dtype=np.float32).reshape(480, 640) + 1
    camera.depth = expected[:, :, None].copy()
    scene, diagnostics, attempts = capture(camera, max_attempts=1)
    np.testing.assert_array_equal(scene.depth_m, expected)
    assert diagnostics.finite_positive_count == 480 * 640
    assert attempts == camera.reads == 1
    camera.depth[:] = 0
    np.testing.assert_array_equal(scene.depth_m, expected)


@pytest.mark.parametrize("shape", [(640, 480, 1), (480, 640, 2), (1, 480, 640, 1)])
def test_capture_rejects_incompatible_depth_axes_without_reshaping(shape):
    camera = FakeCamera()
    camera.depth = np.ones(shape, dtype=np.float32)
    with pytest.raises(MODULE.CaptureFailure) as error:
        capture(camera, max_attempts=3)
    assert error.value.reason == "depth_not_ready"
    assert camera.reads == 3


def test_xyzw_to_wxyz_known_value_and_round_trip():
    original = (-0.5, 0.5, -0.5, 0.5)
    converted = MODULE.xyzw_to_wxyz(original)
    assert converted == (0.5, -0.5, 0.5, -0.5)
    assert isinstance(converted, tuple)
    assert converted[1:] + converted[:1] == original
    assert MODULE.xyzw_to_wxyz((0, 0, 0, 1)) == (1, 0, 0, 0)


def test_mount_and_intrinsics_reuse_scene_rig():
    spec = importlib.util.spec_from_file_location(
        "scene_rig_reference", ROOT / "tools/scene_rig.py"
    )
    rig = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rig)
    scene, diagnostics, attempts = capture(FakeCamera())
    expected = rig.Camera().base_from_optical()
    assert scene.intrinsics == rig.intrinsics()
    assert scene.base_from_optical.source_frame == expected.source_frame
    assert scene.base_from_optical.target_frame == expected.target_frame
    np.testing.assert_array_equal(scene.base_from_optical.rotation, expected.rotation)
    np.testing.assert_array_equal(
        scene.base_from_optical.translation_m, expected.translation_m
    )
    optical = expected.rotation.T @ (np.array([2.75, 0, 0.5]) - expected.translation_m)
    np.testing.assert_allclose(optical, [0, 0, 2], atol=1e-12)


def test_normalize_depth_reports_raw_sentinels_and_preserves_metres():
    raw = np.array([[np.nan, np.inf, -np.inf, 0, -1, 1.25, 3]], dtype=np.float32)
    depth, diagnostics = MODULE.normalize_depth(raw)
    assert isinstance(diagnostics, MODULE.FrameDiagnostics)
    assert asdict(diagnostics) == {
        "nan_count": 1,
        "posinf_count": 1,
        "neginf_count": 1,
        "zero_count": 1,
        "negative_count": 1,
        "finite_positive_count": 2,
        "finite_positive_min": 1.25,
        "finite_positive_max": 3.0,
    }
    assert depth.dtype == np.float64
    assert np.isnan(depth[0, :5]).all()
    np.testing.assert_array_equal(depth[0, 5:], [1.25, 3])
    assert raw[0, 3] == 0 and raw[0, 4] == -1
    assert np.isposinf(raw[0, 1])


def test_depth_without_finite_positive_values_has_no_minimum_or_maximum():
    depth, diagnostics = MODULE.normalize_depth(
        np.array([0, -1, np.nan, np.inf, -np.inf])
    )
    assert np.isnan(depth).all()
    assert diagnostics.finite_positive_count == 0
    assert diagnostics.finite_positive_min is None
    assert diagnostics.finite_positive_max is None


def test_empty_depth_is_rejected():
    with pytest.raises(ValueError):
        MODULE.normalize_depth(np.empty((0, 3)))


def test_capture_success_copies_buffers_and_sets_contract_metadata():
    camera = FakeCamera()
    camera.depth[0, 0] = np.inf
    scene, diagnostics, attempts = capture(camera)
    assert camera.reads == attempts == 1
    assert diagnostics.posinf_count == 1
    assert diagnostics.finite_positive_count == 480 * 640 - 1
    assert scene.rgb.dtype == np.uint8 and scene.rgb.shape == (480, 640, 3)
    assert scene.depth_m.dtype == np.float64 and np.isnan(scene.depth_m[0, 0])
    assert scene.stamp_ns == 123
    assert scene.clock_domain == scene.source_provenance == "synthetic"
    assert scene.rectified and scene.rgb_registered_to_depth_grid
    camera.depth[:] = 9
    camera.rgba[:] = 0
    assert scene.depth_m[1, 1] == 2 and scene.rgb[1, 2, 0] == 200


@pytest.mark.parametrize(
    "defect", ["flat_rgb", "rgb_shape", "none", "empty", "depth_dtype", "depth_shape"]
)
def test_unready_camera_fails_after_bounded_attempts(defect):
    camera = FakeCamera()
    if defect == "flat_rgb":
        camera.rgba[:] = 0
    elif defect == "rgb_shape":
        camera.rgba = camera.rgba[:, :, :3]
    elif defect == "none":
        camera.depth = None
    elif defect == "empty":
        camera.depth = np.empty((0, 0))
    elif defect == "depth_dtype":
        camera.depth = camera.depth.astype(complex)
    else:
        camera.depth = camera.depth[:1]
    with pytest.raises(MODULE.CaptureFailure) as error:
        capture(camera, max_attempts=3)
    assert error.value.reason in {
        "rgba_not_ready",
        "depth_not_ready",
        "stale_frame",
        "timeout",
    }
    assert camera.reads == 3


def test_capture_steps_before_reading_even_on_first_attempt():
    camera = FakeCamera()
    steps = []

    def step():
        steps.append(camera.reads)
        camera.depth[:] = 3

    scene, _, attempts = capture(camera, max_attempts=1, step_fn=step)
    assert steps == [0]
    assert attempts == camera.reads == 1
    assert scene.depth_m[0, 0] == 3


def test_retry_steps_and_accepts_only_when_both_in_place_buffers_change():
    camera = FakeCamera()
    camera.rgba[:] = 0
    steps = []

    def step():
        steps.append(1)
        if len(steps) == 1:
            return  # First render is still warming up.
        if len(steps) == 2:
            camera.rgba[:, ::2, :3] = 200  # Only RGB changes: still stale.
        else:
            camera.rgba[:, ::2, :3] = 180
            camera.depth[:] = 3

    scene, diagnostics, attempts = capture(camera, max_attempts=3, step_fn=step)
    assert len(steps) == camera.reads == attempts == 3
    assert scene.depth_m[0, 0] == 3 and scene.rgb[0, 0, 0] == 180


def test_stale_depth_with_nan_is_not_mistaken_for_a_new_frame():
    camera = FakeCamera()
    camera.rgba[:] = 0
    camera.depth[0, 0] = np.nan

    def step():
        if camera.reads:
            camera.rgba[:, ::2, :3] += 20

    with pytest.raises(MODULE.CaptureFailure) as error:
        capture(camera, max_attempts=3, step_fn=step)
    assert error.value.reason == "stale_frame"
    assert camera.reads == 3


def test_changing_only_depth_does_not_count_as_new_frame():
    camera = FakeCamera()
    camera.depth = None

    def step():
        if camera.reads:
            camera.depth = np.full((480, 640), camera.reads + 1, dtype=np.float32)

    with pytest.raises(MODULE.CaptureFailure) as error:
        capture(camera, max_attempts=3, step_fn=step)
    assert error.value.reason == "stale_frame"
    assert camera.reads == 3


def test_standard_deviation_threshold_is_strict_and_ignores_alpha():
    camera = FakeCamera()
    camera.rgba[:, ::2, :3] = 4  # RGB std == 2, not > 2.
    camera.rgba[:, ::2, 3] = 255
    with pytest.raises(MODULE.CaptureFailure):
        capture(camera, max_attempts=1)
    assert capture(camera, std_threshold=1.9)[0].rgb[0, 0, 0] == 4


@pytest.mark.parametrize("attempts", [0, -1, True, 1.5])
def test_invalid_retry_limit_is_rejected(attempts):
    with pytest.raises(ValueError):
        capture(FakeCamera(), max_attempts=attempts)


def observation(yaw=0, status="valid"):
    left = (-math.sin(yaw) * 0.2, math.cos(yaw) * 0.2)
    return PocketObservation(
        123,
        "synthetic",
        "base_link",
        "synthetic",
        status,
        Pocket((2 + left[0], 1 + left[1], 0.1), 0.15, 0.1)
        if status == "valid"
        else None,
        Pocket((2 - left[0], 1 - left[1], 0.1), 0.15, 0.1)
        if status == "valid"
        else None,
        yaw if status == "valid" else None,
        None,
        None,
        None if status == "valid" else "no_opening_pattern",
    )


@pytest.mark.parametrize(
    "yaw, depth, expected",
    [
        (0, 0.6, (2.3, 1)),
        (math.pi / 2, 0.6, (2, 1.3)),
        (math.pi, 0.6, (1.7, 1)),
        (-math.pi / 2, 0.66, (2, 0.67)),
    ],
)
def test_estimated_center_is_half_a_pallet_depth_inside_front_plane(
    yaw, depth, expected
):
    result = MODULE.estimate_pallet_center_m(observation(yaw), depth)
    assert isinstance(result, tuple)
    assert result == pytest.approx(expected)


@pytest.mark.parametrize("status", ["invalid", "no_pallet"])
def test_nonvalid_observation_cannot_produce_center(status):
    with pytest.raises(ValueError):
        MODULE.estimate_pallet_center_m(observation(status=status), 0.6)


@pytest.mark.parametrize("depth", [0, -1, np.nan, np.inf, True])
def test_invalid_pallet_depth_is_rejected(depth):
    with pytest.raises(ValueError):
        MODULE.estimate_pallet_center_m(observation(), depth)


@pytest.mark.parametrize("sentinel", [0, -1, np.nan, np.inf, -np.inf])
def test_all_unobserved_depth_is_ready_and_reported(sentinel):
    camera = FakeCamera()
    camera.depth[:] = sentinel
    scene, diagnostics, attempts = capture(camera)
    assert attempts == camera.reads == 1
    assert np.isnan(scene.depth_m).all()
    assert diagnostics.finite_positive_count == 0
    assert diagnostics.finite_positive_min is None
    assert diagnostics.finite_positive_max is None


def test_new_frame_id_accepts_unchanged_depth_and_static_rgb():
    camera = FakeCamera()
    camera.depth = None
    ids = iter([None, None, 2])
    seen = []

    def frame_id():
        value = next(ids)
        seen.append(value)
        return value

    def step():
        if camera.reads:
            camera.depth = np.full((480, 640), 2, dtype=np.float32)

    scene, diagnostics, attempts = capture(
        camera,
        max_attempts=3,
        step_fn=step,
        frame_id_fn=frame_id,
    )
    assert seen == [None, None, 2]
    assert attempts == 3
    assert scene.depth_m[0, 0] == 2
    assert diagnostics.finite_positive_count == 480 * 640


def test_unchanged_frame_id_rejects_changed_contents():
    camera = FakeCamera()
    camera.rgba[:] = 0

    def step():
        if camera.reads:
            camera.rgba[:, ::2, :3] += 20
            camera.depth[:] += 1

    with pytest.raises(MODULE.CaptureFailure) as error:
        capture(camera, max_attempts=3, step_fn=step, frame_id_fn=lambda: 7)
    assert error.value.reason == "stale_frame"
    assert camera.reads == 3


def test_stamp_callback_uses_accepted_attempt_and_overrides_fixed_stamp():
    camera = FakeCamera()
    camera.rgba[:] = 0
    now = [100]
    sampled = []

    def step():
        now[0] += 10
        if now[0] == 120:
            camera.rgba[:, ::2, :3] = 200

    def stamp():
        sampled.append(now[0])
        return now[0]

    scene, _, attempts = capture(
        camera,
        max_attempts=3,
        step_fn=step,
        frame_id_fn=lambda: now[0],
        stamp_ns_fn=stamp,
    )
    assert scene.stamp_ns == 120
    assert attempts == camera.reads == 2
    assert sampled == [120]


def test_stamp_callback_can_be_used_without_fixed_stamp():
    scene, _, attempts = MODULE.capture_scene_input(
        FakeCamera(),
        MODULE.default_base_from_optical(),
        stamp_ns_fn=lambda: 456,
    )
    assert scene.stamp_ns == 456
    assert attempts == 1


@pytest.mark.parametrize("stamp", [None, -1, True, 1.5])
def test_invalid_stamp_callback_result_is_rejected(stamp):
    with pytest.raises(ValueError, match="stamp_ns"):
        capture(FakeCamera(), stamp_ns_fn=lambda: stamp)


@pytest.mark.parametrize("with_id", [False, True])
def test_repeated_capture_rejects_previous_accepted_frame_on_first_attempt(with_id):
    camera = FakeCamera()
    state = MODULE.CaptureState()
    options = {"state": state}
    if with_id:
        options["frame_id_fn"] = lambda: 7
    capture(camera, max_attempts=1, **options)
    with pytest.raises(MODULE.CaptureFailure) as error:
        capture(camera, max_attempts=2, **options)
    assert error.value.reason == "stale_frame"
    assert error.value.diagnostics.rejection_counts == {"stale_frame": 2}


def test_cross_call_retry_accepts_new_id_even_for_static_pixels():
    camera = FakeCamera()
    state = MODULE.CaptureState()
    capture(camera, state=state, frame_id_fn=lambda: 7)
    ids = iter([7, 8])
    _, _, attempts = capture(camera, state=state, frame_id_fn=lambda: next(ids))
    assert attempts == 2
    assert state.diagnostics.sensor_frame_id == 8
    assert state.diagnostics.rejection_counts == {"stale_frame": 1}


class SensorCamera(FakeCamera):
    def __init__(self):
        super().__init__()
        self.frame = {
            "rendering_frame": {
                "referenceTimeNumerator": 1,
                "referenceTimeDenominator": 60,
            },
            "rendering_time": 1.0,
        }

    def get_current_frame(self):
        return self.frame


def sensor_capture(camera, step=None, pose=None):
    now = [0.0]

    def advance():
        now[0] += 1.0
        if step:
            step(camera, now[0])

    return MODULE.SensorCapture(
        camera,
        MODULE.default_base_from_optical(),
        step_fn=advance,
        physics_time_fn=lambda: now[0],
        pose_fn=pose or (lambda: ([0, 0, 0], [1, 0, 0, 0])),
    )


def test_sensor_capture_records_acquisition_and_pose_bracket():
    sensor = sensor_capture(SensorCamera())
    scene, _, count = sensor.capture(max_attempts=1)
    diag = asdict(sensor.state.diagnostics)
    assert scene.stamp_ns == 1_000_000_000 and count == 1
    assert diag["acquisition_time_s"] == 1.0
    assert diag["physics_time_before_s"] == 0.0
    assert diag["physics_time_after_s"] == 1.0
    assert diag["pose_before"] == diag["pose_after"]
    assert diag["attempts"] == 1
    with pytest.raises(MODULE.CaptureFailure):
        sensor.capture(max_attempts=1)


@pytest.mark.parametrize(
    "defect, reason",
    [
        ("missing", "sensor_metadata_unavailable"),
        ("old", "acquisition_outside_capture"),
        ("moving", "pose_changed"),
        ("torn", "sensor_frame_changed_during_read"),
    ],
)
def test_sensor_capture_fails_closed(defect, reason):
    camera = SensorCamera()
    poses = iter([([0, 0, 0], [1, 0, 0, 0]), ([0.1, 0, 0], [1, 0, 0, 0])])
    if defect == "missing":
        camera.frame = {}
    if defect == "old":
        camera.frame["rendering_time"] = -1.0
    if defect == "torn":
        original = camera.get_depth

        def depth():
            camera.frame["rendering_frame"]["referenceTimeNumerator"] += 1
            return original()

        camera.get_depth = depth
    sensor = sensor_capture(
        camera, pose=(lambda: next(poses)) if defect == "moving" else None
    )
    with pytest.raises(MODULE.CaptureFailure) as error:
        sensor.capture(max_attempts=1)
    assert error.value.reason == reason


def test_sensor_capture_mutable_id_is_copied_across_calls():
    camera = SensorCamera()

    def update(cam, time):
        cam.frame["rendering_frame"]["referenceTimeNumerator"] = int(time)
        cam.frame["rendering_time"] = time

    sensor = sensor_capture(camera, step=update)
    sensor.capture(max_attempts=1)
    scene, _, attempts = sensor.capture(max_attempts=1)
    assert attempts == 1 and scene.stamp_ns == 2_000_000_000
    assert sensor.state.diagnostics.sensor_frame_id["referenceTimeNumerator"] == 2


@pytest.mark.parametrize(
    "quaternion, reason",
    [
        ([math.cos(0.01), 0, 0, math.sin(0.01)], "pose_changed"),
        ([0, 0, 0, 0], "invalid_capture_pose"),
    ],
)
def test_sensor_capture_rejects_rotation_or_invalid_pose(quaternion, reason):
    poses = iter([([0, 0, 0], [1, 0, 0, 0]), ([0, 0, 0], quaternion)])
    sensor = sensor_capture(SensorCamera(), pose=lambda: next(poses))
    with pytest.raises(MODULE.CaptureFailure) as error:
        sensor.capture(max_attempts=1)
    assert error.value.reason == reason
    assert error.value.diagnostics.rejection_counts == {reason: 1}
    assert sensor.state.previous is None


def test_array_fallback_does_not_forget_accepted_frame_after_rejected_attempt():
    camera = FakeCamera()
    state = MODULE.CaptureState()
    capture(camera, state=state)
    attempts = [0]

    def step():
        attempts[0] += 1
        camera.rgba[:, ::2, :3] = 180 if attempts[0] == 1 else 200
        camera.depth[:] = 2

    with pytest.raises(MODULE.CaptureFailure):
        capture(camera, state=state, step_fn=step, max_attempts=2)
    assert state.previous[0][0, 0, 0] == 200


class DelayedSensorCamera(SensorCamera):
    """Render clock advances at 60 Hz, with N queued frames and zero warm-up."""

    def __init__(self, delay):
        super().__init__()
        self.delay = delay
        self.tick = 0
        self.now = 0.0
        self.queue = [(0, 0.0)] * delay

    def step(self):
        self.tick += 1
        self.now += 1 / 60
        self.queue.append((self.tick, self.now))
        acquired, stamp = self.queue.pop(0)
        self.frame = {"rendering_frame": acquired, "rendering_time": stamp}

    def sensor(self, pose=None):
        return MODULE.SensorCapture(
            self,
            MODULE.default_base_from_optical(),
            step_fn=self.step,
            physics_time_fn=lambda: self.now,
            pose_fn=pose or (lambda: ([0, 0, 0], [1, 0, 0, 0])),
            # This original queue fixture models exact pixel acquisition times,
            # with no additional pixel/reference latency. MeasuredLagCamera
            # below separately exercises the measured default margin.
            render_latency_s=0,
        )


@pytest.mark.parametrize("delay", [2, 4, 7])
def test_delayed_capture_flushes_warmup_and_previous_waypoint(delay):
    camera = DelayedSensorCamera(delay)
    sensor = camera.sensor()
    scene, _, steps = sensor.capture(max_attempts=20)
    assert steps == delay + 1
    assert scene.stamp_ns == int(camera.frame["rendering_time"] * 1e9)
    assert sensor.state.diagnostics.rejection_counts == {
        "sensor_frame_not_ready": delay
    }
    # Travel without rendering leaves the queue at the previous waypoint.
    camera.now += 10
    start = camera.now
    scene, _, steps = sensor.capture(max_attempts=20)
    diag = sensor.state.diagnostics
    assert steps == delay + 1
    assert start <= scene.stamp_ns / 1e9 <= camera.now
    assert diag.capture_start_time_s == start
    assert diag.render_steps == steps
    assert diag.acquisition_since_capture_start_s == pytest.approx(
        camera.frame["rendering_time"] - start
    )
    assert diag.physics_time_after_s - diag.acquisition_time_s == pytest.approx(
        delay / 60
    )
    assert diag.accepted_pose == diag.pose_after
    assert diag.pose_reference == "stationary_capture_end"


def test_delayed_capture_budget_exhaustion_keeps_last_accepted_frame():
    camera = DelayedSensorCamera(4)
    sensor = camera.sensor()
    sensor.capture(max_attempts=10)
    accepted = sensor.state.previous_frame_id
    camera.now += 10
    with pytest.raises(MODULE.CaptureFailure) as error:
        sensor.capture(max_attempts=2)
    assert error.value.reason == "acquisition_outside_capture"
    assert error.value.diagnostics.attempts == 2
    assert error.value.diagnostics.render_steps == 2
    assert sensor.state.previous_frame_id == accepted


def test_sensor_retries_reused_id_even_with_in_window_timestamp():
    camera = SensorCamera()
    sensor = sensor_capture(
        camera, step=lambda cam, now: cam.frame.update(rendering_time=now)
    )
    sensor.capture(max_attempts=1)
    with pytest.raises(MODULE.CaptureFailure) as error:
        sensor.capture(max_attempts=3)
    assert error.value.reason == "stale_frame"
    assert error.value.diagnostics.rejection_counts == {"stale_frame": 3}


def test_delayed_capture_rejects_cumulative_motion_during_warmup():
    camera = DelayedSensorCamera(4)
    sensor = camera.sensor(pose=lambda: ([camera.tick * 0.0004, 0, 0], [1, 0, 0, 0]))
    with pytest.raises(MODULE.CaptureFailure) as error:
        sensor.capture(max_attempts=10)
    assert error.value.reason == "pose_changed"
    assert error.value.diagnostics.attempts == 3
    assert sensor.state.previous is None


@pytest.mark.parametrize("frame, stamp", [(0, 1.0), (3, 0.0)])
def test_zero_sensor_metadata_is_retried(frame, stamp):
    camera = SensorCamera()

    def step(cam, now):
        cam.frame = {
            "rendering_frame": frame if now < 3 else 4,
            "rendering_time": stamp if now < 3 else now,
        }

    sensor = sensor_capture(camera, step=step)
    _, _, steps = sensor.capture(max_attempts=3)
    assert steps == 3
    assert sensor.state.diagnostics.rejection_counts == {"sensor_frame_not_ready": 2}


class MeasuredLagCamera(SensorCamera):
    """ws1 timing model: acquisition trails physics by four 60 Hz renders."""

    def __init__(self, dt=1 / 60):
        super().__init__()
        self.dt = dt
        self.start = 69.025004
        self.now = self.start
        self.tick = 0
        self.frozen_id = None

    def step(self):
        self.tick += 1
        self.now = self.start + self.tick * self.dt
        self.frame = {
            "rendering_frame": self.frozen_id or self.tick,
            "rendering_time": self.start + (self.tick - 4) * self.dt,
        }

    def sensor(self, **options):
        return MODULE.SensorCapture(
            self,
            MODULE.default_base_from_optical(),
            step_fn=self.step,
            physics_time_fn=lambda: self.now,
            pose_fn=lambda: ([0, 0, 0], [1, 0, 0, 0]),
            **options,
        )


@pytest.mark.parametrize("budget", [4, 8])
def test_measured_lag_rejects_capture_start_and_inside_latency_window(budget):
    # At step 4 acquisition equals capture start exactly; step 8 is still
    # short of the measured, rounded-up 0.066667 s margin.
    camera = MeasuredLagCamera()
    sensor = camera.sensor()
    with pytest.raises(MODULE.CaptureFailure) as error:
        sensor.capture(max_attempts=budget)
    assert error.value.reason == "acquisition_outside_capture"
    assert error.value.diagnostics.render_steps == budget
    assert error.value.diagnostics.rejection_counts == {
        "acquisition_outside_capture": budget
    }
    assert sensor.state.previous is None


def test_measured_lag_retries_until_default_latency_margin_is_met():
    camera = MeasuredLagCamera()
    sensor = camera.sensor()
    scene, _, attempts = sensor.capture(max_attempts=9)
    diag = sensor.state.diagnostics
    assert attempts == diag.render_steps == 9
    assert diag.capture_start_time_s == 69.025004
    assert diag.acquisition_since_capture_start_s == pytest.approx(5 / 60)
    assert diag.physics_time_after_s - diag.acquisition_time_s == pytest.approx(4 / 60)
    assert diag.rejection_counts == {"acquisition_outside_capture": 8}
    assert scene.stamp_ns == int(diag.acquisition_time_s * 1e9)


def test_measured_lag_still_rejects_previous_accepted_id_after_margin():
    camera = MeasuredLagCamera()
    sensor = camera.sensor()
    sensor.capture(max_attempts=9)
    accepted = sensor.state.previous_frame_id
    camera.frozen_id = accepted
    with pytest.raises(MODULE.CaptureFailure) as error:
        sensor.capture(max_attempts=11)
    assert error.value.reason == "stale_frame"
    assert error.value.diagnostics.rejection_counts == {
        "acquisition_outside_capture": 8,
        "stale_frame": 3,
    }
    assert error.value.diagnostics.render_steps == 11
    assert sensor.state.previous_frame_id == accepted
    assert sensor.state.diagnostics.accepted_pose is None


def test_measured_lag_margin_is_configurable_and_inclusive():
    # Binary-exact times pin equality at start + L without a float tolerance.
    camera = MeasuredLagCamera(dt=1 / 64)
    camera.start = camera.now = 64.0
    sensor = camera.sensor(render_latency_s=1 / 16)
    _, _, attempts = sensor.capture(max_attempts=8)
    assert attempts == 8
    assert sensor.state.diagnostics.acquisition_since_capture_start_s == 1 / 16
    assert sensor.state.diagnostics.rejection_counts == {
        "acquisition_outside_capture": 7
    }


@pytest.mark.parametrize("latency", [-0.1, float("nan"), float("inf")])
def test_sensor_capture_rejects_invalid_render_latency(latency):
    with pytest.raises(ValueError, match="render_latency_s"):
        MeasuredLagCamera().sensor(render_latency_s=latency)


def test_sensor_capture_rejects_future_acquisition_even_after_multiple_steps():
    camera = SensorCamera()
    sensor = sensor_capture(
        camera, step=lambda cam, now: cam.frame.update(rendering_time=now + 0.1)
    )
    with pytest.raises(MODULE.CaptureFailure) as error:
        sensor.capture(max_attempts=3)
    assert error.value.reason == "acquisition_outside_capture"
    assert error.value.diagnostics.rejection_counts == {
        "acquisition_outside_capture": 3
    }
