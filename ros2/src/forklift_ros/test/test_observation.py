"""Literal independent fixtures for the synthetic reference planes."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest


@pytest.fixture
def api():
    class PendingModule:
        def __getattr__(self, name):
            path = Path(__file__).parents[1] / "forklift_ros" / "observation.py"
            assert path.exists(), "implementation is missing"
            spec = importlib.util.spec_from_file_location("tested_module", path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            return getattr(module, name)

    return PendingModule()


def header(t=1.0, frame="camera_optical_frame"):
    ns = round(t * 1e9)
    return NS(frame_id=frame, stamp=NS(sec=ns // 10**9, nanosec=ns % 10**9))


def info(t=1.0):
    return NS(
        header=header(t),
        width=320,
        height=240,
        k=[160.0, 0.0, 160.0, 0.0, 160.0, 120.0, 0.0, 0.0, 1.0],
        p=[160.0, 0.0, 160.0, 0.0, 0.0, 160.0, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        d=[0.0] * 5,
        distortion_model="plumb_bob",
    )


def depth(t=1.0):
    return NS(
        header=header(t),
        width=320,
        height=240,
        encoding="32FC1",
        is_bigendian=0,
        step=1280,
        data=np.full((240, 320), 2.25, dtype="<f4").tobytes(),
    )


def rgb(t=1.0):
    image = np.zeros((240, 320, 3), dtype=np.uint8)
    image[:, 160:] = [100, 80, 60]
    return NS(
        header=header(t),
        width=320,
        height=240,
        encoding="rgb8",
        is_bigendian=0,
        step=960,
        data=image.tobytes(),
    )


def scan(t=1.0):
    angles = np.linspace(-np.pi, np.pi, 360)
    ranges = np.full(360, np.inf)
    # Independently construct just the known front and left plane beams.
    for target, distance in [(0.0, 2.25), (np.pi / 2, 2.0)]:
        i = int(np.argmin(abs(angles - target)))
        ranges[i] = distance / (np.cos(angles[i]) if target == 0 else np.sin(angles[i]))
    return NS(
        header=header(t, "lidar_link"),
        angle_min=-np.pi,
        angle_max=np.pi,
        angle_increment=2 * np.pi / 359,
        range_min=0.05,
        range_max=10.0,
        ranges=ranges,
        time_increment=0.0,
        scan_time=0.2,
    )


def tf():
    def transform(child, rotation):
        return NS(
            header=header(0, "base_link"),
            child_frame_id=child,
            transform=NS(
                translation=NS(x=0.75, y=0.0, z=0.5),
                rotation=NS(**dict(zip("xyzw", rotation, strict=True))),
            ),
        )

    return NS(
        transforms=[
            transform("camera_optical_frame", [-0.5, 0.5, -0.5, 0.5]),
            transform("lidar_link", [0.0, 0.0, 0.0, 1.0]),
        ]
    )


def feed(api, duration=30.0):
    state = api.ObservationWindow(duration_s=30)
    state.accept("tf_static", tf())
    for index in range(round(duration * 5) + 1):
        t = 1.0 + index * 0.2
        for name, make in [
            ("camera_info", info),
            ("rgb", rgb),
            ("depth", depth),
            ("scan", scan),
        ]:
            state.accept(name, make(t))
        state.accept("clock", NS(clock=header(t).stamp))
    return state


def test_valid_window_reports_actual_counts_and_off_axis_point(api):
    result = feed(api).finish()
    assert result["streams"]["depth"]["count"] == 151
    assert result["streams"]["rgb"]["span_s"] == 30
    assert result["observations"]["point_base_m"] == pytest.approx(
        [3.0, -0.5625, 1.0625]
    )
    assert result["source_provenance"] == "synthetic"


@pytest.mark.parametrize(
    "field,value",
    [("encoding", "16UC1"), ("width", 319), ("step", 100), ("data", b"short")],
)
def test_invalid_depth_metadata_rejected(api, field, value):
    msg = depth()
    setattr(msg, field, value)
    with pytest.raises(ValueError):
        api.ObservationWindow(30).accept("depth", msg)


def test_wrong_frame_rejected(api):
    msg = rgb()
    msg.header.frame_id = "camera_link"
    with pytest.raises(ValueError, match="frame"):
        api.ObservationWindow(30).accept("rgb", msg)


@pytest.mark.parametrize("second", [1.0, 0.9])
def test_duplicate_or_reversed_time_rejected(api, second):
    state = api.ObservationWindow(30)
    state.accept("depth", depth())
    with pytest.raises(ValueError, match="monotonic"):
        state.accept("depth", depth(second))


def test_wrong_known_depth_rejected(api):
    msg = depth()
    msg.data = np.full((240, 320), 4.0, dtype="<f4").tobytes()
    with pytest.raises(ValueError, match="depth"):
        api.ObservationWindow(30).accept("depth", msg)


def test_wrong_lidar_distance_rejected(api):
    msg = scan()
    msg.ranges[:] = 4.0
    with pytest.raises(ValueError, match="plane"):
        api.ObservationWindow(30).accept("scan", msg)


def test_wrong_optical_axis_sign_rejected(api):
    msg = tf()
    msg.transforms[0].transform.rotation = NS(x=0.5, y=0.5, z=0.5, w=0.5)
    with pytest.raises(ValueError, match="transform"):
        api.ObservationWindow(30).accept("tf_static", msg)


def test_empty_and_fresh_replay_do_not_inherit_previous_success(api):
    feed(api).finish()
    with pytest.raises(ValueError, match="missing"):
        api.ObservationWindow(30).finish()


def test_truncated_window_rejected_for_each_stream(api):
    with pytest.raises(ValueError, match="duration"):
        feed(api, 29.8).finish()


def test_sparse_thirty_second_stream_rejected(api):
    state = api.ObservationWindow(30)
    state.accept("tf_static", tf())
    for t in [1.0, 31.0]:
        for name, make in [
            ("camera_info", info),
            ("rgb", rgb),
            ("depth", depth),
            ("scan", scan),
        ]:
            state.accept(name, make(t))
        state.accept("clock", NS(clock=header(t).stamp))
    with pytest.raises(ValueError, match="count"):
        state.finish()


def test_rgb_depth_misalignment_rejected(api):
    state = feed(api)
    state.stamps["rgb"] = [t + 50_000_000 for t in state.stamps["rgb"]]
    with pytest.raises(ValueError, match="aligned"):
        state.finish()


def test_only_one_stream_truncated_is_rejected(api):
    state = feed(api)
    state.stamps["scan"].pop()
    with pytest.raises(ValueError, match="duration"):
        state.finish()


def test_missing_clock_cannot_use_sensor_duration_as_proxy(api):
    state = feed(api)
    state.stamps["clock"].clear()
    with pytest.raises(ValueError, match="missing"):
        state.finish()


def test_nonfinite_depth_roi_is_not_free_space(api):
    msg = depth()
    values = np.full((240, 320), 2.25, dtype="<f4")
    values[80, 200] = np.nan
    msg.data = values.tobytes()
    with pytest.raises(ValueError, match="depth"):
        api.ObservationWindow(30).accept("depth", msg)


def test_invalid_camera_calibration_is_rejected(api):
    msg = info()
    msg.k[0] = 320.0
    with pytest.raises(ValueError, match="calibration"):
        api.ObservationWindow(30).accept("camera_info", msg)


def test_big_endian_float_depth_is_decoded_in_metres(api):
    msg = depth()
    msg.is_bigendian = 1
    msg.data = np.full((240, 320), 2.25, dtype=">f4").tobytes()
    state = api.ObservationWindow(30)
    state.accept("depth", msg)
    assert state.observations["depth_roi_m"] == pytest.approx(2.25)


def test_blank_rgb_is_rejected(api):
    msg = rgb()
    msg.data = bytes(320 * 240 * 3)
    with pytest.raises(ValueError, match="blank"):
        api.ObservationWindow(30).accept("rgb", msg)


def test_fixed_warmup_excludes_startup_black_and_records_excluded_counts(api):
    state = api.ExperimentWindow(30)
    msg = rgb(0.2)
    msg.data = bytes(320 * 240 * 3)
    state.accept("rgb", msg)
    assert state.stamps["rgb"] == []
    assert state.warmup_excluded["rgb"] == [200_000_000]
    state.accept("rgb", rgb(2.0))
    assert state.stamps["rgb"] == [2_000_000_000]


def test_black_at_warmup_boundary_fails_instead_of_extending_warmup(api):
    state = api.ExperimentWindow(30)
    msg = rgb(2.0)
    msg.data = bytes(320 * 240 * 3)
    with pytest.raises(ValueError, match="blank"):
        state.accept("rgb", msg)


def test_warmup_does_not_make_short_observation_complete(api):
    state = api.ExperimentWindow(30)
    state.accept("tf_static", tf())
    for index in range(151):
        t = index * 0.2
        for name, make in [
            ("camera_info", info),
            ("rgb", rgb),
            ("depth", depth),
            ("scan", scan),
        ]:
            state.accept(name, make(t))
        state.accept("clock", NS(clock=header(t).stamp))
    with pytest.raises(ValueError, match="duration"):
        state.finish()


def test_bag_inventory_rejects_missing_row_even_when_duration_is_long_enough(api):
    with pytest.raises(ValueError, match="inventory"):
        api.validate_bag_inventory(
            {"/depth": 165, "/clock": 3288}, {"/depth": 164, "/clock": 3288}, 3453
        )


def test_bag_inventory_rejects_metadata_total_disagreement(api):
    with pytest.raises(ValueError, match="inventory"):
        api.validate_bag_inventory({"/depth": 165}, {"/depth": 165}, 164)


def test_complete_bag_inventory_is_accepted(api):
    api.validate_bag_inventory(
        {"/depth": 165, "/clock": 3288}, {"/depth": 165, "/clock": 3288}, 3453
    )
