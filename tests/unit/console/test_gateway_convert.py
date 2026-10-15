import math
import struct

import pytest

from console.gateway import convert
from console.gateway.convert import (
    CorrectionTracker,
    GridMessage,
    PoseSample,
    ScanMessage,
    Snapshot,
)


def test_yaw_from_quaternion():
    for yaw in (-3.0, -1.0, 0.0, 0.4, 2.9):
        q = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
        assert convert.yaw_from_quat(*q) == pytest.approx(yaw)


def test_rear_axle_pose_moves_back_along_the_heading():
    # base_link at (2, 1) facing +y; the rear axle sits 0.34 m behind it
    x, y, yaw = convert.rear_axle_pose(2.0, 1.0, math.pi / 2, -0.34)
    assert (x, y, yaw) == pytest.approx((2.0, 0.66, math.pi / 2))


def test_grid_cells_use_the_map_server_thresholds():
    data = [-1, 0, 24, 25, 50, 65, 66, 100]
    cells = convert.grid_cells(data, 8, 1)
    # 0 unknown, 1 free, 2 occupied; 0.25 and 0.65 themselves are neither
    assert list(cells) == [0, 1, 1, 0, 0, 0, 2, 2]


def test_grid_cells_refuse_a_size_mismatch():
    with pytest.raises(ValueError):
        convert.grid_cells([0, 0, 0], 2, 2)


def grid(yaw=0.0, **kw):
    base = dict(
        width=4,
        height=2,
        resolution_m=0.05,
        origin_x_m=-1.5,
        origin_y_m=2.0,
        origin_yaw_rad=yaw,
        frame="map",
        data=[0] * 8,
        received_s=10.0,
    )
    base.update(kw)
    return GridMessage(**base)


def test_grid_frame_carries_origin_and_codes():
    head, body = convert.grid_frame(grid(), epoch=7, correction_version=3)
    assert head["origin_m"] == [-1.5, 2.0]
    assert head["values"] == {"unknown": 0, "free": 1, "occupied": 2}
    assert (head["width"], head["height"], head["resolution_m"]) == (4, 2, 0.05)
    assert head["session_epoch"] == 7 and head["frame"] == "map"
    assert body == bytes([1] * 8)


def test_a_rotated_grid_is_not_sent_but_float_noise_is_tolerated():
    assert convert.grid_frame(grid(yaw=1e-6), epoch=1, correction_version=0)
    with pytest.raises(convert.GridRotated):
        convert.grid_frame(grid(yaw=0.01), epoch=1, correction_version=0)


def test_scan_ranges_mark_no_return_as_nan():
    scan = ScanMessage(
        angle_min_rad=-1.0,
        angle_increment_rad=0.5,
        range_min_m=0.15,
        range_max_m=12.0,
        ranges=[1.0, 0.1, math.inf, float("nan"), 13.0, 12.0],
        received_s=5.0,
    )
    laser = PoseSample("map", 1.0, 2.0, 0.3, received_s=4.9)
    head, body = convert.scan_frame(scan, laser, epoch=2)
    values = struct.unpack("<6f", body)
    assert values[0] == pytest.approx(1.0) and values[5] == pytest.approx(12.0)
    assert all(math.isnan(v) for v in values[1:5])
    assert head["pose"] == {"x_m": 1.0, "y_m": 2.0, "yaw_rad": 0.3}
    assert head["frame"] == "map" and head["count"] == 6
    assert head["dtype"] == "float32le"


def test_scan_without_a_laser_pose_is_not_sent():
    scan = ScanMessage(-1.0, 0.5, 0.1, 10.0, [1.0], received_s=1.0)
    assert convert.scan_frame(scan, None, epoch=1) is None


def test_jpeg_is_recognised_by_its_first_bytes_not_the_format_string():
    assert convert.is_jpeg(b"\xff\xd8\xff\xe0rest")
    assert not convert.is_jpeg(b"\x89PNG\r\n")
    assert not convert.is_jpeg(b"")


def test_correction_counts_only_real_changes():
    t = CorrectionTracker()
    assert t.update((0.0, 0.0, 0.0), now=1.0) == (1, 0.0)  # the first one counts
    # slam_toolbox republishes the same transform; noise under 1 mm / 1 mrad is not a correction
    assert t.update((0.0005, 0.0, 0.0005), now=2.0) == (1, 1.0)
    assert t.update((0.01, 0.0, 0.0), now=3.0) == (2, 0.0)
    assert t.age(now=4.5) == pytest.approx(1.5)
    assert CorrectionTracker().age(now=3.0) is None


def test_monitor_welcome_offers_no_control():
    w = convert.welcome("fl-1", "isaac", epoch=9)
    caps = w["capabilities"]
    assert caps["control"] is False
    for key in ("mission", "teleop_direct", "teleop_video", "lift"):
        assert caps[key] is False
    assert caps["camera_model"] is None
    # display-only outline from dls08_measured, measured from the rear axle (x -0.34):
    # fork tip 0.95 = rear extent -0.51 + length 1.46; half width 0.63 / 2
    ch = caps["chassis"]
    assert ch["nose_from_rear_m"] == pytest.approx(1.29)
    assert ch["rear_overhang_m"] == pytest.approx(0.17)
    assert ch["half_width_m"] == pytest.approx(0.315)
    assert ch["model_turning_radius_m"] == pytest.approx(1 / ch["max_curvature_inv_m"])
    assert w["backend"] == "isaac" and w["session_epoch"] == 9
    with pytest.raises(ValueError):
        convert.welcome(
            "fl-1", "mock", epoch=1
        )  # the gateway never claims to be the fake


def test_state_reports_what_is_measured_and_null_for_the_rest():
    snap = Snapshot(
        base=PoseSample("map", 2.0, 1.0, math.pi / 2, received_s=9.6),
        speed_mps=0.31,
        speed_received_s=9.9,
        correction_version=4,
        correction_age_s=0.8,
        grid_received_s=9.0,
        scan_received_s=9.95,
    )
    st = convert.state(snap, now=10.0, epoch=3, seq=17, rear_axle_x_m=-0.34)
    assert st["t"] == "state" and st["state_seq"] == 17 and st["session_epoch"] == 3
    assert st["pose"]["frame"] == "map" and st["pose"]["source"] == "slam"
    assert (st["pose"]["x_m"], st["pose"]["y_m"]) == pytest.approx((2.0, 0.66))
    assert st["pose"]["age_s"] == pytest.approx(0.4)
    assert st["pose"]["correction_version"] == 4
    assert st["rear_axle_speed_mps"] == pytest.approx(0.31)
    for key in (
        "mode",
        "stop_kind",
        "armed",
        "lease",
        "permit",
        "limits",
        "command",
        "applied",
        "mission",
        "steering_rad",
        "lift_m",
        "drive_allowed",
        "lift_allowed",
        "teleop_allowed",
    ):
        assert st[key] is None, key
    assert st["load"] == {"state": None}
    assert st["freshness"]["slam_correction_age_s"] == pytest.approx(0.8)
    assert st["freshness"]["grid_age_s"] == pytest.approx(1.0)
    assert st["freshness"]["scan_age_s"] == pytest.approx(0.05)


def test_state_drops_an_old_speed_and_marks_odom_poses():
    snap = Snapshot(
        base=PoseSample("odom", 0.0, 0.0, 0.0, received_s=9.0),
        speed_mps=0.5,
        speed_received_s=8.5,
    )
    st = convert.state(snap, now=10.0, epoch=1, seq=1, rear_axle_x_m=-0.34)
    assert st["rear_axle_speed_mps"] is None  # older than 1 s
    assert st["pose"]["frame"] == "odom" and st["pose"]["source"] == "odom"
    assert st["pose"]["correction_version"] is None


def test_state_without_any_pose():
    st = convert.state(Snapshot(), now=1.0, epoch=1, seq=1, rear_axle_x_m=-0.34)
    assert st["pose"] is None
    assert st["rear_axle_speed_mps"] is None
    assert st["freshness"] == {
        "slam_correction_age_s": None,
        "grid_age_s": None,
        "scan_age_s": None,
    }
