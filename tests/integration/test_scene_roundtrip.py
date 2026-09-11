"""The ROS-independent producer and core loader agree on actual v1 files."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from forklift_core.perception.scene_dataset import load_scene_sample

ROOT = Path(__file__).resolve().parents[2]


def test_captured_640_by_480_scene_loads_with_original_truth_and_calibration(tmp_path):
    path = ROOT / "ros2/src/forklift_ros/forklift_ros/scene_files.py"
    spec = importlib.util.spec_from_file_location("scene_producer_roundtrip", path)
    api = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(api)
    stamp = 2_400_000_007
    header = NS(frame_id="camera_optical_frame", stamp=NS(sec=2, nanosec=400_000_007))
    camera_info = api.camera_info_to_json(
        NS(
            header=header,
            width=640,
            height=480,
            distortion_model="plumb_bob",
            d=[0.0] * 5,
            k=[460.0, 0.0, 320.0, 0.0, 460.0, 240.0, 0.0, 0.0, 1.0],
            r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p=[460.0, 0.0, 320.0, 0.0, 0.0, 460.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            binning_x=0,
            binning_y=0,
            roi=NS(x_offset=0, y_offset=0, height=0, width=0, do_rectify=False),
        )
    )
    entry = {
        "scene_id": "s001",
        "catalogue_version": "v1",
        "category": "positive",
        "split": "dev",
        "camera": {"width": 640, "height": 480},
        "visibility": {"occluded_side": None},
        "ground_truth": {
            "status": "valid",
            "frame_id": "base_link",
            "stamp_ns": 0,
            "clock_domain": "synthetic",
            "source_provenance": "synthetic_ground_truth",
            "left": {"center_m": [2.0, 0.17, 0.15], "width_m": 0.24, "height_m": 0.2},
            "right": {"center_m": [2.0, -0.17, 0.15], "width_m": 0.24, "height_m": 0.2},
            "insertion_yaw_rad": 0.0,
            "position_sigma_m": 0.0,
            "yaw_sigma_rad": 0.0,
            "reason": None,
        },
    }
    tf = api.tf_static_to_json(
        NS(
            header=NS(frame_id="base_link", stamp=NS(sec=0, nanosec=0)),
            child_frame_id="camera_optical_frame",
            transform=NS(
                translation=NS(x=0.75, y=0.0, z=0.5),
                rotation=NS(x=-0.5, y=0.5, z=-0.5, w=0.5),
            ),
        )
    )
    rgb = np.full((480, 640, 3), [100, 60, 20], dtype=np.uint8)
    depth = np.full((480, 640), 2.25, dtype=np.float32)
    depth[0, 0] = np.nan
    api.write_scene_files(
        tmp_path,
        rgb=rgb,
        depth_m=depth,
        camera_info=camera_info,
        tf=tf,
        ground_truth=api.ground_truth_for_capture(entry, stamp),
        scene=api.build_scene_json(
            entry, stamp, "sha256:image", "a" * 64, "run-1", {"captured": 4.0}
        ),
    )
    sample = load_scene_sample(tmp_path)
    np.testing.assert_array_equal(sample.input.rgb, rgb)
    assert sample.input.depth_m[240, 320] == 2.25
    assert np.isnan(sample.input.depth_m[0, 0])
    assert sample.input.stamp_ns == sample.ground_truth.stamp_ns == stamp
    assert sample.input.clock_domain == sample.ground_truth.clock_domain == "ros_sim"
    assert sample.input.source_provenance == "synthetic"
    assert sample.ground_truth.source_provenance == "synthetic_ground_truth"
    assert sample.input.intrinsics.fx == sample.input.intrinsics.fy == 460.0
    assert sample.input.intrinsics.cx == 320.0
    assert sample.input.intrinsics.cy == 240.0
    np.testing.assert_array_equal(
        sample.input.base_from_optical.translation_m, [0.75, 0.0, 0.5]
    )
    np.testing.assert_array_equal(sample.ground_truth.left.center_m, [2.0, 0.17, 0.15])
    np.testing.assert_array_equal(
        sample.ground_truth.right.center_m, [2.0, -0.17, 0.15]
    )
    assert sample.scene["run_id"] == "run-1"
