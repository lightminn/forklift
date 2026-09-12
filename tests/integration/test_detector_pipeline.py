import dataclasses
import json
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

from forklift_core.perception.evaluation import Outcome, evaluate_scene
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import detect_pockets
from forklift_core.perception.pocket_observation import Pocket, PocketObservation
from forklift_core.perception.scene_dataset import load_scene_sample

REPO_PRIOR = Path(__file__).resolve().parents[2] / "config" / "pallet_prior_v1.yaml"


def test_a_synthetic_scene_survives_the_v1_file_format_and_is_still_detected(
    tmp_path, pallet_scene
):
    scene, truth = pallet_scene(
        centre_xy_m=(2.5, 0.1), yaw_rad=0.2, unknown_patch=(10, 10, 40, 40)
    )
    scene_dir = write_v1_scene(tmp_path / "s001", scene, truth)
    sample = load_scene_sample(scene_dir)
    # the unknown patch must survive millimetre encoding as NaN, not as a 0 mm reading
    assert np.isnan(sample.input.depth_m[20, 20])
    finite = np.isfinite(scene.depth_m) & np.isfinite(sample.input.depth_m)
    assert np.abs(sample.input.depth_m[finite] - scene.depth_m[finite]).max() <= 0.0005
    assert np.isnan(sample.input.depth_m).sum() == np.isnan(scene.depth_m).sum()

    result = detect_pockets(sample.input, load_pallet_prior(REPO_PRIOR))
    outcome = evaluate_scene(
        sample, result.observation, elapsed_s=result.diagnostics.elapsed_s
    )
    assert outcome.outcome is Outcome.TRUE_POSITIVE
    assert outcome.position_error_m < 0.03


def test_depth_beyond_the_encodable_range_is_refused_by_the_writer(
    tmp_path, pallet_scene
):
    scene, truth = pallet_scene()
    too_far = dataclasses.replace(
        scene, depth_m=np.where(np.isfinite(scene.depth_m), 70.0, scene.depth_m)
    )
    with pytest.raises(ValueError):
        write_v1_scene(tmp_path / "s002", too_far, truth)


def write_v1_scene(scene_dir, scene_input, truth):
    """Serialize this synthetic camera fixture; no actual TF reception occurred."""
    depth = scene_input.depth_m
    finite_positive = np.isfinite(depth) & (depth > 0)
    if np.any(depth[finite_positive] > 65.535):
        raise ValueError("Depth exceeds the uint16 millimetre encoding range")
    raw = np.zeros(depth.shape, dtype=np.uint16)
    raw[finite_positive] = np.rint(depth[finite_positive] * 1000).astype(np.uint16)
    intrinsics = scene_input.intrinsics
    k = [
        intrinsics.fx,
        0.0,
        intrinsics.cx,
        0.0,
        intrinsics.fy,
        intrinsics.cy,
        0.0,
        0.0,
        1.0,
    ]
    projection = (
        np.column_stack((np.array(k).reshape(3, 3), np.zeros(3))).ravel().tolist()
    )
    # This writer is deliberately scoped to the plan's synthetic camera mount.
    # Validate it before writing the declared quaternion, rather than silently
    # serializing another input transform with that camera's orientation.
    expected_rotation = np.array(((0, 0, 1), (-1, 0, 0), (0, -1, 0)))
    if not np.array_equal(scene_input.base_from_optical.rotation, expected_rotation):
        raise ValueError("Writer requires the synthetic fixture camera orientation")
    observation = PocketObservation(
        stamp_ns=scene_input.stamp_ns,
        clock_domain=scene_input.clock_domain,
        frame_id="base_link",
        source_provenance="synthetic_ground_truth",
        status="valid",
        left=Pocket(truth["left_centre_m"], truth["opening_width_m"], 0.20),
        right=Pocket(truth["right_centre_m"], truth["opening_width_m"], 0.20),
        insertion_yaw_rad=truth["yaw_rad"],
        position_sigma_m=None,
        yaw_sigma_rad=None,
        reason=None,
    )
    objects = {
        "depth_meta.json": {
            "unit": "mm",
            "meters_per_unit": 0.001,
            "unknown_value": 0,
            "kind": "optical_axis_z",
        },
        "camera_info.json": {
            "frame_id": intrinsics.frame_id,
            "stamp_ns": scene_input.stamp_ns,
            "width": intrinsics.width,
            "height": intrinsics.height,
            "distortion_model": "plumb_bob",
            "d": [0.0] * 5,
            "k": k,
            "r": np.eye(3).ravel().tolist(),
            "p": projection,
            "binning_x": 0,
            "binning_y": 0,
            "roi": {
                "x_offset": 0,
                "y_offset": 0,
                "height": 0,
                "width": 0,
                "do_rectify": False,
            },
        },
        # v1 origin is a contract label here, NOT evidence of a received TF.
        "tf.json": {
            "target_frame": "base_link",
            "source_frame": "camera_optical_frame",
            "translation_m": scene_input.base_from_optical.translation_m.tolist(),
            "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
            "origin": "received_tf_static",
        },
        "ground_truth.json": observation.to_json(),
        "scene.json": {
            "scene_id": scene_dir.name,
            "catalogue_version": "v1",
            "category": "positive",
            "split": "dev",
            "stamp_ns": scene_input.stamp_ns,
            "clock_domain": scene_input.clock_domain,
            "source_provenance": scene_input.source_provenance,
        },
    }
    scene_dir.mkdir(parents=True)
    Image.fromarray(raw).save(scene_dir / "depth_mm.png")
    Image.fromarray(scene_input.rgb).save(scene_dir / "rgb.png")
    for name, data in objects.items():
        (scene_dir / name).write_text(
            json.dumps(data, allow_nan=False) + "\n", encoding="utf-8"
        )
    return scene_dir
