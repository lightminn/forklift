import dataclasses
import importlib.util
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.evaluation import Outcome, evaluate_scene
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import detect_pockets
from forklift_core.perception.scene_dataset import load_scene_sample

REPO_PRIOR = Path(__file__).resolve().parents[2] / "config" / "pallet_prior_v1.yaml"


_WRITER_SPEC = importlib.util.spec_from_file_location(
    "scene_writer", Path(__file__).resolve().parents[1] / "fixtures" / "scene_writer.py"
)
_WRITER = importlib.util.module_from_spec(_WRITER_SPEC)
_WRITER_SPEC.loader.exec_module(_WRITER)
write_v1_scene = _WRITER.write_v1_scene


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
