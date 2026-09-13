"""Replay the untracked, frozen v1 observations without changing expectations."""

import json
from pathlib import Path

import pytest
import yaml

from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.scene_dataset import load_scene_sample

ROOT = Path(__file__).resolve().parents[2]
RUNS = (
    ("20260912T170442Z_pocket_eval_dev_02", 70),
    ("20260912T170558Z_pocket_eval_eval_01", 30),
)


@pytest.mark.parametrize("run_name,expected_count", RUNS)
def test_saved_v1_observations_are_exactly_reproduced(run_name, expected_count):
    run_dir = ROOT / "artifacts" / run_name
    if not run_dir.is_dir():
        pytest.skip(f"Untracked v1 evaluation artifacts are absent: {run_dir}")
    run = json.loads((run_dir / "run.json").read_text())
    observations = sorted((run_dir / "observations").glob("*.json"))
    assert len(observations) == expected_count
    assert {path.stem for path in observations} == set(run["scene_ids"])
    prior = load_pallet_prior(ROOT / "config/pallet_prior_v1.yaml")
    params_data = yaml.safe_load((ROOT / "config/detector_params_v1.yaml").read_text())
    assert params_data == run["params"]  # both historical runs used this frozen set
    params = DetectorParams(**params_data)
    dataset = Path(run["dataset_dir"])
    if not dataset.is_dir():
        dataset = ROOT / "data/synthetic_scenes/catalogue_v1"
    assert dataset.is_dir(), (
        f"Stored observations exist but replay input is missing: {dataset}"
    )
    differences = {}
    for path in observations:
        expected = json.loads(path.read_text())
        expected.pop("diagnostics")
        sample = load_scene_sample(dataset / "scenes" / path.stem)
        result = detect_pockets(sample.input, prior, params)
        actual = result.observation.to_json()
        if actual != expected:
            differences[path.stem] = {
                key: {"expected": expected.get(key), "actual": actual.get(key)}
                for key in expected.keys() | actual.keys()
                if key not in expected
                or key not in actual
                or expected[key] != actual[key]
            }
    matched = expected_count - len(differences)
    print(
        f"{run_name}: {matched}/{expected_count} observations match (excluding diagnostics)"
    )
    assert not differences, json.dumps(differences, indent=2, sort_keys=True)
