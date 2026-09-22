"""Replay frozen v1 observations, allowing only explicitly recorded fixes."""

import dataclasses
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
# Historical observations remain immutable. Each delta is
# (scene_id, old_status, old_reason, new_status, new_reason).
#
# A delta also records which fields it is allowed to change. Without that, a
# delta that only renames the reason trips the field-set check written for a
# delta that adds geometry, and the failure names the wrong cause.
EXPECTED_REPLAY_DELTAS = {
    "20260912T170442Z_pocket_eval_dev_02": {
        # Repairing plane extraction made this scene detectable.
        ("s009", "no_pallet", "no_opening_pattern", "valid", None),
        # Per-opening upper-deck evidence runs before the ray check, so its
        # reason replaces the ray one. It names the side whose upper count fell
        # short, not an obstruction. In these nine it happened to be the side
        # the ray classification named -- an observation about this set, not a
        # property of the two checks.
        ("s007", "invalid", "pocket_occluded:right", "invalid", "upper_deck_occluded:right"),
        ("s027", "invalid", "pocket_occluded:left", "invalid", "upper_deck_occluded:left"),
        ("s039", "invalid", "pocket_occluded:left", "invalid", "upper_deck_occluded:left"),
        ("s051", "invalid", "pocket_occluded:right", "invalid", "upper_deck_occluded:right"),
        ("s076", "invalid", "pocket_occluded:left", "invalid", "upper_deck_occluded:left"),
        ("s090", "invalid", "pocket_occluded:right", "invalid", "upper_deck_occluded:right"),
    },
    "20260912T170558Z_pocket_eval_eval_01": {
        ("s060", "invalid", "pocket_occluded:right", "invalid", "upper_deck_occluded:right"),
        ("s063", "invalid", "pocket_occluded:left", "invalid", "upper_deck_occluded:left"),
        ("s100", "invalid", "pocket_occluded:right", "invalid", "upper_deck_occluded:right"),
    },
}
# A newly valid observation adds geometry; a renamed reason must not.
GEOMETRY_FIELDS = {"status", "reason", "left", "right", "insertion_yaw_rad"}
REASON_ONLY_FIELDS = {"reason"}


def allowed_fields(delta):
    _, old_status, _, new_status, _ = delta
    return GEOMETRY_FIELDS if old_status != new_status else REASON_ONLY_FIELDS


@pytest.mark.parametrize("run_name,expected_count", RUNS)
def test_saved_v1_observations_have_only_expected_deltas(run_name, expected_count):
    run_dir = ROOT / "artifacts" / run_name
    if not run_dir.is_dir():
        pytest.skip(f"Untracked v1 evaluation artifacts are absent: {run_dir}")
    run = json.loads((run_dir / "run.json").read_text())
    observations = sorted((run_dir / "observations").glob("*.json"))
    assert len(observations) == expected_count
    assert {path.stem for path in observations} == set(run["scene_ids"])
    prior = load_pallet_prior(ROOT / "config/pallet_prior_v1.yaml")
    params_data = yaml.safe_load((ROOT / "config/detector_params_v1.yaml").read_text())
    # The frozen file is a SUBSET of the parameter set, not the whole of it:
    # deck_evidence_tol_m and upper_band_points are absent and come from the
    # code's defaults. Asserting containment rather than equality says that
    # plainly and keeps working if a later run records more keys than the file
    # carries. It does not make the gap safe -- changing a default still moves
    # this "frozen" set silently.
    assert params_data.items() <= run["params"].items()
    missing = {f.name for f in dataclasses.fields(DetectorParams)} - set(params_data)
    assert missing == {"deck_evidence_tol_m", "upper_band_points"}, (
        "a parameter left the frozen file without anyone recording it"
    )
    params = DetectorParams(**params_data)
    dataset = Path(run["dataset_dir"])
    if not dataset.is_dir():
        dataset = ROOT / "data/synthetic_scenes/catalogue_v1"
    assert dataset.is_dir(), (
        f"Stored observations exist but replay input is missing: {dataset}"
    )
    differences = {}
    deltas = set()
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
            deltas.add(
                (
                    path.stem,
                    expected["status"],
                    expected["reason"],
                    actual["status"],
                    actual["reason"],
                )
            )
    # Compare the deltas first. Checking field sets inside the loop made an
    # unexpected delta surface as a field-set mismatch on some other scene,
    # which hid the cause.
    assert deltas == EXPECTED_REPLAY_DELTAS[run_name], json.dumps(
        differences, indent=2, sort_keys=True
    )
    # Then hold each delta to the fields its kind is allowed to touch. Every
    # observation must preserve its acquisition metadata and the unknown
    # uncertainty fields either way.
    for delta in sorted(deltas):
        scene_id = delta[0]
        assert set(differences[scene_id]) == allowed_fields(delta), json.dumps(
            differences[scene_id], indent=2, sort_keys=True
        )
    matched = expected_count - len(differences)
    print(
        f"{run_name}: {matched}/{expected_count} observations match (excluding diagnostics)"
    )
