"""Merge real PNG/JSON batch fixtures and reject inconsistent evaluation data."""

import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys
from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from forklift_core.perception.scene_dataset import load_scene_sample

ROOT = Path(__file__).resolve().parents[2]


def _fixture_writer():
    spec = importlib.util.spec_from_file_location(
        "scene_dataset_fixtures", ROOT / "tests/unit/perception/test_scene_dataset.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.write_scene


write_scene = _fixture_writer()


def api():
    path = ROOT / "tools/merge_scene_batches.py"
    assert path.exists(), "merge_scene_batches implementation is missing"
    spec = importlib.util.spec_from_file_location("tested_scene_merge", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_json(path):
    return json.loads(path.read_text())


def write_json(path, payload):
    path.write_text(json.dumps(payload, allow_nan=False) + "\n")


def hashes(scene):
    return {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in scene.iterdir()
        if p.is_file()
    }


def manifest_path(batch):
    return batch / "scene_capture/manifest.json"


def scene_path(batch, scene_id="s001"):
    return batch / "scene_capture/scenes" / scene_id


def refresh_scene_hashes(batch, scene_id="s001"):
    path = manifest_path(batch)
    manifest = read_json(path)
    manifest["scenes"][scene_id]["files"] = hashes(scene_path(batch, scene_id))
    write_json(path, manifest)


@pytest.fixture
def batches(tmp_path):
    catalogue = {
        "catalogue_version": "test",
        "camera": {"width": 8, "height": 6},
        "scenes": [],
    }
    metadata = {
        "catalogue_version": "test",
        "camera": catalogue["camera"],
        "image_id": "sha256:image",
        "source_snapshot_sha256": "a" * 64,
    }
    paths = [tmp_path / "batch-a", tmp_path / "batch-b"]
    for index, (category, split) in enumerate(
        [
            ("positive", "dev"),
            ("positive", "eval"),
            ("occluded", "dev"),
            ("negative_no_pallet", "dev"),
            ("negative_lookalike", "eval"),
        ],
        start=1,
    ):
        scene_id = f"s{index:03d}"
        batch = paths[0 if index <= 3 else 1]
        scene = write_scene(batch / "scene_capture/scenes", scene_id=scene_id)
        shutil.copyfile(scene / "rgb.png", scene / "depth_preview.png")
        payload = read_json(scene / "scene.json")
        payload.update(metadata, category=category, split=split, run_id=batch.name)
        write_json(scene / "scene.json", payload)
        truth = read_json(scene / "ground_truth.json")
        if category.startswith("negative_"):
            truth.update(
                status="no_pallet",
                left=None,
                right=None,
                insertion_yaw_rad=None,
                position_sigma_m=None,
                yaw_sigma_rad=None,
                reason="no target pallet",
            )
        write_json(scene / "ground_truth.json", truth)
        catalogue["scenes"].append(
            {
                "scene_id": scene_id,
                "category": category,
                "split": split,
                "ground_truth": {**truth, "stamp_ns": 0, "clock_domain": "synthetic"},
            }
        )
    catalogue_path = tmp_path / "catalogue.yaml"
    catalogue_path.write_text(yaml.safe_dump(catalogue))
    for batch in paths:
        ids = sorted(p.name for p in (batch / "scene_capture/scenes").iterdir())
        manifest = {
            **metadata,
            "catalogue_sha256": hashlib.sha256(catalogue_path.read_bytes()).hexdigest(),
            "run_id": batch.name,
            "requested_scenes": ids,
            "failed_count": 0,
            "scenes": {
                sid: {
                    "passed": True,
                    "error": None,
                    "files": hashes(scene_path(batch, sid)),
                    "wall_times_s": {"captured": 3.0},
                }
                for sid in ids
            },
        }
        write_json(manifest_path(batch), manifest)
    return catalogue_path, paths, tmp_path / "merged"


def merge(batches):
    catalogue, paths, output = batches
    return api().merge_batches(catalogue=catalogue, batches=paths, output=output)


def rejected(batches, match):
    with pytest.raises(ValueError, match=match):
        merge(batches)
    assert not batches[2].exists()


def test_merges_three_plus_two_scenes_and_preserves_bytes_counts_and_batch_ids(batches):
    catalogue, paths, output = batches
    before = {
        str(p): p.read_bytes()
        for batch in paths
        for p in batch.rglob("*")
        if p.is_file()
    }
    manifest = merge(batches)
    assert read_json(output / "manifest.json") == manifest
    assert (
        manifest["catalogue_sha256"]
        == hashlib.sha256(catalogue.read_bytes()).hexdigest()
    )
    assert manifest["category_counts"] == {
        "positive": 2,
        "occluded": 1,
        "negative_no_pallet": 1,
        "negative_lookalike": 1,
    }
    assert manifest["split_counts"] == {"dev": 3, "eval": 2}
    assert set(manifest["scenes"]) == {"s001", "s002", "s003", "s004", "s005"}
    for index in range(1, 6):
        sid = f"s{index:03d}"
        record = manifest["scenes"][sid]
        batch = paths[0 if index <= 3 else 1]
        assert record["batch_run_id"] == batch.name
        assert (
            hashes(output / "scenes" / sid)
            == record["files"]
            == hashes(scene_path(batch, sid))
        )
        assert load_scene_sample(output / "scenes" / sid).ground_truth.status == (
            "valid" if index <= 3 else "no_pallet"
        )
    assert all(Path(p).read_bytes() == value for p, value in before.items())


def test_merge_cli_writes_the_set(batches):
    catalogue, paths, output = batches
    result = subprocess.run(
        [
            sys.executable,
            "tools/merge_scene_batches.py",
            "--catalogue",
            str(catalogue),
            "--batches",
            *map(str, paths),
            "--output",
            str(output),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    assert len(read_json(output / "manifest.json")["scenes"]) == 5


def test_duplicate_scene_across_batches_is_rejected(batches):
    catalogue, paths, output = batches
    rejected((catalogue, [*paths, paths[0]], output), "duplicate")


def test_missing_catalogue_scene_is_rejected(batches):
    catalogue, paths, output = batches
    rejected((catalogue, paths[:1], output), "missing|coverage")


@pytest.mark.parametrize(
    "field,value",
    [
        ("catalogue_version", "different"),
        ("catalogue_sha256", "0" * 64),
        ("camera", {"width": 9}),
        ("image_id", "sha256:other"),
        ("source_snapshot_sha256", "b" * 64),
    ],
)
def test_batch_metadata_mismatch_is_rejected(batches, field, value):
    path = manifest_path(batches[1][1])
    payload = read_json(path)
    payload[field] = value
    write_json(path, payload)
    rejected(batches, field)


def test_hash_must_match_actual_catalogue_even_when_all_batches_agree(batches):
    with batches[0].open("a") as stream:
        stream.write("\n# Changed bytes, identical parsed content\n")
    rejected(batches, "catalogue_sha256")


@pytest.mark.parametrize("kind", ["failed-count", "passed", "error", "batch-error"])
def test_failed_batches_are_rejected_without_partial_recovery(batches, kind):
    path = manifest_path(batches[1][0])
    payload = read_json(path)
    if kind == "failed-count":
        payload["failed_count"] = 1
    elif kind == "batch-error":
        payload["error"] = "setup failed"
    else:
        payload["scenes"]["s001"][kind] = (
            False if kind == "passed" else "cleanup failed"
        )
    write_json(path, payload)
    rejected(batches, "failed|error|passed")


@pytest.mark.parametrize(
    "kind",
    [
        "duplicate-request",
        "missing-result",
        "extra-result",
        "directory",
        "extra-directory",
        "scene-id",
    ],
)
def test_requested_result_directory_and_scene_ids_must_match(batches, kind):
    batch = batches[1][0]
    path = manifest_path(batch)
    payload = read_json(path)
    if kind == "duplicate-request":
        payload["requested_scenes"].append("s001")
    elif kind == "missing-result":
        del payload["scenes"]["s001"]
    elif kind == "extra-result":
        payload["scenes"]["s099"] = deepcopy(payload["scenes"]["s001"])
    elif kind == "directory":
        scene_path(batch).rename(scene_path(batch, "s099"))
    elif kind == "extra-directory":
        scene_path(batch, "s099").mkdir()
    else:
        scene = scene_path(batch) / "scene.json"
        obj = read_json(scene)
        obj["scene_id"] = "s002"
        write_json(scene, obj)
        refresh_scene_hashes(batch)
        payload = read_json(path)
    write_json(path, payload)
    rejected(batches, "ID|scene_id|director|request|result")


@pytest.mark.parametrize(
    "field,value",
    [
        ("category", "occluded"),
        ("split", "eval"),
        ("catalogue_version", "other"),
        ("camera", {"width": 9}),
        ("image_id", "sha256:other"),
        ("source_snapshot_sha256", "b" * 64),
        ("run_id", "other"),
    ],
)
def test_scene_metadata_must_match_catalogue_and_batch(batches, field, value):
    batch = batches[1][0]
    path = scene_path(batch) / "scene.json"
    payload = read_json(path)
    payload[field] = value
    write_json(path, payload)
    refresh_scene_hashes(batch)
    rejected(batches, field)


def test_corrupt_file_hash_is_rejected(batches):
    with (scene_path(batches[1][0]) / "rgb.png").open("ab") as stream:
        stream.write(b"corruption")
    rejected(batches, "hash")


@pytest.mark.parametrize(
    "defect", ["missing-hash", "missing-file", "unsafe-name", "symlink"]
)
def test_only_complete_verified_regular_scene_files_are_copied(batches, defect):
    batch = batches[1][0]
    path = manifest_path(batch)
    payload = read_json(path)
    if defect == "missing-hash":
        del payload["scenes"]["s001"]["files"]["rgb.png"]
    elif defect == "unsafe-name":
        payload["scenes"]["s001"]["files"]["../rgb.png"] = "b" * 64
    else:
        (scene_path(batch) / "rgb.png").unlink()
        if defect == "symlink":
            (scene_path(batch) / "rgb.png").symlink_to(
                scene_path(batch) / "depth_preview.png"
            )
    write_json(path, payload)
    rejected(batches, "file|symlink")


@pytest.mark.parametrize(
    "field,value",
    [
        ("left", {"center_m": [1.8, 0.175, 0.15], "width_m": 0.25, "height_m": 0.2}),
        ("reason", "changed"),
        ("clock_domain", "synthetic"),
        ("stamp_ns", 0),
        ("source_provenance", "synthetic"),
    ],
)
def test_ground_truth_must_be_catalogue_copy_with_only_acquisition_time_replaced(
    batches, field, value
):
    batch = batches[1][0]
    path = scene_path(batch) / "ground_truth.json"
    payload = read_json(path)
    payload[field] = value
    write_json(path, payload)
    refresh_scene_hashes(batch)
    rejected(batches, "ground.truth")


@pytest.mark.parametrize("scene_id", ["s001", "s003", "s004", "s005"])
def test_category_status_contract_is_enforced_even_when_catalogue_and_scene_agree(
    batches, scene_id
):
    catalogue, paths, _ = batches
    data = yaml.safe_load(catalogue.read_text())
    entry = next(e for e in data["scenes"] if e["scene_id"] == scene_id)
    entry["ground_truth"]["status"] = "unsupported"
    catalogue.write_text(yaml.safe_dump(data))
    batch = paths[0 if scene_id < "s004" else 1]
    path = scene_path(batch, scene_id) / "ground_truth.json"
    payload = read_json(path)
    payload["status"] = "unsupported"
    write_json(path, payload)
    refresh_scene_hashes(batch, scene_id)
    for batch in paths:
        path = manifest_path(batch)
        payload = read_json(path)
        payload["catalogue_sha256"] = hashlib.sha256(catalogue.read_bytes()).hexdigest()
        write_json(path, payload)
    rejected(batches, "status")


def test_loader_contract_is_checked_after_hash_verification(batches):
    batch = batches[1][0]
    path = scene_path(batch) / "depth_meta.json"
    payload = read_json(path)
    payload["meters_per_unit"] = 1.0
    write_json(path, payload)
    refresh_scene_hashes(batch)
    rejected(batches, "meters_per_unit")


def test_existing_output_is_never_overwritten(batches):
    output = batches[2]
    output.mkdir()
    marker = output / "keep.txt"
    marker.write_text("preserve me")
    with pytest.raises(FileExistsError):
        merge(batches)
    assert marker.read_text() == "preserve me"
    assert list(output.iterdir()) == [marker]


def test_copy_failure_does_not_leave_a_partial_set(batches, monkeypatch):
    module = api()

    def fail_copy(*args, **kwargs):
        raise OSError("copy failed")

    monkeypatch.setattr(module.shutil, "copyfile", fail_copy)
    with pytest.raises(OSError, match="copy failed"):
        module.merge_batches(
            catalogue=batches[0], batches=batches[1], output=batches[2]
        )
    assert not batches[2].exists()
