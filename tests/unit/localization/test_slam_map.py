"""The survey map bundle and the localization start checks (SLAM localization plan L2, L4)."""

import json
import math

import numpy as np
import pytest

from forklift_core.localization import slam_map as sm


def _grid(origin=(0.0, 0.0, 0.0), shape=(20, 20), walls=((5, slice(2, 18)),)):
    data = np.zeros(shape, np.int8)
    for row, cols in walls:
        data[row, cols] = 100
    return sm.Grid(data, tuple(origin), 0.1)


def _bundle(tmp_path, grid):
    folder = tmp_path / "bundle"
    folder.mkdir()
    (folder / "map.posegraph").write_bytes(b"graph")
    (folder / "map.data").write_bytes(b"data")
    np.savez(folder / "grid.npz", data=grid.data, origin=np.asarray(grid.origin), resolution_m=grid.resolution_m)
    hashes = {name: sm.file_sha256(folder / name) for name in sm.BUNDLE_FILES}
    manifest = {"format": sm.FORMAT, "map_id": sm.map_id_of(hashes), "files": hashes, "seed": 0,
                "inputs": {name: {"path": name, "sha256": "h_" + name} for name in sm.SHARED_INPUTS}}
    (folder / "manifest.json").write_text(json.dumps(manifest))
    return folder, manifest


def test_a_bundle_loads_with_its_map_id_and_any_change_is_a_mismatch(tmp_path):
    folder, manifest = _bundle(tmp_path, _grid())
    bundle = sm.load_bundle(folder, manifest["map_id"])
    assert bundle.map_id == manifest["map_id"] and bundle.graph_prefix == folder / "map"
    with pytest.raises(sm.MapBundleError, match="slam_map_mismatch: map_id"):
        sm.load_bundle(folder, "0" * 64)
    (folder / "map.data").write_bytes(b"other")
    with pytest.raises(sm.MapBundleError, match="slam_map_mismatch: the files differ"):
        sm.load_bundle(folder)
    (folder / "map.data").unlink()
    with pytest.raises(sm.MapBundleError, match="slam_map_mismatch: map.data missing"):
        sm.load_bundle(folder)


def test_a_run_on_another_seed_or_input_file_does_not_match_the_survey(tmp_path):
    _, manifest = _bundle(tmp_path, _grid())
    same = {name: "h_" + name for name in sm.SHARED_INPUTS}
    assert sm.run_mismatches(manifest, seed=0, input_sha256=same) == []
    assert sm.run_mismatches(manifest, seed=1, input_sha256=same) == ["seed 1 is not the survey's 0"]
    assert sm.run_mismatches(manifest, seed=0, input_sha256={**same, "lidar": "x"}) == ["lidar differs from the survey's"]


def test_coverage_compares_cells_in_world_whatever_the_two_grids_origins():
    survey = _grid()
    shifted = _grid(origin=(-0.5, -0.5, 0.0), shape=(30, 30), walls=((10, slice(7, 23)),))  # the same wall
    assert sm.map_coverage(survey, shifted) == 1.0
    empty = sm.Grid(np.zeros((30, 30), np.int8), (-0.5, -0.5, 0.0), 0.1)
    assert sm.map_coverage(survey, empty) == 0.0
    assert math.isnan(sm.map_coverage(sm.Grid(np.zeros((5, 5), np.int8), (0, 0, 0), 0.1), survey))
    # a rotated origin: the same wall seen by a grid turned a quarter about its own corner
    rotated = sm.Grid(np.rot90(survey.data, k=-1).copy(), (0.0, 2.0, -math.pi / 2), 0.1)
    assert sm.map_coverage(survey, rotated) == 1.0


def test_endpoints_on_the_survey_walls_count_and_nan_or_no_return_fails():
    survey = _grid()  # wall row 5: y 0.5-0.6, x 0.2-1.8
    laser = (1.0, 0.1, math.pi / 2)  # looking up at the wall 0.4-0.5 m away
    ranges = np.array([0.45, 0.45, np.inf, np.nan])
    assert sm.endpoint_fraction(survey, laser, ranges, -0.01, 0.01, 0.2, 12.0) == 1.0
    assert sm.endpoint_fraction(survey, laser, np.array([1.5, 0.45]), 0.0, 0.0, 0.2, 12.0) == 0.5
    assert math.isnan(sm.endpoint_fraction(survey, laser, np.array([np.inf, np.nan]), 0.0, 0.0, 0.2, 12.0))
    assert not sm.localized([math.nan], [(0.0, 0.0, 0.0)], (0.0, 0.0, 0.0))


def test_readiness_needs_every_scan_on_the_walls_and_near_the_start_across_pi():
    start = (1.0, 2.0, math.pi - 0.01)
    good = [(1.02, 2.0, -math.pi + 0.02)] * 3  # 0.03 rad across the wrap
    assert sm.localized([0.9, 0.85, 0.81], good, start)
    assert not sm.localized([0.9, 0.79, 0.95], good, start)
    assert not sm.localized([0.9, 0.9, 0.9], good[:2] + [(1.2, 2.0, math.pi)], start)
    assert not sm.localized([], [], start)


@pytest.mark.parametrize("manifest", ["null", "[]", '{"format": "slam_map_bundle/1"}'])
def test_a_manifest_that_is_not_a_bundle_manifest_is_a_mismatch_not_a_crash(tmp_path, manifest):
    """Codex S3 review P2: null or [] raised AttributeError, past the supervisor's and the bridge's reason paths."""
    folder, _ = _bundle(tmp_path, _grid())
    (folder / "manifest.json").write_text(manifest)
    with pytest.raises(sm.MapBundleError, match="slam_map_mismatch"):
        sm.load_bundle(folder)


def test_a_grid_that_is_not_a_map_is_a_mismatch(tmp_path):
    folder, manifest = _bundle(tmp_path, _grid())
    np.savez(folder / "grid.npz", data=np.zeros(4, np.int8), origin=np.zeros(3), resolution_m=0.05)
    hashes = {name: sm.file_sha256(folder / name) for name in sm.BUNDLE_FILES}
    (folder / "manifest.json").write_text(json.dumps({**manifest, "files": hashes, "map_id": sm.map_id_of(hashes)}))
    with pytest.raises(sm.MapBundleError, match="not a 2-D map"):
        sm.load_bundle(folder)


def test_another_scene_or_scene_argument_does_not_match_the_survey(tmp_path):
    """Codex S3 review P2: the base scene file alone missed a referenced USD and the colliders."""
    _, manifest = _bundle(tmp_path, _grid())
    manifest["run"] = {"scene_sha256": {"scene.usda": "s", "props.usda": "p"}, "arguments": {"prism_colliders": True}}
    same = {name: "h_" + name for name in sm.SHARED_INPUTS}
    ok = dict(seed=0, input_sha256=same, scene_sha256={"scene.usda": "s", "props.usda": "p"},
              arguments={"prism_colliders": True})
    assert sm.run_mismatches(manifest, **ok) == []
    assert sm.run_mismatches(manifest, **{**ok, "scene_sha256": {"scene.usda": "s", "props.usda": "q"}}) == [
        "the scene's USD files differ from the survey's"]
    assert sm.run_mismatches(manifest, **{**ok, "arguments": {"prism_colliders": False}}) == [
        "argument prism_colliders differs from the survey's"]
    noise = {**ok, "arguments": {"prism_colliders": True, "slam_noise_seed": 7}}
    assert sm.run_mismatches(manifest, **noise) == ["argument slam_noise_seed differs from the survey's"]  # Codex r2
    assert "obstacle_layer differs from the survey's" in sm.run_mismatches(
        manifest, **{**ok, "input_sha256": {**same, "obstacle_layer": "x"}})
