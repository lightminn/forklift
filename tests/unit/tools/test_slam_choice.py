"""The live launcher's SLAM mode choice (live-default plan S3 D2 and 5d's switch, deploy/console/slam_choice.sh): unset
MAP_BUNDLE takes the registry's accepted bundle of the scene file and odometry model and, with none, maps online as a
recorded fallback (never silently); MAP_BUNDLE=registry asks for the bundle and stops without one; MAP_BUNDLE=none
maps whatever the registry says; an explicit folder is used as given; a broken registry stops the launch."""

import json
import subprocess
import sys
from pathlib import Path

from tools import slam_bundle_registry as reg

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "deploy" / "console" / "slam_choice.sh"
BASE = {"ROOT": str(ROOT), "VENV_PY": sys.executable, "ENCODER_MODEL": "team_spec_quantized", "PATH": "/usr/bin:/bin",
        "PYTHONPATH": str(ROOT)}


def _run(shown, env):
    return subprocess.run(["bash", "-c", f'set -u; . "{SCRIPT}"; {shown}'], env={**BASE, **env}, capture_output=True,
                          text=True, timeout=60)


def _choose(tmp_path, **env):
    out = _run("printf '%s|%s|%s|%s' \"$MAP_BUNDLE\" \"$EXPECTED_MAP_ID\" \"$MAP_START_POSE\" \"$SLAM_CHOICE\"", env)
    return out.returncode, out.stdout.split("|") if out.returncode == 0 else out.stderr


def _choose_err(tmp_path, **env):
    """The choice and the launch's stderr: a fallback says so where the operator looks."""
    out = _run('printf %s "$SLAM_CHOICE"', env)
    return out.returncode, out.stdout, out.stderr


def _registry(tmp_path, relocalizable=False):
    scene = tmp_path / "line_m_session.json"
    scene.write_text(json.dumps({"name": "line_m"}))
    manifest = {"map_id": "a" * 64, "inputs": {"scene_file": {"path": scene.name, "sha256": reg.sha256(scene)}},
                "run": {"arguments": {"layout": "scene", "encoder_model": "team_spec_quantized"}}}
    row = reg.entry(manifest, {"ok": True, "map_id": "a" * 64, "world": {"ok": True, "position_max_m": 0.09, "yaw_max_rad": 0.006}},
                    folder="line_m_tsq_aaaaaaaaaaaa", start=[1.5, -2.0, 0.25], name="line_m")
    book = tmp_path / "bundles.yaml"
    marked = reg.put({"bundles": []}, row)
    if relocalizable:
        marked = reg.mark(marked, "line_m", True, "a test's smoke")
    reg.write(book, marked)
    return scene, book


def test_the_registry_row_fills_the_bundle_its_id_and_start_pose(tmp_path):
    scene, book = _registry(tmp_path)
    (tmp_path / "root" / "line_m_tsq_aaaaaaaaaaaa").mkdir(parents=True)
    for env in ({"MAP_BUNDLE": "registry"}, {}):  # asked for, and the default (plan 5d, user decision 2026-10-27)
        rc, (bundle, map_id, start, choice) = _choose(tmp_path, SCENE_FILE=str(scene), SLAM_BUNDLE_ROOT=str(tmp_path / "root"),
                                                      SLAM_BUNDLE_REGISTRY=str(book), **env)
        assert rc == 0 and bundle == str(tmp_path / "root" / "line_m_tsq_aaaaaaaaaaaa")
        assert map_id == "a" * 64 and start == "1.5,-2.0,0.25" and choice.startswith("localization (registry")


def test_the_default_without_a_row_maps_as_a_recorded_fallback(tmp_path):
    # no row for the scene and model (the reported model here; any unregistered scene) or no scene file at all (the
    # factory hall): online mapping, and the choice and stderr both say it is a fallback -- never a silent one
    scene, book = _registry(tmp_path)
    for env in ({"SCENE_FILE": str(scene), "ENCODER_MODEL": "reported"}, {}):
        rc, out, err = _choose_err(tmp_path, SLAM_BUNDLE_ROOT=str(tmp_path), SLAM_BUNDLE_REGISTRY=str(book), **env)
        assert rc == 0 and out.startswith("mapping (fallback: "), (env, out, err)
        assert "fallback" in err
    rc, (bundle, map_id, start, choice) = _choose(tmp_path, SCENE_FILE=str(scene), ENCODER_MODEL="reported",
                                                  SLAM_BUNDLE_ROOT=str(tmp_path), SLAM_BUNDLE_REGISTRY=str(book),
                                                  MAP_START_POSE="9,9,9", EXPECTED_MAP_ID="c" * 64)
    assert rc == 0 and bundle == "" and map_id == "" and start == "" and "no registry row" in choice
    rc, (bundle, _, _, choice) = _choose(tmp_path)
    assert rc == 0 and bundle == "" and choice.startswith("mapping (fallback: no SCENE_FILE")


def test_a_registered_scene_without_a_bundle_root_stops_the_launch(tmp_path):
    # a registered scene means localization: a missing root is a configuration error, not a reason to map
    scene, book = _registry(tmp_path)
    rc, err = _choose(tmp_path, SCENE_FILE=str(scene), SLAM_BUNDLE_REGISTRY=str(book))
    assert rc != 0 and "SLAM_BUNDLE_ROOT" in err


def test_an_explicit_registry_stops_without_a_row_and_none_forces_mapping(tmp_path):
    scene, book = _registry(tmp_path)
    rc, err = _choose(tmp_path, MAP_BUNDLE="registry", SCENE_FILE=str(scene), ENCODER_MODEL="reported",
                      SLAM_BUNDLE_ROOT=str(tmp_path), SLAM_BUNDLE_REGISTRY=str(book))
    assert rc != 0 and "no registry row" in err
    rc, err = _choose(tmp_path, MAP_BUNDLE="registry", SLAM_BUNDLE_ROOT=str(tmp_path), SLAM_BUNDLE_REGISTRY=str(book))
    assert rc != 0 and "SCENE_FILE" in err
    rc, (bundle, _, _, choice) = _choose(tmp_path, MAP_BUNDLE="none", SCENE_FILE=str(scene))
    assert rc == 0 and bundle == "" and choice == "mapping (MAP_BUNDLE=none)"


def test_an_explicit_folder_is_used_as_given_and_a_broken_registry_stops(tmp_path):
    scene, book = _registry(tmp_path)
    folder = tmp_path / "given"
    folder.mkdir()
    rc, (bundle, map_id, start, choice) = _choose(tmp_path, MAP_BUNDLE=str(folder), MAP_START_POSE="0,0,0",
                                                  EXPECTED_MAP_ID="b" * 64)
    assert rc == 0 and bundle == str(folder) and map_id == "b" * 64 and choice == f"localization (given: {folder})"
    # a folder whose name is a word the chooser could use internally is still a folder (Codex P3)
    rc, (bundle, _, _, choice) = _choose(tmp_path, MAP_BUNDLE="default", MAP_START_POSE="0,0,0")
    assert rc == 0 and bundle == "default" and choice == "localization (given: default)"
    book.write_text("bundles: [ {broken")
    for env in ({"MAP_BUNDLE": "registry"}, {}):  # the default too: a registry that does not read is never "no row"
        rc, err = _choose(tmp_path, SCENE_FILE=str(scene), SLAM_BUNDLE_ROOT=str(tmp_path), SLAM_BUNDLE_REGISTRY=str(book),
                          **env)
        assert rc != 0 and "registry" in err


def test_the_registry_row_says_whether_its_start_relocalizes(tmp_path):
    # lead 2026-10-31 (a): MAP_START_RELOC is the row's mark, yes or no; empty wherever no registry row was taken
    shown = 'printf %s "$MAP_START_RELOC"'
    for relocalizable, want in ((False, "no"), (True, "yes")):
        d = tmp_path / want
        d.mkdir()
        scene, book = _registry(d, relocalizable=relocalizable)
        out = _run(shown, {"SCENE_FILE": str(scene), "SLAM_BUNDLE_ROOT": str(d), "SLAM_BUNDLE_REGISTRY": str(book)})
        assert out.returncode == 0 and out.stdout == want, out.stderr
    for env in ({"MAP_BUNDLE": "none", "SCENE_FILE": str(scene)}, {"MAP_BUNDLE": str(tmp_path), "MAP_START_POSE": "0,0,0"},
                {"SCENE_FILE": str(scene), "ENCODER_MODEL": "reported", "SLAM_BUNDLE_REGISTRY": str(book)}):
        out = _run(shown, {"MAP_START_RELOC": "yes", **env})  # never inherited from the caller
        assert out.returncode == 0 and out.stdout == "", (env, out.stderr)
