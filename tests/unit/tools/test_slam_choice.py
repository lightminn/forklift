"""The live launcher's SLAM mode choice (live-default plan S3 D2, deploy/console/slam_choice.sh): MAP_BUNDLE=registry
takes the registry's accepted bundle of the scene file and odometry model, none registered maps online, MAP_BUNDLE=none
maps whatever the registry says, an explicit folder is used as given, a broken registry stops the launch."""

import json
import subprocess
import sys
from pathlib import Path

from tools import slam_bundle_registry as reg

ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "deploy" / "console" / "slam_choice.sh"


def _choose(tmp_path, **env):
    shown = "printf '%s|%s|%s|%s' \"$MAP_BUNDLE\" \"$EXPECTED_MAP_ID\" \"$MAP_START_POSE\" \"$SLAM_CHOICE\""
    base = {"ROOT": str(ROOT), "VENV_PY": sys.executable, "ENCODER_MODEL": "team_spec_quantized",
            "PATH": "/usr/bin:/bin", "PYTHONPATH": str(ROOT)}
    out = subprocess.run(["bash", "-c", f'set -u; . "{SCRIPT}"; {shown}'], env={**base, **env}, capture_output=True,
                         text=True, timeout=60)
    return out.returncode, out.stdout.split("|") if out.returncode == 0 else out.stderr


def _registry(tmp_path):
    scene = tmp_path / "line_m_session.json"
    scene.write_text(json.dumps({"name": "line_m"}))
    manifest = {"map_id": "a" * 64, "inputs": {"scene_file": {"path": scene.name, "sha256": reg.sha256(scene)}},
                "run": {"arguments": {"layout": "scene", "encoder_model": "team_spec_quantized"}}}
    row = reg.entry(manifest, {"ok": True, "world": {"ok": True, "position_max_m": 0.09, "yaw_max_rad": 0.006}},
                    folder="line_m_tsq_aaaaaaaaaaaa", start=[1.5, -2.0, 0.25], name="line_m")
    book = tmp_path / "bundles.yaml"
    reg.write(book, reg.put({"bundles": []}, row))
    return scene, book


def test_the_registry_row_fills_the_bundle_its_id_and_start_pose(tmp_path):
    scene, book = _registry(tmp_path)
    (tmp_path / "root" / "line_m_tsq_aaaaaaaaaaaa").mkdir(parents=True)
    rc, (bundle, map_id, start, choice) = _choose(tmp_path, MAP_BUNDLE="registry", SCENE_FILE=str(scene),
                                                  SLAM_BUNDLE_ROOT=str(tmp_path / "root"), SLAM_BUNDLE_REGISTRY=str(book))
    assert rc == 0 and bundle == str(tmp_path / "root" / "line_m_tsq_aaaaaaaaaaaa")
    assert map_id == "a" * 64 and start == "1.5,-2.0,0.25" and choice.startswith("localization (registry")


def test_no_row_maps_online_and_none_forces_mapping(tmp_path):
    scene, book = _registry(tmp_path)
    rc, (bundle, map_id, _, choice) = _choose(tmp_path, MAP_BUNDLE="registry", SCENE_FILE=str(scene),
                                              ENCODER_MODEL="reported", SLAM_BUNDLE_ROOT=str(tmp_path),
                                              SLAM_BUNDLE_REGISTRY=str(book))
    assert rc == 0 and bundle == "" and map_id == "" and choice.startswith("mapping (no registry row")
    rc, (bundle, _, _, choice) = _choose(tmp_path, MAP_BUNDLE="none", SCENE_FILE=str(scene))
    assert rc == 0 and bundle == "" and choice == "mapping (MAP_BUNDLE=none)"
    rc, (bundle, _, _, choice) = _choose(tmp_path)
    assert rc == 0 and bundle == "" and choice == "mapping (MAP_BUNDLE unset)"


def test_an_explicit_folder_is_used_as_given_and_a_broken_registry_stops(tmp_path):
    scene, book = _registry(tmp_path)
    folder = tmp_path / "given"
    folder.mkdir()
    rc, (bundle, map_id, start, choice) = _choose(tmp_path, MAP_BUNDLE=str(folder), MAP_START_POSE="0,0,0",
                                                  EXPECTED_MAP_ID="b" * 64)
    assert rc == 0 and bundle == str(folder) and map_id == "b" * 64 and choice == f"localization (given: {folder})"
    book.write_text("bundles: [ {broken")
    rc, err = _choose(tmp_path, MAP_BUNDLE="registry", SCENE_FILE=str(scene), SLAM_BUNDLE_ROOT=str(tmp_path),
                      SLAM_BUNDLE_REGISTRY=str(book))
    assert rc != 0 and "registry" in err
