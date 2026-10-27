"""Pose lists for the scan capture (tools/global_match_capture.py, plan B4a): map and hall poses, keep-outs from a run
record, and earlier lists of another map kept apart in the hall frame."""

import json
import math

import numpy as np

from forklift_core.localization import relocalization_samples as S
from forklift_core.localization.slam_pose import compose
from tools import global_match_capture as T


def _bundle(path, map_from_world):
    path.mkdir()
    data = np.zeros((160, 240), np.int8)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    np.savez(path / "grid.npz", data=data, origin=np.array([0.0, 0.0, 0.0]), resolution_m=0.05)
    (path / "manifest.json").write_text(json.dumps({"map_id": "m" * 64, "map_from_world": list(map_from_world)}))
    return path


def _result(path, pickup=(6.0, 4.0, 0.0)):
    sc = {"props": [], "pickup": {"x_m": pickup[0], "y_m": pickup[1], "yaw_rad": pickup[2]}}
    path.write_text(json.dumps({"arguments": {}, "scenario": sc}))
    return path


def test_poses_carry_their_hall_pose_and_avoid_the_pickup_pallet(tmp_path):
    mfw = (1.0, -2.0, 0.3)
    b = _bundle(tmp_path / "b", mfw)
    out = tmp_path / "p.json"
    T.main(["poses", "--bundle", str(b), "--keep-out", str(_result(tmp_path / "r.json")), "--split", "dev",
            "--map-index", "0", "--seed", "5", "--sep-m", "1.5", "--count", "6", "--out", str(out)])
    rec = json.loads(out.read_text())
    assert rec["format"] == "scan_capture_poses/1" and rec["split"] == 0 and len(rec["poses"]) == 6
    pallet = T.keep_out_rectangles(tmp_path / "r.json")
    for row in rec["poses"]:
        assert np.allclose(compose(mfw, tuple(row["base_world"])), row["base"], atol=1e-9)
        corners = S.footprint_corners(row["base"])
        assert not S._overlap(corners, pallet[0])


def test_a_later_list_keeps_apart_from_another_maps_list_in_the_hall_frame(tmp_path):
    b1 = _bundle(tmp_path / "b1", (0.0, 0.0, 0.0))
    b2 = _bundle(tmp_path / "b2", (0.5, 0.0, 0.0))  # the second map's frame is the hall moved 0.5 m
    first, second = tmp_path / "a.json", tmp_path / "b.json"
    T.main(["poses", "--bundle", str(b1), "--split", "dev", "--map-index", "0", "--seed", "1", "--sep-m", "1.5",
            "--count", "8", "--out", str(first)])
    T.main(["poses", "--bundle", str(b2), "--split", "val", "--map-index", "1", "--seed", "2", "--sep-m", "3.0",
            "--fill", "--max-attempts", "4000", "--taken", str(first), "--out", str(second)])
    a = [tuple(r["base_world"]) for r in json.loads(first.read_text())["poses"]]
    b = [tuple(r["base_world"]) for r in json.loads(second.read_text())["poses"]]
    assert b and all(S.independent(p, a, sep_m=3.0) for p in b)
    assert all(math.hypot(*np.subtract(S.laser_xy(p), S.laser_xy(q))) >= 3.0 for p in b for q in b if p != q)


def test_the_capture_command_takes_the_surveys_world():
    scene = {"run": {"arguments": {"layout": "scene", "seed": 0, "prism_colliders": False, "min_obstacle_height_m": None,
                                   "encoder_model": "team_spec_quantized", "pallet_urdf": "p.urdf",
                                   "pallet_geometry": "g.yaml", "settings": "s.yaml", "lidar_keepout_m": 0.15,
                                   "exit_clearance_m": None}},
             "inputs": {k: {"path": f"/in/{k}"} for k in ("base_scene", "forklift_urdf", "factory_layout", "lidar",
                                                         "obstacle_layer", "scene_file")}}
    argv = T.capture_argv(scene, bundle="B", poses="P", out_npz="O.npz", output_dir="R")
    assert argv[argv.index("--scene-file") + 1] == "/in/scene_file" and "--near-capture-redraw" in argv
    assert argv[argv.index("--lidar-keepout-m") + 1] == "0.15" and "--exit-clearance-m" not in argv
    assert argv[argv.index("--obstacle-layer") + 1] == "/in/obstacle_layer" and "--prism-colliders" not in argv
    hall = json.loads(json.dumps(scene))
    hall["run"]["arguments"].update(layout="factory", prism_colliders=True, min_obstacle_height_m=1.15,
                                    lidar_keepout_m=None)
    argv = T.capture_argv(hall, bundle="B", poses="P", out_npz="O.npz", output_dir="R")
    assert "--scene-file" not in argv and "--prism-colliders" in argv
    assert argv[argv.index("--min-obstacle-height-m") + 1] == "1.15" and argv[argv.index("--layout") + 1] == "factory"
