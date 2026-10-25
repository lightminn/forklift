"""The survey map bundle registry (live-default plan S3 D1/D2): accepted bundles only, found by the scene file's content
and the odometry model, each with its full map id and its start pose as map <- base_link."""

import json
import math

import numpy as np
import pytest

from tools import slam_bundle_registry as reg


def _manifest(scene_hash="s" * 64, model="team_spec_quantized", map_id="a" * 64):
    return {"map_id": map_id, "inputs": {"scene_file": {"path": "line_m_session.json", "sha256": scene_hash}},
            "run": {"arguments": {"layout": "scene", "encoder_model": model}}}


def _report(ok=True, world_ok=True, position=0.092, yaw=0.006):
    return {"ok": ok, "world": {"ok": world_ok, "position_max_m": position, "yaw_max_rad": yaw}}


def test_the_start_pose_is_the_base_link_pose_whatever_its_heading():
    yaw = 2.5  # not 0: the quaternion's heading, not the rear axle's place
    row = np.array([1.0, -2.0, 0.1, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
    x, y, h = reg.start_pose(row)
    assert (x, y) == (1.0, -2.0) and math.isclose(h, yaw)


def test_only_an_accepted_survey_enters_the_registry():
    e = reg.entry(_manifest(), _report(), folder="line_m_tsq_aaaaaaaaaaaa", start=[1.0, 2.0, 0.5], name="line_m")
    assert e["map_id"] == "a" * 64 and e["scene_sha256"] == "s" * 64 and e["encoder_model"] == "team_spec_quantized"
    assert e["start_pose"] == [1.0, 2.0, 0.5] and e["v2"] == {"position_max_m": 0.092, "yaw_max_rad": 0.006}
    for bad in (_report(ok=False), _report(world_ok=False)):
        with pytest.raises(ValueError, match="not accepted"):
            reg.entry(_manifest(), bad, folder="f", start=[0, 0, 0], name="line_m")
    with pytest.raises(ValueError, match="scene file"):
        reg.entry({**_manifest(), "inputs": {}}, _report(), folder="f", start=[0, 0, 0], name="factory")


def test_lookup_by_scene_content_and_odometry_model():
    a = reg.entry(_manifest(), _report(), folder="fa", start=[0, 0, 0], name="line_m")
    b = reg.entry(_manifest(scene_hash="t" * 64, map_id="b" * 64), _report(), folder="fb", start=[0, 0, 0],
                  name="random_m")
    book = reg.put(reg.put({"bundles": []}, a), b)
    assert reg.lookup(book, "s" * 64, "team_spec_quantized")["folder"] == "fa"
    assert reg.lookup(book, "t" * 64, "team_spec_quantized")["folder"] == "fb"
    assert reg.lookup(book, "s" * 64, "reported") is None  # a bundle is bound to its odometry model
    assert reg.lookup(book, "u" * 64, "team_spec_quantized") is None
    # a new accepted survey of the same scene and model replaces the old row
    a2 = reg.entry(_manifest(map_id="c" * 64), _report(position=0.05), folder="fa2", start=[0, 0, 0], name="line_m")
    book = reg.put(book, a2)
    assert len(book["bundles"]) == 2 and reg.lookup(book, "s" * 64, "team_spec_quantized")["map_id"] == "c" * 64


def test_the_lookup_command_prints_what_the_launcher_reads(tmp_path, capsys):
    scene = tmp_path / "line_m_session.json"
    scene.write_text(json.dumps({"name": "line_m"}))
    row = reg.entry(_manifest(scene_hash=reg.sha256(scene)), _report(), folder="fa", start=[1.0, -2.0, 2.5],
                    name="line_m")
    book = tmp_path / "bundles.yaml"
    reg.write(book, reg.put({"bundles": []}, row))
    assert reg.main(["lookup", str(book), "--scene-file", str(scene), "--encoder-model", "team_spec_quantized"]) == 0
    assert capsys.readouterr().out.split() == ["fa", "a" * 64, "1.0,-2.0,2.5"]
    assert reg.main(["lookup", str(book), "--scene-file", str(scene), "--encoder-model", "reported"]) == reg.NONE
