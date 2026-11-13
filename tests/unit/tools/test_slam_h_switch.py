"""H's switch (online-SLAM plan 1-H): the start pose handed to the localization and the switch summary."""

from __future__ import annotations

import json
import math

import pytest

from tools import slam_h_switch as hs


def _line(k, unit, recv, wall, mfo=(0.0, 0.0, 0.0), odom=(0.0, 0.0, 0.0)):
    return json.dumps({"scan_id": k, "unit": unit, "slam_wall_s": wall, "recv_mono": recv, "sent_mono": recv - wall,
                       "map_from_odom": list(mfo), "odom_base": list(odom), "stamp_s": float(k)})


def _write(path, lines):
    path.write_text("\n".join(lines) + "\n")
    return path


def test_the_start_pose_is_the_mappings_own_last_estimate(tmp_path):
    drive = _write(tmp_path / "map.jsonl", [_line(0, 0, 1.0, 0.1),
                                            _line(1, 0, 2.0, 0.1, mfo=(1.0, 0.0, math.pi / 2), odom=(2.0, 0.0, 0.0))])
    x, y, yaw = (float(v) for v in hs.start_pose(drive).split(","))
    assert (x, y, yaw) == pytest.approx((1.0, 2.0, math.pi / 2))


def test_the_switch_wait_runs_from_the_last_mapping_reply_to_the_first_localization_reply(tmp_path):
    m = _write(tmp_path / "map.jsonl", [_line(k, 0, 10.0 + k, 0.2) for k in range(5)])
    loc = _write(tmp_path / "loc.jsonl", [_line(5, 1, 50.0, 0.3), _line(6, 1, 51.0, 1.2), _line(7, 2, 52.0, 0.4)])
    out = hs.summarize(m, loc, steps={"map_driver_end": 14.5, "map_container_end": 30.0, "bundle_end": 31.0})
    assert out["switch_wait_s"] == pytest.approx(50.0 - 14.0)
    assert out["switch_pass"] is True
    assert out["after_keyframe_max_s"] == pytest.approx(1.2)
    assert out["after_keyframes_at_or_over_1s"] == 1 and out["after_pass"] is False
    assert out["steps_from_last_mapping_reply_s"] == pytest.approx({"map_driver_end": 0.5, "map_container_end": 16.0,
                                                                     "bundle_end": 17.0})
    assert out["mapping_units"] == [0] and out["localization_units"] == [1, 2]


def test_a_wait_over_60_s_fails_and_phases_that_overlap_or_mix_units_are_refused(tmp_path):
    m = _write(tmp_path / "map.jsonl", [_line(0, 0, 10.0, 0.2)])
    late = _write(tmp_path / "late.jsonl", [_line(1, 1, 70.5, 0.2)])
    assert hs.summarize(m, late)["switch_pass"] is False
    for bad in ([_line(1, 1, 9.0, 0.2)],  # a localization reply before the mapping ended
                [_line(1, 0, 20.0, 0.2)]):  # unit 0 again after the switch
        with pytest.raises(SystemExit):
            hs.summarize(m, _write(tmp_path / "bad.jsonl", bad))
    with pytest.raises(SystemExit):  # the mapping phase sent a later unit
        hs.summarize(_write(tmp_path / "m2.jsonl", [_line(0, 0, 1.0, 0.1), _line(1, 1, 2.0, 0.1)]), late)
    with pytest.raises(SystemExit):  # an empty phase
        hs.summarize(_write(tmp_path / "empty.jsonl", []), late)
