"""Equivalence of two cost replays of the same input (online-SLAM plan 5 S1 equivalence, patch build design v3 4): the
processed scans in order, the reply poses within 1e-6 m / 1e-6 rad, the end graph's node and edge counts and the
effective parameters; each run's own slam_toolbox check passed."""

from __future__ import annotations

import json

import pytest

from tools import slam_equivalence as eq


def make_run(root, name, rows, *, nodes=3, edges=2, params="a: 1\n", patch_ok=True, kind=""):
    d = root / name
    (d / "bridge" / "dump").mkdir(parents=True)
    (d / "drive.jsonl").write_text("".join(json.dumps(r) + "\n" for r in rows))
    (d / "bridge" / "dump" / "nodes.csv").write_text("unique_id\n" + "".join(f"{i}\n" for i in range(nodes)))
    (d / "bridge" / "dump" / "edges.csv").write_text("source,target\n" + "".join(f"{i},{i + 1}\n" for i in range(edges)))
    (d / "bridge" / "params_effective.yaml").write_text(params)
    (d / "bridge" / "patch_check.json").write_text(json.dumps({"ok": patch_ok, "kind": kind}))
    return d


def row(k, x, y=0.0, yaw=0.0):
    return {"scan_id": k, "stamp_s": 0.1 * k, "map_from_odom": [x, y, yaw], "slam_wall_s": 0.01}


def test_identical_replays_are_equivalent(tmp_path):
    rows = [row(k, 0.001 * k) for k in range(5)]
    a, b = make_run(tmp_path, "r0", rows), make_run(tmp_path, "p0", rows, kind="patched")
    out = eq.compare(a, b)
    assert out["equivalent"] and out["max_position_diff_m"] == 0.0 and out["scans"] == 5


def test_a_pose_difference_above_1e6_or_a_different_scan_sequence_breaks_it(tmp_path):
    rows = [row(k, 0.001 * k) for k in range(5)]
    a = make_run(tmp_path, "a", rows)
    near = [dict(r) for r in rows]
    near[3] = row(3, 0.003 + 5e-7)
    assert eq.compare(a, make_run(tmp_path, "b", near))["equivalent"]
    far = [dict(r) for r in rows]
    far[3] = row(3, 0.003 + 2e-6)
    out = eq.compare(a, make_run(tmp_path, "c", far))
    assert not out["equivalent"] and any("position" in p for p in out["problems"])
    yaw = [dict(r) for r in rows]
    yaw[2] = row(2, 0.002, yaw=2e-6)
    assert any("heading" in p for p in eq.compare(a, make_run(tmp_path, "d", yaw))["problems"])
    missing = rows[:2] + rows[3:]
    assert any("scan" in p for p in eq.compare(a, make_run(tmp_path, "e", missing))["problems"])


def test_the_graph_parameters_and_each_runs_own_check_must_agree(tmp_path):
    rows = [row(k, 0.0) for k in range(3)]
    a = make_run(tmp_path, "a", rows)
    assert any("graph" in p for p in eq.compare(a, make_run(tmp_path, "b", rows, edges=3))["problems"])
    assert any("parameters" in p for p in eq.compare(a, make_run(tmp_path, "c", rows, params="a: 2\n"))["problems"])
    assert any("check" in p for p in eq.compare(a, make_run(tmp_path, "d", rows, patch_ok=False))["problems"])


def test_a_missing_graph_dump_is_a_failure_not_a_pass(tmp_path):
    rows = [row(k, 0.0) for k in range(3)]
    a, b = make_run(tmp_path, "a", rows), make_run(tmp_path, "b", rows)
    (b / "bridge" / "dump" / "edges.csv").unlink()
    with pytest.raises(SystemExit):
        eq.compare(a, b)
