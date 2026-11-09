"""Real-time plan 5.23 G2-1a: the check's measures on synthetic traces -- the capture tick (the control tick after the
capture step), control-made gaps, the capture files' completeness -- and the registered pass lines."""

import importlib.util
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "g2_1a_analyze", ROOT / "docs" / "validation" / "2026-11-10-g2-1a-check" / "g2_1a_analyze.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)
C, BIT = tool.C, tool.BIT


def trace(n=40, dt=1 / 120):
    D = np.zeros((n, len(C)))
    S = np.arange(n) * dt
    D[:, C["S"]] = S
    D[:, C["q"]] = S
    D[:, C["world_start"]] = S
    D[:, C["world_end"]] = S + 0.002
    D[:, C["step_end"]] = S + 0.003
    D[:, C["after_end"]] = S + 0.004
    D[:, C["ctrl_start"]] = S - 0.001
    D[:, C["ctrl_end"]] = S
    return D


def shift(D, row, by):
    D[row:, [C["S"], C["q"], C["world_start"], C["world_end"], C["step_end"], C["after_end"], C["ctrl_start"],
             C["ctrl_end"]]] += by


def test_the_capture_tick_is_the_control_tick_after_the_capture_step():
    D = trace()
    D[10, C["flags"]] = BIT["capture"]
    shift(D, 11, 0.05)
    D[11, C["ctrl_start"]] = D[11, C["ctrl_end"]] - 0.05  # the save in the tick before step 11
    c = tool.capture_ticks(D)
    assert c["rows"] == [10]
    assert abs(c["control_next_s"]["median"] - 0.05) < 1e-9 and c["control_same_row_s"]["median"] < 0.01
    assert abs(c["gap_s"]["median"] - (1 / 120 + 0.05)) < 1e-9


def test_gaps_are_counted_near_and_over_and_split_by_their_largest_part():
    D = trace(60)
    shift(D, 11, 0.22)  # near, control-made
    D[11, C["ctrl_start"]] = D[11, C["ctrl_end"]] - 0.21
    shift(D, 31, 0.30)  # over, world-made
    D[30, C["world_end"]] = D[30, C["world_start"]] + 0.29
    D[30, C["step_end"]] = D[30, C["world_end"]] + 0.001
    D[30, C["after_end"]] = D[30, C["step_end"]] + 0.001
    g = tool.gap_counts(D, 0)
    assert g == {"near": 1, "near_control": 1, "over": 1, "over_control": 0}


def test_capture_files_must_be_whole_and_open(tmp_path):
    from PIL import Image

    Image.new("RGB", (4, 4)).save(tmp_path / "perception_capture_1_rgb.png")
    np.save(tmp_path / "perception_capture_1_depth_m.npy", np.ones((4, 4)))
    Image.new("RGB", (4, 4)).save(tmp_path / "perception_capture_2_rgb.png")
    (tmp_path / "perception_capture_3_depth_vis.png").write_bytes(b"not a png")
    f = tool.capture_files(tmp_path, [{"attempt": 1, "done": True}, {"attempt": 4, "done": True}])
    assert f["attempts"] == [1, 2, 3]
    assert "2:depth_m.npy" in f["missing"] and "3:rgb.png" in f["missing"]
    assert any(m.startswith("4:") for m in f["missing"]) and f["unreadable"] == ["perception_capture_3_depth_vis.png: UnidentifiedImageError"]


def row(n, scene, mode, *, near=(0, 0), over=(0, 0), cap=0.06, done=True, problems=False):
    return {"name": f"g2a_{n:02d}_{scene}_{mode}", "n": n, "scene": scene, "mode": mode, "invalid": [],
            "mission_done": done, "gaps": {"near": near[1], "near_control": near[0], "over": over[1],
                                           "over_control": over[0]},
            "capture": {"control_next_s": {"median": cap}},
            "files": {"missing": ["x"] if problems else [], "unreadable": []},
            "capture_dump": {"mode": "async", "dropped_full": 0, "errors": 0, "unsettled": 0} if mode == "async" else None,
            "log_dropped_lines": 0}


def good():
    rows = []
    for (a, b), scene in [((1, 2), "H"), ((5, 6), "H"), ((9, 10), "H"), ((3, 4), "F"), ((7, 8), "F"), ((11, 12), "F")]:
        first, second = ("sync", "async") if a in (1, 9, 3, 11) else ("async", "sync")
        for n, m in ((a, first), (b, second)):
            rows.append(row(n, scene, m, near=(2, 5) if m == "sync" else (0, 3), over=(0, 1),
                            cap=0.10 if m == "sync" else 0.05))
    return rows


def test_the_registered_lines_pass_and_each_one_can_fail():
    assert tool.verdict(good())["pass"]
    rows = good()
    rows[1]["gaps"]["over_control"] = 1  # an async control-made gap over 0.25 s
    assert not tool.verdict(rows)["lines"]["1_no_control_over_0_25_async"]["pass"]
    rows = good()
    for r in rows:
        r["capture"]["control_next_s"]["median"] = 0.07 if r["mode"] == "sync" else 0.05  # drops of 0.02 only
    v = tool.verdict(rows)["lines"]["3_capture_control_lower"]
    assert not v["pass"] and abs(v["median_drop_s"] - 0.02) < 1e-12
    rows = good()
    next(r for r in rows if r["mode"] == "async")["mission_done"] = False
    assert not tool.verdict(rows)["lines"]["4_missions_no_worse"]["pass"]
    rows = good()
    rows[3]["files"]["missing"] = ["7:rgb.png"]
    assert not tool.verdict(rows)["lines"]["5_no_loss"]["pass"]
    rows = good()
    for r in rows:
        r["gaps"]["over"] = 2 if r["mode"] == "async" else 1
    assert not tool.verdict(rows)["lines"]["6_any_source_over_0_25_not_more"]["pass"]
    rows = good()
    rows[0]["invalid"] = ["probe exit 1"]
    v = tool.verdict(rows)
    assert v["lines"]["3_capture_control_lower"]["left_out"] == [[1, 2]] and not v["pass"]


def test_no_control_made_near_gap_in_either_arm_holds_line_2():
    rows = good()
    for r in rows:
        r["gaps"]["near_control"] = 0
    assert tool.verdict(rows)["lines"]["2_near_control_share_lower"]["pass"]
