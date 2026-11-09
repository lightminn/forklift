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


def test_the_saved_capture_tick_follows_the_last_row_of_a_capture_run():
    D = trace(60)
    D[10:20, C["flags"]] = BIT["capture"]  # ten retry renders, the frame taken at row 19 (Codex review P1)
    D[30, C["flags"]] = BIT["capture"]  # a second capture taken at once
    for row in (20, 31):
        shift(D, row, 0.05)
        D[row, C["ctrl_start"]] = D[row, C["ctrl_end"]] - 0.05  # the save in the tick after the run
    c = tool.capture_ticks(D)
    assert c["rows"] == [19, 30] and c["runs"] == 2 and c["flagged_rows"] == 11 and c["runs_suspect"] == 0
    assert abs(c["control_next_s"]["median"] - 0.05) < 1e-9 and c["control_last_flagged_row_s"]["median"] < 0.01


def test_gaps_are_counted_near_and_over_and_split_by_their_largest_part():
    D = trace(80)
    shift(D, 11, 0.22)  # near, control-made
    D[11, C["ctrl_start"]] = D[11, C["ctrl_end"]] - 0.21
    shift(D, 51, 0.22)  # near, mostly outside the measured parts: control the largest part but under half the gap
    D[51, C["ctrl_start"]] = D[51, C["ctrl_end"]] - 0.01
    shift(D, 31, 0.30)  # over, world-made
    D[30, C["world_end"]] = D[30, C["world_start"]] + 0.29
    D[30, C["step_end"]] = D[30, C["world_end"]] + 0.001
    D[30, C["after_end"]] = D[30, C["step_end"]] + 0.001
    g = tool.gap_counts(D, 0)
    assert g == {"near": 2, "near_control": 1, "over": 1, "over_control": 0}


def test_capture_files_must_be_whole_and_open(tmp_path):
    from PIL import Image

    Image.new("RGB", (4, 4)).save(tmp_path / "perception_capture_1_rgb.png")
    np.save(tmp_path / "perception_capture_1_depth_m.npy", np.ones((4, 4)))  # finite: its picture is due
    Image.new("RGB", (4, 4)).save(tmp_path / "perception_capture_2_rgb.png")
    np.save(tmp_path / "perception_capture_2_depth_m.npy", np.full((4, 4), np.nan))  # no finite value: no picture
    (tmp_path / "perception_capture_4_depth_vis.png").write_bytes(b"not a png")
    f = tool.capture_files(tmp_path, [{"attempt": 2, "done": True}, {"attempt": 5, "done": True}])
    assert f["attempts"] == [1, 2, 4]
    assert "1:depth_vis.png" in f["missing"] and not any(m.startswith("2:") for m in f["missing"])
    assert not any(m.startswith("3:") for m in f["missing"])  # a number never present: an abandoned attempt
    assert "4:rgb.png" in f["missing"] and any(m.startswith("5:") for m in f["missing"])
    assert f["unreadable"] == ["perception_capture_4_depth_vis.png: UnidentifiedImageError"]


def row(n, scene, mode, *, near=(0, 0), over=(0, 0), cap=0.06, done=True, problems=False):
    return {"name": f"g2a_{n:02d}_{scene}_{mode}", "n": n, "scene": scene, "mode": mode, "invalid": [],
            "mission_done": done, "gaps": {"near": near[1], "near_control": near[0], "over": over[1],
                                           "over_control": over[0]},
            "capture": {"control_next_s": {"median": cap}, "control_next_clean_s": {"median": cap}},
            "files": {"missing": ["x"] if problems else [], "unreadable": []},
            "capture_dump": {"mode": "async", "dropped_full": 0, "errors": 0, "unsettled": 0} if mode == "async" else None,
            "log_dropped_lines": 0, "mission_outcomes": [("done", None)], "forbidden_pocket_contacts": 0}


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
        r["capture"]["control_next_s"]["median"] = 0.07 if r["mode"] == "sync" else 0.05  # drops of 0.02 only (A)
    v = tool.verdict(rows)["lines"]["3_capture_control_lower"]
    assert not v["pass"] and abs(v["populations"]["A"]["median_drop_s"] - 0.02) < 1e-12 and v["populations"]["B"]["pass"]
    rows = good()
    next(r for r in rows if r["mode"] == "async")["mission_done"] = False
    assert not tool.verdict(rows)["lines"]["4_missions_no_worse"]["pass"]
    rows = good()
    next(r for r in rows if r["mode"] == "async")["forbidden_pocket_contacts"] = 1  # a safety degradation
    assert not tool.verdict(rows)["lines"]["4_missions_no_worse"]["pass"]
    rows = good()
    next(r for r in rows if r["mode"] == "sync")["mission_done"] = None  # an unknown sync outcome: undecided
    assert not tool.verdict(rows)["lines"]["4_missions_no_worse"]["pass"]
    rows = good()
    rows[3]["files"]["missing"] = ["7:rgb.png"]
    assert not tool.verdict(rows)["lines"]["5_no_loss"]["pass"]
    rows = good()
    for r in rows:
        if r["scene"] == "H":  # worse in hall only, better in line_m: per scene it fails
            r["gaps"]["over"] = 2 if r["mode"] == "async" else 1
        else:
            r["gaps"]["over"] = 0 if r["mode"] == "async" else 5
    assert not tool.verdict(rows)["lines"]["6_any_source_over_0_25_not_more"]["pass"]
    rows = good()
    for r in rows:  # line 2: the share falls but the count of control-made near gaps rises
        r["gaps"]["near"], r["gaps"]["near_control"] = ((10, 2) if r["mode"] == "sync" else (40, 3))
    assert not tool.verdict(rows)["lines"]["2_near_control_share_lower"]["pass"]
    rows = good()
    rows[0]["invalid"] = ["probe exit 1"]
    v = tool.verdict(rows)
    assert v["lines"]["3_capture_control_lower"]["left_out"] == [[1, 2]] and not v["pass"]


def test_no_control_made_near_gap_in_either_arm_holds_line_2():
    rows = good()
    for r in rows:
        r["gaps"]["near_control"] = 0
    assert tool.verdict(rows)["lines"]["2_near_control_share_lower"]["pass"]


def test_suspect_runs_form_population_b():
    D = trace(60)
    D[:, C["live_step"]] = np.arange(60)
    D[10:13, C["flags"]] = BIT["capture"]  # a STOP in its last tick: suspect
    D[30:33, C["flags"]] = BIT["capture"]
    cmd = [{"step": k, "mode": "AUTO", "stop_kind": "none"} for k in range(60)]
    cmd[12]["stop_kind"] = "preempt"
    c = tool.capture_ticks(D, cmd)
    assert c["rows"] == [12, 32] and c["rows_clean"] == [32] and c["runs_suspect"] == 1


def test_files_by_number_present_and_mission_contact_records(tmp_path):
    from PIL import Image

    Image.new("RGB", (4, 4)).save(tmp_path / "perception_capture_2_rgb.png")  # number 1 abandoned: a gap is fine
    np.save(tmp_path / "perception_capture_2_depth_m.npy", np.full((4, 4), np.nan))
    f = tool.capture_files(tmp_path, None)
    assert f["missing"] == [] and f["attempts"] == [2]
    assert tool.capture_files(tmp_path, [{"attempt": 3, "done": True}])["missing"]  # async: an entry's files missing
    out = tmp_path / "s"
    run = out / "run"
    (run / "missions" / "m1").mkdir(parents=True)
    (out / "probe.json").write_text('{"confirm": {"mission_result": {"mission_id": "m1"}}}')
    (run / "missions" / "m1" / "records.json").write_text('{"state": {"forbidden_pocket_contacts": [1]}}')
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": []}) == 1
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": None}) is None
    (run / "missions" / "m1" / "records.json").write_text('{"state": {"forbidden_pocket_contacts": null}}')
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": []}) is None
    (run / "missions" / "m1" / "records.json").unlink()  # a reported mission without its record
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": []}) is None
    rows = good()
    next(r for r in rows if r["mode"] == "async")["forbidden_pocket_contacts"] = None
    assert not tool.verdict(rows)["lines"]["4_missions_no_worse"]["pass"]


def test_a_reported_mission_without_an_id_makes_the_contacts_unknown(tmp_path):
    out, run = tmp_path / "s", tmp_path / "s" / "run"
    run.mkdir(parents=True)
    (out / "probe.json").write_text('{"confirm": {"mission_result": {"outcome": "done"}}}')
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": []}) is None
    (out / "probe.json").write_text('{}')  # no mission reported: the session's own list only
    assert tool.forbidden_contacts(out, run, {"forbidden_pocket_contacts": []}) == 0
