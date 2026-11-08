"""v2 main batch verdict pieces (docs/plans/2026-11-08-live-riib-defaults-v2.md 3.3): the longest ok=false run's end
rules, stalls with the C2 replan steps apart and the previous row's split, the four pooled comparisons, trace breaks."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import riib_v2_verdict as V  # noqa: E402


def test_the_longest_false_run_ends_at_the_next_true_step_a_gap_or_the_window_end():
    steps = {0: (0.0, 0, True), 1: (1.0, 0, False), 2: (2.0, 0, False), 3: (3.0, 0, True),  # 1 -> 3: 2 s
             4: (4.0, 0, False), 5: (5.0, 0, False), 7: (20.0, 0, False),  # gap after 5: ends at 5 (1 s), 7 alone
             8: (21.0, 0, False), 9: (24.0, 0, False)}  # 7..9 false to the window's end: 24 - 20 = 4 s
    assert V.longest_false(steps, 0.0, 100.0) == pytest.approx(4.0)
    assert V.longest_false(steps, 0.0, 6.0) == pytest.approx(2.0)


def trace(S, flags=None):
    names = ["live_camera", "video", "render_arg", "scan", "rendered", "capture", "pocket", "far"]
    S = np.asarray(S, float)
    return {"S": S, "live_step": np.arange(len(S), dtype=float),
            "flags": np.asarray(flags if flags is not None else [math.nan] * len(S), float), "flag_names": names}


def test_stalls_split_by_the_previous_row_and_c2_steps_apart():
    rendered, capture = 1 << 4, (1 << 5) | (1 << 4)
    tr = trace([0.0, 0.03, 0.40, 0.43, 0.80, 0.83, 1.30], [math.nan, rendered, math.nan, capture, math.nan, 0, math.nan])
    out = V.stall_rows(tr, 0.0, 10.0, exclude={6})
    assert (out["total"], out["rendered"], out["capture"], out["other"], out["c2_excluded"]) == (2, 1, 1, 0, 1)
    assert V.stall_rows(tr, 0.5, 10.0, exclude=set())["total"] == 2  # S[k] in the interval: steps 4 and 6


def test_the_four_pooled_rules():
    a = {"sim": 300.0, "wall": 1000.0, "valid_wall": 1000.0, "false_wall": 50.0, "longest": 10.0, "stalls": 3}
    ok = {"sim": 292.0, "wall": 1000.0, "valid_wall": 1000.0, "false_wall": 60.0, "longest": 14.0, "stalls": 5}
    out = V.arm_rules(a, ok)
    assert out["pass"] and out["F"]["pass"] and out["R"]["pass"] and out["longest"]["pass"] and out["stalls"]["pass"]
    assert not V.arm_rules(a, {**ok, "sim": 290.0})["F"]["pass"]  # 0.290 < 0.97 x 0.300
    assert not V.arm_rules(a, {**ok, "false_wall": 61.0})["R"]["pass"]  # 0.061 > 0.05 + 0.01
    assert not V.arm_rules(a, {**ok, "longest": 14.1})["longest"]["pass"]  # > 1.2 x 10 + 2
    assert not V.arm_rules(a, {**ok, "stalls": 6})["stalls"]["pass"]  # > 3 + 2


def test_a_trace_break_inside_the_interval_is_missing():
    tr = trace([0.0, 0.1, 0.2, 0.3])
    assert not V.trace_missing(tr, 0.0, 1.0)
    tr["live_step"] = np.array([0.0, 1.0, 3.0, 4.0])
    assert V.trace_missing(tr, 0.0, 1.0) and not V.trace_missing(tr, 0.15, 1.0)


def test_pooled_r_uses_the_valid_intervals_not_the_wall_with_gaps():
    # Codex: R x wall_s overcounts when allowed gaps sit inside wall_s
    a = {"sim": 300.0, "wall": 1000.0, "valid_wall": 1000.0, "false_wall": 90.6, "longest": 1.0, "stalls": 0}
    b = {"sim": 300.0, "wall": 1009.0, "valid_wall": 1000.0, "false_wall": 100.45, "longest": 1.0, "stalls": 0}
    assert V.arm_rules(a, b)["R"]["pass"]  # 0.10045 <= 0.0906 + 0.01


def test_the_lifecycle_order_needs_each_step_after_the_previous():
    ev = [{"event": "reached", "label": "x"}, {"event": "reply", "kind": "rearm", "t": "ack"},
          {"event": "reply", "kind": "mission_resume", "t": "ack"}]
    ok, order = V.in_order(ev, [("x", V._label("x")), ("rearm", V._reply("rearm")), ("resume", V._reply("mission_resume"))])
    assert ok and [j for _, j in order] == [0, 1, 2]
    ok2, _ = V.in_order(list(reversed(ev)), [("x", V._label("x")), ("rearm", V._reply("rearm"))])
    assert not ok2  # the rearm before the label does not count


def test_expected_missions_must_have_their_files_and_contact_field(tmp_path):
    import json as _json

    run = tmp_path / "run"
    (run / "missions" / "m1").mkdir(parents=True)
    (run / "spool").mkdir()
    (run / "spool" / "manifest.json").write_text(_json.dumps({"complete": True}))
    for f in ("result.json", "records.json"):
        (run / "missions" / "m1" / f).write_text("{}")
    res = {"forbidden_pocket_contacts": [], "session_neighbour_gap": {}, "session_rest_largest": {},
           "obstacle_layer": {"unpermitted_dir_entries": 0},
           "mission_results": [{"mission_id": "m1"}, {"mission_id": "m2"}]}
    sess = {"run": run, "result": res, "rows": [], "probe": {},
            "records": {"m1": {"forbidden_pocket_contacts": []}}}
    out = V.safety(sess)
    assert "undecidable" in out and out["undecidable"]["missing_missions"] == ["m2"]
    res["mission_results"] = [{"mission_id": "m1"}]
    sess["records"] = {"m1": {}}  # parsed, but without the contact field
    out = V.safety(sess)
    assert "undecidable" in out and out["undecidable"]["records_without_contacts"] == ["m1"]
    sess["records"] = {"m1": {"forbidden_pocket_contacts": []}}
    assert V.safety(sess)["pass"] is True
