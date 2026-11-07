"""The agreement audit's table, on synthetic records only (the revision plan 9.16: no value is read before the rule)."""

import json
import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import pose_agreement_audit as A  # noqa: E402


def _session(tmp_path, name, record):
    run = tmp_path / name / "epoch_1" / "run"
    run.mkdir(parents=True)
    (run / "result.json").write_text(json.dumps({"pose_check": record}))
    return str(tmp_path / name)


SAMPLES = [{"t": 1.0, "o": [0.0, 0.0, 0.0], "applied": [5.0, 0.0, 0.0], "truth": [5.0, 0.0, 0.0]},
           {"t": 9.0, "o": [-2.0, 0.0, 0.0], "applied": [3.0, 0.0, 0.01], "truth": [3.0, 0.1, 0.0]}]


def test_an_on_session_reads_its_agreement_events_and_says_which_pose_is_wider(tmp_path):
    record = {"samples": SAMPLES, "start": {"result": {"confirmed": False}},
              "events": [{"t": 9.5, "kind": "slam_agreement", "agree_m": 0.05, "agree_rad": 0.02, "evidence": [3.0, 0.1, 0.03]},
                         {"t": 9.6, "kind": "ended"}]}
    record["events"][0]["ok"] = False  # a refused agreement is a row too
    record["rematches"] = [{"t": 9.5, "outcome": "confirm", "why": [], "g": [3.0, 0.1, 0.03], "o_begin": [-2.0, 0.0, 0.0],
                            "o_scan": [-2.0, 0.0, 0.0], "attempts": [{"reason": "ok", "state": "done"}],
                            "scan_stamps": [[8.6, 8.7, 8.8, 8.9, 9.0]]}]
    rows, counts = A.session_rows(_session(tmp_path, "b_val3_032_full", record))
    row, = rows
    assert row["class"] == "natural_unconfirmed" and row["ok"] is False and row["reverse_m"] == pytest.approx(2.0)
    assert row["stand"] == 1 and row["generation"] == 1 and row["attempts"] == ["ok"] and row["scan_t"] == 9.0
    assert row["g_err"] == pytest.approx([0.0, 0.0, 0.03]) and row["source"] == "rematch"
    assert row["wider"] == "g" and row["delta_o"] == pytest.approx([0.0, 0.0])
    assert "res_base" in row["unrecorded"] and counts["generations"] == 1 and counts["competitor"] == 0


def test_a_shadow_session_reads_its_raw_generations_in_order(tmp_path):
    raw = {"start": {"confirmed": True}, "rematches": [{"t": 2.0, "g": [5.0, 0.0, 0.0], "agree_m": 0.01, "agree_rad": 0.001},
                                                       {"t": 3.0, "g": None},
                                                       {"t": 9.5, "g": [3.0, 0.1, 0.0], "agree_m": 0.02, "agree_rad": 0.002}]}
    rows, _ = A.session_rows(_session(tmp_path, "t_val3_006", {"samples": SAMPLES, "raw": raw}))
    assert [r["agreement"] for r in rows] == [1, 2, 3] and {r["class"] for r in rows} == {"confirmed"}
    assert rows[2]["wider"] == "c" and rows[2]["kind"] == "raw" and rows[1]["g_err"] is None


def test_a_start_with_a_competitor_is_its_own_class(tmp_path):
    raw = {"start": {"confirmed": False}, "rematches": []}
    assert A.start_class({"raw": raw}, "s/t_val3_021") == "competitor"
    assert A.start_class({"raw": raw}, "s/b_val3_052_half") == "natural_unconfirmed"


def test_it_prints_no_value(tmp_path, capsys):
    record = {"samples": SAMPLES, "start": {"result": {"confirmed": True}},
              "events": [{"t": 9.5, "kind": "rematch_agreement", "agree_m": 0.1234, "agree_rad": 0.0567, "evidence": [3.0, 0.0, 0.0]}]}
    _session(tmp_path, "t_val3_006", record)
    out = tmp_path / "out.json"
    assert A.main([str(out), str(tmp_path / "t_*")]) == 0
    text = capsys.readouterr().out
    assert "0.1234" not in text and "1 generations" in text
    assert json.loads(out.read_text())["sessions"][0]["session"] == "t_val3_006"
    assert json.loads(out.read_text())["rows"][0]["agree_m"] == pytest.approx(0.1234)
    assert math.isfinite(json.loads(out.read_text())["rows"][0]["reverse_m"])


def test_the_applied_pose_is_the_event_s_own_and_g_meets_the_truth_at_its_scan(tmp_path):
    # Codex: the sample before the event had heading 0, the event applied 0.10: c is the wider one
    samples = [{"t": 2.0, "o": [0.0, 0.0, 0.0], "applied": [0.0, 0.0, 0.0], "truth": [0.0, 0.0, 0.0]}]
    record = {"samples": samples, "start": {"result": {"confirmed": True}}, "rematches": [],
              "events": [{"t": 2.05, "kind": "slam_agreement", "agree_m": 0.0, "agree_rad": 0.07, "ok": False,
                          "evidence": [0.0, 0.0, 0.03], "applied": [0.0, 0.0, 0.10]}]}
    (row,), _ = A.session_rows(_session(tmp_path, "t_val3_006", record))
    assert row["c_err"] == pytest.approx([0.0, 0.0, 0.10]) and row["wider"] == "c" and row["source"] == "start"


def test_a_shadow_row_takes_the_result_s_correction_at_the_nearest_odometry(tmp_path):
    samples = [{"t": 2.0, "o": [1.0, 0.0, 0.0], "applied": [1.0, 0.0, 0.0], "truth": [1.0, 0.0, 0.0]}]
    raw = {"start": {"confirmed": True}, "rematches": [{"t": 2.04, "g": [1.0, 0.0, 0.0], "agree_m": 0.0, "agree_rad": 0.05,
                                                        "residual_confirmed": True, "applicable": True,
                                                        "C": [0.0, 0.0, 0.05]}]}
    (row,), _ = A.session_rows(_session(tmp_path, "t_val3_006", {"samples": samples, "raw": raw}))
    assert row["c_err"][2] == pytest.approx(0.05) and row["wider"] == "c"

