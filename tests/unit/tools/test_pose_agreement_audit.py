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
    row, = A.session_rows(_session(tmp_path, "b_val3_032_full", record))
    assert row["class"] == "natural_unconfirmed" and row["attempt"] == 1 and row["reverse_m"] == pytest.approx(2.0)
    assert row["g_err"] == pytest.approx([0.0, 0.03]) and row["c_err"] == pytest.approx([0.1, 0.01]) and row["wider"] == "g"


def test_a_shadow_session_reads_its_raw_generations_in_order(tmp_path):
    raw = {"start": {"confirmed": True}, "rematches": [{"t": 2.0, "g": [5.0, 0.0, 0.0], "agree_m": 0.01, "agree_rad": 0.001},
                                                       {"t": 3.0, "g": None},
                                                       {"t": 9.5, "g": [3.0, 0.1, 0.0], "agree_m": 0.02, "agree_rad": 0.002}]}
    rows = A.session_rows(_session(tmp_path, "t_val3_006", {"samples": SAMPLES, "raw": raw}))
    assert [r["attempt"] for r in rows] == [1, 2] and {r["class"] for r in rows} == {"confirmed"}
    assert rows[1]["wider"] == "c" and rows[1]["kind"] == "raw"


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
    assert json.loads(out.read_text())["rows"][0]["agree_m"] == pytest.approx(0.1234)
    assert math.isfinite(json.loads(out.read_text())["rows"][0]["reverse_m"])
