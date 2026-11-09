"""Lifecycle 4 confirmation's CPU pre-check (docs/plans/2026-11-09-live-riib-v2-lifecycle4-confirmation.md 3): the start
poses taken from m1's approach record, and the verdict over every start."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import riib_life4_plan_check as C  # noqa: E402


REC = {"state": {"paths": {"approach": {"poses": [[0.0, 0.0, 0.0], [0.5, 0.0, 0.0], [1.0, 0.0, 0.0],
                                                  [1.5, 0.0, 0.0], [2.0, 0.0, 0.0]]}}}}


def test_starts_from_an_approach_record_cover_its_start_middle_end_and_the_backed_off_ends():
    starts = C.approach_starts(REC, "s0B", backs=(0.31, 1.0))
    assert [s["name"] for s in starts] == ["s0B:approach_start", "s0B:approach_mid", "s0B:approach_end",
                                           "s0B:approach_end_back_0.31", "s0B:approach_end_back_1.00"]
    assert starts[1]["pose"] == [1.0, 0.0, 0.0]
    assert starts[3]["pose"] == pytest.approx([1.69, 0.0, 0.0])  # backed along its heading
    assert starts[4]["pose"] == pytest.approx([1.0, 0.0, 0.0])


def test_the_back_off_follows_the_heading():
    rec = {"state": {"paths": {"approach": {"poses": [[0.0, 0.0, 1.5707963267948966], [0.0, 1.0, 1.5707963267948966]]}}}}
    end_back = C.approach_starts(rec, "x", backs=(0.5,))[-1]["pose"]
    assert end_back[0] == pytest.approx(0.0, abs=1e-12) and end_back[1] == pytest.approx(0.5)


def _entries(names, outcome="passed"):
    return [{"name": n, "outcome": outcome, "submit_check": None} for n in names]


def test_the_gate_leaves_the_approach_middle_and_end_as_reports():
    # independent review P2-3: clear_out runs because those poses may fail the next start check
    gate = C.gate_names(("s0B", "s1B"), (0.31, 1.0))
    assert len(gate) == 7 and "s0B:approach_mid" not in gate and "s1B:approach_end" not in gate
    assert "s0B:approach_start" in gate and "s1B:approach_end_back_1.00" in gate
    entries = _entries(C.expected_names(("s0B", "s1B"), (0.31, 1.0)))
    entries[2]["outcome"] = "observe_failed"  # s0B:approach_mid
    out = C.verdict(entries, C.expected_names(("s0B", "s1B"), (0.31, 1.0)), gate)
    assert out["pass"] is True and out["report_failed"] == ["s0B:approach_mid"]


def test_the_formal_check_passes_only_on_every_registered_start():
    expected = C.expected_names(("s0B", "s1B"), (0.31, 1.0))
    assert len(expected) == 11 and expected[0] == "session_start"
    assert C.verdict(_entries(expected), expected)["pass"] is True
    out = C.verdict(_entries(expected[:-1]), expected)
    assert out["pass"] is False and out["missing"] == [expected[-1]]  # a start not checked is not a pass
    bad = _entries(expected)
    bad[3]["outcome"] = "observe_failed"
    assert C.verdict(bad, expected)["failed"] == [expected[3]]
    refused = _entries(expected)
    refused[0]["submit_check"] = "destination_occupied"
    assert C.verdict(refused, expected)["pass"] is False
    assert C.verdict(_entries(expected + ["session_start"]), expected)["pass"] is False  # a duplicate


def test_the_formal_mode_refuses_records_other_than_the_registered_two(tmp_path):
    with pytest.raises(SystemExit):
        C.main([str(tmp_path / "s.json"), "--records", "s0B=x.json", "--out", str(tmp_path / "o.json")])
