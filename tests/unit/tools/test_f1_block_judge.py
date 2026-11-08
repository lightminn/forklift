"""tools/f1_block_judge.py on synthetic session folders (docs/plans/2026-11-08-f1-vehicle-block.md 5)."""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f1_block_judge as J  # noqa: E402


def _session(tmp_path, *, verdict="pass", source="operator_placed", arg="block", autonomy="blocked", auto=0,
             missions=0, log=True, dropped=0):
    d = tmp_path / "s"
    run = d / "epoch_1" / "run"
    run.mkdir(parents=True)
    (d / "probe.json").write_text(json.dumps({"block_check": {"verdict": verdict}}))
    (run / "result.json").write_text(json.dumps({"start_reloc": {"source": source, "autonomy": autonomy},
                                                 "arguments": {"operator_placed_auto": arg}}))
    if log:
        lines = [json.dumps({"log_writer": "ordered"})]
        lines += [json.dumps({"tick_t": i, "mode": "AUTO" if i < auto else "PAUSED"}) for i in range(5)]
        if log != "unclosed":
            lines.append(json.dumps({"log_end": {"dropped_lines": dropped, "line_errors": 0}}))
        (run / "live_log.jsonl").write_text("\n".join(lines) + "\n")
    for k in range(missions):
        (run / "missions" / f"m{k + 1}").mkdir(parents=True)
    return d


def test_a_clean_block_passes(tmp_path):
    out = J.judge(_session(tmp_path), "block")
    assert out["verdict"] == "pass", out["failed"]


def test_an_auto_tick_or_a_mission_fails_the_block(tmp_path):
    assert J.judge(_session(tmp_path / "a", auto=1), "block")["failed"] == ["no_auto_tick"]
    assert J.judge(_session(tmp_path / "b", missions=1), "block")["failed"] == ["no_mission"]


def test_the_record_must_say_operator_placed_and_the_setting(tmp_path):
    assert "source_operator_placed" in J.judge(_session(tmp_path / "a", source="matcher"), "block")["failed"]
    assert "argument_applied" in J.judge(_session(tmp_path / "b", arg="allow"), "block")["failed"]
    assert "decision_recorded" in J.judge(_session(tmp_path / "c", autonomy="allowed"), "block")["failed"]


def test_missing_records_fail(tmp_path):
    out = J.judge(tmp_path / "none", "block")
    assert out["verdict"] == "fail" and {"probe_pass", "one_epoch", "log_whole"} <= set(out["failed"])
    assert "log_whole" in J.judge(_session(tmp_path / "a", log=False), "block")["failed"]


def test_a_log_that_lost_lines_or_never_closed_is_not_whole(tmp_path):
    assert "log_whole" in J.judge(_session(tmp_path / "a", dropped=1), "block")["failed"]
    assert "log_whole" in J.judge(_session(tmp_path / "b", log="unclosed"), "block")["failed"]


def test_a_rolled_part_gone_is_not_whole(tmp_path):
    d = _session(tmp_path)
    (d / "epoch_1" / "run" / "live_log.index.json").write_text(json.dumps({"rotations": 4, "keep": 3}))
    assert "log_whole" in J.judge(d, "block")["failed"]


def test_allow_needs_a_mission(tmp_path):
    d = _session(tmp_path / "a", arg="allow", autonomy="allowed", auto=2, missions=1)
    assert J.judge(d, "allow")["verdict"] == "pass"
    d = _session(tmp_path / "b", arg="allow", autonomy="allowed")
    assert J.judge(d, "allow")["failed"] == ["mission_made"]


def test_the_cli_writes_the_verdict_and_exits_by_it(tmp_path):
    d = _session(tmp_path)
    assert J.main([str(d), "--expect", "block"]) == 0 and json.loads((d / "f1_block_verdict.json").read_text())
    assert J.main([str(d), "--expect", "allow"]) == 1
