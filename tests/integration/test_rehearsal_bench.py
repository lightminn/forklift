"""The rehearsal bench end to end (docs/design/2026-11-15-arrival-day-rehearsal.md 2.1-2.3): the model holds the pty
master, the tools link by path, records are marked synthetic and refused by measurement entry points, and the
bridge's synthetic permit opens only on the bench's model."""

from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_bringup_tool import mcu_model  # noqa: E402,F401,I001 -- the module-scoped model build

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
import bringup  # noqa: E402
import rehearsal_bench  # noqa: E402
import teleop_bridge as tb  # noqa: E402

pytestmark = pytest.mark.skipif(os.geteuid() == 0, reason="the bench refuses root")


@pytest.fixture
def bench(mcu_model, tmp_path):  # noqa: F811
    proc, path, record = rehearsal_bench.start("drive", tmp_path / "reh", mcu_model.parent, [], 30000)
    yield proc, path, record
    if proc.poll() is None:
        proc.terminate()
        proc.wait(timeout=5)


def test_a_record_on_the_bench_is_marked_and_refused_for_measurement(bench, tmp_path, capsys):
    proc, path, record = bench
    out = tmp_path / "run.jsonl"
    assert bringup.main(["record", "--device", record["pts"], "--bench", str(path), "--out", str(out),
                         "--seconds", "1.0"]) == 0
    capsys.readouterr()
    rows = [json.loads(x) for x in out.read_text().splitlines()]
    assert rows[0]["t"] == "header" and rows[0]["synthetic"]["bench"]["model_pid"] == proc.pid
    assert any(r.get("t") == "state" for r in rows), "the model answered nothing on the slave"
    assert bringup.main(["clock", "--record", str(out)]) == 2
    assert "synthetic" in capsys.readouterr().out
    assert bringup.main(["clock", "--record", str(out), "--allow-synthetic"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["not_for_judgement"] is True and report["result"] == "ok"


def test_markers_asked_for_without_a_terminal_are_said_and_recorded(bench, tmp_path, capsys):
    # rehearsal 2 R2-1: pytest's stdin is not a terminal -- the header says the markers are off and why, and stderr warns
    proc, path, record = bench
    out = tmp_path / "m.jsonl"
    assert bringup.main(["record", "--device", record["pts"], "--bench", str(path), "--out", str(out),
                         "--seconds", "0.5", "--markers"]) == 0
    err = capsys.readouterr().err
    head = json.loads(out.read_text().splitlines()[0])
    # under pytest's capture stdin has no descriptor ("no stdin"); from a pipe it is "stdin is not a terminal"
    assert head["markers"]["requested"] is True and head["markers"]["active"] is False
    assert head["markers"]["why"] in ("no stdin", "stdin is not a terminal")
    assert "--markers is off" in err


def test_a_pty_without_the_bench_and_a_bench_on_a_real_device_are_refused(bench, tmp_path, capsys):
    _proc, path, record = bench
    assert bringup.main(["record", "--device", record["pts"], "--out", str(tmp_path / "a.jsonl"),
                         "--seconds", "0.2"]) == 2
    assert "needs --bench" in capsys.readouterr().out
    assert bringup.main(["record", "--device", "/dev/null", "--bench", str(path), "--out", str(tmp_path / "b.jsonl"),
                         "--seconds", "0.2"]) == 2
    assert "not a pty" in capsys.readouterr().out


def test_the_bridges_synthetic_permit_is_refused_off_the_bench(bench, tmp_path, capsys):
    _proc, path, record = bench
    flags = ["--empty-forks", "--forks-clear", "--lift-low", "--no-open-recovery", "--port", "0"]
    assert tb.main(["--device", "/dev/null", "--synthetic-permit", *flags]) == 2  # a real device
    assert tb.main(["--device", "/dev/null", "--bench", str(path), "--synthetic-permit", *flags]) == 2
    assert tb.main(["--device", record["pts"], "--synthetic-permit", *flags]) == 2  # no bench
    assert not list(tmp_path.glob("*.jsonl"))


def test_the_synthetic_permit_opens_on_the_model_and_closes_for_good_when_it_goes(bench):
    proc, _path, record = bench
    clock = {"t": 0}
    permit = tb.SyntheticPermit(record["pts"], record, recheck_s=1.0, clock=lambda: clock["t"])
    assert permit([], 1, None) == (1.0, "synthetic_permit")
    proc.terminate()
    proc.wait(timeout=5)
    # the model's master closed with it: the pty is gone, so the next call closes without waiting for a recheck
    assert permit([], 1, None) == (0.0, "synthetic_permit_lost")
    clock["t"] = int(10e9)
    assert permit([], 1, None) == (0.0, "synthetic_permit_lost") and permit.lost is not None


def test_the_permit_closes_in_the_same_call_when_the_path_names_another_device(bench, tmp_path):
    _proc, _path, record = bench
    link = tmp_path / "mcu"
    link.symlink_to(record["pts"])
    permit = tb.SyntheticPermit(str(link), record, recheck_s=1000.0, clock=lambda: 0)
    assert permit([], 1, None) == (1.0, "synthetic_permit")
    link.unlink()
    link.symlink_to("/dev/null")  # retargeted to another device; no recheck is due
    assert permit([], 1, None) == (0.0, "synthetic_permit_lost") and permit.lost == "device_changed"


def test_a_rehearsal_result_on_stdout_is_marked(bench, tmp_path, capsys):
    _proc, path, record = bench
    assert bringup.main(["cpr", "--device", record["pts"], "--bench", str(path), "--out", str(tmp_path / "c.jsonl"),
                         "--seconds", "0.5", "--turns-left", "1", "--turns-right", "1"]) == 0
    out = json.loads(capsys.readouterr().out.strip().splitlines()[-1])
    assert out["not_for_judgement"] is True and out["synthetic_inputs"] is True
    rows = [json.loads(x) for x in (tmp_path / "c.jsonl").read_text().splitlines()]
    assert rows[0]["device_id"]["major"] in range(136, 144)


def test_absent_parts_go_into_the_header(bench, tmp_path, capsys):
    """Partial-parts plan v4: the capture names the parts not fitted; first boot and kth read them from here."""
    proc, path, record = bench
    out = tmp_path / "run.jsonl"
    assert bringup.main(["record", "--device", record["pts"], "--bench", str(path), "--out", str(out),
                         "--seconds", "0.3", "--absent", "steer", "--absent", "lift", "--absent", "steer"]) == 0
    capsys.readouterr()
    headers = [json.loads(x) for x in out.read_text().splitlines() if json.loads(x)["t"] == "header"]
    assert [h["parts_absent"] for h in headers] == [["lift", "steer"]]  # none absent: [] (the clear-faults test)


def _bench_with(mcu_model, tmp_path, extra):  # noqa: F811
    return rehearsal_bench.start("drive", tmp_path / "reh", mcu_model.parent, extra, 30000)


@pytest.mark.parametrize("extra, expected, mode_after", [
    ([], "refused", "DISARMED"),                                                   # nothing latched: not sent
    (["--estop-at-ms", "100", "--estop-release-ms", "400"], "cleared", "DISARMED"),  # the cause gone: cleared
    (["--fault-at-ms", "100"], "cleared", "DISARMED"),        # a driver nFAULT: the re-init's CLR_FLT clears it
    (["--estop-at-ms", "100"], "not_accepted", "ESTOP"),      # the e-stop still pressed: refused, seq not echoed
])
def test_clear_faults_only_on_a_latch_and_never_arms(mcu_model, tmp_path, capsys, extra, expected,  # noqa: F811
                                                     mode_after):
    proc, path, record = _bench_with(mcu_model, tmp_path, extra)
    try:
        time.sleep(0.8)  # past the injected latch (and the e-stop's release)
        out = tmp_path / "cf.jsonl"
        code = bringup.main(["clear-faults", "--device", record["pts"], "--bench", str(path), "--out", str(out),
                             "--cause", "test: injected latch"])
        res = json.loads(capsys.readouterr().out)
        assert res["result"] == expected, res
        assert code == (0 if expected == "cleared" else 1)
        rows = [json.loads(x) for x in out.read_text().splitlines()]
        assert rows[0]["cause"] == "test: injected latch" and rows[0]["cmd"] == "clear-faults"
        assert rows[0]["parts_absent"] == []
        kinds = [r["kind"] for r in rows if r["t"] == "written"]
        assert "ARM" not in kinds and kinds.count("CLEAR_FAULTS") == (0 if expected == "refused" else 1)
        if expected != "refused":
            assert kinds.index("DISARM") < kinds.index("CLEAR_FAULTS")
        if expected == "refused":
            assert res["why"] == "not_latched"
        else:
            assert res["before"]["mode"] in ("ESTOP", "FAULT") and res["after"]["mode"] == mode_after
            if expected == "not_accepted":
                assert res["after"]["reject_reason"] == "ESTOP_ACTIVE"
            else:
                assert res["after"]["drivers_ready"] and not res["after"]["fault_flags"]
    finally:
        proc.terminate()
        proc.wait(timeout=5)
