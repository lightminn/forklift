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
    assert permit([], 1, None) == (1.0, "synthetic_permit")  # not rechecked yet
    clock["t"] = int(1.5e9)
    assert permit([], 1, None) == (0.0, "synthetic_permit_lost")
    clock["t"] = int(10e9)
    assert permit([], 1, None) == (0.0, "synthetic_permit_lost") and permit.lost is not None
