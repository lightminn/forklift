"""The R4 fault injector edits only a copy of a record, at the place it reports (M-a1 replay plan R4)."""

import importlib.util
import json
from pathlib import Path

from forklift_core.mission.boundary_record import BoundaryRecorder, RecordingGate, boundary_ending, recorded_function, sink_effect

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("boundary_inject", ROOT / "tools/boundary_inject.py")
I = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(I)


def _record(run: Path) -> None:
    rec = BoundaryRecorder(run / "boundary", value_format=2)
    gate = RecordingGate(rec)
    for tick in range(3):
        rec.tick_begin(tick, tick_t=float(tick), work_t=float(tick), commands=[], stop_kind="none")
        recorded_function(lambda: 0.5 + tick, "Clock", "monotonic", gate)()
        recorded_function(lambda x: x, "Planner", "plan", gate)(1.25)
        sink_effect(gate, "echo", "stdout", f"line {tick}")
        rec.tick_end(outputs={}, branches=[])
    rec.close(ending=boundary_ending(None))


def _calls(run: Path, tick: int):
    for chunk in sorted((run / "boundary").glob("boundary_*.jsonl")):
        for line in chunk.read_text().splitlines():
            t = json.loads(line)
            if t["tick"] == tick:
                return [(c["port"], c["method"]) for c in t["calls"]], t["calls"]
    raise AssertionError(tick)


def test_each_mutation_changes_the_copy_at_the_reported_place_and_never_the_original(tmp_path):
    _record(tmp_path / "orig")
    before = (tmp_path / "orig/boundary/boundary_00000.jsonl").read_text()
    seen = {}
    for name in ("drop", "extra", "swap", "truncate", "request", "response", "sink", "ending"):
        work = tmp_path / name
        where = I.make_copy(tmp_path / "orig", work, name, after=1)
        seen[name] = where
        if name == "ending":
            assert json.loads((work / "run/boundary/boundary_meta.json").read_text())["ending"]["kind"] == "exception"
            continue
        assert where["tick"] == 1
        names, calls = _calls(work / "run", 1)
        if name == "drop":
            assert names == [("Planner", "plan"), ("Sink", "echo")]
        elif name == "extra":
            assert names == [("Clock", "monotonic"), ("Clock", "monotonic"), ("Planner", "plan"), ("Sink", "echo")]
        elif name == "swap":
            assert names == [("Planner", "plan"), ("Clock", "monotonic"), ("Sink", "echo")]
        elif name == "truncate":
            assert names == []
        elif name == "request":
            assert "1.250001" in json.dumps(calls[1]["request"])
        elif name == "response":
            assert abs(calls[0]["response"] - 1.501) < 1e-12
        elif name == "sink":
            orig = _calls(tmp_path / "orig", 1)[1][2]["request"]
            assert calls[2]["request"] != orig
    assert (tmp_path / "orig/boundary/boundary_00000.jsonl").read_text() == before
