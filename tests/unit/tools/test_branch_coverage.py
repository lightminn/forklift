"""The D2 branch coverage over format-2 records (M-a1 replay plan R5/R6)."""

import importlib.util
from pathlib import Path

from forklift_core.mission.boundary_record import BoundaryRecorder, boundary_ending

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("branch_coverage", ROOT / "tools/branch_coverage.py")
C = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(C)


def _run(base: Path, branches, ending=None) -> Path:
    rec = BoundaryRecorder(base / "boundary", value_format=2)
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    rec.tick_end(outputs={}, branches=branches)
    rec.close(ending=boundary_ending(ending))
    return base


def test_items_are_covered_by_the_runs_that_show_them_and_a_refusal_counts_from_the_ending(tmp_path):
    a = _run(tmp_path / "a", ["state.transitions[from=settle,to=return_home,time_s=3.5]", "obstacle.backoff_records[phase=transport]"])
    (a / "result.json").write_text('{"obstacle_layer": {"backoff_records": [{"escape": {"chosen": 1}}, {"escape": false}]}}')
    b = _run(tmp_path / "b", [], ending=RuntimeError("Mission planning failed: no_path"))
    out = C.coverage([a, b])
    assert out["items"]["return home"]["covered"] and out["items"]["obstacle backoff"]["covered"]
    assert out["items"]["mission refusal"]["covered"] and str(b) in out["items"]["mission refusal"]["runs"]
    assert out["items"]["obstacle escape"]["covered"] and "stall replan" in out["not_covered"]
    assert "realignment" in out["not_covered"]
    assert "state.transitions[from=settle,to=return_home]" in out["branches"]
