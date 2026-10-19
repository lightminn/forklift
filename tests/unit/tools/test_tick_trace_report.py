"""Real-time plan 5.12 D12a: the trace report's work, overrun and render counts on a made-up trace."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from sim.isaac.tick_trace import TickTrace

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("tick_trace_report", ROOT / "tools/tick_trace_report.py")
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)
report = REPORT.report

DT = 1 / 120


def make_run(tmp_path, works, flags_of, *, late=0.0, world_share=0.5, last=0.001):
    """One row per physics step: the step starts when due (q + d) plus a late wake-up, then takes works[i] up to the
    next wait (world.step its world_share)."""
    (tmp_path / "run").mkdir()
    t = {"now": 0.0}
    trace = TickTrace(len(works) + 1, lambda: t["now"])
    start = 0.0
    for i, w in enumerate(works + [last]):
        trace.control("ctrl_start", t["now"])
        trace.control("ctrl_end", t["now"])
        trace.begin()
        q = t["now"]
        d = max(0.0, start + DT - q) if i else 0.0
        trace.set("q", q)
        trace.set("d", d)
        start = q + d + (late if i else 0.0)
        trace.set("S", start)
        trace.set("tick", float(i))
        for f in flags_of(i):
            trace.flag(f)
        trace.set("world_start", start)
        trace.set("world_end", start + w * world_share)
        trace.set("step_end", start + w * world_share)
        trace.set("after_end", start + w)
        t["now"] = start + w
    trace.save(tmp_path / "run" / "tick_trace.npz")
    (tmp_path / "run" / "slam_records.json").write_text(json.dumps([{"stamp_s": 0.0, "mode": "tracking"}]))
    return tmp_path


def test_the_overrun_is_the_work_past_the_spacing_by_kind(tmp_path):
    # Every 12th step renders and scans (30 ms), the rest take 2 ms: overrun 30 - 8.33 ms per 0.1 s.
    works = [0.030 if i % 12 == 0 else 0.002 for i in range(120)]
    run = make_run(tmp_path, works, lambda i: ["live_camera", "scan", "rendered"] if i % 12 == 0 else [])
    r = report(run)
    assert r["rows"] == 120 and r["sim_s"] == pytest.approx(1.0)
    assert r["overrun_per_sim_s"] == pytest.approx(10 * (0.030 - DT), rel=1e-6)
    assert r["overrun_by_kind"]["render+scan"] == pytest.approx(r["overrun_per_sim_s"])
    assert r["overrun_by_kind"]["plain"] == 0.0
    assert r["overrun_by_phase"][0] == pytest.approx(r["overrun_per_sim_s"])
    assert r["renders"] == {"live_camera": 10, "video": 0, "render_arg": 0, "capture": 0, "pocket": 0,
                            "rendered": 10, "unattributed": 0}
    assert r["wall_per_sim_s"] == pytest.approx(10 * 0.030 + 110 * DT, rel=1e-6)
    assert r["model_wall_per_sim_s"] == pytest.approx(r["wall_per_sim_s"], rel=1e-6)
    parts = r["mean_ms_by_kind"]["plain"]
    w_ms = sum(parts[p] for p in ("gate", "world", "step_rest", "after", "between", "control", "to_wait"))
    assert w_ms == pytest.approx(2.0)  # the parts add up to W


def test_a_late_wake_up_is_in_the_measured_wall_not_the_model(tmp_path):
    # Codex 5.12 implementation P2-1: every wake-up 1 ms late adds 1 ms a step to the wall, not to the model.
    run = make_run(tmp_path, [0.002] * 120, lambda i: [], late=0.001)
    r = report(run)
    assert r["model_wall_per_sim_s"] == pytest.approx(1.0)
    assert r["wall_per_sim_s"] == pytest.approx(1.0 + 120 * 0.001, rel=1e-6)
    assert r["mean_ms_by_kind"]["plain"]["overshoot"] == pytest.approx(1.0)


def test_a_long_world_step_is_listed_as_a_stall(tmp_path):
    works = [0.002] * 30 + [0.300] + [0.002] * 30
    run = make_run(tmp_path, works, lambda i: ["live_camera", "rendered"] if i == 30 else [])
    r = report(run, stall_s=0.1)
    assert [(s["tick"], s["kind"], round(s["world_s"], 3)) for s in r["stalls"]] == [(30, "render", 0.15)]


def test_a_stall_after_the_world_step_and_on_the_last_step_are_listed(tmp_path):
    # Codex 5.12 implementation P2-3: 2 ms of world.step then 200 ms of processing; a 200 ms last step.
    works = [0.002] * 30 + [0.202] + [0.002] * 30
    run = make_run(tmp_path, works, lambda i: [], world_share=0.01, last=0.200)
    r = report(run, stall_s=0.1)
    assert [s["tick"] for s in r["stalls"]] == [30, 61]
    assert r["stalls"][0]["world_s"] < 0.01 and r["stalls"][0]["step_s"] == pytest.approx(0.202)


def test_a_far_frame_render_counts_as_a_render_unattributed(tmp_path):
    # Codex 5.12 implementation P2-4: a render with no request bit (the far frames, asked inside step_world).
    run = make_run(tmp_path, [0.002] * 24, lambda i: ["rendered"] if i % 12 == 3 else [])
    r = report(run)
    assert r["rows_by_kind"]["render"] == 2 and r["renders"]["unattributed"] == 2


def test_a_trace_with_other_columns_is_refused(tmp_path):
    (tmp_path / "run").mkdir()
    np.savez(tmp_path / "run" / "tick_trace.npz", data=np.zeros((2, 2)), columns=np.array(["a", "b"]),
             flags=np.array(["x"]))
    (tmp_path / "run" / "slam_records.json").write_text("[]")
    with pytest.raises(ValueError, match="columns"):
        report(tmp_path)
