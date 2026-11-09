"""Real-time plan 5.22 results tables: the gap reading keeps a large control tick inside a world-led gap in view
(Codex round 1 P2-1), the on arm's ``name|parent`` keys fold onto the off arm's names, and the periodic reading finds
one period with a free phase per session."""

import importlib.util
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "g2m_tables", ROOT / "docs" / "validation" / "2026-11-09-g2-measure" / "g2m_tables.py")
tables = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tables)


def _gap(gap, world, control):
    return {"gap_s": gap, "parts": {"world": world, "step_rest": 0.0, "after": 0.0, "control_next": control},
            "top_calls": [{"name": "hook.detect_pockets_expected"}]}


def test_gaps_keep_the_control_inside_a_world_led_gap():
    row = {"M1": [_gap(0.93, 0.79, 0.138), _gap(0.84, 0.83, 0.003), _gap(0.30, 0.10, 0.19), _gap(0.235, 0.011, 0.217)]}
    g = tables.gaps(row)
    assert (g["over_0_25"], g["world_alone_over_0_25"]) == (3, 2)  # the third would pass with control gone
    assert g["max_control_inside_over"] == 0.19
    assert g["near"] == [{"gap_s": 0.235, "control_s": 0.217, "world_s": 0.011, "top": ["hook.detect_pockets_expected"]}]


def test_folded_adds_parent_keys_onto_the_name():
    t = {"grid._erode|grid._clear_group|x": [0.2, 8], "grid._erode|grid.snapshot<refresh|x": [0.1, 2],
         "lidar.cast": [0.5, 10]}
    assert tables.folded(t) == {"grid._erode": [0.30000000000000004, 10], "lidar.cast": [0.5, 10]}


def test_periodic_finds_the_period_with_a_free_phase_per_session():
    P = 17.1
    starts = {"a": 100.0 + P * np.arange(12), "b": 3000.0 + 5.3 + P * np.arange(10), "c": 9000.0 + 11.0 + P * np.arange(9)}
    r = tables.periodic(starts)
    assert abs(r["median_spacing_s"] - P) < 1e-6 and abs(r["period_s"] - P) < 0.002
    assert all(s["coherence"] > 0.99 for s in r["sessions"].values())


def test_timers_take_whole_windows_and_only_auto_ones(monkeypatch):
    def cmd(mono, mode):
        return {"pacing_ok": True, "speed_mps": 0.1, "mono": mono, "mode": mode}

    def line(mono, tick, s):
        return {"status": True, "mono": mono, "tick_t": tick, "timers": {"x": [s, 1]}}
    log = [line(1.0, 0.1, 9.0),  # the first line opens no window: its timers are before any window here
           cmd(1.5, "PAUSED"), line(2.0, 1.1, 1.0),  # a window with a non-AUTO command
           cmd(2.5, "AUTO"), line(3.0, 2.1, 2.0),  # AUTO
           cmd(3.5, "AUTO"), line(4.0, 3.1, 3.0)]  # AUTO
    monkeypatch.setattr(tables, "read_live_log", lambda run: log)
    t, span = tables.timers(Path("."))
    assert (t["x"], round(span, 6)) == ([5.0, 2], 2.0)
    t, span = tables.timers(Path("."), auto_only=False)
    assert (t["x"], round(span, 6)) == ([6.0, 3], 3.0)
