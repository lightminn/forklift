"""v2 main batch verdict pieces (docs/plans/2026-11-08-live-riib-defaults-v2.md 3.3): the longest ok=false run's end
rules, stalls with the C2 replan steps apart and the previous row's split, the four pooled comparisons, trace breaks."""

import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import riib_v2_verdict as V  # noqa: E402


def test_the_longest_false_run_ends_at_the_next_true_step_a_gap_or_the_window_end():
    steps = {0: (0.0, 0, True), 1: (1.0, 0, False), 2: (2.0, 0, False), 3: (3.0, 0, True),  # 1 -> 3: 2 s
             4: (4.0, 0, False), 5: (5.0, 0, False), 7: (20.0, 0, False),  # gap after 5: ends at 5 (1 s), 7 alone
             8: (21.0, 0, False), 9: (24.0, 0, False)}  # 7..9 false to the window's end: 24 - 20 = 4 s
    assert V.longest_false(steps, 0.0, 100.0) == pytest.approx(4.0)
    assert V.longest_false(steps, 0.0, 6.0) == pytest.approx(2.0)


def trace(S, flags=None):
    names = ["live_camera", "video", "render_arg", "scan", "rendered", "capture", "pocket", "far"]
    S = np.asarray(S, float)
    return {"S": S, "live_step": np.arange(len(S), dtype=float),
            "flags": np.asarray(flags if flags is not None else [math.nan] * len(S), float), "flag_names": names}


def test_stalls_split_by_the_previous_row_and_c2_steps_apart():
    rendered, capture = 1 << 4, (1 << 5) | (1 << 4)
    tr = trace([0.0, 0.03, 0.40, 0.43, 0.80, 0.83, 1.30], [math.nan, rendered, math.nan, capture, math.nan, 0, math.nan])
    out = V.stall_rows(tr, 0.0, 10.0, exclude={6})
    assert (out["total"], out["rendered"], out["capture"], out["other"], out["c2_excluded"]) == (2, 1, 1, 0, 1)
    assert V.stall_rows(tr, 0.5, 10.0, exclude=set())["total"] == 2  # S[k] in the interval: steps 4 and 6


def test_the_four_pooled_rules():
    a = {"sim": 300.0, "wall": 1000.0, "false_wall": 50.0, "longest": 10.0, "stalls": 3}
    ok = {"sim": 292.0, "wall": 1000.0, "false_wall": 60.0, "longest": 14.0, "stalls": 5}
    out = V.arm_rules(a, ok)
    assert out["pass"] and out["F"]["pass"] and out["R"]["pass"] and out["longest"]["pass"] and out["stalls"]["pass"]
    assert not V.arm_rules(a, {**ok, "sim": 290.0})["F"]["pass"]  # 0.290 < 0.97 x 0.300
    assert not V.arm_rules(a, {**ok, "false_wall": 61.0})["R"]["pass"]  # 0.061 > 0.05 + 0.01
    assert not V.arm_rules(a, {**ok, "longest": 14.1})["longest"]["pass"]  # > 1.2 x 10 + 2
    assert not V.arm_rules(a, {**ok, "stalls": 6})["stalls"]["pass"]  # > 3 + 2


def test_a_trace_break_inside_the_interval_is_missing():
    tr = trace([0.0, 0.1, 0.2, 0.3])
    assert not V.trace_missing(tr, 0.0, 1.0)
    tr["live_step"] = np.array([0.0, 1.0, 3.0, 4.0])
    assert V.trace_missing(tr, 0.0, 1.0) and not V.trace_missing(tr, 0.15, 1.0)
