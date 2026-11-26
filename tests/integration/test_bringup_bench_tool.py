"""DIAGNOSTIC BRANCH ONLY (diag/fw-bench, never merged): the commissioning tool on the bench image (the MCU model with
--bench 1) -- the BENCH_SET sent before every ARM and read back by its seq, the steering run with its range turned
off, the session's limits, and the refusals. The model's plant is synthetic: no currents, speeds or real turns."""

import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "integration"))
import bringup  # noqa: E402
from test_bringup_commission_tool import LR1, args, kinds_written, model  # noqa: E402,F401
from test_bringup_tool import mcu_model  # noqa: E402,F401

from forklift_core.mcu_link.messages import FaultFlag  # noqa: E402

BENCH = "forklift-mcu commission OPEN v0-test rec=00000000 lim=lr1 bench"


def rows(b):
    return [json.loads(x) for x in b.out.getvalue().splitlines()]


def test_a_steering_run_with_its_range_off_and_its_own_limits(model):
    b = model("--bench", "1")
    b.host.usb_product = BENCH
    res = bringup.execute(args(axis="M", duty=0.2, seconds=1.0, bench_bypass="steer_range", bench_duty_max=0.25,
                               bench_time_max=5.0, stop_kth_turns=3.0, observed="none"), b)
    assert res["result"] == "ok" and res["cleanup_confirmed"], res
    taken = res["bench_set"]
    assert taken["bypass"] == ["steer_range"] and taken["overrides"]
    assert taken["duty_max_e4"] == [10000, 10000, 2500, 1500] and taken["time_ms"][2] == 5000
    # the BENCH_SET went out before the ARM, and the board shows its overrides from then on
    order = [k for k in kinds_written(b) if k in ("BENCH_SET", "ARM")]
    assert order[:2] == ["BENCH_SET", "ARM"]
    assert max(abs(s.state.duty_e4[2]) for s in b.samples) == 2000  # 0.2 > the image's 0.15
    armed = [s for s in b.samples if s.state.mode.name == "ARMED"]
    assert armed and all(s.state.fault_flags & FaultFlag.BENCH_OVERRIDES for s in armed)
    assert res["not_for_gate"] and "bench:steer_range" in res["bypassed"]
    assert set(res["kth"]) >= {"driven_turns", "driven_s", "mean_turns_per_s", "to_end_turns", "ambiguous"}
    assert res["bench"] == {"bypass": ["steer_range"], "duty_max": 0.25, "time_max_s": 5.0}
    (bs,) = [r for r in rows(b) if r["t"] == "bench_set"]
    assert bs["seq"] == taken["seq"]


def test_a_bench_board_gets_its_defaults_when_nothing_is_asked(model):
    """A previous run's overrides never carry: every run on the bench image sends its BENCH_SET, the defaults here."""
    b = model("--bench", "1")
    b.host.usb_product = BENCH
    res = bringup.execute(args(axis="L", duty=0.1, seconds=0.6, observed="none"), b)
    assert res["result"] == "ok", res
    assert res["bench_set"]["overrides"] is False and res["bench_set"]["bypass"] == []
    assert not any(s.state.fault_flags & FaultFlag.BENCH_OVERRIDES for s in b.samples[-5:])


def test_bench_values_on_another_image_are_refused_before_anything(model):
    b = model()
    b.host.usb_product = LR1
    res = bringup.execute(args(axis="M", duty=0.12, seconds=1.0, bench_bypass="steer_range", observed="none"), b)
    assert res["result"] == "refused" and "not the bench image" in res["why"]
    assert "ARM" not in kinds_written(b) and "BENCH_SET" not in kinds_written(b)


@pytest.mark.parametrize("kw,why", [
    (dict(bench_bypass="steer_range", window=(-3.2, 3.2)), "give no --window"),
    (dict(bench_bypass="overcurrent"), "unknown name"),
    (dict(bench_bypass="lift_range"), "lift axis (H) only"),
    (dict(bench_duty_max=0.25, duty=0.3), "within +-0.25"),
    (dict(bench_duty_max=1.5), "bench-duty-max"),
    (dict(bench_time_max=1.0, seconds=0.8), "seconds must be in"),
    (dict(bench_bypass="steer_range", stop_kth_turns=0.0), "stop-kth-turns"),
    (dict(bench_bypass="steer_range", single_axis=True, motor_detached=True), "a plain run only"),
])
def test_bench_argument_refusals(model, kw, why):
    b = model("--bench", "1")
    b.host.usb_product = BENCH
    base = dict(axis="M", duty=0.12, seconds=1.0, observed="none")
    res = bringup.execute(args(**{**base, **kw}), b)
    assert res["result"] == "refused" and why in res["why"], res
    assert "ARM" not in kinds_written(b)


def test_without_the_range_bypass_the_steering_still_needs_its_window(model):
    b = model("--bench", "1")
    b.host.usb_product = BENCH
    res = bringup.execute(args(axis="M", duty=0.12, seconds=1.0, bench_duty_max=0.25, observed="none"), b)
    assert res["result"] == "refused" and "--window" in res["why"]


def _stopper(turns, window=None):
    from types import SimpleNamespace

    return SimpleNamespace(axis=2, ignore=FaultFlag(0), sensorless=False, stop_kth_turns=turns, _kth_prev=None,
                           _kth_rad=0.0, window=window)


def _st(rad):
    from types import SimpleNamespace

    return SimpleNamespace(steering_rad=rad, lift_m=0.0, fault_flags=FaultFlag(0))


def test_the_kth_turn_stop_counts_across_the_wrap_and_refuses_a_step_too_far():
    import math

    s = _stopper(1.0)
    stop = bringup.CommissionSession.early_stop
    why, i = None, 0
    while why is None and i < 1000:
        why = stop(s, _st(math.remainder(-0.4 * i, math.tau)), FaultFlag(0))  # the negative way, 0.4 rad per STATE
        i += 1
    assert why == "kth_turns" and i == 17  # 16 steps x 0.4 = 6.4 rad >= one turn (6.28)
    s = _stopper(3.0)
    assert stop(s, _st(0.0), FaultFlag(0)) is None
    assert stop(s, _st(2.5), FaultFlag(0)) == "kth_step_ambiguous"  # 143 deg in one STATE: never guessed
    s = _stopper(None, window=(-0.2, 0.2))
    assert stop(s, _st(0.3), FaultFlag(0)) == "window"  # no turn stop: the window as before
