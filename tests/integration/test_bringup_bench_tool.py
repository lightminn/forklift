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
    # a fresh boot's last_rx_seq equals the first seq: a second identical BENCH_SET is the one read (P2-1)
    assert order.index("ARM") > 0 and set(order[:order.index("ARM")]) == {"BENCH_SET"}
    assert max(abs(s.state.duty_e4[2]) for s in b.samples) == 2000  # 0.2 > the image's 0.15
    armed = [s for s in b.samples if s.state.mode.name == "ARMED"]
    assert armed and all(s.state.fault_flags & FaultFlag.BENCH_OVERRIDES for s in armed)
    assert res["not_for_gate"] and "bench:steer_range" in res["bypassed"]
    assert set(res["kth"]) >= {"driven_turns", "driven_s", "mean_turns_per_s", "to_end_turns", "source"}
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


def _stopper(turns, window=None, names=()):
    s = object.__new__(bringup.CommissionSession)
    s.axis, s.ignore, s.sensorless, s.stop_kth_turns, s.window = 2, FaultFlag(0), False, turns, window
    s.bench = bringup.bl.BenchAsk(2, tuple(names), None, None) if names else None
    s.bench_taken = {"seq": 1} if names or turns is not None else None
    s._kth_base, s._prog_us, s._prog_value = None, None, None
    return s


def _st(counts, us, duty=1200, rad=0.0):
    from types import SimpleNamespace

    counts &= 0xFFFFFFFF
    return SimpleNamespace(steering_rad=rad, lift_m=0.0, fault_flags=FaultFlag(0), mcu_time_us=us,
                           duty_e4=(0, 0, duty, 0), kth_alt_crc_errors=(counts & 0xFFFF, counts >> 16))


def test_the_turn_stop_reads_the_boards_counter_either_way_and_across_its_wrap():
    stop = bringup.CommissionSession.early_stop
    for way in (1, -1):
        s = _stopper(3.0, names=("steer_range",))
        base = 0xFFFFF000  # the counter wraps inside the run
        why, i = None, 0
        while why is None and i < 1000:
            why = stop(s, _st(base + way * 1000 * i, i * 10_000), FaultFlag(0))  # ~0.24 turn per STATE
            i += 1
        assert why == "kth_turns" and i == 14  # 13 x 1000 = 13000 >= 3 x 4096 = 12288
    s = _stopper(None, window=(-0.2, 0.2))
    assert stop(s, _st(0, 0, rad=0.3), FaultFlag(0)) == "window"  # no turn stop, range on: the window as before


def test_a_driven_steering_whose_kth_stops_is_stopped_when_its_range_is_off():
    """Codex fw-bench rounds 1-2 P2: a KTH stuck from the start, or stuck after moving, with the duty on."""
    stop = bringup.CommissionSession.early_stop
    s = _stopper(3.0, names=("steer_range",))
    why = None
    for i in range(80):  # stuck from the start
        why = stop(s, _st(0, i * 10_000), FaultFlag(0))
        if why:
            break
    assert why == "kth_no_motion" and 50 <= i <= 52
    s = _stopper(3.0, names=("steer_range",))
    why, c = None, 0
    for i in range(200):  # moves 0.1 s (10 x 205 counts = 18 deg each), then stays
        c += 205 if i < 10 else 0
        why = stop(s, _st(c, i * 10_000), FaultFlag(0))
        if why:
            break
    assert why == "kth_no_motion" and 59 <= i <= 62
    s = _stopper(3.0, names=("steer_range",))  # the duty off: no judgement
    assert all(stop(s, _st(0, i * 10_000, duty=0), FaultFlag(0)) is None for i in range(80))


def test_the_turn_stop_needs_the_bench_image(model):
    b = model()
    b.host.usb_product = LR1
    res = bringup.execute(args(axis="M", duty=0.12, seconds=0.2, window=(-0.5, 0.5), stop_kth_turns=1.0,
                               observed="none"), b)
    assert res["result"] == "refused" and "bench image only" in res["why"]


def test_a_second_run_on_the_same_boot_gets_the_defaults_back(model):
    """Codex fw-bench round 1 P3-3: an override run, then a plain one on the same board: the second's BENCH_SET
    takes the defaults back and the bit goes off."""
    b = model("--bench", "1")
    b.host.usb_product = BENCH
    first = bringup.execute(args(axis="L", duty=0.1, seconds=0.6, bench_duty_max=0.5, observed="none"), b)
    assert first["result"] == "ok" and first["bench_set"]["overrides"], first
    assert any(s.state.fault_flags & FaultFlag.BENCH_OVERRIDES for s in b.samples)
    n = len(b.samples)
    second = bringup.execute(args(axis="L", duty=0.1, seconds=0.6, observed="none"), b)
    assert second["result"] == "ok" and second["bench_set"]["overrides"] is False, second
    assert second["bench_set"]["seq"] != first["bench_set"]["seq"]
    armed = [s for s in b.samples[n:] if s.state.mode.name == "ARMED"]
    assert armed and not any(s.state.fault_flags & FaultFlag.BENCH_OVERRIDES for s in armed)
    assert second["not_for_gate"] and "bench_image" in second["bypassed"]
