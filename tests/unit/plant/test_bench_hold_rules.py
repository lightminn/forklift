"""bench7 hold asks (bench console plan 4.3 A1b-2 decisions 2-3, 10): the CLI's target words, the defaults (11-27
measured starting values, lead 2026-11-28) and the value checks the firmware applies (bench_hold_values_ok)."""

import pytest

from forklift_core.mcu_link.messages import BenchHoldTarget, BenchMode
from forklift_core.plant import bench_limits as bl


@pytest.mark.parametrize(("text", "want"), [
    ("zero", (BenchHoldTarget.ZERO, 0)), ("here", (BenchHoldTarget.HERE, 0)),
    ("abs:0", (BenchHoldTarget.ABS, 0)), ("abs:4095", (BenchHoldTarget.ABS, 4095)),
    ("rel:-4096", (BenchHoldTarget.REL, -4096)), ("rel:+40", (BenchHoldTarget.REL, 40)),
    ("rel:1073741823", (BenchHoldTarget.REL, 2**30 - 1)),
])
def test_hold_target_words(text, want):
    assert bl.parse_hold_target(text) == (int(want[0]), want[1])


@pytest.mark.parametrize("text", ["", "abs", "abs:4096", "abs:-1", "abs:x", "rel:1073741824", "rel:-1073741824",
                                  "rel:", "zero:1", "here:0", "home"])
def test_hold_target_refused(text):
    with pytest.raises(bl.LimitRefused):
        bl.parse_hold_target(text)


def test_hold_defaults_are_the_measured_starting_values():
    h = bl.HoldAsk(int(BenchHoldTarget.HERE), 0)
    assert (h.kp, h.ki, h.kd, h.deadband, h.min_duty, h.cap) == (0.5, 0.0, 0.0, 12, 0.12, 0.15)
    assert h.duty_max_e4 == 1500 and h.min_duty_e4 == 1200
    assert bl.hold_refusal(h, 1500) is None


@pytest.mark.parametrize(("kw", "cap_e4"), [
    ({"kp": 0.0}, 1500), ({"kp": 100.5}, 1500), ({"ki": -1.0}, 1500), ({"ki": 1000.5}, 1500), ({"kd": 10.5}, 1500),
    ({"kp": float("nan")}, 1500), ({"kd": float("inf")}, 1500), ({"deadband": 513}, 1500), ({"deadband": -1}, 1500),
    ({"cap": 0.2}, 1500), ({"cap": 0.0}, 1500), ({"min_duty": 0.16}, 1500), ({"min_duty": -0.01}, 1500),
])
def test_hold_values_refused_as_the_firmware_would(kw, cap_e4):
    h = bl.HoldAsk(int(BenchHoldTarget.HERE), 0, **kw)
    assert bl.hold_refusal(h, cap_e4) is not None


def test_a_hold_bench_ask():
    h = bl.HoldAsk(int(BenchHoldTarget.REL), 100)
    a = bl.BenchAsk(2, ("steer_range",), None, None, source=1, mode=int(BenchMode.HOLD), hold=h)
    assert a.hold_run and not a.distance and bl.bench_refusal(a) is None
    # a hold needs the KTH source, M or H, and no distance stop
    assert bl.bench_refusal(bl.BenchAsk(2, (), None, None, source=0, mode=int(BenchMode.HOLD), hold=h))
    assert bl.bench_refusal(bl.BenchAsk(0, (), None, None, source=1, mode=int(BenchMode.HOLD), hold=h))
    assert bl.bench_refusal(bl.BenchAsk(2, (), None, None, source=1, stop_counts=4096, mode=int(BenchMode.HOLD),
                                        hold=h))
    assert bl.bench_refusal(bl.BenchAsk(2, (), None, None, source=1, mode=int(BenchMode.HOLD), hold=None))
    # the hold's cap within the run axis's duty limit (the image's default 0.15, or the run's own)
    big = bl.HoldAsk(int(BenchHoldTarget.HERE), 0, cap=0.3)
    assert bl.bench_refusal(bl.BenchAsk(2, (), None, None, source=1, mode=int(BenchMode.HOLD), hold=big))
    assert bl.bench_refusal(bl.BenchAsk(2, (), 0.3, None, source=1, mode=int(BenchMode.HOLD), hold=big)) is None
