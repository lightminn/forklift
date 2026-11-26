"""DIAGNOSTIC BRANCH ONLY (diag/fw-bench, never merged): the bench image's pure rules -- names, the BENCH_SET it
builds, the defaults pinned to board_config.c, and the KTH turn count."""

import math
import re
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import BENCH_LIFT_RANGE, BENCH_STEER_RANGE
from forklift_core.plant import bench_limits as bl

REPO = Path(__file__).resolve().parents[3]
BOARD = (REPO / "firmware" / "boards" / "weact_h743" / "src" / "board_config.c").read_text()
DRIVE_H = (REPO / "firmware" / "core" / "include" / "fw" / "drive.h").read_text()


def test_the_names_are_the_firmware_bits_and_nothing_stops_less():
    assert bl.BENCH_BYPASS_BITS == {"steer_range": BENCH_STEER_RANGE, "lift_range": BENCH_LIFT_RANGE}
    assert re.search(r"#define DRIVE_BENCH_STEER_RANGE \(1u << 0\)", DRIVE_H)
    assert re.search(r"#define DRIVE_BENCH_LIFT_RANGE \(1u << 1\)", DRIVE_H)
    assert bl.parse_bench_bypass("steer_range, steer_range") == ("steer_range",)
    for stopping in ("overcurrent", "estop", "expiry", "iwdg", "cycle_overrun", "driver"):
        with pytest.raises(bl.LimitRefused):
            bl.parse_bench_bypass(stopping)


def test_the_defaults_are_the_open_commissioning_builds():
    assert "c.commission_duty_max_e4[ch] = 1500;" in BOARD and "c.commission_time_ms[ch] = 2000;" in BOARD
    assert "commission_duty_max_e4[DRIVE_CH_R] = 10000;" in BOARD
    assert bl.BENCH_DEFAULT_DUTY_E4 == (10000, 10000, 1500, 1500)
    assert bl.BENCH_DEFAULT_TIME_MS == (2147483, 2147483, 2000, 2000)


def test_a_bench_ask_builds_its_bench_set():
    a = bl.BenchAsk(2, ("steer_range",), 0.25, 30.0)
    assert a.bypass == BENCH_STEER_RANGE and a.overrides()
    assert a.duty_max_e4() == (10000, 10000, 2500, 1500) and a.time_ms() == (2147483, 2147483, 30000, 2000)
    assert not bl.BenchAsk(2, (), None, None).overrides()
    assert bl.bench_refusal(bl.BenchAsk(0, ("steer_range",), None, None))
    assert bl.bench_refusal(bl.BenchAsk(2, (), 0.0, None)) and bl.bench_refusal(bl.BenchAsk(2, (), None, 3000.0))


def test_the_bench_image_is_told_by_its_exact_suffix():
    assert bl.bench_image("forklift-mcu commission OPEN diag-fw-bench-1-0-gabc rec=1f4207f4 lim=lr1 bench3")
    assert not bl.bench_image("forklift-mcu commission OPEN v rec=1 lim=lr1")
    assert not bl.bench_image("forklift-mcu commission OPEN v rec=1 lim=lr1 benchx")
    assert not bl.bench_image(None)
    # an older bench generation (no travel counter) is not this image, yet stays out of gates
    old = "forklift-mcu commission OPEN v rec=1 lim=lr1 bench"
    assert not bl.bench_image(old)
    from forklift_core.plant import judgement
    assert judgement.diagnostic_image(old) and judgement.diagnostic_image(old + "2")


def test_the_travel_counter_is_read_mod_2_32():
    from types import SimpleNamespace

    assert bl.kth_counts(SimpleNamespace(kth_alt_crc_errors=(0x5678, 0x1234))) == 0x12345678
    assert bl.counts_delta(0xFFFFFFF0, 0x10) == 0x20
    assert bl.counts_delta(0x10, 0xFFFFFFF0) == -0x20
    assert bl.KTH_COUNTS_PER_TURN == 4096


def test_hold_targets_and_the_boards_steering_zero():
    """diag/fw-bench hold: here, the nearest raw 0, an offset; the raw angle read with the board's own steering zero."""
    import re as _re

    block = BOARD[BOARD.index("c.steer = (abs_axis_config)"):]
    block = block[:block.index("};")]
    assert ".zero_offset_rad = PI_F" in block and ".sign = 1" in block and ".sensor_per_joint = 1.0f" in block
    assert _re.search(r"zero_offset_rad = PI_F", block) and bl.STEER_ZERO_OFFSET_RAD == pytest.approx(math.pi)
    assert bl.kth_raw_angle(-math.pi) == 0 and bl.kth_raw_angle(0.0) == 2048 and bl.kth_raw_angle(float("nan")) is None
    assert bl.hold_target("here", 1000, 0.0) == 1000
    # raw 2048 (joint 0): the nearest raw 0 is half a turn either way -- the negative way by the rule (a <= 2048)
    assert bl.hold_target("zero", 10_000, 0.0) == 10_000 - 2048
    a100 = -math.pi + 100 * math.tau / 4096  # raw 100: back 100
    assert bl.hold_target("zero", 10_000, a100) == 10_000 - 100
    a4000 = -math.pi + 4000 * math.tau / 4096  # raw 4000: on 96
    assert bl.hold_target("zero", 10_000, a4000) == 10_000 + 96
    assert bl.hold_target("-500", 100, 0.0) == (100 - 500) & 0xFFFFFFFF
    for bad in ("zero", "nope", str(101 * 4096)):
        with pytest.raises(bl.LimitRefused):
            bl.hold_target(bad, 0, float("nan") if bad == "zero" else 0.0)
