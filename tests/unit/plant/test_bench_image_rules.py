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
    assert bl.bench_image("forklift-mcu commission OPEN diag-fw-bench-1-0-gabc rec=1f4207f4 lim=lr1 bench2")
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
