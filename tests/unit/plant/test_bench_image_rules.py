"""The bench image's pure rules (bench console plan A1a, mcu-link contract 8): the names, the BENCH_SET a run builds,
the defaults pinned to board_config.c, the image told by its exact suffix, and every bench image kept out of gates."""

import re
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import BENCH_BYPASS_KNOWN, BenchBypass
from forklift_core.plant import bench_limits as bl
from forklift_core.plant import judgement

REPO = Path(__file__).resolve().parents[3]
BOARD = (REPO / "firmware" / "boards" / "weact_h743" / "src" / "board_config.c").read_text()
DRIVE_H = (REPO / "firmware" / "core" / "include" / "fw" / "drive.h").read_text()
BENCH5 = "forklift-mcu commission OPEN v0-1-gabcdefabcdef rec=1f4207f4 lim=lr1 bench7"


def test_the_names_are_the_firmware_bits_and_nothing_stops_less():
    assert bl.BENCH_BYPASS_BITS == {"steer_range": int(BenchBypass.STEER_RANGE),
                                    "lift_range": int(BenchBypass.LIFT_RANGE)}
    assert sum(bl.BENCH_BYPASS_BITS.values()) == BENCH_BYPASS_KNOWN
    assert re.search(r"#define DRIVE_BENCH_STEER_RANGE \(1u << 0\)", DRIVE_H)
    assert re.search(r"#define DRIVE_BENCH_LIFT_RANGE \(1u << 1\)", DRIVE_H)
    assert bl.parse_bench_bypass("steer_range, steer_range") == ("steer_range",)
    for stopping in ("overcurrent", "current_sense", "estop", "expiry", "iwdg", "cycle_overrun", "driver"):
        with pytest.raises(bl.LimitRefused):
            bl.parse_bench_bypass(stopping)


def test_the_defaults_are_the_open_commissioning_builds():
    assert "c.commission_duty_max_e4[ch] = 1500;" in BOARD and "c.commission_time_ms[ch] = 2000;" in BOARD
    assert "commission_duty_max_e4[DRIVE_CH_R] = 10000;" in BOARD
    assert "commission_time_ms[DRIVE_CH_R] = 0x7FFFFFFFu / 1000u;" in BOARD
    assert bl.BENCH_DEFAULT_DUTY_E4 == (10000, 10000, 1500, 1500)
    assert bl.BENCH_DEFAULT_TIME_MS == (2147483, 2147483, 2000, 2000)


def test_a_bench_ask_builds_its_bench_set():
    a = bl.BenchAsk(2, ("steer_range",), 0.25, 30.0)
    assert a.bypass == int(BenchBypass.STEER_RANGE) and a.overrides()
    assert a.duty_max_e4() == (10000, 10000, 2500, 1500) and a.time_ms() == (2147483, 2147483, 30000, 2000)
    assert not bl.BenchAsk(2, (), None, None).overrides()
    assert bl.bench_refusal(bl.BenchAsk(0, ("steer_range",), None, None))
    assert bl.bench_refusal(bl.BenchAsk(2, ("lift_range",), None, None))
    assert bl.bench_refusal(bl.BenchAsk(2, (), 0.0, None)) and bl.bench_refusal(bl.BenchAsk(2, (), None, 3000.0))
    assert bl.bench_refusal(bl.BenchAsk(3, ("lift_range",), 0.3, 10.0)) is None


def test_the_bench_image_is_told_by_its_exact_suffix():
    assert bl.bench_image(BENCH5)
    for other in ("forklift-mcu commission OPEN v rec=1 lim=lr1",
                  "forklift-mcu commission OPEN v rec=1 lim=lr1 bench4",
                  "forklift-mcu commission OPEN v rec=1 lim=lr1 bench5",  # A1a's generation (plan 4.2 A1b-1)
                  "forklift-mcu commission OPEN v rec=1 lim=lr1 bench6",  # A1b-1's generation (plan 4.3 A1b-2)
                  "forklift-mcu commission OPEN diag-fw-bench-2 rec=1f4207f4 lim=lr1 bench3",
                  "forklift-mcu commission OPEN v rec=1 lim=lr1 bench7x",
                  "forklift-mcu normal OPEN v rec=1 lim=lr1 bench7", None):
        assert not bl.bench_image(other), other
    assert bl.any_bench_image("forklift-mcu commission OPEN v rec=1 lim=lr1 bench4")


@pytest.mark.parametrize("product", [BENCH5, "forklift-mcu commission OPEN v rec=1 lim=lr1 bench4",
                                     "forklift-mcu commission OPEN v rec=1 lim=lr1 bench"])
def test_every_bench_image_stays_out_of_gates(product):
    """A run on any bench generation is no gate evidence, a run without any override included (plan A1a)."""
    assert judgement.diagnostic_image(product)


def test_other_images_keep_their_gate_reading():
    assert not judgement.diagnostic_image("forklift-mcu commission OPEN v rec=1 lim=lr1")
    assert not judgement.diagnostic_image("forklift-mcu commission OPEN v rec=1 lim=lr1 sa1")
    assert judgement.diagnostic_image("forklift-mcu commission OPEN diag-commission-unlimited-1 rec=1 lim=lr1")
    assert not judgement.diagnostic_image(None)
