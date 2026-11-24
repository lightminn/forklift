"""Bench commissioning limits (docs/design/2026-11-25-bench-commission-limits.md 1.2, PR-1): the pure rules."""

import re
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.plant import bench_limits as bl

REPO = Path(__file__).resolve().parents[3]
BOARD = (REPO / "firmware" / "boards" / "weact_h743" / "src" / "board_config.c").read_text()


def test_the_tool_constants_are_the_boards():
    """The tool's copies of the board's values: the wheel slew, the open commissioning build's wheel limits, and the
    default 0.15 / 2 s on every axis."""
    assert re.search(r"c\.wheel_duty_slew_per_s = 2\.0f;", BOARD) and bl.WHEEL_SLEW_PER_S == 2.0
    block = BOARD[BOARD.index("#if defined(FW_BUILD_commission) && defined(FW_MOTOR_OUTPUT_OPEN)"):]
    block = block[:block.index("#endif")]
    assert "commission_duty_max_e4[DRIVE_CH_R] = 10000;" in block and bl.WHEEL_DUTY_MAX_OPEN == 1.0
    assert "commission_time_ms[DRIVE_CH_R] = 0x7FFFFFFFu / 1000u;" in block
    assert bl.WHEEL_TIME_MAX_S_OPEN == 2147.483
    assert "c.commission_duty_max_e4[ch] = 1500;" in BOARD and "c.commission_time_ms[ch] = 2000;" in BOARD


def test_bypass_names_and_bits():
    assert bl.parse_bypass(None) == () and bl.parse_bypass("") == ()
    assert bl.parse_bypass("enc_stuck, enc_polarity,enc_stuck") == ("enc_stuck", "enc_polarity")
    with pytest.raises(bl.LimitRefused):
        bl.parse_bypass("enc_stuck,overcurrent")
    assert bl.ignored_bits(("enc_stuck",), 0) == FaultFlag.ENCODER_STUCK_L
    assert bl.ignored_bits(("enc_stuck", "enc_polarity"), 1) == FaultFlag.ENCODER_STUCK_R | FaultFlag.ENCODER_POLARITY_R
    assert bl.ignored_bits(("enc_stuck",), 2) == FaultFlag(0)


def test_triangle_pieces_sign_and_shape():
    p = bl.triangle(0.05, 0.3, 1.0, 2, -1)
    assert len(p.pieces) == 20 and p.duration_s == pytest.approx(2.0)
    assert all(x < 0 for x in p.pieces)  # the sign on every piece
    assert p.pieces[:5] == (-750, -1250, -1750, -2250, -2750) and p.pieces[5:10] == (-2750, -2250, -1750, -1250, -750)
    assert p.max_abs_e4 == 2750  # piece centres: the peak sits just under HI (11-24: 0.2975 for 0.30)
    for bad in ((0.3, 0.3, 1.0, 1, 1), (0.05, 1.1, 1.0, 1, 1), (0.05, 0.3, 1.05, 1, 1), (0.05, 0.3, 0.1, 1, 1),
                (0.05, 0.3, 1.0, 0, 1), (0.05, 0.3, 1.0, 1, 0)):
        with pytest.raises(bl.LimitRefused):
            bl.triangle(*bad)


def test_slope_on_the_generated_pieces_first_step_included():
    assert bl.slope_problem((2000, 4000, 2000, 0)) is None  # 0.2 per 0.1 s = 2.0 /s: at the slew
    assert "piece 0" in bl.slope_problem((2001,))
    assert "piece 2" in bl.slope_problem((1000, 2000, 4100))
    # LO 0.20 HI 0.80 PERIOD 1.0: both written slopes pass, the first piece (0.26) does not (Codex S 3rd P3)
    assert "piece 0" in bl.slope_problem(bl.triangle(0.2, 0.8, 1.0, 1, 1).pieces)


@pytest.mark.parametrize("ask,large", [
    (bl.RunAsk(0, 0.15, 1.5), False), (bl.RunAsk(0, 0.1501, 1.0), True), (bl.RunAsk(1, -0.1, 1.51), True),
    (bl.RunAsk(2, 0.5, 5.0), False),  # steering/lift are never large: their own checks refuse
    (bl.RunAsk(0, 0.5, 5.0, "limit_disarm"), False),
])
def test_what_is_large(ask, large):
    assert bl.is_large(ask) is large


def test_large_refusals():
    ask = bl.RunAsk(0, 0.3, 10.0)
    ok = dict(wheels_lifted=True, vcc_watched=False, synthetic=False)
    assert bl.large_refusal(ask, **ok) is None
    assert "wheels lifted" in bl.large_refusal(ask, **{**ok, "wheels_lifted": False})
    assert "vcc" in bl.large_refusal(bl.RunAsk(0, 0.31, 1.0), **ok)
    assert bl.large_refusal(bl.RunAsk(0, 0.31, 1.0), **{**ok, "vcc_watched": True}) is None
    assert bl.large_refusal(bl.RunAsk(0, 0.31, 1.0), **{**ok, "synthetic": True}) is None
    assert "at most 1.0" in bl.large_refusal(bl.RunAsk(0, 1.01, 1.0), **ok)
    assert bl.large_refusal(bl.RunAsk(0, 0.3, 600.0), **ok) is None
    assert "at most 600" in bl.large_refusal(bl.RunAsk(0, 0.3, 600.01), **ok)
    assert bl.large_refusal(bl.RunAsk(0, 0.1, 1.0), **{**ok, "wheels_lifted": False}) is None  # not large


@pytest.mark.parametrize("product,ok", [
    ("forklift-mcu commission OPEN v1-2-gabc rec=1f4207f4 lim=lr1", True),
    (None, False),
    ("forklift-mcu commission OPEN v1-2-gabc rec=1f4207f4", False),
    ("forklift-mcu commission OPEN v1-2-gabc rec=1f4207f4 lim=lr", False),  # cut at 126 characters
    ("forklift-mcu commission locked v1 lim=lr1", False),
    ("forklift-mcu normal OPEN v1 rec=1f4207f4 waiver=00000000", False),
    ("Silicon Labs CP2102", False),
])
def test_board_check(product, ok):
    assert (bl.board_refusal(product) is None) is ok


def test_the_confirmation_phrase():
    assert bl.confirmation_phrase(bl.RunAsk(0, -0.3, 10.0)) == "duty 0.3, 10 s, wheels lifted and fixed"
