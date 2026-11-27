"""The bench console's copy of the board's command timeout (the ARM echo window, plan 3.1) equals board_config.c."""

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import bench_console  # noqa: E402


def test_the_arm_echo_window_is_the_boards_command_timeout():
    src = (REPO / "firmware" / "boards" / "weact_h743" / "src" / "board_config.c").read_text()
    (ms,) = re.findall(r"c\.command_timeout_ms\s*=\s*(\d+)u?;", src)
    assert bench_console.BOARD_COMMAND_TIMEOUT_MS == int(ms)
