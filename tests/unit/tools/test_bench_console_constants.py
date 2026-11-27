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


def test_an_ended_model_is_a_lost_device(tmp_path):
    """bench_console --model (plan 4.1): a model process that ended is told to the host as a lost device, so the
    console stops being ready instead of showing its last state (Codex C round 3 P2-5)."""
    import time

    dev = bench_console.ModelDevice("/bin/true", gate=lambda f, w: (w(), True)[1])
    console = bench_console.BenchConsole(dev, runs_root=tmp_path)
    try:
        end = time.monotonic() + 5.0
        while dev.proc.poll() is None and time.monotonic() < end:
            time.sleep(0.01)
        console.cycle()
        console.cycle()
        assert dev.dead and console.not_ready() is not None
    finally:
        dev.close()
