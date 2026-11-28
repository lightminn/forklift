"""The console's model device takes the model's own product (bench console plan 4.5 A1c decision 2): an old model that
says nothing, or one that says another generation, is refused within the deadline and its child is gone."""
import os
import stat
import sys
import time
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))

import bench_console  # noqa: E402


def fake_model(tmp_path: Path, line: str | None) -> tuple[Path, Path]:
    pid_file = tmp_path / "pid"
    say = f"echo '{line}'\n" if line is not None else ""
    script = tmp_path / "mcu_model"
    script.write_text(f"#!/bin/sh\necho $$ > {pid_file}\n{say}exec sleep 30\n")
    script.chmod(script.stat().st_mode | stat.S_IXUSR)
    return script, pid_file


def gone(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return True
    return False


@pytest.mark.parametrize("line", [None, "product: forklift-mcu commission OPEN model rec=00000000 lim=lr1 bench7"])
def test_an_old_or_other_model_is_refused_and_stopped(tmp_path, line):
    exe, pid_file = fake_model(tmp_path, line)
    t0 = time.monotonic()
    with pytest.raises(RuntimeError, match="not this tool's bench image"):
        bench_console.ModelDevice(str(exe), gate=lambda *a, **k: True)
    assert time.monotonic() - t0 < 5.0  # the 2 s deadline, never a wait on a silent model
    pid = int(pid_file.read_text())
    assert gone(pid)  # the child is stopped and reaped with the refusal


def test_a_bench8_model_line_is_its_product(tmp_path):
    exe, pid_file = fake_model(tmp_path, "product: forklift-mcu commission OPEN model rec=00000000 lim=lr1 bench8")
    dev = bench_console.ModelDevice(str(exe), gate=lambda *a, **k: True)
    try:
        assert dev.product.endswith("lim=lr1 bench8")
    finally:
        dev.close()
    assert gone(int(pid_file.read_text()))
