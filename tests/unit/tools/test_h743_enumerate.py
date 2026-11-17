"""tools/h743_check/2_enumerate.sh (copied into data/h743_check by stage.sh): the product and serial read from
``udevadm info -a`` must not die of SIGPIPE under ``set -o pipefail`` (2026-11-17: ``grep -m1`` closed the pipe while
udevadm was still writing the parent devices' attributes, and the script exited 141 before its check)."""

from __future__ import annotations

import os
import shutil
import stat
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
SCRIPT = REPO / "tools" / "h743_check" / "2_enumerate.sh"
WANT = "forklift-mcu fw-hil 65f252a5c4b6"

pytestmark = pytest.mark.skipif(shutil.which("bash") is None, reason="needs bash")


def stage(tmp_path: Path, product: str, *, lines: int = 200_000, writable: bool = True):
    h = tmp_path / "h743_check"
    (h / "fw").mkdir(parents=True)
    shutil.copy(SCRIPT, h / "2_enumerate.sh")
    (h / "fw" / "BUILD.txt").write_text(WANT + "\n")
    dev = tmp_path / "forklift-mcu"
    dev.write_text("")
    if not writable:
        dev.chmod(stat.S_IRUSR)
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    # a udevadm that names the device first and then keeps writing parent attributes, far past a pipe buffer
    fake = bin_ / "udevadm"
    fake.write_text(
        "#!/usr/bin/env bash\n"
        f"echo '    ATTRS{{product}}==\"{product}\"'\n"
        "echo '    ATTRS{serial}==\"200364500000\"'\n"
        f"seq 1 {lines} | sed 's/.*/    ATTRS{{idle_&}}==\"&\"/'\n"
    )
    fake.chmod(0o755)
    env = {
        **os.environ,
        "PATH": f"{bin_}:{os.environ['PATH']}",
        "FORKLIFT_MCU_DEV": str(dev),
    }
    return h / "2_enumerate.sh", env


def run(script, env):
    return subprocess.run(
        ["bash", str(script)], env=env, capture_output=True, text=True, timeout=60
    )


def test_a_long_udevadm_listing_reads_the_product_without_sigpipe(tmp_path):
    script, env = stage(tmp_path, WANT)
    done = run(script, env)
    assert done.returncode == 0, (done.returncode, done.stdout, done.stderr)
    assert f"product: {WANT}" in done.stdout and "serial:  200364500000" in done.stdout
    assert "OK: the board runs the staged image" in done.stdout


def test_a_different_product_still_fails_the_check(tmp_path):
    script, env = stage(tmp_path, "forklift-mcu fw-hil 000000000000")
    done = run(script, env)
    assert (
        done.returncode == 1
        and "FAIL: product string is not the staged image" in done.stdout
    )


@pytest.mark.skipif(os.geteuid() == 0, reason="root can write a read-only file")
def test_no_write_permission_fails(tmp_path):
    script, env = stage(tmp_path, WANT, lines=10, writable=False)
    done = run(script, env)
    assert done.returncode == 1 and "no read/write permission" in done.stdout
