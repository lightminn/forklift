"""Cross build of the WeAct H743 board firmware (board design 2026-11-12 sections 3.6, 10).

Needs the pinned Arm GNU Toolchain 13.3.rel1: ARM_GNU_TOOLCHAIN_DIR, or the default install
directory in the home folder. Skipped where it is absent (e.g. ws1).
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
FIRMWARE = REPO / "firmware"
BOARD = FIRMWARE / "boards" / "weact_h743"
TOOLCHAIN_FILE = BOARD / "arm-gnu-13.3.cmake"

FLASH_LIMIT = 256 * 1024  # board design 10
DTCM_LIMIT = 96 * 1024
MAIN_STACK = 8 * 1024
EXCEPTION_FRAME = 108  # FPU lazy-stacking frame 104 B + 4 B alignment padding (design 6)
NESTING = 3  # priorities 1, 2, 3 above the main loop


def _toolchain() -> Path | None:
    env = os.environ.get("ARM_GNU_TOOLCHAIN_DIR")
    candidates = [Path(env)] if env else []
    candidates.append(Path.home() / "opt" / "arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi")
    for c in candidates:
        if (c / "bin" / "arm-none-eabi-gcc").is_file():
            return c
    return None


TOOLCHAIN = _toolchain()
pytestmark = pytest.mark.skipif(
    TOOLCHAIN is None or shutil.which("cmake") is None,
    reason="Arm GNU Toolchain 13.3.rel1 or cmake not available",
)


def _configure(build: Path, *extra: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["cmake", "-S", str(FIRMWARE), "-B", str(build), "-DFW_BOARD=weact_h743",
         f"-DARM_GNU_TOOLCHAIN_DIR={TOOLCHAIN}", f"-DCMAKE_TOOLCHAIN_FILE={TOOLCHAIN_FILE}", *extra],
        capture_output=True, text=True,
    )


@pytest.fixture(scope="module")
def locked_build(tmp_path_factory) -> Path:
    build = tmp_path_factory.mktemp("weact_h743")
    cfg = _configure(build)
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    res = subprocess.run(["cmake", "--build", str(build), "-j2"], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "warning" not in (res.stdout + res.stderr).lower(), res.stdout + res.stderr
    return build / "boards" / "weact_h743"


def _sections(elf: Path) -> dict[str, int]:
    out = subprocess.run([str(TOOLCHAIN / "bin" / "arm-none-eabi-size"), "-A", str(elf)],
                         capture_output=True, text=True, check=True).stdout
    sizes = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith(".") and parts[1].isdigit():
            sizes[parts[0]] = int(parts[1])
    return sizes


def test_size_within_limits(locked_build):
    s = _sections(locked_build / "fw.elf")
    flash = s.get(".isr_vector", 0) + s.get(".text", 0) + s.get(".ARM.exidx", 0) + s.get(".data", 0)
    dtcm = s.get(".data", 0) + s.get(".bss", 0)
    assert 0 < flash <= FLASH_LIMIT
    assert dtcm <= DTCM_LIMIT


def test_build_identity_in_elf(locked_build):
    data = (locked_build / "fw.elf").read_bytes()
    m = re.search(rb"forklift-mcu (normal|commission|hil) locked \S+ link=(usb|uart7) cc=[^\x00]*"
                  rb"Arm GNU Toolchain 13\.3\.Rel1", data)
    assert m, "build identity string missing"


def test_locked_build_links_no_tim1_output(locked_build):
    map_text = (locked_build / "fw.map").read_text()
    assert "motor_out_tim1" not in map_text
    assert "motor_out_locked" in map_text


def test_stack_upper_bound(locked_build):
    """B1: the sum of every frame bounds any call chain (no recursion); B2 adds the call graph."""
    total = 0
    for su in locked_build.parent.parent.rglob("*.su"):
        for line in su.read_text().splitlines():
            fields = line.split("\t")
            if len(fields) >= 3:
                assert fields[2] in ("static", "static,bounded"), line
                total += int(fields[1])
    assert total + NESTING * EXCEPTION_FRAME <= MAIN_STACK // 2, total


def test_tim1_is_poisoned_in_locked_builds(tmp_path):
    """Negative control (design 3.6): naming TIM1 under the lock guard must not compile."""
    src = tmp_path / "uses_tim1.c"
    src.write_text("void f(void) { TIM1->CCR1 = 1; }\n")
    tp = FIRMWARE / "third_party"
    res = subprocess.run(
        [str(TOOLCHAIN / "bin" / "arm-none-eabi-gcc"), "-mcpu=cortex-m7", "-mthumb", "-c",
         "-DSTM32H743xx", "-isystem", str(tp / "cmsis_core" / "CMSIS" / "Core" / "Include"),
         "-isystem", str(tp / "cmsis_device_h7" / "Include"),
         "-include", str(BOARD / "include" / "board" / "lock_guard.h"),
         str(src), "-o", str(tmp_path / "uses_tim1.o")],
        capture_output=True, text=True,
    )
    assert res.returncode != 0
    assert "poisoned" in res.stderr


def test_guard_alone_compiles(tmp_path):
    """The positive side of the control: the guard itself is clean under -Werror."""
    src = tmp_path / "clean.c"
    src.write_text("void g(void) { TIM2->CNT = 0; }\n")
    tp = FIRMWARE / "third_party"
    res = subprocess.run(
        [str(TOOLCHAIN / "bin" / "arm-none-eabi-gcc"), "-mcpu=cortex-m7", "-mthumb", "-c",
         "-Wall", "-Wextra", "-Werror", "-DSTM32H743xx",
         "-isystem", str(tp / "cmsis_core" / "CMSIS" / "Core" / "Include"),
         "-isystem", str(tp / "cmsis_device_h7" / "Include"),
         "-include", str(BOARD / "include" / "board" / "lock_guard.h"),
         str(src), "-o", str(tmp_path / "clean.o")],
        capture_output=True, text=True,
    )
    assert res.returncode == 0, res.stderr


def test_motor_output_cannot_be_opened(tmp_path):
    """Opening the outputs is the user's decision and a separate PR (lead decision Q6)."""
    cfg = _configure(tmp_path / "open", "-DFW_MOTOR_OUTPUT=1")
    assert cfg.returncode != 0
    assert "outputs stay locked" in cfg.stderr


def test_unknown_build_kind_refused(tmp_path):
    cfg = _configure(tmp_path / "bad", "-DFW_BUILD=fast")
    assert cfg.returncode != 0
