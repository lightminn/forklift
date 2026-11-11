"""Cross build of the WeAct H743 board firmware (board design 2026-11-12 sections 3.6, 10).

Needs the pinned Arm GNU Toolchain 13.3.rel1: ARM_GNU_TOOLCHAIN_DIR, or the default install
directory in the home folder. Skipped where it is absent (e.g. ws1).
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
FIRMWARE = REPO / "firmware"
BOARD = FIRMWARE / "boards" / "weact_h743"
TOOLCHAIN_FILE = BOARD / "arm-gnu-13.3.cmake"

FLASH_LIMIT = 256 * 1024  # board design 10
DTCM_LIMIT = 96 * 1024
MAIN_STACK = 8 * 1024


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
         f"-DARM_GNU_TOOLCHAIN_DIR={TOOLCHAIN}", f"-DCMAKE_TOOLCHAIN_FILE={TOOLCHAIN_FILE}",
         f"-DFW_PYTHON={sys.executable}", *extra],
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


def _symbols(elf: Path) -> dict[str, int]:
    out = subprocess.run([str(TOOLCHAIN / "bin" / "arm-none-eabi-nm"), str(elf)],
                         capture_output=True, text=True, check=True).stdout
    syms = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            syms[parts[2]] = int(parts[0], 16)
    return syms


def _vectors(elf: Path, tmp: Path) -> list[int]:
    raw = tmp / "isr_vector.bin"
    subprocess.run([str(TOOLCHAIN / "bin" / "arm-none-eabi-objcopy"), "-O", "binary", "-j", ".isr_vector",
                    str(elf), str(raw)], check=True)
    data = raw.read_bytes()
    return [int.from_bytes(data[i:i + 4], "little") for i in range(0, len(data), 4)]


def test_every_enabled_interrupt_has_its_handler(locked_build, tmp_path):
    """Codex B2a review: an IRQ enabled with no vector entry jumped to address 0."""
    elf = locked_build / "fw.elf"
    syms = _symbols(elf)
    vec = _vectors(elf, tmp_path)
    header = (FIRMWARE / "third_party" / "cmsis_device_h7" / "Include" / "stm32h743xx.h").read_text()
    irqn = {m.group(1): int(m.group(2)) for m in re.finditer(r"^\s+(\w+)_IRQn\s+=\s+(\d+),", header, re.M)}
    assert all(v != 0 for v in vec[1:]), "a vector slot is empty"
    default = syms["Default_Handler"] | 1
    enabled = set()
    for src in (BOARD / "src").glob("*.c"):
        enabled |= set(re.findall(r"NVIC_EnableIRQ\((\w+)_IRQn\)", src.read_text()))
    assert enabled, "no interrupt found"
    for name in enabled:
        slot = vec[16 + irqn[name]]
        assert slot != default, f"{name} has no handler"
        assert slot == syms[f"{name}_IRQHandler"] | 1, name


def test_locked_build_links_no_tim1_output(locked_build):
    map_text = (locked_build / "fw.map").read_text()
    assert "motor_out_tim1" not in map_text
    assert "motor_out_locked" in map_text


def test_stack_worst_case_from_call_graphs(locked_build):
    """Board design 6: call-graph worst case per preemption level plus exception frames, within
    half of the main stack (tools/fw_stack_check.py)."""
    res = subprocess.run(
        [sys.executable, str(REPO / "tools" / "fw_stack_check.py"), str(locked_build.parent.parent),
         "--limit", str(MAIN_STACK // 2), "--level", "board_main", "--level", "TIM6_DAC_IRQHandler",
         "--level", "SPI1_IRQHandler,SPI2_IRQHandler",
         "--level", "NMI_Handler,HardFault_Handler,Default_Handler",
         "--indirect", "TIM6_DAC_IRQHandler=on_tick", "--indirect", "*=hal_*"],
        capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr


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


def _identity(build: Path) -> str:
    data = (build / "boards" / "weact_h743" / "fw.elf").read_bytes()
    m = re.search(rb"forklift-mcu \S+ locked (\S+) link=", data)
    assert m, "build identity string missing"
    return m.group(1).decode()


def test_identity_follows_the_tree_in_an_incremental_build(tmp_path):
    """Codex B1 review: a new commit or a dirty tree must reach the ELF without reconfiguring."""
    if shutil.which("git") is None:
        pytest.skip("git not available")
    repo = tmp_path / "repo"
    shutil.copytree(FIRMWARE, repo / "firmware", ignore=shutil.ignore_patterns("build"))

    def git(*args: str) -> str:
        return subprocess.run(["git", "-c", "user.email=t@example.invalid", "-c", "user.name=t",
                               *args], cwd=repo, capture_output=True, text=True, check=True).stdout

    header = tmp_path / "vehicle_config.h"
    gen = subprocess.run([sys.executable, str(REPO / "tools" / "real_vehicle_header.py"), "--config",
                          str(REPO / "config" / "real" / "vehicle.yaml"), "--out", str(header)],
                         env={**os.environ, "PYTHONPATH": str(REPO / "src")}, capture_output=True, text=True)
    assert gen.returncode == 0, gen.stderr
    git("init", "-q")
    git("add", "-A")
    git("commit", "-q", "-m", "a")
    build = tmp_path / "build"
    cfg = subprocess.run(
        ["cmake", "-S", str(repo / "firmware"), "-B", str(build), "-DFW_BOARD=weact_h743",
         f"-DARM_GNU_TOOLCHAIN_DIR={TOOLCHAIN}", f"-DFW_VEHICLE_HEADER={header}",
         f"-DCMAKE_TOOLCHAIN_FILE={repo / 'firmware/boards/weact_h743/arm-gnu-13.3.cmake'}"],
        capture_output=True, text=True)
    assert cfg.returncode == 0, cfg.stderr

    def build_once() -> str:
        res = subprocess.run(["cmake", "--build", str(build), "-j2"], capture_output=True, text=True)
        assert res.returncode == 0, res.stdout + res.stderr
        return _identity(build)

    head_a = git("rev-parse", "--short=12", "HEAD").strip()
    assert build_once() == head_a
    src = repo / "firmware" / "boards" / "weact_h743" / "src" / "led.c"
    src.write_text(src.read_text() + "/* changed */\n")
    assert build_once() == head_a + "-dirty"
    git("commit", "-q", "-am", "b")
    head_b = git("rev-parse", "--short=12", "HEAD").strip()
    assert head_b != head_a
    assert build_once() == head_b


def test_unknown_build_kind_refused(tmp_path):
    cfg = _configure(tmp_path / "bad", "-DFW_BUILD=fast")
    assert cfg.returncode != 0
