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
import yaml

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


def _build(tmp_path_factory, name: str, *extra: str) -> Path:
    build = tmp_path_factory.mktemp(name)
    cfg = _configure(build, *extra)
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    res = subprocess.run(["cmake", "--build", str(build), "-j2"], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "warning" not in (res.stdout + res.stderr).lower(), res.stdout + res.stderr
    return build / "boards" / "weact_h743"


# Every build the board design names (2.2, 5, 9), all with the outputs locked: name -> (cmake
# arguments, link, build kind in the identity, image name).
BUILDS = {
    "usb": ((), "usb", "normal", "fw"),
    "uart7": (("-DFW_LINK=uart7",), "uart7", "normal", "fw"),
    "commission": (("-DFW_BUILD=commission",), "usb", "commission", "fw"),
    "hil": (("-DFW_BUILD=hil",), "usb", "hil", "fw"),
    "iwdg_stall": (("-DFW_TEST_IWDG_STALL=5",), "usb", "normal+iwdg-stall-5s", "fw_iwdg_stall"),
}


@pytest.fixture(scope="module")
def build_of(tmp_path_factory):
    made: dict[str, Path] = {}

    def get(name: str) -> Path:
        if name not in made:
            made[name] = _build(tmp_path_factory, f"weact_h743_{name}", *BUILDS[name][0])
        return made[name]
    return get


@pytest.fixture(scope="module")
def locked_build(build_of) -> Path:
    """The default build: normal, outputs locked, USB CDC link."""
    return build_of("usb")


@pytest.fixture(params=list(BUILDS))
def any_build(request, build_of) -> tuple[str, Path, Path]:
    """(name, build directory, ELF) for each build."""
    d = build_of(request.param)
    return request.param, d, d / f"{BUILDS[request.param][3]}.elf"


def _sections(elf: Path) -> dict[str, int]:
    out = subprocess.run([str(TOOLCHAIN / "bin" / "arm-none-eabi-size"), "-A", str(elf)],
                         capture_output=True, text=True, check=True).stdout
    sizes = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 2 and parts[0].startswith(".") and parts[1].isdigit():
            sizes[parts[0]] = int(parts[1])
    return sizes


def test_size_within_limits(any_build):
    s = _sections(any_build[2])
    flash = s.get(".isr_vector", 0) + s.get(".text", 0) + s.get(".ARM.exidx", 0) + s.get(".data", 0)
    dtcm = s.get(".data", 0) + s.get(".bss", 0)
    assert 0 < flash <= FLASH_LIMIT
    assert dtcm <= DTCM_LIMIT


def test_build_identity_in_elf(any_build):
    name, _, elf = any_build
    data = elf.read_bytes()
    m = re.search(rb"forklift-mcu (\S+) locked \S+ link=(usb|uart7) cc=[^\x00]*"
                  rb"Arm GNU Toolchain 13\.3\.Rel1", data)
    assert m, "build identity string missing"
    assert m.group(1).decode() == BUILDS[name][2]
    assert m.group(2).decode() == BUILDS[name][1]


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


def test_every_enabled_interrupt_has_its_handler(any_build, tmp_path):
    """Codex B2a review: an IRQ enabled with no vector entry jumped to address 0. An IRQ whose
    priority a board source sets counts as enabled too: TinyUSB enables the OTG one itself."""
    elf = any_build[2]
    syms = _symbols(elf)
    vec = _vectors(elf, tmp_path)
    header = (FIRMWARE / "third_party" / "cmsis_device_h7" / "Include" / "stm32h743xx.h").read_text()
    irqn = {m.group(1): int(m.group(2)) for m in re.finditer(r"^\s+(\w+)_IRQn\s+=\s+(\d+),", header, re.M)}
    assert all(v != 0 for v in vec[1:]), "a vector slot is empty"
    default = syms["Default_Handler"] | 1
    compiled = set(re.findall(r"src/(\w+)\.c\.obj", elf.with_suffix(".map").read_text()))
    enabled = set()
    for src in (BOARD / "src").glob("*.c"):
        if src.stem in compiled:
            enabled |= set(re.findall(r"NVIC_(?:EnableIRQ|SetPriority)\((\w+)_IRQn",
                                      src.read_text()))
    assert enabled, "no interrupt found"
    for name in enabled:
        slot = vec[16 + irqn[name]]
        assert slot != default, f"{name} has no handler"
        assert slot == syms[f"{name}_IRQHandler"] | 1, name


def test_locked_build_links_no_tim1_output(any_build):
    map_text = any_build[2].with_suffix(".map").read_text()
    assert "motor_out_tim1" not in map_text
    assert "motor_out_locked" in map_text


# TinyUSB's indirect calls in this image, per caller (B3, usbd.c and osal_none.h of 0.21.0): the
# class driver table holds only the CDC driver (driver->init/reset/open/control_xfer_cb/xfer_cb);
# a control transfer completes through the class's callback or the weak vendor default
# (CFG_TUD_TEST_MODE is off); the OSAL-none queue and spinlock switch interrupts through
# usbd_int_set. Nothing calls usbd_defer_func, so no deferred function is ever queued, and the CDC
# driver's sof and xfer_isr are NULL, so dcd_event_handler reaches only the interrupt switch.
TUSB_RULES = [
    "tud_rhport_init=cdcd_init|usbd_int_set",
    "tud_task_ext=cdcd_open|cdcd_control_xfer_cb|cdcd_xfer_cb|tud_vendor_control_xfer_cb|usbd_int_set",
    "configuration_reset=cdcd_reset",
    "usbd_*=usbd_int_set",
    "dcd_event_handler=usbd_int_set",
]


@pytest.mark.parametrize("name", list(BUILDS))
def test_stack_worst_case_from_call_graphs(name, build_of):
    """Board design 6: call-graph worst case per preemption level plus exception frames, within
    half of the main stack (tools/fw_stack_check.py). Levels from the lowest priority up: main
    loop, USB OTG (3), TIM6 (2), SPI and UART7 (1)."""
    build = build_of(name)
    link = BUILDS[name][1]
    levels = ["board_main"]
    if link == "usb":
        levels.append("OTG_FS_IRQHandler")
    levels += ["TIM6_DAC_IRQHandler",
               "SPI1_IRQHandler,SPI2_IRQHandler" + (",UART7_IRQHandler" if link == "uart7" else ""),
               # An unexpected IRQ runs at priority 0, HardFault above it, NMI above all (Codex B2b).
               "Default_Handler", "HardFault_Handler", "NMI_Handler"]
    rules = ["TIM6_DAC_IRQHandler=on_tick", "fw_app_*=hal_*"]
    if link == "usb":
        rules += TUSB_RULES
    args = [sys.executable, str(REPO / "tools" / "fw_stack_check.py"), str(build.parent.parent),
            "--limit", str(MAIN_STACK // 2), "--elf", str(build / f"{BUILDS[name][3]}.elf"),
            "--objdump", str(TOOLCHAIN / "bin" / "arm-none-eabi-objdump")]
    for lv in levels:
        args += ["--level", lv]
    for rule in rules:
        args += ["--indirect", rule]
    res = subprocess.run(args, capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr


def test_vendored_tinyusb_is_exactly_what_the_usb_build_reads(locked_build):
    """Lead condition (B3): only the TinyUSB files the build uses are vendored. The compiler's
    dependency files list every TinyUSB source and header the USB build opened."""
    used = set()
    for dep in locked_build.parent.parent.rglob("*.d"):
        for tok in dep.read_text().replace("\\\n", " ").split():
            if "third_party/tinyusb/" in tok and not tok.endswith(":"):
                used.add(tok.split("third_party/tinyusb/", 1)[1])
    m = yaml.safe_load((FIRMWARE / "third_party" / "MANIFEST.yaml").read_text())
    listed = next(p for p in m["packages"] if p["name"] == "tinyusb")["files"]
    assert used == {f for f in listed if f != "LICENSE"}


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


def _symbols_of(elf: Path) -> set[str]:
    return set(_symbols(elf))


def test_hil_build_runs_the_model_and_never_wakes_the_drivers(build_of):
    """Board design 9: the HIL build fills the inputs from the shared vehicle model and starts no
    motor-side peripheral; nSLEEP is never raised (motor_out_wake_drivers is not even linked)."""
    d = build_of("hil")
    syms = _symbols_of(d / "fw.elf")
    assert {"plant_step", "plant_inputs", "plant_spi_start"} <= syms
    for absent in ("motor_out_wake_drivers", "drv_spi_init", "kth_spi_init", "encoder_init", "adc_init"):
        assert absent not in syms, absent
    normal = _symbols_of(build_of("usb") / "fw.elf")
    assert "motor_out_wake_drivers" in normal and "plant_step" not in normal


def test_iwdg_stall_build_stops_the_tick(build_of):
    """Board design 9: only the S5 test build can stop the tick."""
    assert "tick_stop" in _symbols_of(build_of("iwdg_stall") / "fw_iwdg_stall.elf")
    assert "tick_stop" not in _symbols_of(build_of("usb") / "fw.elf")


@pytest.mark.parametrize("args, message", [
    (("-DFW_TEST_IWDG_STALL=5", "-DFW_MOTOR_OUTPUT=1"), "needs the outputs locked"),
    (("-DFW_TEST_IWDG_STALL=61",), "1..60"),
    (("-DFW_TEST_IWDG_STALL=x",), "1..60"),
])
def test_iwdg_stall_build_refusals(tmp_path, args, message):
    """Lead condition 2026-11-12: the stall test build never opens the outputs."""
    cfg = _configure(tmp_path / "stall", *args)
    assert cfg.returncode != 0
    assert message in cfg.stderr


def test_unknown_link_refused(tmp_path):
    cfg = _configure(tmp_path / "badlink", "-DFW_LINK=spi")
    assert cfg.returncode != 0
    assert "FW_LINK must be usb or uart7" in cfg.stderr


def test_unknown_build_kind_refused(tmp_path):
    cfg = _configure(tmp_path / "bad", "-DFW_BUILD=fast")
    assert cfg.returncode != 0
