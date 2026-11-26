"""Cross build of the WeAct H743 board firmware (board design 2026-11-12 sections 3.6, 10).

Needs the pinned Arm GNU Toolchain 13.3.rel1: ARM_GNU_TOOLCHAIN_DIR, or the default install
directory in the home folder. Skipped where it is absent (e.g. ws1).
"""
from __future__ import annotations

import hashlib
import json
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
         f"-DFW_PYTHON={sys.executable}", "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON", *extra],
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
    assert b"lim=" not in data  # only the open commissioning image raises its limits
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


def test_every_board_source_of_a_locked_build_has_tim1_poisoned(any_build):
    """Board design 3.6: in a locked build no board C source may name TIM1 (per-source guard)."""
    cmds = json.loads((any_build[1].parent.parent / "compile_commands.json").read_text())
    board = [c for c in cmds if "/boards/weact_h743/src/" in c["file"] and c["file"].endswith(".c")]
    assert len(board) > 10
    for c in board:
        assert "lock_guard.h" in c["command"], c["file"]


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


def test_motor_output_needs_the_protection_record(tmp_path):
    """Motor output design 2: no record, no output build -- and the refusal says whose decision it
    is (lead decision Q6)."""
    cfg = _configure(tmp_path / "open", "-DFW_MOTOR_OUTPUT=1", "-DFW_BUILD=commission")
    assert cfg.returncode != 0
    assert "user's decision" in _flat(cfg.stderr) and "IPROPI" in cfg.stderr
    assert "(lead decision Q6); being able" in _flat(cfg.stderr)  # the text kept whole (Codex)


def _flat(text: str) -> str:
    """CMake wraps long messages: compare with the whitespace collapsed."""
    return " ".join(text.split())


MARKER = ("IPROPI protection record: confirmed-by=test fixture; date=2026-11-12; "
          "evidence=not a real record (tests/integration/test_firmware_cross.py)\n")
WAIVER_MARKER = ("E-stop waiver record: confirmed-by=test fixture; date=2026-11-18; "
                 "evidence=not a real record (tests/integration/test_firmware_cross.py)\n")


@pytest.fixture(scope="module")
def record_repo(tmp_path_factory):
    """The firmware in a scratch git repository outside this one, with a committed fake record
    (lead condition: tests never write a record into the project)."""
    root = tmp_path_factory.mktemp("record_repo")
    repo = root / "repo"
    shutil.copytree(FIRMWARE, repo / "firmware", ignore=shutil.ignore_patterns("build"))
    # the operating (open normal) build generates its vehicle header from the copy's own vehicle.yaml
    # (operating plan 2026-11-26 2.4): the config, its chassis model, the generator and the package
    shutil.copytree(REPO / "config" / "real", repo / "config" / "real")
    shutil.copytree(REPO / "sim" / "models" / "dls08_measured", repo / "sim" / "models" / "dls08_measured")
    (repo / "tools").mkdir()
    shutil.copy2(REPO / "tools" / "real_vehicle_header.py", repo / "tools" / "real_vehicle_header.py")
    shutil.copytree(REPO / "src" / "forklift_core", repo / "src" / "forklift_core",
                    ignore=shutil.ignore_patterns("__pycache__"))
    (repo / "records").mkdir()
    (repo / "records" / "ipropi.md").write_text("# fake\n" + MARKER)
    (repo / "records" / "waiver.md").write_text("# fake\n" + WAIVER_MARKER)
    header = root / "vehicle_config.h"
    gen = subprocess.run([sys.executable, str(REPO / "tools" / "real_vehicle_header.py"), "--config",
                          str(REPO / "config" / "real" / "vehicle.yaml"), "--out", str(header)],
                         env={**os.environ, "PYTHONPATH": str(REPO / "src")}, capture_output=True, text=True)
    assert gen.returncode == 0, gen.stderr
    _git(repo, "init", "-q")
    _git(repo, "add", "-A")
    _git(repo, "commit", "-q", "-m", "a")
    return repo, header


def _git(repo: Path, *args: str) -> str:
    return subprocess.run(["git", "-c", "user.email=t@example.invalid", "-c", "user.name=t", *args],
                          cwd=repo, capture_output=True, text=True, check=True).stdout


def _operating(extra) -> bool:
    return "-DFW_MOTOR_OUTPUT=1" in extra and "-DFW_BUILD=normal" in extra


def _configure_in(repo: Path, header: Path, build: Path, *extra: str, header_given: bool | None = None
                  ) -> subprocess.CompletedProcess:
    """The open normal (operating) build generates its header from the copy's vehicle.yaml (FW_PYTHON); every other
    build takes the pre-generated one. ``header_given`` forces either (the refusal test)."""
    given = not _operating(extra) if header_given is None else header_given
    vehicle = [f"-DFW_VEHICLE_HEADER={header}"] if given else [f"-DFW_PYTHON={sys.executable}"]
    return subprocess.run(
        ["cmake", "-S", str(repo / "firmware"), "-B", str(build), "-DFW_BOARD=weact_h743",
         f"-DARM_GNU_TOOLCHAIN_DIR={TOOLCHAIN}", *vehicle,
         f"-DCMAKE_TOOLCHAIN_FILE={repo / 'firmware/boards/weact_h743/arm-gnu-13.3.cmake'}", *extra],
        capture_output=True, text=True)


OPEN = ("-DFW_MOTOR_OUTPUT=1", "-DFW_BUILD=commission")


@pytest.fixture(scope="module")
def open_build(record_repo, tmp_path_factory):
    repo, header = record_repo
    build = tmp_path_factory.mktemp("open_build")
    cfg = _configure_in(repo, header, build, *OPEN, "-DFW_IPROPI_PROTECTION_RECORD=records/ipropi.md",
                        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON")
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    res = subprocess.run(["cmake", "--build", str(build), "-j2"], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "warning" not in (res.stdout + res.stderr).lower(), res.stdout + res.stderr
    return build


def test_output_build_carries_the_record_and_tim1(open_build, record_repo):
    repo, _ = record_repo
    d = open_build / "boards" / "weact_h743"
    blob = subprocess.run(["git", "show", "HEAD:records/ipropi.md"], cwd=repo, capture_output=True,
                          check=True).stdout
    sha8 = hashlib.sha256(blob).hexdigest()[:8]
    data = (d / "fw.elf").read_bytes()
    # design 2026-11-25-bench-commission-limits 1.1: the open commissioning image ends its identity with lim=lr1
    assert re.search(rb"forklift-mcu commission OPEN \S+ rec=" + sha8.encode() + rb" lim=lr1 link=usb", data)
    map_text = (d / "fw.map").read_text()
    assert "motor_out_tim1" in map_text and "motor_out_locked" not in map_text
    syms = _symbols(d / "fw.elf")
    assert {"motor_out_apply", "motor_out_force_safe", "iwdg_unfreeze_in_debug"} <= set(syms)


def test_only_the_output_file_escapes_the_tim1_guard(open_build):
    """Motor output design 2: in the output build every board C source but motor_out_tim1.c is
    still compiled with TIM1 poisoned."""
    cmds = json.loads((open_build / "compile_commands.json").read_text())
    board = [c for c in cmds if "/boards/weact_h743/src/" in c["file"] and c["file"].endswith(".c")]
    assert len(board) > 10
    for c in board:
        guarded = "lock_guard.h" in c["command"]
        assert guarded == (not c["file"].endswith("motor_out_tim1.c")), c["file"]


def test_output_build_stack(open_build):
    d = open_build / "boards" / "weact_h743"
    args = [sys.executable, str(REPO / "tools" / "fw_stack_check.py"), str(open_build),
            "--limit", str(MAIN_STACK // 2), "--elf", str(d / "fw.elf"),
            "--objdump", str(TOOLCHAIN / "bin" / "arm-none-eabi-objdump")]
    for lv in ["board_main", "OTG_FS_IRQHandler", "TIM6_DAC_IRQHandler", "SPI1_IRQHandler,SPI2_IRQHandler",
               "Default_Handler", "HardFault_Handler", "NMI_Handler"]:
        args += ["--level", lv]
    for rule in ["TIM6_DAC_IRQHandler=on_tick", "fw_app_*=hal_*", *TUSB_RULES]:
        args += ["--indirect", rule]
    res = subprocess.run(args, capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr


def test_output_build_rechecks_the_record_at_every_build(record_repo, tmp_path):
    """Codex design review: configure-time evidence is not enough; a record changed after the
    configure stops the next build."""
    repo, header = record_repo
    build = tmp_path / "b"
    cfg = _configure_in(repo, header, build, *OPEN, "-DFW_IPROPI_PROTECTION_RECORD=records/ipropi.md")
    assert cfg.returncode == 0, cfg.stderr
    rec = repo / "records" / "ipropi.md"
    original = rec.read_text()
    try:
        rec.write_text(original + "edited\n")
        res = subprocess.run(["cmake", "--build", str(build), "--target", "protection_record"],
                             capture_output=True, text=True)
        assert res.returncode != 0
        assert "differs from its committed version" in _flat(res.stdout + res.stderr)
    finally:
        rec.write_text(original)


@pytest.mark.parametrize("case, message", [
    ("untracked", "is not committed"),
    ("staged_only", "is not committed"),
    ("index_differs", "differs from its committed version"),
    ("tree_differs", "differs from its committed version"),
    ("assume_unchanged", "differs from its committed version"),
    ("skip_worktree", "differs from its committed version"),
    ("symlink", "symbolic link"),
    ("no_marker", "no marker line"),
    ("outside", "outside the repository"),
])
def test_output_build_refuses_a_bad_record(record_repo, tmp_path, case, message):
    """Motor output design 2: a regular file in the repository, its HEAD blob present, index and
    work tree equal to HEAD, the marker line in the HEAD blob (Codex design reviews 1 and 2)."""
    repo, header = record_repo
    path = "records/ipropi.md"
    restore = []
    if case == "untracked":
        path = "records/new.md"
        (repo / path).write_text(MARKER)
        restore.append(lambda: (repo / "records" / "new.md").unlink())
    elif case == "staged_only":
        path = "records/staged.md"
        (repo / path).write_text(MARKER)
        _git(repo, "add", path)
        restore.append(lambda: (_git(repo, "rm", "-q", "--cached", "records/staged.md"),
                                (repo / "records" / "staged.md").unlink()))
    elif case == "index_differs":  # HEAD = A, index = B, work tree = A (Codex review 2)
        rec = repo / path
        a = rec.read_text()
        rec.write_text(a + "b\n")
        _git(repo, "add", path)
        rec.write_text(a)
        restore.append(lambda: _git(repo, "reset", "-q", "--", "records/ipropi.md"))
    elif case == "tree_differs":
        rec = repo / path
        a = rec.read_text()
        rec.write_text(a + "b\n")
        restore.append(lambda: rec.write_text(a))
    elif case in ("assume_unchanged", "skip_worktree"):  # git diff skips the file (Codex code review)
        flag = "--" + case.replace("_", "-")
        rec = repo / path
        a = rec.read_text()
        _git(repo, "update-index", flag, path)
        rec.write_text(a.replace("confirmed-by=test fixture", "confirmed-by=someone else"))
        restore.append(lambda: (rec.write_text(a), _git(repo, "update-index", "--no-" + flag[2:], path)))
    elif case == "symlink":
        path = "records/link.md"
        (repo / path).symlink_to(repo / "records" / "ipropi.md")
        _git(repo, "add", path)
        _git(repo, "commit", "-q", "-m", "link")
        restore.append(lambda: (_git(repo, "rm", "-q", "records/link.md"),
                                _git(repo, "commit", "-q", "-m", "unlink")))
    elif case == "no_marker":
        path = "records/plain.md"
        (repo / path).write_text("# no marker\n")
        _git(repo, "add", path)
        _git(repo, "commit", "-q", "-m", "plain")
        restore.append(lambda: (_git(repo, "rm", "-q", "records/plain.md"),
                                _git(repo, "commit", "-q", "-m", "rm plain")))
    elif case == "outside":
        path = str(tmp_path / "elsewhere.md")
        Path(path).write_text(MARKER)
    try:
        cfg = _configure_in(repo, header, tmp_path / "b", *OPEN, f"-DFW_IPROPI_PROTECTION_RECORD={path}")
        assert cfg.returncode != 0
        assert message in _flat(cfg.stderr), cfg.stderr
        assert "user's decision" in _flat(cfg.stderr)
    finally:
        for r in restore:
            r()


@pytest.mark.parametrize("extra", [("-DFW_BUILD=normal",), ("-DFW_BUILD=hil",),
                                   ("-DFW_BUILD=commission", "-DFW_TEST_IWDG_STALL=5")])
def test_only_the_commissioning_build_may_open_the_outputs(record_repo, tmp_path, extra):
    """Motor output design 2, lead condition 2: normal needs the e-stop waiver record too (plan
    2026-11-18), hil and the IWDG stall build stay locked -- even with a good record."""
    repo, header = record_repo
    cfg = _configure_in(repo, header, tmp_path / "b", "-DFW_MOTOR_OUTPUT=1",
                        "-DFW_IPROPI_PROTECTION_RECORD=records/ipropi.md", *extra)
    assert cfg.returncode != 0


WAIVED = ("-DFW_MOTOR_OUTPUT=1", "-DFW_BUILD=normal", "-DFW_IPROPI_PROTECTION_RECORD=records/ipropi.md")


def _sha8(repo: Path, rel: str) -> str:
    blob = subprocess.run(["git", "show", f"HEAD:{rel}"], cwd=repo, capture_output=True, check=True).stdout
    return hashlib.sha256(blob).hexdigest()[:8]


def test_normal_build_opens_only_with_the_waiver_record(record_repo, tmp_path):
    """Plan 2026-11-18 (V1.0 e-stop waiver): the open normal build is built only with both
    committed records, and its identity carries both hashes, waiver= last (the console's host check
    reads the product string's tokens)."""
    repo, header = record_repo
    build = tmp_path / "b"
    cfg = _configure_in(repo, header, build, *WAIVED, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    res = subprocess.run(["cmake", "--build", str(build), "-j2"], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "warning" not in (res.stdout + res.stderr).lower(), res.stdout + res.stderr
    d = build / "boards" / "weact_h743"
    data = (d / "fw.elf").read_bytes()
    want = (rb"forklift-mcu normal OPEN \S+ rec=" + _sha8(repo, "records/ipropi.md").encode() +
            rb" waiver=" + _sha8(repo, "records/waiver.md").encode() + rb" link=usb")
    assert re.search(want, data), "identity without both hashes"
    map_text = (d / "fw.map").read_text()
    assert "motor_out_tim1" in map_text and "motor_out_locked" not in map_text
    # Re-checked at every build like the IPROPI record (Codex design review of the IPROPI gate).
    rec = repo / "records" / "waiver.md"
    original = rec.read_text()
    try:
        rec.write_text(original + "edited\n")
        # the image target itself, so a missing add_dependencies(fw waiver_record) fails here (Codex)
        res = subprocess.run(["cmake", "--build", str(build), "--target", "fw"],
                             capture_output=True, text=True)
        assert res.returncode != 0
        assert "differs from its committed version" in _flat(res.stdout + res.stderr)
    finally:
        rec.write_text(original)
    # A git description long enough to cut waiver= out of the 126-character USB string is refused.
    _git(repo, "tag", "-a", "-m", "long", "t" + "x" * 80)  # git describe reads annotated tags
    try:
        res = subprocess.run(["cmake", "--build", str(build), "--target", "build_info"],
                             capture_output=True, text=True)
        assert res.returncode != 0
        assert "would cut the waiver= token" in _flat(res.stdout + res.stderr)
    finally:
        _git(repo, "tag", "-d", "t" + "x" * 80)


def test_the_operating_build_refuses_a_given_vehicle_header(record_repo, tmp_path):
    """Operating plan 2026-11-26 2.4 (C12, U3d): the open normal build's header comes only from the commit's
    vehicle.yaml -- a header handed in is refused, before any other judgement."""
    repo, header = record_repo
    cfg = _configure_in(repo, header, tmp_path / "b", *WAIVED, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md",
                        header_given=True)
    assert cfg.returncode != 0
    assert "FW_VEHICLE_HEADER is not for the open normal (operating) build" in _flat(cfg.stderr), cfg.stderr


def test_the_operating_build_generates_its_header_from_the_commits_vehicle_yaml(record_repo, tmp_path):
    """The header the operating build compiled is the generator's output for the copy's vehicle.yaml: the same
    calibration id as the host's config (``RealVehicleConfig.calibration_id``)."""
    from forklift_core.plant.real_vehicle import load_real_vehicle

    repo, header = record_repo
    build = tmp_path / "b"
    cfg = _configure_in(repo, header, build, *WAIVED, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    res = subprocess.run(["cmake", "--build", str(build), "--target", "fw_vehicle_header"],
                         capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    made = (build / "boards" / "weact_h743" / "generated" / "vehicle_config.h").read_text()
    calib = load_real_vehicle(repo / "config" / "real" / "vehicle.yaml").calibration_id
    assert f'"{calib}"' in made and made == header.read_text()  # the same generator, the same yaml


def test_a_header_left_in_the_build_directory_is_not_taken_by_the_operating_build(record_repo, tmp_path):
    """Codex U3d P2-1: a build directory configured first with a handed-in header (another calibration) and then
    reconfigured as the operating build regenerates its header from the commit's vehicle.yaml at the build."""
    repo, header = record_repo
    build = tmp_path / "b"
    other = tmp_path / "other_vehicle_config.h"
    text = header.read_text()
    calib = re.search(r'FW_VEHICLE_CALIBRATION_ID "([0-9a-f]+)"', text).group(1)
    other.write_text(text.replace(calib, "f" * len(calib)))
    cfg = _configure_in(repo, other, build, *OPEN, "-DFW_IPROPI_PROTECTION_RECORD=records/ipropi.md")
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    made = build / "boards" / "weact_h743" / "generated" / "vehicle_config.h"
    assert "f" * len(calib) in made.read_text()  # the copy sits in the build directory
    cfg = _configure_in(repo, header, build, "-UFW_VEHICLE_HEADER", "-UFW_BUILD", *WAIVED,
                        "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    made.write_text(other.read_text())  # left behind after the reconfiguration too: only the build can replace it
    res = subprocess.run(["cmake", "--build", str(build), "-j2"], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    assert made.read_text() == text  # the commit's values, not the copy left behind
    elf = (build / "boards" / "weact_h743" / "fw.elf").read_bytes()
    assert b"forklift-mcu normal OPEN" in elf


def test_the_operating_build_refuses_an_uncommitted_vehicle_config(record_repo, tmp_path):
    """Codex U3d P2-2: the operating build's vehicle.yaml (and the chassis model it names) must be the HEAD blob --
    at configure time and at every build."""
    repo, header = record_repo
    yaml_path = repo / "config" / "real" / "vehicle.yaml"
    original = yaml_path.read_text()
    try:
        yaml_path.write_text(original + "# edited\n")
        cfg = _configure_in(repo, header, tmp_path / "b", *WAIVED, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
        assert cfg.returncode != 0
        assert "differs from its committed version" in _flat(cfg.stderr), cfg.stderr
        # a locked or commissioning build may read the working copy (a bench edit), as before -- at its build
        yaml_path.write_text(original.replace("rad_s: 8.0", "rad_s: 7.5"))
        cfg = _configure_in(repo, header, tmp_path / "c", *OPEN, "-DFW_IPROPI_PROTECTION_RECORD=records/ipropi.md",
                            header_given=False)
        assert cfg.returncode == 0, cfg.stderr
        res = subprocess.run(["cmake", "--build", str(tmp_path / "c"), "--target", "fw_vehicle_header"],
                             capture_output=True, text=True)
        assert res.returncode == 0, res.stdout + res.stderr
        made = (tmp_path / "c" / "boards" / "weact_h743" / "generated" / "vehicle_config.h").read_text()
        assert "7.5" in made and made != header.read_text()
    finally:
        yaml_path.write_text(original)
    build = tmp_path / "d"
    cfg = _configure_in(repo, header, build, *WAIVED, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    # the chassis model is not a configure dependency: only the build's own check can catch its edit
    chassis = repo / "sim" / "models" / "dls08_measured" / "parameters.yaml"
    chassis_text = chassis.read_text()
    try:
        chassis.write_text(chassis_text + "# edited after configure\n")
        res = subprocess.run(["cmake", "--build", str(build), "--target", "fw_vehicle_header"],
                             capture_output=True, text=True)
        assert res.returncode != 0
        assert "differs from its committed version" in _flat(res.stdout + res.stderr)
    finally:
        chassis.write_text(chassis_text)


def _copy_repo(repo: Path, dest: Path) -> Path:
    shutil.copytree(repo, dest, symlinks=True, ignore=shutil.ignore_patterns("build"))
    return dest


def test_the_operating_build_refuses_a_chassis_model_through_a_symbolic_link(record_repo, tmp_path):
    """Codex U3d round 2 P2-1: the chassis path the config writes is the one checked -- a committed symbolic link
    to another committed model is refused, not followed."""
    repo, header = record_repo
    r2 = _copy_repo(repo, tmp_path / "r2")
    (r2 / "sim" / "models" / "alias").symlink_to("dls08_measured")
    cfg_path = r2 / "config" / "real" / "vehicle.yaml"
    cfg_path.write_text(cfg_path.read_text().replace("sim/models/dls08_measured/", "sim/models/alias/"))
    _git(r2, "add", "-A")
    _git(r2, "commit", "-q", "-m", "alias")
    cfg = _configure_in(r2, header, tmp_path / "b", *WAIVED, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
    assert cfg.returncode != 0
    assert "goes through a symbolic link" in _flat(cfg.stderr), cfg.stderr


def test_a_symbolic_link_folded_away_by_dot_dot_is_still_refused(record_repo, tmp_path):
    """Codex U3d round 3 P2-1: "alias/../dls08_measured" normalises to a link-free path; the components as written
    are checked first."""
    repo, header = record_repo
    r2 = _copy_repo(repo, tmp_path / "r2")
    (r2 / "sim" / "models" / "alias").symlink_to("dls08_measured")
    cfg_path = r2 / "config" / "real" / "vehicle.yaml"
    cfg_path.write_text(cfg_path.read_text().replace("sim/models/dls08_measured/", "sim/models/alias/../dls08_measured/"))
    _git(r2, "add", "-A")
    _git(r2, "commit", "-q", "-m", "alias dot-dot")
    cfg = _configure_in(r2, header, tmp_path / "b", *WAIVED, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
    assert cfg.returncode != 0
    assert "goes through a symbolic link" in _flat(cfg.stderr), cfg.stderr


def test_the_header_check_and_the_identity_name_one_commit(record_repo, tmp_path):
    """Codex U3d round 3 P2-2: HEAD moving after the identity was made: the header's check refuses."""
    repo, header = record_repo
    r2 = _copy_repo(repo, tmp_path / "r2")
    build = tmp_path / "b"
    cfg = _configure_in(r2, header, build, *WAIVED, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    res = subprocess.run(["cmake", "--build", str(build), "--target", "build_info"], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    info = build / "boards" / "weact_h743" / "generated" / "build_info.h"
    (r2 / "moved.txt").write_text("b\n")
    _git(r2, "add", "moved.txt")
    _git(r2, "commit", "-q", "-m", "b")  # HEAD moves between the two targets
    gen = subprocess.run(
        ["cmake", f"-DREPO={r2}", f"-DPYTHON={sys.executable}", f"-DYAML={r2 / 'config/real/vehicle.yaml'}",
         f"-DOUT={tmp_path / 'v.h'}", "-DCOMMITTED=1", f"-DBUILD_INFO={info}",
         "-P", str(r2 / "firmware" / "cmake" / "gen_vehicle_header.cmake")], capture_output=True, text=True)
    assert gen.returncode != 0
    assert "is not the commit the build identity names" in _flat(gen.stderr), gen.stderr
    # the whole build makes the identity again first, so it passes on the new HEAD
    res = subprocess.run(["cmake", "--build", str(build), "--target", "fw_vehicle_header"],
                         capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr


def test_a_stray_build_info_beside_the_board_source_is_not_compiled(record_repo, tmp_path):
    """Codex U3d round 4 P2-1: the identity compiled is the generated build_info.h (the one the header check read)."""
    repo, header = record_repo
    r2 = _copy_repo(repo, tmp_path / "r2")
    (r2 / "firmware" / "boards" / "weact_h743" / "src" / "build_info.h").write_text(
        '#error "a stray build_info.h was compiled"\n')  # untracked: git describe stays clean
    build = tmp_path / "b"
    cfg = _configure_in(r2, header, build, *WAIVED, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    res = subprocess.run(["cmake", "--build", str(build), "-j2"], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    head = _git(r2, "rev-parse", "--short=12", "HEAD").strip()
    assert f"forklift-mcu normal OPEN {head} ".encode() in (build / "boards" / "weact_h743" / "fw.elf").read_bytes()


def test_the_operating_header_is_generated_from_the_committed_blobs(record_repo, tmp_path):
    """Codex U3d round 4 P2-2: the generator reads a staged copy of the HEAD blobs, never the work tree."""
    repo, header = record_repo
    log = tmp_path / "argv.log"
    wrapper = tmp_path / "py.sh"
    wrapper.write_text(f'#!/bin/sh\necho "$@" >> {log}\nexec {sys.executable} "$@"\n')
    wrapper.chmod(0o755)
    out = tmp_path / "v.h"
    gen = subprocess.run(
        ["cmake", f"-DREPO={repo}", f"-DPYTHON={wrapper}", f"-DYAML={repo / 'config/real/vehicle.yaml'}",
         f"-DOUT={out}", "-DCOMMITTED=1", "-P", str(repo / "firmware" / "cmake" / "gen_vehicle_header.cmake")],
        capture_output=True, text=True)
    assert gen.returncode == 0, gen.stderr
    generating = [ln for ln in log.read_text().splitlines() if "--out" in ln]
    assert len(generating) == 1 and f"{out}.stage/config/real/vehicle.yaml" in generating[0], generating
    assert out.read_text() == header.read_text() and not Path(f"{out}.stage").exists()


def test_a_stray_vehicle_header_beside_the_board_source_is_not_compiled(record_repo, tmp_path):
    """Codex U3d round 2 P2-2: the board source includes the generated header by its absolute path."""
    repo, header = record_repo
    r2 = _copy_repo(repo, tmp_path / "r2")
    (r2 / "firmware" / "boards" / "weact_h743" / "src" / "vehicle_config.h").write_text(
        '#error "a stray vehicle_config.h was compiled"\n')  # untracked: git describe stays clean
    build = tmp_path / "b"
    cfg = _configure_in(r2, header, build, *WAIVED, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    res = subprocess.run(["cmake", "--build", str(build), "-j2"], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr


def test_the_commissioning_open_build_carries_no_waiver(open_build):
    data = (open_build / "boards" / "weact_h743" / "fw.elf").read_bytes()
    assert re.search(rb"forklift-mcu commission OPEN \S+ rec=[0-9a-f]{8} lim=lr1 link=usb", data)
    assert b"waiver=" not in data


@pytest.mark.parametrize("extra, message", [
    ((), "E-stop waiver record"),                                       # normal, outputs, no waiver
    (("-DFW_ESTOP_WAIVER_RECORD=records/ipropi.md",), "no marker line"),  # the other record's marker
    (("-DFW_ESTOP_WAIVER_RECORD=records/missing.md",), "is not a file"),
    (("-DFW_ESTOP_WAIVER_RECORD=records/waiver.md", "-DFW_IPROPI_PROTECTION_RECORD="), "IPROPI"),
])
def test_normal_build_refuses_without_a_good_waiver(record_repo, tmp_path, extra, message):
    repo, header = record_repo
    cfg = _configure_in(repo, header, tmp_path / "b", *WAIVED, *extra)
    assert cfg.returncode != 0
    assert message in _flat(cfg.stderr), cfg.stderr
    assert "user's decision" in _flat(cfg.stderr)


@pytest.mark.parametrize("extra", [
    ("-DFW_BUILD=commission", "-DFW_MOTOR_OUTPUT=1", "-DFW_IPROPI_PROTECTION_RECORD=records/ipropi.md"),
    ("-DFW_BUILD=normal",),                                           # locked normal
    ("-DFW_BUILD=hil",),
    ("-DFW_BUILD=normal", "-DFW_TEST_IWDG_STALL=5"),
])
def test_the_waiver_is_only_for_the_open_normal_build(record_repo, tmp_path, extra):
    """A waiver record given anywhere else is refused, not ignored (plan 2026-11-18)."""
    repo, header = record_repo
    cfg = _configure_in(repo, header, tmp_path / "b", *extra, "-DFW_ESTOP_WAIVER_RECORD=records/waiver.md")
    assert cfg.returncode != 0
    assert "FW_ESTOP_WAIVER_RECORD" in _flat(cfg.stderr), cfg.stderr


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


SINGLE = (*OPEN, "-DFW_SINGLE_AXIS=1")


@pytest.fixture(scope="module")
def single_build(record_repo, tmp_path_factory):
    """fw-single (partial-parts plan 2026-11-25 PR-B): the open commissioning build, one axis per arming."""
    repo, header = record_repo
    build = tmp_path_factory.mktemp("single_build")
    cfg = _configure_in(repo, header, build, *SINGLE, "-DFW_IPROPI_PROTECTION_RECORD=records/ipropi.md")
    assert cfg.returncode == 0, cfg.stdout + cfg.stderr
    res = subprocess.run(["cmake", "--build", str(build), "-j2"], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    assert "warning" not in (res.stdout + res.stderr).lower(), res.stdout + res.stderr
    return build


def test_single_axis_build_identity_size_and_stack(single_build, open_build):
    """The tool tells fw-single apart only by the exact suffix " lim=lr1 sa1"; the open commissioning
    image never carries sa1; the stack stays within half the main stack."""
    d = single_build / "boards" / "weact_h743"
    data = (d / "fw.elf").read_bytes()
    assert re.search(rb"forklift-mcu commission OPEN \S+ rec=[0-9a-f]{8} lim=lr1 sa1 link=usb", data)
    assert b" sa1" not in (open_build / "boards" / "weact_h743" / "fw.elf").read_bytes()
    sizes = _sections(d / "fw.elf")
    assert sizes[".text"] + sizes.get(".data", 0) < FLASH_LIMIT
    args = [sys.executable, str(REPO / "tools" / "fw_stack_check.py"), str(single_build),
            "--limit", str(MAIN_STACK // 2), "--elf", str(d / "fw.elf"),
            "--objdump", str(TOOLCHAIN / "bin" / "arm-none-eabi-objdump")]
    for lv in ["board_main", "OTG_FS_IRQHandler", "TIM6_DAC_IRQHandler", "SPI1_IRQHandler,SPI2_IRQHandler",
               "Default_Handler", "HardFault_Handler", "NMI_Handler"]:
        args += ["--level", lv]
    for rule in ["TIM6_DAC_IRQHandler=on_tick", "fw_app_*=hal_*", *TUSB_RULES]:
        args += ["--indirect", rule]
    res = subprocess.run(args, capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr


@pytest.mark.parametrize("extra", [("-DFW_BUILD=commission",), ("-DFW_BUILD=normal",), ("-DFW_BUILD=hil",)])
def test_single_axis_only_on_the_open_commissioning_build(record_repo, tmp_path, extra):
    repo, header = record_repo
    cfg = _configure_in(repo, header, tmp_path / "b", "-DFW_SINGLE_AXIS=1", *extra)
    assert cfg.returncode != 0
    assert "FW_SINGLE_AXIS=1 is only for" in _flat(cfg.stderr), cfg.stderr


def test_other_images_carry_no_single_axis_core(build_of, open_build):
    """The other images keep their core: compiled without FW_CORE_SINGLE_AXIS (their bytes match the
    commit before fw-single but for the git description; checked by hand for PR-B)."""
    for d in (build_of("usb"), build_of("commission"), open_build / "boards" / "weact_h743"):
        cmds = d.parent.parent / "compile_commands.json"
        if cmds.is_file():
            assert "FW_CORE_SINGLE_AXIS" not in cmds.read_text()
        assert b" sa1" not in (d / "fw.elf").read_bytes()


def test_single_axis_only_on_the_usb_link(record_repo, tmp_path):
    """Codex PR-B P2-1: UART7 has no product string to tell fw-single apart by."""
    repo, header = record_repo
    cfg = _configure_in(repo, header, tmp_path / "b", *SINGLE, "-DFW_LINK=uart7",
                        "-DFW_IPROPI_PROTECTION_RECORD=records/ipropi.md")
    assert cfg.returncode != 0
    assert "only for FW_LINK=usb" in _flat(cfg.stderr), cfg.stderr
