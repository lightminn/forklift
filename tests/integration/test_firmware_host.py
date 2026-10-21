"""Host build of the hardware-independent firmware core (firmware/) and its C tests.

Builds with CMake into a temporary directory; skipped (not passed) when CMake or a
C compiler is missing.
"""

import random
import shutil
import struct
import subprocess
from pathlib import Path

import pytest

from forklift_core.mcu_link.framing import cobs_decode
from forklift_core.mcu_link.messages import (
    Arm,
    ClearFaults,
    Disarm,
    Drive,
    FaultFlag,
    McuState,
    Mode,
    RejectReason,
    StateFlag,
    decode,
    encode,
)

FIRMWARE = Path(__file__).resolve().parents[2] / "firmware"


@pytest.fixture(scope="session")
def firmware_build(tmp_path_factory) -> Path:
    if shutil.which("cmake") is None:
        pytest.skip("cmake not installed")
    if shutil.which("cc") is None and shutil.which("gcc") is None:
        pytest.skip("no C compiler")
    build = tmp_path_factory.mktemp("firmware_build")
    result = subprocess.run(
        ["cmake", "-S", str(FIRMWARE), "-B", str(build), "-DCMAKE_BUILD_TYPE=Debug"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    result = subprocess.run(
        ["cmake", "--build", str(build), "-j2"], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return build


def test_c_core_unit_tests_pass(firmware_build):
    result = subprocess.run(
        ["ctest", "--output-on-failure"],
        cwd=firmware_build,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr


# --- C <-> Python agreement on mcu-link/1 (contract section 3) -----------------------

SEED = 20261022


def _f32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


def _random_messages(rng: random.Random) -> list[tuple[str, int, object, list[str]]]:
    """(kind, seq, message, link_tool encode arguments) with float32-exact values."""
    cases = []

    def rf(scale: float) -> float:
        return _f32(rng.uniform(-scale, scale))

    for _ in range(200):
        drive = Drive(
            rf(20),
            rf(20),
            rf(0.5),
            rf(1),
            rng.randint(1, 0xFFFF),
            rng.random() < 0.5,
            rng.randint(1, 0xFFFF),
            rng.randint(1, 2**32 - 1),
            rng.randint(0, 2**64 - 1),
        )
        args = [repr(drive.left_wheel_rad_s), repr(drive.right_wheel_rad_s)]
        args += [repr(drive.steering_rad), repr(drive.lift_m)]
        args += [str(drive.valid_after_echo_ms), str(int(drive.lift_hold))]
        args += [str(drive.arm_gen), str(drive.boot_id), str(drive.echo_mcu_time_us)]
        cases.append(("drive", rng.randint(0, 0xFFFF), drive, args))
    for _ in range(20):
        arm = Arm(rng.randint(1, 2**32 - 1), rng.randint(0, 2**64 - 1))
        args = [str(arm.boot_id), str(arm.echo_mcu_time_us)]
        cases.append(("arm", rng.randint(0, 0xFFFF), arm, args))
        clear = ClearFaults(rng.randint(1, 2**32 - 1))
        cases.append(("clear", rng.randint(0, 0xFFFF), clear, [str(clear.boot_id)]))
        cases.append(("disarm", rng.randint(0, 0xFFFF), Disarm(), []))
    for _ in range(50):
        state = McuState(
            rng.randint(0, 2**64 - 1),
            rng.randint(1, 2**32 - 1),
            rng.randint(0, 0xFFFF),
            rng.randint(0, 0xFFFF),
            Mode(rng.randint(0, 5)),
            RejectReason(rng.randint(0, 13)),
            FaultFlag(rng.randint(0, (1 << 17) - 1)),
            rng.randint(-(2**31), 2**31 - 1),
            rng.randint(-(2**31), 2**31 - 1),
            rf(20),
            rf(20),
            rf(0.5),
            rf(1),
            tuple(rng.randint(-32768, 32767) for _ in range(4)),
            tuple(rng.randint(-10000, 10000) for _ in range(4)),
            tuple(rng.randint(0, 255) for _ in range(4)),
            rng.randint(0, 0xFFFF),
            rng.randint(0, 0xFFFF),
            StateFlag(rng.randint(0, 15)),
        )
        values = [
            state.mcu_time_us,
            state.boot_id,
            state.arm_gen,
            state.last_rx_seq,
            int(state.mode),
            int(state.reject_reason),
            int(state.fault_flags),
            state.wheel_count_left,
            state.wheel_count_right,
        ]
        args = [str(v) for v in values]
        args += [
            repr(v)
            for v in (
                state.wheel_rate_left_rad_s,
                state.wheel_rate_right_rad_s,
                state.steering_rad,
                state.lift_m,
            )
        ]
        args += [str(v) for v in (*state.current_ma, *state.duty_e4)]
        args += [str(v) for v in state.driver_status]
        args += [str(state.rx_errors), str(state.cycle_overruns), str(int(state.flags))]
        cases.append(("state", rng.randint(0, 0xFFFF), state, args))
    return cases


def _parse_c_line(line: str):
    """link_tool decode output -> (kind, seq, Python message)."""
    parts = line.split()
    kind, seq, rest = parts[0], int(parts[1]), parts[2:]
    if kind == "drive":
        floats = [_f32(float(v)) for v in rest[:4]]
        ints = [int(v) for v in rest[4:]]
        message = Drive(*floats, ints[0], bool(ints[1]), *ints[2:])
    elif kind == "arm":
        message = Arm(int(rest[0]), int(rest[1]))
    elif kind == "clear":
        message = ClearFaults(int(rest[0]))
    elif kind == "disarm":
        message = Disarm()
    elif kind == "state":
        ints = [int(v) for v in rest[:9]]
        floats = [_f32(float(v)) for v in rest[9:13]]
        tail = [int(v) for v in rest[13:]]
        message = McuState(
            *ints[:4],
            Mode(ints[4]),
            RejectReason(ints[5]),
            FaultFlag(ints[6]),
            ints[7],
            ints[8],
            *floats,
            tuple(tail[0:4]),
            tuple(tail[4:8]),
            tuple(tail[8:12]),
            tail[12],
            tail[13],
            StateFlag(tail[14]),
        )
    else:
        raise AssertionError(line)
    return kind, seq, message


def _link_tool(build: Path) -> Path:
    tool = build / "link_tool"
    assert tool.exists(), "link_tool was not built"
    return tool


def test_python_frames_decode_in_c(firmware_build):
    cases = _random_messages(random.Random(SEED))
    wires = "\n".join(encode(message, seq).hex() for _, seq, message, _ in cases)
    result = subprocess.run(
        [str(_link_tool(firmware_build)), "decode"],
        input=wires + "\n",
        capture_output=True,
        text=True,
        check=True,
    )
    lines = result.stdout.splitlines()
    assert len(lines) == len(cases), f"seed {SEED}"
    for index, ((kind, seq, message, _), line) in enumerate(
        zip(cases, lines, strict=True)
    ):
        assert _parse_c_line(line) == (kind, seq, message), f"seed {SEED} case {index}"


def test_c_frames_decode_in_python_and_match_bytes(firmware_build):
    tool = str(_link_tool(firmware_build))
    for index, (kind, seq, message, args) in enumerate(
        _random_messages(random.Random(SEED))
    ):
        result = subprocess.run(
            [tool, "encode", kind, str(seq), *args],
            capture_output=True,
            text=True,
            check=True,
        )
        wire = bytes.fromhex(result.stdout.strip())
        assert wire == encode(message, seq), f"seed {SEED} case {index}"
        body = cobs_decode(wire[:-1])[:-2]
        assert decode(body) == (seq, message), f"seed {SEED} case {index}"
