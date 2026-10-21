"""mcu-link/1 messages: encode/decode and a stream decoder that stamps host time.

Contract: ``docs/interfaces/2026-10-22-mcu-link-contract.md`` sections 2-4. Units are
SI except where the contract fixes raw fields (``current_ma``, ``duty_e4``); the
``current_a`` and ``duty`` properties give the SI views.
"""

from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag

from .framing import BODY_MAX, FrameDecoder, cobs_encode, crc16_ccitt_false

VERSION = 1
U32_MAX = 2**32 - 1

_HEADER = struct.Struct("<BBH")
_DRIVE = struct.Struct("<ffffHBHII")
_ARM = struct.Struct("<II")
_CLEAR = struct.Struct("<I")
_STATE = struct.Struct("<IIHHBBIiiffff4h4h4BHHB")

TYPE_DRIVE = 0x01
TYPE_ARM = 0x02
TYPE_DISARM = 0x03
TYPE_CLEAR_FAULTS = 0x04
TYPE_STATE = 0x81

_LIFT_HOLD = 0x01


class LinkError(ValueError):
    """A frame body that is not a valid mcu-link/1 message."""


class Mode(IntEnum):
    BOOT = 0
    DISARMED = 1
    ARMED = 2
    STOPPING = 3
    FAULT = 4
    ESTOP = 5


class RejectReason(IntEnum):
    NONE = 0
    BAD_VALUE = 1
    NOT_ARMED = 2
    LATCHED = 3
    ESTOP_ACTIVE = 4
    DRIVER_NOT_READY = 5
    STALE_SEQ = 6
    ZERO_FIRST = 7
    BOOT_MISMATCH = 8
    ARM_GEN_MISMATCH = 9
    STALE_ECHO = 10
    STOPPING = 11
    NOT_DISARMED = 12
    SENSOR_INVALID = 13


class FaultFlag(IntFlag):
    ESTOP_INPUT = 1 << 0
    COMMAND_TIMEOUT = 1 << 1
    DRIVER_FAULT_L = 1 << 2
    DRIVER_FAULT_R = 1 << 3
    DRIVER_FAULT_M = 1 << 4
    DRIVER_FAULT_H = 1 << 5
    STEER_SENSOR_INVALID = 1 << 6
    LIFT_SENSOR_INVALID = 1 << 7
    OVERCURRENT_L = 1 << 8
    OVERCURRENT_R = 1 << 9
    OVERCURRENT_M = 1 << 10
    OVERCURRENT_H = 1 << 11
    DRIVER_NOT_READY = 1 << 12
    STEER_OUT_OF_RANGE = 1 << 13
    LIFT_OUT_OF_RANGE = 1 << 14
    CYCLE_OVERRUN = 1 << 15
    CURRENT_SENSE_INVALID = 1 << 16


class StateFlag(IntFlag):
    LIMITED = 1 << 0
    COMMAND_FRESH = 1 << 1
    ZERO_SEEN = 1 << 2
    DRIVERS_READY = 1 << 3


_FAULT_MASK = (1 << 17) - 1
_STATE_FLAG_MASK = (1 << 4) - 1


def _check_range(name: str, value: int, low: int, high: int) -> None:
    if not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in {low}..{high}")


def _check_finite(**values: float) -> None:
    for name, value in values.items():
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")


@dataclass(frozen=True)
class Drive:
    """Wheel rates (rad/s, vehicle-forward rolling +), steering (rad), lift (m)."""

    left_wheel_rad_s: float
    right_wheel_rad_s: float
    steering_rad: float
    lift_m: float
    valid_after_echo_ms: int
    lift_hold: bool
    arm_gen: int
    boot_id: int
    echo_mcu_time_us: int

    def __post_init__(self) -> None:
        _check_finite(
            left_wheel_rad_s=self.left_wheel_rad_s,
            right_wheel_rad_s=self.right_wheel_rad_s,
            steering_rad=self.steering_rad,
            lift_m=self.lift_m,
        )
        _check_range("valid_after_echo_ms", self.valid_after_echo_ms, 1, 0xFFFF)
        _check_range("arm_gen", self.arm_gen, 1, 0xFFFF)
        _check_range("boot_id", self.boot_id, 1, U32_MAX)
        _check_range("echo_mcu_time_us", self.echo_mcu_time_us, 0, U32_MAX)


@dataclass(frozen=True)
class Arm:
    boot_id: int
    echo_mcu_time_us: int

    def __post_init__(self) -> None:
        _check_range("boot_id", self.boot_id, 1, U32_MAX)
        _check_range("echo_mcu_time_us", self.echo_mcu_time_us, 0, U32_MAX)


@dataclass(frozen=True)
class Disarm:
    pass


@dataclass(frozen=True)
class ClearFaults:
    boot_id: int

    def __post_init__(self) -> None:
        _check_range("boot_id", self.boot_id, 1, U32_MAX)


@dataclass(frozen=True)
class McuState:
    """One STATE report. ``mcu_time_us`` is the MCU clock: never mix it with host time.

    ``wheel_count_*`` are 32-bit modular; difference them with
    :func:`wheel_count_delta`. Invalid positions are NaN.
    """

    mcu_time_us: int
    boot_id: int
    arm_gen: int
    last_rx_seq: int
    mode: Mode
    reject_reason: RejectReason
    fault_flags: FaultFlag
    wheel_count_left: int
    wheel_count_right: int
    wheel_rate_left_rad_s: float
    wheel_rate_right_rad_s: float
    steering_rad: float
    lift_m: float
    current_ma: tuple[int, int, int, int]
    duty_e4: tuple[int, int, int, int]
    driver_status: tuple[int, int, int, int]
    rx_errors: int
    cycle_overruns: int
    flags: StateFlag

    @property
    def current_a(self) -> tuple[float, ...]:
        """Motor currents L, R, M, H in amperes."""
        return tuple(value / 1000 for value in self.current_ma)

    @property
    def duty(self) -> tuple[float, ...]:
        """Applied duties L, R, M, H in -1..1."""
        return tuple(value / 10000 for value in self.duty_e4)


Message = Drive | Arm | Disarm | ClearFaults | McuState


def _payload(message: Message) -> tuple[int, bytes]:
    if isinstance(message, Drive):
        return TYPE_DRIVE, _DRIVE.pack(
            message.left_wheel_rad_s,
            message.right_wheel_rad_s,
            message.steering_rad,
            message.lift_m,
            message.valid_after_echo_ms,
            _LIFT_HOLD if message.lift_hold else 0,
            message.arm_gen,
            message.boot_id,
            message.echo_mcu_time_us,
        )
    if isinstance(message, Arm):
        return TYPE_ARM, _ARM.pack(message.boot_id, message.echo_mcu_time_us)
    if isinstance(message, Disarm):
        return TYPE_DISARM, b""
    if isinstance(message, ClearFaults):
        return TYPE_CLEAR_FAULTS, _CLEAR.pack(message.boot_id)
    if isinstance(message, McuState):
        return TYPE_STATE, _STATE.pack(
            message.mcu_time_us,
            message.boot_id,
            message.arm_gen,
            message.last_rx_seq,
            int(message.mode),
            int(message.reject_reason),
            int(message.fault_flags),
            message.wheel_count_left,
            message.wheel_count_right,
            message.wheel_rate_left_rad_s,
            message.wheel_rate_right_rad_s,
            message.steering_rad,
            message.lift_m,
            *message.current_ma,
            *message.duty_e4,
            *message.driver_status,
            message.rx_errors,
            message.cycle_overruns,
            int(message.flags),
        )
    raise TypeError(f"not an mcu-link message: {message!r}")


def encode(message: Message, seq: int) -> bytes:
    """Wire bytes of ``message`` (COBS-encoded, with the 0x00 delimiter)."""
    _check_range("seq", seq, 0, 0xFFFF)
    type_, payload = _payload(message)
    body = _HEADER.pack(VERSION, type_, seq) + payload
    body += crc16_ccitt_false(body).to_bytes(2, "little")
    if len(body) > BODY_MAX:
        raise ValueError("message exceeds the frame size")
    return cobs_encode(body) + b"\x00"


def _state(fields: tuple) -> McuState:
    mode, reject, faults, flags = fields[4], fields[5], fields[6], fields[27]
    if mode > max(Mode) or reject > max(RejectReason):
        raise LinkError("mode or reject reason out of range")
    if faults & ~_FAULT_MASK or flags & ~_STATE_FLAG_MASK:
        raise LinkError("undefined fault or state flag bit set")
    return McuState(
        mcu_time_us=fields[0],
        boot_id=fields[1],
        arm_gen=fields[2],
        last_rx_seq=fields[3],
        mode=Mode(mode),
        reject_reason=RejectReason(reject),
        fault_flags=FaultFlag(faults),
        wheel_count_left=fields[7],
        wheel_count_right=fields[8],
        wheel_rate_left_rad_s=fields[9],
        wheel_rate_right_rad_s=fields[10],
        steering_rad=fields[11],
        lift_m=fields[12],
        current_ma=tuple(fields[13:17]),
        duty_e4=tuple(fields[17:21]),
        driver_status=tuple(fields[21:25]),
        rx_errors=fields[25],
        cycle_overruns=fields[26],
        flags=StateFlag(flags),
    )


def decode(body: bytes) -> tuple[int, Message]:
    """Decode a CRC-checked body without its CRC (as :class:`FrameDecoder` returns)."""
    if len(body) < _HEADER.size:
        raise LinkError("body too short")
    version, type_, seq = _HEADER.unpack_from(body)
    if version != VERSION:
        raise LinkError(f"unsupported version {version}")
    payload = body[_HEADER.size :]
    layouts = {
        TYPE_DRIVE: _DRIVE,
        TYPE_ARM: _ARM,
        TYPE_CLEAR_FAULTS: _CLEAR,
        TYPE_STATE: _STATE,
    }
    if type_ == TYPE_DISARM:
        if payload:
            raise LinkError("DISARM carries no payload")
        return seq, Disarm()
    layout = layouts.get(type_)
    if layout is None:
        raise LinkError(f"unknown message type 0x{type_:02x}")
    if len(payload) != layout.size:
        raise LinkError(f"type 0x{type_:02x} needs {layout.size} payload bytes")
    fields = layout.unpack(payload)
    try:
        if type_ == TYPE_DRIVE:
            if fields[5] & ~_LIFT_HOLD:
                raise LinkError("undefined DRIVE flag bit set")
            message: Message = Drive(
                *fields[:5], bool(fields[5] & _LIFT_HOLD), *fields[6:]
            )
        elif type_ == TYPE_ARM:
            message = Arm(*fields)
        elif type_ == TYPE_CLEAR_FAULTS:
            message = ClearFaults(*fields)
        else:
            message = _state(fields)
    except LinkError:
        raise
    except ValueError as error:
        raise LinkError(str(error)) from error
    return seq, message


@dataclass(frozen=True)
class Received:
    """A decoded message and the host ``time.monotonic_ns()`` its bytes were read at."""

    seq: int
    message: Message
    received_ns: int


class LinkDecoder:
    """Byte stream -> :class:`Received` messages; framing and message errors counted."""

    def __init__(self) -> None:
        self._frames = FrameDecoder()
        self._message_errors = 0

    @property
    def errors(self) -> int:
        return self._frames.errors + self._message_errors

    def feed(self, data: bytes, received_ns: int) -> list[Received]:
        received = []
        for body in self._frames.feed(data):
            try:
                seq, message = decode(body)
            except LinkError:
                self._message_errors += 1
                continue
            received.append(Received(seq, message, received_ns))
        return received


def valid_after_echo_ms(expires_ns: int, state_received_ns: int) -> int:
    """DRIVE validity counted from the echoed STATE (contract section 2).

    Both times are host ``time.monotonic_ns()``; ``state_received_ns`` is when the
    STATE whose ``mcu_time_us`` is echoed was read. Rounds down and clamps to the
    field; raises ``ValueError`` when less than 1 ms is left.
    """
    remaining_ms = (expires_ns - state_received_ns) // 1_000_000
    if remaining_ms < 1:
        raise ValueError("the command has less than 1 ms left")
    return min(remaining_ms, 0xFFFF)


def wheel_count_delta(newer: int, older: int) -> int:
    """Difference of two 32-bit modular wheel counts, in -2**31 .. 2**31 - 1."""
    return (newer - older + 2**31) % 2**32 - 2**31
