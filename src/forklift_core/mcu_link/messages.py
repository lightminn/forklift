"""mcu-link/3 messages: encode/decode and a stream decoder that stamps host time.

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

VERSION = 3
U32_MAX = 2**32 - 1
U64_MAX = 2**64 - 1

_HEADER = struct.Struct("<BBH")
_DRIVE = struct.Struct("<ffffHBHIQ")
_ARM = struct.Struct("<IQ")
_CLEAR = struct.Struct("<I")
_STATE = struct.Struct("<QIHHBBIiiffff4h4h4BHHBB6H")  # v3: the KTH counters
_COMMISSION = struct.Struct("<BhHHIQ")
# Bench extension (contract section 8, wire generation bench5): the bench image only; the operating v3 messages above
# are unchanged. BENCH_SET: boot_id, echo, run_id, mode, axis, source, count_sign, bypass, duty_max_e4[4], time_ms[4],
# stop_counts. BENCH_ARM: boot_id, echo, run_id. BENCH_STATE: 48 bytes, the order of the contract's table.
_BENCH_SET = struct.Struct("<IQIBBBbI4H4Ii")
_BENCH_ARM = struct.Struct("<IQI")
_BENCH_STATE = struct.Struct("<HIIBBBBHiiiiBBHHHHBBBBBB")

TYPE_DRIVE = 0x01
TYPE_ARM = 0x02
TYPE_DISARM = 0x03
TYPE_CLEAR_FAULTS = 0x04
TYPE_COMMISSION = 0x05
TYPE_BENCH_SET = 0x06  # bench extension (contract 8): the bench image only, unknown to every other image
TYPE_BENCH_ARM = 0x08  # bench extension: ARM with the run id of the BENCH_SET it consumes
TYPE_STATE = 0x81
TYPE_BENCH_STATE = 0x82  # bench extension: sent by the bench image right after each STATE, as one pair

BENCH_TIME_MAX_MS = 0x7FFFFFFF // 1000  # the firmware's validator ceiling for a per-axis time limit

_LIFT_HOLD = 0x01


class LinkError(ValueError):
    """A frame body that is not a valid mcu-link/2 message."""


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
    WRONG_BUILD = 14  # v2: a COMMISSION to a normal build, a DRIVE to the commissioning build
    ESTOP_UNWIRED = 15  # v2: a normal build with no e-stop pin refuses ARM


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
    # v2: the rear wheels' encoder checks (gap shared: both encoders are read in the same cycle)
    ENCODER_GAP = 1 << 17
    ENCODER_STUCK_L = 1 << 18
    ENCODER_STUCK_R = 1 << 19
    ENCODER_POLARITY_L = 1 << 20
    ENCODER_POLARITY_R = 1 << 21
    ENCODER_JUMP_L = 1 << 22
    ENCODER_JUMP_R = 1 << 23
    COMMISSION_TIME_LIMIT = 1 << 24  # v2: a nonzero commissioning duty held past its limit (kept until ARM)


class StateFlag(IntFlag):
    LIMITED = 1 << 0
    COMMAND_FRESH = 1 << 1
    ZERO_SEEN = 1 << 2
    DRIVERS_READY = 1 << 3
    COMMISSION_BUILD = 1 << 4  # v2: FW_COMMISSION -- an executor never connects
    HIL_PLANT = 1 << 5  # v2: FW_HIL_PLANT (outputs locked, a model fills the inputs)
    OUTPUTS_LOCKED = 1 << 6  # v2: motor outputs refused by the build -- no teleop or autonomy
    ESTOP_UNWIRED = 1 << 7  # v2: no e-stop pin -- no teleop or autonomy (a normal build refuses ARM)


class ResetCause(IntEnum):
    """v2: why the MCU last started (the same value in every STATE of a boot)."""

    POWER = 0  # power-on or brown-out
    PIN = 1
    IWDG = 2
    SOFTWARE = 3
    OTHER = 4


class BenchMode(IntEnum):
    """Bench extension: the run's drive mode, fixed by its BENCH_SET (contract 8)."""

    DUTY = 1
    HOLD = 2  # A1b


class BenchSource(IntEnum):
    """Bench extension: the run's measurement source (contract 8, plan 0.1)."""

    NONE = 0
    KTH = 1  # A1b
    MT = 2  # A1c


class BenchBypass(IntFlag):
    """Bench extension: the checks a BENCH_SET may turn off for one run. The stopping side has no bit."""

    STEER_RANGE = 1 << 0  # M: the steering joint range (KTH validity stays)
    LIFT_RANGE = 1 << 1  # H: the lift joint range (KTH validity stays)


BENCH_BYPASS_KNOWN = int(BenchBypass.STEER_RANGE | BenchBypass.LIFT_RANGE)


class BenchDriveState(IntEnum):
    """Bench extension: where the consumed run is (contract 8)."""

    NONE = 0  # no run consumed since boot
    PREP = 1  # armed, no accepted non-zero COMMISSION yet
    DRIVING = 2
    ENDED = 3


class BenchStop(IntEnum):
    """Bench extension: the first cause that ended a run that drove (the result's stop reason)."""

    NONE = 0
    END_ZERO = 1  # an accepted zero COMMISSION after driving
    COMMAND_TIMEOUT = 2
    TIME_LIMIT = 3
    DISARM = 4
    FAULT = 5
    ESTOP = 6
    LINK_BOUNDARY = 7
    DISTANCE_REACHED = 8  # A1b
    DISTANCE_PROVISIONAL = 9  # A1b
    NO_PROGRESS = 10  # A1b
    KTH_ZERO_SUSPECT = 11  # A1b
    KTH_LOST = 12  # A1b


_FAULT_MASK = (1 << 25) - 1
_STATE_FLAG_MASK = (1 << 8) - 1


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
        _check_range("echo_mcu_time_us", self.echo_mcu_time_us, 0, U64_MAX)


@dataclass(frozen=True)
class Commission:
    """v2 commissioning command (FW_COMMISSION builds only): one axis's duty, the others stopped. Axis 0 L, 1 R,
    2 M (steering), 3 H (lift); ``duty_e4`` -10000..10000. Freshness as ``Drive``."""

    axis: int
    duty_e4: int
    valid_after_echo_ms: int
    arm_gen: int
    boot_id: int
    echo_mcu_time_us: int

    def __post_init__(self) -> None:
        _check_range("axis", self.axis, 0, 3)
        _check_range("duty_e4", self.duty_e4, -10000, 10000)
        _check_range("valid_after_echo_ms", self.valid_after_echo_ms, 1, 0xFFFF)
        _check_range("arm_gen", self.arm_gen, 1, 0xFFFF)
        _check_range("boot_id", self.boot_id, 1, U32_MAX)
        _check_range("echo_mcu_time_us", self.echo_mcu_time_us, 0, U64_MAX)


@dataclass(frozen=True)
class Arm:
    boot_id: int
    echo_mcu_time_us: int

    def __post_init__(self) -> None:
        _check_range("boot_id", self.boot_id, 1, U32_MAX)
        _check_range("echo_mcu_time_us", self.echo_mcu_time_us, 0, U64_MAX)


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
    reset_cause: ResetCause = ResetCause.POWER
    # v3 (contract 3.3): per axis (M steering, H lift), 16-bit modular -- difference adjacent STATEs of one boot with
    # :func:`kth_counter_delta`. Fresh frames, those failing the configured CRC variant, those that would fail the other
    kth_frames: tuple[int, int] = (0, 0)
    kth_crc_errors: tuple[int, int] = (0, 0)
    kth_alt_crc_errors: tuple[int, int] = (0, 0)

    @property
    def current_a(self) -> tuple[float, ...]:
        """Motor currents L, R, M, H in amperes."""
        return tuple(value / 1000 for value in self.current_ma)

    @property
    def duty(self) -> tuple[float, ...]:
        """Applied duties L, R, M, H in -1..1."""
        return tuple(value / 10000 for value in self.duty_e4)


@dataclass(frozen=True)
class BenchSet:
    """Bench extension (contract 8): one run's settings, taken in DISARMED only and consumed once by the BENCH_ARM
    carrying the same ``run_id``. ``count_sign`` is the duty sign that raises the source's count (+1 or -1)."""

    boot_id: int
    echo_mcu_time_us: int
    run_id: int
    mode: int
    axis: int
    source: int
    count_sign: int
    bypass: int
    duty_max_e4: tuple
    time_ms: tuple
    stop_counts: int

    def __post_init__(self) -> None:
        _check_range("boot_id", self.boot_id, 1, U32_MAX)
        _check_range("echo_mcu_time_us", self.echo_mcu_time_us, 0, U64_MAX)
        _check_range("run_id", self.run_id, 1, U32_MAX)
        _check_range("mode", self.mode, 0, 0xFF)
        _check_range("axis", self.axis, 0, 0xFF)
        _check_range("source", self.source, 0, 0xFF)
        _check_range("count_sign", self.count_sign, -128, 127)
        _check_range("bypass", self.bypass, 0, U32_MAX)
        if len(self.duty_max_e4) != 4 or len(self.time_ms) != 4:
            raise ValueError("duty_max_e4 and time_ms carry four axes (L, R, M, H)")
        for i in range(4):
            _check_range("duty_max_e4", self.duty_max_e4[i], 0, 0xFFFF)
            _check_range("time_ms", self.time_ms[i], 0, U32_MAX)
        _check_range("stop_counts", self.stop_counts, -(2**31), 2**31 - 1)


@dataclass(frozen=True)
class BenchArm:
    """Bench extension (contract 8): ARM for the bench image, naming the run whose BENCH_SET it consumes."""

    boot_id: int
    echo_mcu_time_us: int
    run_id: int

    def __post_init__(self) -> None:
        _check_range("boot_id", self.boot_id, 1, U32_MAX)
        _check_range("echo_mcu_time_us", self.echo_mcu_time_us, 0, U64_MAX)
        _check_range("run_id", self.run_id, 1, U32_MAX)


@dataclass(frozen=True)
class BenchState:
    """Bench extension (contract 8): the bench image's run state, sent with the STATE whose header seq is
    ``pair_seq``. Ages are milliseconds, 0xFFFF = none yet (saturating)."""

    pair_seq: int
    run_id: int
    result_run_id: int
    axis: int
    source: int
    count_state: int
    kth_lost: int
    raw_angle: int
    cumulative: int
    target: int
    distance_ref: int
    cut_position: int
    stop_reason: int
    distance_confirm: int
    blocked: int
    dropped_pairs: int
    evidence_age_ms: int
    end_evidence_age_ms: int
    mcu_bypass: int
    drive: int  # bits 0-1 BenchDriveState, bits 4-5 BenchMode
    conn_gen: int
    reserved: int
    result_axis: int
    result_source: int

    @property
    def drive_state(self) -> BenchDriveState:
        return BenchDriveState(self.drive & 0x03)

    @property
    def mode(self) -> int:
        return (self.drive >> 4) & 0x03


Message = Drive | Commission | Arm | Disarm | ClearFaults | McuState | BenchSet | BenchArm | BenchState


def _kth(message: McuState) -> tuple[int, ...]:
    out = []
    for name in ("kth_frames", "kth_crc_errors", "kth_alt_crc_errors"):
        pair = tuple(getattr(message, name))
        if len(pair) != 2:
            raise LinkError(f"{name} holds two counters (M, H)")
        for v in pair:
            _check_range(name, v, 0, 0xFFFF)
        out += pair
    return tuple(out)


def kth_counter_delta(new: int, old: int) -> int:
    """How much a v3 KTH counter rose between two adjacent STATEs of one boot: unsigned modulo 2^16 (contract 3.3).
    Never :func:`wheel_count_delta`'s signed half-range reading -- these counters only rise."""
    return (int(new) - int(old)) & 0xFFFF


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
    if isinstance(message, Commission):
        return TYPE_COMMISSION, _COMMISSION.pack(
            message.axis, message.duty_e4, message.valid_after_echo_ms, message.arm_gen, message.boot_id,
            message.echo_mcu_time_us)
    if isinstance(message, BenchSet):
        return TYPE_BENCH_SET, _BENCH_SET.pack(
            message.boot_id, message.echo_mcu_time_us, message.run_id, message.mode, message.axis, message.source,
            message.count_sign, message.bypass, *message.duty_max_e4, *message.time_ms, message.stop_counts)
    if isinstance(message, BenchArm):
        return TYPE_BENCH_ARM, _BENCH_ARM.pack(message.boot_id, message.echo_mcu_time_us, message.run_id)
    if isinstance(message, BenchState):
        return TYPE_BENCH_STATE, _BENCH_STATE.pack(
            message.pair_seq, message.run_id, message.result_run_id, message.axis, message.source,
            message.count_state, message.kth_lost, message.raw_angle, message.cumulative, message.target,
            message.distance_ref, message.cut_position, message.stop_reason, message.distance_confirm,
            message.blocked, message.dropped_pairs, message.evidence_age_ms, message.end_evidence_age_ms,
            message.mcu_bypass, message.drive, message.conn_gen, message.reserved, message.result_axis,
            message.result_source)
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
            int(message.reset_cause),
            *_kth(message),
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
    mode, reject, faults, flags, cause = fields[4], fields[5], fields[6], fields[27], fields[28]
    if mode > max(Mode) or reject > max(RejectReason) or cause > max(ResetCause):
        raise LinkError("mode, reject reason or reset cause out of range")
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
        reset_cause=ResetCause(cause),
        kth_frames=tuple(fields[29:31]),
        kth_crc_errors=tuple(fields[31:33]),
        kth_alt_crc_errors=tuple(fields[33:35]),
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
        TYPE_COMMISSION: _COMMISSION,
        TYPE_BENCH_SET: _BENCH_SET,
        TYPE_BENCH_ARM: _BENCH_ARM,
        TYPE_BENCH_STATE: _BENCH_STATE,
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
        elif type_ == TYPE_COMMISSION:
            message = Commission(*fields)
        elif type_ == TYPE_BENCH_SET:
            message = BenchSet(*fields[:8], tuple(fields[8:12]), tuple(fields[12:16]), fields[16])
        elif type_ == TYPE_BENCH_ARM:
            message = BenchArm(*fields)
        elif type_ == TYPE_BENCH_STATE:
            message = BenchState(*fields)
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

    def discard_partial(self) -> None:
        """A new connection starts clean: a half frame of the old one is dropped, the error counts kept."""
        self._frames.discard()

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
