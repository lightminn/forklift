"""Bench commissioning limits (docs/design/2026-11-25-bench-commission-limits.md 1.2, PR-1): pure rules the
commissioning tool applies before anything is armed -- which runs are "large", what a large run needs, the
encoder early-stop bypass names, and the triangle profile with its checks. No I/O here: the tool supplies the
board's USB product string and the operator's answers."""

from __future__ import annotations

from dataclasses import dataclass

from forklift_core.mcu_link.messages import FaultFlag
from forklift_core.plant.commission import COMMISSION_DUTY_MAX, COMMISSION_HOLD_MAX_S

# The open commissioning image's limit generation (firmware build identity, gen_build_info.cmake): its wheels take
# duty 1.0 and the validator's time ceiling. A board without the whole token is refused large values.
LIMITS_TOKEN = "lim=lr1"
# fw-single (partial-parts plan 2026-11-25 PR-B): the open commissioning image with one axis per arming ends its USB
# product string with exactly these two tokens; the tool tells it apart by nothing else.
SINGLE_AXIS_SUFFIX = ("lim=lr1", "sa1")
# Tool-derived bypass names of a fw-single steering/lift run (never typed with --bypass): the firmware's M/H sensor
# checks are off on that image, and a sensorless run has no window either. Both make the run no gate evidence.
SINGLE_BYPASS_NAMES = ("m_sensor_checks_off", "sensorless")
# A detached motor's M/H hold on fw-single may use the firmware's 2 s, kept TIME_MARGIN_S under it like the wheels.
SINGLE_DETACHED_HOLD_MAX_S = 1.5
OPEN_COMMISSION_PREFIX = "forklift-mcu commission OPEN "
# The open commissioning build's wheel limits (board_config.c; tests/unit/plant/test_bench_limits.py pins them to
# the source): duty 1.0, 0x7FFFFFFF us / 1000 ms.
WHEEL_DUTY_MAX_OPEN = 1.0
WHEEL_TIME_MAX_S_OPEN = (0x7FFFFFFF // 1000) / 1000.0
TIME_MARGIN_S = 0.5  # the tool's hold stays this far under the board's time limit
VCC_WATCH_ABOVE = 0.3  # |duty| above this needs --vcc-watched on every real run (R5: VCC = VM on V1.0)
WHEEL_SLEW_PER_S = 2.0  # board_config.c wheel_duty_slew_per_s (pinned by the unit test)
PIECE_S = 0.1  # the triangle's piece length
PROFILE_VALIDITY_S = 0.06  # COMMISSION_CAP_MS: a piece's command validity after its last send

BYPASS_BITS = {
    "enc_stuck": {0: FaultFlag.ENCODER_STUCK_L, 1: FaultFlag.ENCODER_STUCK_R},
    "enc_polarity": {0: FaultFlag.ENCODER_POLARITY_L, 1: FaultFlag.ENCODER_POLARITY_R},
}


class LimitRefused(ValueError):
    """A commissioning run's bench-limit rule refused it before anything was armed."""


def parse_bypass(text: str | None) -> tuple[str, ...]:
    if not text:
        return ()
    names = tuple(dict.fromkeys(n.strip() for n in text.split(",") if n.strip()))
    unknown = [n for n in names if n not in BYPASS_BITS]
    if unknown:
        raise LimitRefused(f"bypass: unknown name(s) {unknown}; known {sorted(BYPASS_BITS)}")
    return names


def ignored_bits(names: tuple[str, ...], axis: int) -> FaultFlag:
    """The encoder bits the early stop leaves out for this wheel (none for steering and lift)."""
    out = FaultFlag(0)
    for n in names:
        out |= BYPASS_BITS.get(n, {}).get(axis, FaultFlag(0))  # the fw-single names leave no encoder bit out
    return out


@dataclass(frozen=True)
class Profile:
    pieces: tuple[int, ...]  # signed duty_e4 per PIECE_S
    period_s: float
    cycles: int

    @property
    def duration_s(self) -> float:
        return len(self.pieces) * PIECE_S

    @property
    def max_abs_e4(self) -> int:
        return max(abs(p) for p in self.pieces)


def triangle(lo: float, hi: float, period_s: float, cycles: int, sign: int) -> Profile:
    """lo -> hi -> lo per period as signed duty_e4 pieces of PIECE_S (piece centres, so the peak lies just under hi).
    The sign goes on every piece (the diagnostic branch sent them all positive whatever was asked)."""
    if sign not in (1, -1):
        raise LimitRefused("profile: the sign is +1 or -1")
    if not (0 <= lo < hi <= WHEEL_DUTY_MAX_OPEN) or cycles < 1:
        raise LimitRefused("profile: need 0 <= LO < HI <= 1.0 and CYCLES >= 1")
    n_f = period_s / PIECE_S
    n = int(round(n_f))
    if n < 2 or abs(n_f - n) > 1e-9:
        raise LimitRefused(f"profile: PERIOD must be a whole number (>= 2) of {PIECE_S} s pieces")
    pieces = []
    for _c in range(cycles):
        for k in range(n):
            phase = (k + 0.5) / n
            frac = 2 * phase if phase < 0.5 else 2 * (1 - phase)
            pieces.append(sign * int(round((lo + (hi - lo) * frac) * 10000)))
    return Profile(tuple(pieces), period_s, cycles)


def slope_problem(pieces: tuple[int, ...]) -> str | None:
    """The generated pieces, the first step from 0 included, must not ask for more than the board's wheel slew
    (else the firmware cuts the ramp and the applied wave differs from the asked one)."""
    step_max = WHEEL_SLEW_PER_S * PIECE_S * 10000 + 1e-9
    prev = 0
    for i, p in enumerate(pieces):
        if abs(p - prev) > step_max:
            return f"profile: piece {i} steps {abs(p - prev)} e4 in {PIECE_S} s, over the wheel slew {WHEEL_SLEW_PER_S}/s"
        prev = p
    return None


@dataclass(frozen=True)
class RunAsk:
    axis: int  # 0 L, 1 R, 2 M, 3 H
    duty: float  # signed, the largest asked |duty| (a profile's peak)
    seconds: float  # the longest non-zero output asked, ramp included
    scenario: str | None = None


def is_large(ask: RunAsk) -> bool:
    return ask.axis < 2 and ask.scenario is None and (
        abs(ask.duty) > COMMISSION_DUTY_MAX + 1e-12 or ask.seconds > COMMISSION_HOLD_MAX_S + 1e-12)


def large_refusal(ask: RunAsk, *, wheels_lifted: bool, vcc_watched: bool, synthetic: bool) -> str | None:
    """What a large wheel run needs besides the board check (design 1.2): the wheels lifted and fixed, a VCC watch
    above 0.3 on a real board, and the PR-1 time bound."""
    if not is_large(ask):
        return None
    if abs(ask.duty) > WHEEL_DUTY_MAX_OPEN + 1e-12:
        return f"duty: at most {WHEEL_DUTY_MAX_OPEN}"
    # the open board's own wheel limit, less the tool's margin (PR-2a matches acceptances across the seq's wrap, so
    # PR-1's 600 s interim bound is gone)
    top = WHEEL_TIME_MAX_S_OPEN - TIME_MARGIN_S
    if ask.seconds > top:
        return f"seconds: at most {top:g} s in one run (the board's wheel time limit less {TIME_MARGIN_S} s)"
    if not wheels_lifted:
        return ("wheels lifted: a large run needs --wheels-lifted (wheels off the floor, the truck fixed against the "
                "fast wheel's vibration and reaction, the wires clear of the turning parts)")
    if abs(ask.duty) > VCC_WATCH_ABOVE + 1e-12 and not vcc_watched and not synthetic:
        return f"vcc: above duty {VCC_WATCH_ABOVE} watch VCC (--vcc-watched); step 0.3 -> 0.5 -> 0.7 -> 1.0"
    return None


def single_axis_image(product: str | None) -> bool:
    """Whether a USB product string is fw-single's: the open commissioning prefix and the exact suffix."""
    return (product is not None and product.startswith(OPEN_COMMISSION_PREFIX)
            and tuple(product.split()[-len(SINGLE_AXIS_SUFFIX):]) == SINGLE_AXIS_SUFFIX)


def board_refusal(product: str | None) -> str | None:
    """A large run only on the open commissioning image carrying the limit generation, read from the open fd's USB
    product string (UART7 has none; a USB-UART adapter returns its own)."""
    if product is None:
        return "board: no USB product string to read the limit generation from (UART7 or no read): no large values"
    if not product.startswith(OPEN_COMMISSION_PREFIX) or LIMITS_TOKEN not in product.split():
        return f"board: {product!r} is not an open commissioning image with {LIMITS_TOKEN}: no large values"
    return None


def confirmation_phrase(ask: RunAsk) -> str:
    return f"duty {abs(ask.duty):g}, {ask.seconds:g} s, wheels lifted and fixed"


# ---- DIAGNOSTIC BRANCH ONLY (diag/fw-bench, never merged): the bench image's session overrides ----------------------
# The bench image ends its USB product string with exactly these tokens (gen_build_info.cmake); the generation number
# names what its STATE carries (2: the steering KTH travel counter) -- an older bench image is not this one (Codex
# fw-bench round 3 P2-1).
BENCH_SUFFIX = ("lim=lr1", "bench2")
# The checks a BENCH_SET may turn off for one run (firmware drive.h DRIVE_BENCH_*). The stopping side -- overcurrent,
# driver faults, STOP/DISARM, command expiry, IWDG, cycle overrun -- has no name and never will.
BENCH_BYPASS_BITS = {"steer_range": 1 << 0, "lift_range": 1 << 1}
# The bench image's defaults (the open commissioning build's limits, board_config.c, pinned by the unit test):
# L, R duty 1.0 and the validator's ms ceiling, M, H 0.15 / 2 s.
BENCH_DEFAULT_DUTY_E4 = (10000, 10000, 1500, 1500)
BENCH_DEFAULT_TIME_MS = (0x7FFFFFFF // 1000, 0x7FFFFFFF // 1000, 2000, 2000)
BENCH_TIME_MAX_MS = 0x7FFFFFFF // 1000
BENCH_KTH_TURNS_MAX = 100.0
# Codex fw-bench round 2 P2-2: the 100 Hz STATE cannot tell a fast turn's way round; the bench image counts the steering
# KTH's travel itself at every valid frame (1 kHz) and carries it in STATE's kth_alt_crc_errors pair.
KTH_COUNTS_PER_TURN = 4096
# Codex fw-bench round 1 P2-3: with its range off a KTH stuck low (0x0000 passes the CRC) is a valid fixed angle; a
# driven axis whose sensor has not moved this much within this long is stopped
BENCH_NO_MOTION_S = 0.5
BENCH_NO_MOTION_COUNTS = 57  # 5 deg of the steering KTH (4096 a turn)
BENCH_NO_MOTION_LIFT_M = 0.002


def bench_image(product: str | None) -> bool:
    """Whether a USB product string is the bench image's: the open commissioning prefix and the exact suffix."""
    return (product is not None and product.startswith(OPEN_COMMISSION_PREFIX)
            and tuple(product.split()[-len(BENCH_SUFFIX):]) == BENCH_SUFFIX)


def any_bench_image(product: str | None) -> bool:
    """Any bench generation (``lim=lr1 bench*``): told apart from this tool's own by ``bench_image``."""
    toks = str(product).split() if product else []
    return (product is not None and product.startswith(OPEN_COMMISSION_PREFIX) and len(toks) >= 2
            and toks[-2] == "lim=lr1" and toks[-1].startswith("bench"))


def parse_bench_bypass(text: str | None) -> tuple[str, ...]:
    if not text:
        return ()
    names = tuple(dict.fromkeys(n.strip() for n in text.split(",") if n.strip()))
    unknown = [n for n in names if n not in BENCH_BYPASS_BITS]
    if unknown:
        raise LimitRefused(f"bench-bypass: unknown name(s) {unknown}; known {sorted(BENCH_BYPASS_BITS)} "
                           "(the stopping side is never bypassed)")
    return names


@dataclass(frozen=True)
class BenchAsk:
    """One run's BENCH_SET: the bypass names, and the run axis's duty and time limits (the other axes at the image's
    defaults)."""

    axis: int
    names: tuple[str, ...]
    duty_max: float | None  # None: the image's default for the axis
    time_max_s: float | None

    @property
    def bypass(self) -> int:
        out = 0
        for n in self.names:
            out |= BENCH_BYPASS_BITS[n]
        return out

    def duty_max_e4(self) -> tuple[int, ...]:
        out = list(BENCH_DEFAULT_DUTY_E4)
        if self.duty_max is not None:
            out[self.axis] = int(round(self.duty_max * 10000))
        return tuple(out)

    def time_ms(self) -> tuple[int, ...]:
        out = list(BENCH_DEFAULT_TIME_MS)
        if self.time_max_s is not None:
            out[self.axis] = int(round(self.time_max_s * 1000))
        return tuple(out)

    def overrides(self) -> bool:
        return (self.bypass != 0 or self.duty_max_e4() != BENCH_DEFAULT_DUTY_E4
                or self.time_ms() != BENCH_DEFAULT_TIME_MS)


def bench_refusal(ask: BenchAsk) -> str | None:
    if ask.duty_max is not None and not 0 < ask.duty_max <= 1.0:
        return "bench-duty-max: in (0, 1]"
    if ask.time_max_s is not None and not 0.001 <= ask.time_max_s <= BENCH_TIME_MAX_MS / 1000.0:
        return f"bench-time-max: in [0.001, {BENCH_TIME_MAX_MS / 1000.0:g}] s"
    if "steer_range" in ask.names and ask.axis != 2:
        return "bench-bypass steer_range: the steering axis (M) only"
    if "lift_range" in ask.names and ask.axis != 3:
        return "bench-bypass lift_range: the lift axis (H) only"
    return None


# diag/fw-bench hold: the board's steering axis (board_config.c: sign +1, sensor_per_joint 1, zero_offset pi, pinned by
# the unit test) -- the KTH raw angle from STATE.steering_rad, for a hold at the sensor's own zero
STEER_ZERO_OFFSET_RAD = 3.141592653589793


def kth_raw_angle(steering_rad: float) -> int | None:
    """The KTH raw angle (0..4095) a STATE's steering_rad reads (the board's sign, ratio and zero), or None."""
    import math

    if steering_rad is None or not math.isfinite(steering_rad):
        return None
    return int(round((steering_rad + STEER_ZERO_OFFSET_RAD) * 4096 / math.tau)) % 4096


def hold_target(spec: str, counts_now: int, steering_rad: float) -> int:
    """The travel-counter target (mod 2^32) of ``--hold-target``: ``here`` (now), ``zero`` (the nearest sensor raw
    angle 0) or a signed count offset from now. LimitRefused when it cannot be told."""
    if spec == "here":
        return counts_now & 0xFFFFFFFF
    if spec == "zero":
        a = kth_raw_angle(steering_rad)
        if a is None:
            raise LimitRefused("hold-target zero: the KTH angle is not valid now")
        delta = -a if a <= 2048 else 4096 - a
        return (counts_now + delta) & 0xFFFFFFFF
    try:
        off = int(spec)
    except ValueError:
        raise LimitRefused("hold-target: here, zero or a signed count offset") from None
    if abs(off) > 100 * KTH_COUNTS_PER_TURN:
        raise LimitRefused("hold-target: an offset within 100 turns")
    return (counts_now + off) & 0xFFFFFFFF


def kth_counts(state) -> int:
    """The bench image's steering KTH travel counter (mod 2^32) from STATE's kth_alt_crc_errors pair (low, high)."""
    lo, hi = state.kth_alt_crc_errors
    return (int(hi) << 16) | int(lo)


def counts_delta(a: int, b: int) -> int:
    """b - a of two travel counters, as a signed 32-bit difference."""
    d = (int(b) - int(a)) & 0xFFFFFFFF
    return d - (1 << 32) if d & 0x80000000 else d
