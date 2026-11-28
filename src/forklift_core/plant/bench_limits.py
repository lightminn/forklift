"""Bench commissioning limits (docs/design/2026-11-25-bench-commission-limits.md 1.2, PR-1): pure rules the
commissioning tool applies before anything is armed -- which runs are "large", what a large run needs, the
encoder early-stop bypass names, and the triangle profile with its checks. No I/O here: the tool supplies the
board's USB product string and the operator's answers."""

from __future__ import annotations

import math

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


# ---- the bench image (bench console plan A1a, mcu-link contract 8) ------------------------------------------------------
# The bench image ends its USB product string with exactly these tokens (gen_build_info.cmake); the generation names the
# wire contract it speaks (bench6: BENCH_SET / BENCH_ARM / BENCH_STATE). Any other generation is not this one: it is
# refused for every run and stays out of gates (judgement.diagnostic_image).
BENCH_SUFFIX = ("lim=lr1", "bench8")  # bench8: the MT source (bench console plan 4.5 A1c), no bench7 fallback
# The checks a BENCH_SET may turn off for one run (firmware drive.h DRIVE_BENCH_*). The stopping side -- overcurrent,
# current sense, driver faults, STOP/DISARM, command expiry, IWDG, cycle overrun -- has no name and never will.
BENCH_BYPASS_BITS = {"steer_range": 1 << 0, "lift_range": 1 << 1}
# The bench image's defaults: the open commissioning build's limits (board_config.c, pinned by the unit test) --
# L, R duty 1.0 and the validator's ms ceiling, M, H 0.15 / 2 s.
BENCH_DEFAULT_DUTY_E4 = (10000, 10000, 1500, 1500)
BENCH_DEFAULT_TIME_MS = (0x7FFFFFFF // 1000, 0x7FFFFFFF // 1000, 2000, 2000)
BENCH_TIME_MAX_MS = 0x7FFFFFFF // 1000


def bench_image(product: str | None) -> bool:
    """Whether a USB product string is this tool's bench image: the open commissioning prefix and the exact suffix."""
    return (product is not None and product.startswith(OPEN_COMMISSION_PREFIX)
            and tuple(product.split()[-len(BENCH_SUFFIX):]) == BENCH_SUFFIX)


# The MCU model's bench image says its own product on its first stdout line (plan 4.5 A1c decision 2), so an old model
# binary is never given a product it cannot speak: the host reads it within a deadline and refuses otherwise.
MODEL_PRODUCT_LINE = "product: "
MODEL_PRODUCT_DEADLINE_S = 2.0


def read_model_product(fd: int, deadline_s: float = MODEL_PRODUCT_DEADLINE_S) -> str | None:
    """The product the model says on its first stdout line, read from ``fd`` (non-blocking reads under a deadline),
    or None when none came in time (an old model says nothing and keeps running) or the line is not one."""
    import os
    import select
    import time

    end = time.monotonic() + deadline_s
    buf = b""
    while b"\n" not in buf:
        left = end - time.monotonic()
        if left <= 0:
            return None
        ready, _w, _x = select.select([fd], [], [], left)
        if not ready:
            return None
        chunk = os.read(fd, 256)
        if not chunk:
            return None  # the model ended
        buf += chunk
        if len(buf) > 512:
            return None
    line = buf.split(b"\n", 1)[0].decode("utf-8", "replace")
    return line[len(MODEL_PRODUCT_LINE):] if line.startswith(MODEL_PRODUCT_LINE) else None


def any_bench_image(product: str | None) -> bool:
    """Any bench generation (``lim=lr1 bench*``), this tool's own or not."""
    toks = str(product).split() if product else []
    return (product is not None and product.startswith(OPEN_COMMISSION_PREFIX) and len(toks) >= 2
            and toks[-2] == LIMITS_TOKEN and toks[-1].startswith("bench"))


def parse_bench_bypass(text: str | None) -> tuple[str, ...]:
    if not text:
        return ()
    names = tuple(dict.fromkeys(n.strip() for n in text.split(",") if n.strip()))
    unknown = [n for n in names if n not in BENCH_BYPASS_BITS]
    if unknown:
        raise LimitRefused(f"bench-bypass: unknown name(s) {unknown}; known {sorted(BENCH_BYPASS_BITS)} "
                           "(the stopping side is never bypassed)")
    return names


# ---- bench7 hold (plan 4.3 A1b-2 decisions 2-3, 10) ----------------------------------------------------------------
HOLD_TARGETS = {"zero": 0, "abs": 1, "here": 2, "rel": 3}  # BenchHoldTarget


def parse_hold_target(text: str) -> tuple[int, int]:
    """``zero`` | ``here`` | ``abs:V`` (raw 0..4095) | ``rel:N`` (|N| < 2^30 counts) -> (target_kind, target_value)."""
    word, sep, rest = str(text).strip().partition(":")
    if word not in HOLD_TARGETS:
        raise LimitRefused(f"hold: {text!r} is not zero, here, abs:V or rel:N")
    if word in ("zero", "here"):
        if sep:
            raise LimitRefused(f"hold: {word} takes no value")
        return HOLD_TARGETS[word], 0
    try:
        value = int(rest, 10)
    except ValueError:
        raise LimitRefused(f"hold: {text!r} needs an integer value") from None
    if word == "abs" and not 0 <= value <= 4095:
        raise LimitRefused("hold abs:V: a raw angle 0..4095")
    if word == "rel" and not -(2**30) < value < 2**30:
        raise LimitRefused("hold rel:N: |N| < 2^30 counts")
    return HOLD_TARGETS[word], value


@dataclass(frozen=True)
class HoldAsk:
    """A hold run's settings (one BENCH_HOLD, sent again unchanged to extend it). The defaults are the 11-27 measured
    starting values (lead 2026-11-28): cap 0.15, kp 0.5, deadband 12, minimum duty 0.12, ki 0, kd 0 (kd untested)."""

    kind: int
    value: int
    kp: float = 0.5
    ki: float = 0.0
    kd: float = 0.0
    deadband: int = 12
    min_duty: float = 0.12
    cap: float = 0.15
    source: int = 1  # bench8 (plan 4.5 A1c): 1 KTH (cpr 4096), 2 MT (the wheel's motor shaft CPR from vehicle.yaml)
    cpr: int = 4096

    @property
    def duty_max_e4(self) -> int:
        return int(round(self.cap * 10000))

    @property
    def min_duty_e4(self) -> int:
        return int(round(self.min_duty * 10000))


def hold_refusal(h: HoldAsk, axis_duty_max_e4: int) -> str | None:
    """The firmware's value checks (bench_hold_values_ok), refused here before anything is sent."""
    for name, value, lo, hi, lo_open in (("hold-kp", h.kp, 0.0, 100.0, True), ("hold-ki", h.ki, 0.0, 1000.0, False),
                                         ("hold-kd", h.kd, 0.0, 10.0, False)):
        if not math.isfinite(value) or value > hi or value < lo or (lo_open and value <= lo):
            return f"{name}: {'(' if lo_open else '['}{lo:g}, {hi:g}]"
    if not isinstance(h.deadband, int) or not 0 <= h.deadband <= 512:
        return "hold-deadband: 0..512 counts"
    if not math.isfinite(h.cap) or not 1 <= h.duty_max_e4 <= axis_duty_max_e4:
        return f"hold duty cap: (0, {axis_duty_max_e4 / 10000:g}] (the run axis's duty limit)"
    if not math.isfinite(h.min_duty) or h.min_duty_e4 < 0 or h.min_duty_e4 > h.duty_max_e4:
        return "hold-min-duty: 0 .. the duty cap"
    if h.source == MT_SOURCE:
        if h.kind in (HOLD_TARGETS["zero"], HOLD_TARGETS["abs"]):
            return "hold: the MT count is relative -- here or rel:N only (plan 4.5 A1c decision 5)"
        if not isinstance(h.cpr, int) or not 1 <= h.cpr <= MT_CPR_MAX:
            return f"hold: the MT motor shaft CPR 1..{MT_CPR_MAX}"
    elif h.cpr != KTH_COUNTS_PER_TURN:
        return "hold: the KTH CPR is 4096"
    return None


@dataclass(frozen=True)
class BenchAsk:
    """One run's BENCH_SET: the bypass names, and the run axis's duty and time limits (the other axes at the image's
    defaults)."""

    axis: int
    names: tuple[str, ...]
    duty_max: float | None  # None: the image's default for the axis
    time_max_s: float | None
    source: int = 0  # bench6: 1 the run axis's KTH count (M, H); bench8: 2 the run wheel's MT count (L, R)
    stop_counts: int = 0  # bench6: the signed distance stop in counts, 0 none
    count_sign: int = 1  # bench6: the duty sign x the count's direction
    mode: int = 1  # BenchMode: 1 duty, 2 hold (bench7)
    hold: HoldAsk | None = None  # bench7: the hold run's settings
    units: dict | None = None  # bench8: an MT run's units as read (MtUnits.record), fixed for the record

    @property
    def hold_run(self) -> bool:
        return self.mode == 2

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

    @property
    def distance(self) -> bool:
        return self.stop_counts != 0


HIGH_DUTY_DISTANCE = 0.4  # above this a distance stop is shown as unverified, never refused (lead 2026-11-28)
KTH_COUNTS_PER_TURN = 4096
KTH_SOURCE, MT_SOURCE = 1, 2
MT_CPR_MAX = 65535


def turns_stop_counts(turns: float, counts_per_turn: int, duty: float, count_sign: int,
                      label: str = "stop-kth-turns") -> int:
    """The distance stop for ``turns`` turns of the source's shaft in the duty's direction (plan 4.2 A1b-1 decision 8,
    4.5 A1c decision 11): ``round(turns x counts_per_turn) x count_sign x sign(duty)``, refused unless 1 <= |counts| <
    2^31 (a tiny distance never rounds to 0, which would mean no distance stop)."""
    if not math.isfinite(turns) or turns <= 0:
        raise LimitRefused(f"{label}: a positive finite number of turns")
    if turns >= 2 ** 31 / counts_per_turn:
        raise LimitRefused(f"{label}: {turns} turns is past 2^31-1 counts")
    if count_sign not in (1, -1):
        raise LimitRefused("count-sign: 1 or -1")
    if duty == 0 or not math.isfinite(duty):
        raise LimitRefused(f"{label}: needs a non-zero duty (its sign is the direction)")
    mag = round(turns * counts_per_turn)
    if not 1 <= mag < 2 ** 31:
        raise LimitRefused(f"{label}: {turns} turns is {mag} counts; 1 .. 2^31-1 counts")
    return mag * count_sign * (1 if duty > 0 else -1)


def kth_stop_counts(turns: float, duty: float, count_sign: int) -> int:
    """The KTH sensor turns' distance stop (plan 4.2 A1b-1 decision 8)."""
    return turns_stop_counts(turns, KTH_COUNTS_PER_TURN, duty, count_sign)


def counts_stop(counts, duty: float, count_sign: int) -> int:
    """A distance stop given in counts (plan 4.5 A1c decision 11): a positive whole magnitude 1..2^31-1 (0 would mean
    no stop on the wire), signed by ``count_sign x sign(duty)``."""
    if isinstance(counts, bool) or not isinstance(counts, int):
        raise LimitRefused("stop-counts: a whole number of counts")
    if not 1 <= counts < 2 ** 31:
        raise LimitRefused("stop-counts: 1 .. 2^31-1 counts (its sign comes from the duty and count-sign)")
    if count_sign not in (1, -1):
        raise LimitRefused("count-sign: 1 or -1")
    if duty == 0 or not math.isfinite(duty):
        raise LimitRefused("stop-counts: needs a non-zero duty (its sign is the direction)")
    return counts * count_sign * (1 if duty > 0 else -1)


@dataclass(frozen=True)
class MtUnits:
    """One wheel's MT units from ``config/real/vehicle.yaml`` ``wheel_counts.<side>`` (plan 0.1, 4.5 A1c decision 11):
    the motor shaft CPR (None when the side gives only counts per wheel turn, or a CPR outside 1..65535 or not whole --
    then MT turns and MT hold are refused), the gear ratio, the counts per wheel turn and whether they were measured."""

    side: str
    motor_cpr: int | None
    gear_ratio: float | None
    counts_per_wheel_rev: float | None  # None when the CPR is not a number (never NaN in a record)
    measured: bool
    why: str | None = None

    def record(self) -> dict:
        return {"side": self.side, "motor_cpr": self.motor_cpr, "gear_ratio": self.gear_ratio,
                "counts_per_wheel_rev": self.counts_per_wheel_rev, "measured": self.measured,
                "no_motor_cpr": self.why}


def _finite_pos(v) -> float | None:
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        return None
    try:
        f = float(v)
    except OverflowError:
        return None  # an integer past the float range (Codex A1c-2 r3 P3)
    return f if math.isfinite(f) and f > 0 else None


def mt_units_from_mapping(data: dict) -> tuple[MtUnits, MtUnits]:
    """L and R from vehicle.yaml's content (its ``wheel_counts``); the loader keeps the motor shaft CPR the vehicle
    config folds into counts per wheel turn. Every recorded number is finite (never NaN or Infinity in a JSON row --
    Codex A1c-2 r2 P2-2): a value that is not a finite positive number is recorded as None with the reason."""
    out = []
    for side in ("left", "right"):
        entry = (data.get("wheel_counts") or {}).get(side)
        if not isinstance(entry, dict):
            raise ValueError(f"wheel_counts.{side} must be a mapping")
        keys = set(entry) - {"provenance"}
        prov = entry.get("provenance") or {}
        measured = bool(prov.get("measured", False)) if isinstance(prov, dict) else False
        if keys == {"motor_counts_per_rev", "gear_ratio"}:
            raw = _finite_pos(entry["motor_counts_per_rev"])
            gear = _finite_pos(entry["gear_ratio"])
            cpr, why = None, None
            if raw is None:
                why = "motor_counts_per_rev is not a finite positive number"
            elif raw != int(raw):
                why = f"motor_counts_per_rev {raw:g} is not whole"
            elif not 1 <= int(raw) <= MT_CPR_MAX:
                why = f"motor_counts_per_rev {int(raw)} is outside 1..{MT_CPR_MAX}"
            else:
                cpr = int(raw)
            if gear is None:
                cpr, why = None, why or "gear_ratio is not a finite positive number"
            per_wheel = raw * gear if raw is not None and gear is not None else None
            if per_wheel is not None and not (math.isfinite(per_wheel) and per_wheel > 0):
                per_wheel, cpr, why = None, None, why or "the counts per wheel turn are not a finite positive number"
            out.append(MtUnits(side, cpr, gear, per_wheel, measured, why))
        elif keys == {"counts_per_wheel_rev"}:
            out.append(MtUnits(side, None, None, _finite_pos(entry["counts_per_wheel_rev"]), measured,
                               "this side gives counts per wheel turn only"))
        else:
            raise ValueError(f"wheel_counts.{side} takes either motor_counts_per_rev + gear_ratio or counts_per_wheel_rev")
    return out[0], out[1]


def mt_units(path) -> tuple[MtUnits, MtUnits]:
    from pathlib import Path

    import yaml

    return mt_units_from_mapping(yaml.safe_load(Path(path).read_text(encoding="utf-8")))


def bench_refusal(ask: BenchAsk) -> str | None:
    if ask.duty_max is not None and not 0 < ask.duty_max <= 1.0:
        return "bench-duty-max: in (0, 1]"
    if ask.time_max_s is not None and not 0.001 <= ask.time_max_s <= BENCH_TIME_MAX_MS / 1000.0:
        return f"bench-time-max: in [0.001, {BENCH_TIME_MAX_MS / 1000.0:g}] s"
    if "steer_range" in ask.names and ask.axis != 2:
        return "bench-bypass steer_range: the steering axis (M) only"
    if "lift_range" in ask.names and ask.axis != 3:
        return "bench-bypass lift_range: the lift axis (H) only"
    # the source and axis pairs (bench8, plan 4.5 A1c decision 3): KTH on M or H, MT on L or R
    paired = (ask.source == KTH_SOURCE and ask.axis in (2, 3)) or (ask.source == MT_SOURCE and ask.axis in (0, 1))
    if ask.mode == 2 and not paired:
        return "hold: the KTH count of M or H, or the MT count of L or R"  # before the distance words below
    if ask.source not in (0, KTH_SOURCE, MT_SOURCE):
        return "bench source: none, kth or mt"
    if ask.source == KTH_SOURCE and ask.axis not in (2, 3):
        return "stop-kth-turns: the KTH count is M's or H's"
    if ask.source == MT_SOURCE and ask.axis not in (0, 1):
        return "source mt: the MT count is L's or R's"
    if ask.stop_counts != 0 and ask.source == 0:
        return "stop: a distance stop needs a measured source"
    if ask.mode not in (1, 2):
        return "bench mode: duty or hold"
    if ask.hold_run:
        if not paired:
            return "hold: the KTH count of M or H, or the MT count of L or R"
        if ask.stop_counts != 0:
            return "hold: no distance stop"
        if ask.hold is None:
            return "hold: no settings"
        if ask.hold.source != ask.source:
            return "hold: its source must be the run's"
        return hold_refusal(ask.hold, ask.duty_max_e4()[ask.axis])
    return None
