"""Encoder models of the team's selected parts, for the simulation's measurement model.

Sources: the week-7 deck (team EasyEDA material, 2026-10-14) and the user's answers (2026-10-15).
The rear L/R (driven) wheels carry an MT6701QT-ACD incremental encoder -- A/B 1,024 pulses per
revolution as shipped, counted x4 by the STM32 timer's encoder mode (4,096 counts per revolution);
the steering (and lift) carry a KTH7801-X-C-SOP8 absolute encoder whose SPI frame holds a 12-bit
angle and a 4-bit CRC. Datasheet integral non-linearity: MT6701 +-1.0 deg typical (+-1.5 deg max,
Rev.1.5), KTH7801 +-0.35 deg typical. The INL's *shape* is not specified; these models use a
first harmonic of the shaft angle with the typical amplitude, which is an assumption. MT6701 ABZ
hysteresis 0.088 deg (Rev.1.5) is modelled as a Schmitt band around each count transition; its
transition noise (0.01 deg rms) and the KTH7801's output noise (0.015 deg) are left out (both well
below one count of 2 pi / 4096 = 0.088 deg).
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import numpy as np

MT6701_PULSES_PER_REV = 1024
QUADRATURE_FACTOR = 4  # both edges of A and B (user, 2026-10-15)
MT6701_COUNTS_PER_REV = MT6701_PULSES_PER_REV * QUADRATURE_FACTOR
MT6701_INL_TYPICAL_RAD = math.radians(1.0)
MT6701_ABZ_HYSTERESIS_RAD = math.radians(0.088)
KTH7801_C_ANGLE_BITS = 12
KTH7801_INL_TYPICAL_RAD = math.radians(0.35)
TWO_PI = 2.0 * math.pi


def _finite(value, name: str) -> float:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


class AngleUnwrapper:
    """A continuous angle from successive wrapped readings of a revolute joint (any wrap period
    that is a multiple of 2 pi; the step between readings must stay below pi)."""

    def __init__(self) -> None:
        self._raw = None
        self.angle_rad = None

    def update(self, raw_rad) -> float:
        raw = _finite(raw_rad, "raw_rad")
        if self._raw is None:
            self.angle_rad = raw
        else:
            step = raw - self._raw
            step -= TWO_PI * round(step / TWO_PI)
            self.angle_rad += step
        self._raw = raw
        return self.angle_rad


def first_harmonic_inl(angle_rad: float, amplitude_rad: float, phase_rad: float) -> float:
    return amplitude_rad * math.sin(angle_rad + phase_rad)


class QuadratureWheelEncoder:
    """An incremental wheel encoder read once per control tick. The count follows the encoder angle
    theta + INL(theta) of the continuous shaft angle: it starts at floor(angle / step) and steps up
    once the angle passes the next transition plus half the hysteresis, down once it falls below the
    current one minus half (no hysteresis: floor). Each reading returns the rotation the count moved
    since the previous reading -- 0 on the first, which only sets the reference."""

    def __init__(self, counts_per_rev: int = MT6701_COUNTS_PER_REV, *, inl_amplitude_rad: float = 0.0,
                 inl_phase_rad: float = 0.0, hysteresis_rad: float = 0.0) -> None:
        if int(counts_per_rev) != counts_per_rev or counts_per_rev <= 0:
            raise ValueError("counts_per_rev must be a positive integer")
        self.counts_per_rev = int(counts_per_rev)
        self.step_rad = TWO_PI / self.counts_per_rev
        self.inl_amplitude_rad = abs(_finite(inl_amplitude_rad, "inl_amplitude_rad"))
        self.inl_phase_rad = _finite(inl_phase_rad, "inl_phase_rad")
        if self.inl_amplitude_rad >= 1.0:
            raise ValueError("inl amplitude must be below one radian")
        self.hysteresis_rad = abs(_finite(hysteresis_rad, "hysteresis_rad"))
        # MT6701: 0.088 deg, about one count of 4,096 (2 pi / 4096 = 0.0879 deg).
        if self.hysteresis_rad >= 2 * self.step_rad:
            raise ValueError("hysteresis must be below two counts")
        self._unwrap = AngleUnwrapper()
        self.count = None

    def encoder_angle(self, angle_rad: float) -> float:
        angle = _finite(angle_rad, "angle_rad")
        return angle + first_harmonic_inl(angle, self.inl_amplitude_rad, self.inl_phase_rad)

    def counts_at(self, angle_rad: float) -> int:
        """The count without history (no hysteresis): floor of the encoder angle over a step."""
        return math.floor(self.encoder_angle(angle_rad) / self.step_rad)

    def update_continuous(self, angle_rad: float) -> float:
        """As update, from an already continuous shaft angle."""
        value = self.encoder_angle(angle_rad)
        if self.count is None:
            self.count = math.floor(value / self.step_rad)
            return 0.0
        count, half = self.count, self.hysteresis_rad / 2
        while value >= (count + 1) * self.step_rad + half:
            count += 1
        while value < count * self.step_rad - half:
            count -= 1
        increment = (count - self.count) * self.step_rad
        self.count = count
        return increment

    def update(self, raw_joint_angle_rad) -> float:
        """The measured rotation (rad) since the previous reading, from the count increment."""
        return self.update_continuous(self._unwrap.update(raw_joint_angle_rad))

    @property
    def angle_rad(self) -> float | None:
        return self._unwrap.angle_rad


class AbsoluteAngleEncoder:
    """An absolute encoder read as an n-bit angle: round((theta + INL(theta) - INL(0)) / step) x step,
    i.e. its zero calibrated at the straight-ahead position; wrapped to (-pi, pi]."""

    def __init__(self, bits: int = KTH7801_C_ANGLE_BITS, *, inl_amplitude_rad: float = 0.0,
                 inl_phase_rad: float = 0.0) -> None:
        if int(bits) != bits or not 1 <= bits <= 24:
            raise ValueError("bits must be an integer in 1..24")
        self.bits = int(bits)
        self.step_rad = TWO_PI / (1 << self.bits)
        self.inl_amplitude_rad = abs(_finite(inl_amplitude_rad, "inl_amplitude_rad"))
        self.inl_phase_rad = _finite(inl_phase_rad, "inl_phase_rad")

    def read(self, angle_rad) -> float:
        angle = _finite(angle_rad, "angle_rad")
        error = (first_harmonic_inl(angle, self.inl_amplitude_rad, self.inl_phase_rad)
                 - first_harmonic_inl(0.0, self.inl_amplitude_rad, self.inl_phase_rad))
        value = round((angle + error) / self.step_rad) * self.step_rad
        return math.atan2(math.sin(value), math.cos(value))


ENCODER_MODELS = ("reported", "team_spec_quantized", "team_spec_typical_inl")


@dataclass(frozen=True)
class EncoderModelSpec:
    """What a --encoder-model name means (recorded in meta.json with the run's INL phases)."""

    name: str
    algorithm: int  # bumped when the counting or reading rule changes; descriptions are not compared
    wheel_counts_per_rev: int
    wheel_hysteresis_rad: float
    wheel_inl_amplitude_rad: float
    steering_bits: int
    steering_inl_amplitude_rad: float
    wheels: str = "rear left, rear right (MT6701, driven wheels)"
    steering: str = ("each front steering joint read as a KTH7801-C 12-bit angle with an ideal internal "
                     "zero at straight ahead (the truck has one steering encoder on an unknown linkage)")
    inl_shape: str = "first harmonic of the shaft angle, typical amplitude, phase per sensor from the seed"
    sample: str = "one reading per physics tick; row i is the rotation over (t[i-1], t[i]], row 0 is 0"


SPEC_COMPARED_FIELDS = ("name", "algorithm", "wheel_counts_per_rev", "wheel_hysteresis_rad", "wheel_inl_amplitude_rad",
                        "steering_bits", "steering_inl_amplitude_rad")
ENCODER_ARRAYS = ("wheel_counts", "wheel_increments_rad", "steering_measured_rad")


def encoder_model_spec(name: str) -> EncoderModelSpec | None:
    """None for "reported" (the engine's joint velocity, the runner's behaviour before the model)."""
    if name not in ENCODER_MODELS:
        raise ValueError(f"unknown encoder model {name!r}")
    if name == "reported":
        return None
    inl = name == "team_spec_typical_inl"
    return EncoderModelSpec(
        name=name,
        algorithm=1,
        wheel_counts_per_rev=MT6701_COUNTS_PER_REV,
        wheel_hysteresis_rad=MT6701_ABZ_HYSTERESIS_RAD,
        wheel_inl_amplitude_rad=MT6701_INL_TYPICAL_RAD if inl else 0.0,
        steering_bits=KTH7801_C_ANGLE_BITS,
        steering_inl_amplitude_rad=KTH7801_INL_TYPICAL_RAD if inl else 0.0,
    )


def sensor_phases(seed: int | None) -> tuple[float, float, float, float]:
    """INL phases of the rear-left, rear-right wheel and front-left, front-right steering sensors: one
    sensor unit per seed (fixed for a run), all 0 without a seed."""
    if seed is None:
        return (0.0, 0.0, 0.0, 0.0)
    return tuple(float(v) for v in np.random.default_rng([int(seed), 7]).uniform(0, 2 * math.pi, 4))


class TeamEncoderModel:
    """The online model: two rear wheel encoders and two steering reads per physics tick."""

    def __init__(self, spec: EncoderModelSpec, phases: tuple[float, float, float, float]) -> None:
        self.spec, self.phases = spec, tuple(float(p) for p in phases)
        self.wheels = [QuadratureWheelEncoder(spec.wheel_counts_per_rev, inl_amplitude_rad=spec.wheel_inl_amplitude_rad,
                                              inl_phase_rad=self.phases[i], hysteresis_rad=spec.wheel_hysteresis_rad)
                       for i in range(2)]
        self.steering = [AbsoluteAngleEncoder(spec.steering_bits, inl_amplitude_rad=spec.steering_inl_amplitude_rad,
                                              inl_phase_rad=self.phases[2 + i]) for i in range(2)]

    def read(self, rear_wheel_angles_rad, steering_rad):
        """(increments, measured steering, counts) from the rear wheels' continuous angles and the true
        steering angles."""
        increments = [enc.update_continuous(a) for enc, a in zip(self.wheels, rear_wheel_angles_rad, strict=True)]
        steering = [enc.read(a) for enc, a in zip(self.steering, steering_rad, strict=True)]
        return tuple(increments), tuple(steering), tuple(enc.count for enc in self.wheels)

    def record(self) -> dict:
        return {**asdict(self.spec), "phases_rad": list(self.phases)}


def encoder_odometry_inputs(log, meta: dict, *, phase_seed: int | None = None, check_recorded: bool = True) -> dict:
    """The odometry inputs a replay integrates from a recorded run (slam_log.npz + meta.json):
    {"kind": "rates", "rates", "steering"} for a "reported" run (meta without encoder_model reads as
    reported); {"kind": "increments", "increments", "steering", "counts"} for a team-spec run, the
    counts regenerated over the whole log from the recorded continuous rear wheel angles and true
    steering with the run's phases (phase_seed None; checked against the recorded counts, increments
    and measured steering) or another sensor unit's (phase_seed given). Anything inconsistent raises."""
    record = meta.get("encoder_model")
    name = "reported" if record is None else record.get("name")
    spec = encoder_model_spec(name)
    present = [k for k in ENCODER_ARRAYS if k in getattr(log, "files", log)]
    if spec is None and present:
        raise ValueError(f"encoder arrays {present} in a log recorded as {name!r} (lost or contradicting model record)")
    stamps = np.asarray(log["joint_stamps_s"], dtype=float)
    if stamps.ndim != 1 or len(stamps) < 2 or not np.isfinite(stamps).all() or np.any(np.diff(stamps) <= 0):
        raise ValueError("joint_stamps_s must be finite and strictly increasing")
    steering_true = np.asarray(log["steering_rad"], dtype=float)
    if spec is None:
        return {"kind": "rates", "rates": np.asarray(log["wheel_rates_rad_s"], dtype=float)[:, 2:4],
                "steering": steering_true}
    if any(record.get(k) != getattr(spec, k) for k in SPEC_COMPARED_FIELDS):
        raise ValueError(f"the log's encoder model record differs from {name!r} as this code defines it")
    if len(present) != len(ENCODER_ARRAYS) and check_recorded:
        raise ValueError(f"a {name!r} log must carry {list(ENCODER_ARRAYS)}, has {present}")
    angles = np.asarray(log["wheel_angles_rad"], dtype=float)
    if angles.shape != (len(stamps), 4) or not np.isfinite(angles).all():
        raise ValueError("wheel_angles_rad must be finite and (N, 4)")
    if steering_true.shape != (len(stamps), 2) or not np.isfinite(steering_true).all():
        raise ValueError("steering_rad must be finite and (N, 2)")
    phases = tuple(record["phases_rad"]) if phase_seed is None else sensor_phases(phase_seed)
    model = TeamEncoderModel(spec, phases)
    rows = [model.read(angles[k, 2:4], steering_true[k]) for k in range(len(stamps))]
    increments = np.array([r[0] for r in rows], dtype=float)
    steering = np.array([r[1] for r in rows], dtype=float)
    counts = np.array([r[2] for r in rows], dtype=np.int64)
    if phase_seed is None and check_recorded:
        for key, value, integer in (("wheel_counts", counts, True), ("wheel_increments_rad", increments, False),
                                    ("steering_measured_rad", steering, False)):
            recorded = np.asarray(log[key])
            kind_ok = (np.issubdtype(recorded.dtype, np.integer) if integer
                       else np.issubdtype(recorded.dtype, np.floating) and np.isfinite(recorded).all())
            if not kind_ok or recorded.dtype == np.bool_:
                raise ValueError(f"the recorded {key} has dtype {recorded.dtype}")
            # Compare in the regenerated value's own type: a record narrowed to another dtype differs.
            if recorded.shape != value.shape or not np.array_equal(recorded.astype(value.dtype), value) \
                    or (not integer and recorded.dtype != np.float64):
                raise ValueError(f"the recorded {key} does not match the encoder model")
        if not np.allclose(np.diff(counts, axis=0) * (2 * math.pi / spec.wheel_counts_per_rev), increments[1:],
                           rtol=0, atol=1e-12):
            raise ValueError("wheel increments are not the count differences")
    return {"kind": "increments", "increments": increments, "steering": steering, "counts": counts}


__all__ = [
    "ENCODER_MODELS",
    "EncoderModelSpec",
    "MT6701_ABZ_HYSTERESIS_RAD",
    "TeamEncoderModel",
    "encoder_model_spec",
    "encoder_odometry_inputs",
    "sensor_phases",
    "AbsoluteAngleEncoder",
    "AngleUnwrapper",
    "KTH7801_C_ANGLE_BITS",
    "KTH7801_INL_TYPICAL_RAD",
    "MT6701_COUNTS_PER_REV",
    "MT6701_INL_TYPICAL_RAD",
    "MT6701_PULSES_PER_REV",
    "QUADRATURE_FACTOR",
    "QuadratureWheelEncoder",
    "first_harmonic_inl",
]
