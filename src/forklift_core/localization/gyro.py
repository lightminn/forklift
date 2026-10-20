"""Gyro heading for the wheel odometry (gyro-heading plan docs/plans/2026-10-21-gyro-heading-odometry.md, G2 design (A)
docs/plans/2026-10-21-gyro-g2-runtime-safety.md D1, D2, D5).

Two parts:

- ``GyroSampler`` -- the simulated D435i gyro (a setup service: it holds the RNG): the body's angular velocity rotated into
  the IMU axes by the *true* mount (the camera's mount, then an injected mount error), and on each axis (1 + scale) x rate
  + initial bias + temperature ramp + bias random walk + white noise, quantised. ``sample`` returns ``(stamp_s, w_imu,
  seq)``.
- the heading state -- plain values in a dict (``new_heading_state``; a gyro run's boundary record is refused by name for
  now, tools/boundary_replay.py UNREPLAYABLE_ON -- the sampler holds its RNG): the algorithm rotates the sample back to the body by the *known* mount (the camera's only), takes the
  vertical rate, keeps a standstill bias from non-overlapping blocks (both rear wheel counts unchanged, the applied
  command zero, the block's variance at most k N^2/dt), and gives the heading increment (raw - bias) dt once a bias exists.
  A sample gap past the model's deadline, a NaN or a rate beyond the range on any of the three sensor axes, or a stamp
  that does not advance is a fault (``GyroFault``): the run ends its session (design D5). Each bias update is kept in
  ``history`` as [block end s, bias rad/s] (design D7's heading record).
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

FORMAT = "gyro_model/1"
FIRST_BIAS_LIMIT_S = 60.0  # design D2: no first bias by this sim second ends the session as slam_prewarm_timeout does
D2R = math.pi / 180.0


class GyroFault(RuntimeError):
    """The gyro cannot be read as it must (design D5); the message starts with ``slam_gyro_fault``."""


@dataclass(frozen=True)
class GyroModel:
    """A ``gyro_model/1`` file, the datasheet's units (deg, s)."""

    scale: float
    initial_bias_dps: float  # each axis' b0 from U(-x, x)
    temperature_dps: float  # a linear ramp reaching this over temperature_s, sign drawn per axis
    temperature_s: float
    random_walk_dps_rts: float  # deg/s/sqrt(s)
    noise_dps_rthz: float
    lsb_dps: float
    range_dps: float
    mount_error_deg: float  # about the vehicle's y axis (left), unknown to the algorithm
    zupt_s: float
    zupt_k: float
    gap_s: float  # a sample later than this after the last is a fault

    @classmethod
    def load(cls, path: Path) -> tuple["GyroModel", str]:
        import yaml

        text = Path(path).read_text()
        data = yaml.safe_load(text)
        if not isinstance(data, dict) or data.get("format") != FORMAT:
            raise ValueError(f"{path}: not a {FORMAT} file")
        fields = {k: float(data[k]) for k in cls.__dataclass_fields__}
        model = cls(**fields)
        if model.zupt_s <= 0 or model.gap_s <= 0 or model.noise_dps_rthz < 0 or model.lsb_dps <= 0 or model.range_dps <= 0:
            raise ValueError(f"{path}: non-positive zupt_s, gap_s, lsb_dps or range_dps, or negative noise")
        return model, hashlib.sha256(text.encode()).hexdigest()

    def record(self) -> dict:
        return {"format": FORMAT, **asdict(self)}


def quat_matrix(wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in wxyz)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def rot_y(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


class GyroSampler:
    """The simulated gyro. ``mount_rotation``: vehicle (base_link) <- IMU, 3x3 -- the run's camera optical frame (the
    D435i IMU axes are the depth axes). ``fault_at_s``: from this stamp on every sample is NaN (design D5's injection)."""

    def __init__(self, model: GyroModel, mount_rotation, seed: int, *, fault_at_s: float | None = None):
        self.model = model
        self.known = np.asarray(mount_rotation, float).reshape(3, 3)
        self.true = rot_y(model.mount_error_deg * D2R) @ self.known
        rng = np.random.default_rng([int(seed), 11])
        self.rng = rng
        self.b0 = rng.uniform(-model.initial_bias_dps, model.initial_bias_dps, 3) * D2R
        self.temperature_sign = rng.choice((-1.0, 1.0), 3)
        self.walk = np.zeros(3)
        self.seq = 0
        self.last_stamp: float | None = None
        self.fault_at_s = fault_at_s

    def sample(self, stamp_s: float, w_world, body_wxyz) -> tuple[float, list, int]:
        m = self.model
        stamp = float(stamp_s)
        dt = 1.0 / 120.0 if self.last_stamp is None else max(stamp - self.last_stamp, 1e-9)
        self.last_stamp = stamp
        w_body = quat_matrix(body_wxyz).T @ np.asarray(w_world, float)
        w_imu = self.true.T @ w_body
        self.walk = self.walk + self.rng.normal(0, m.random_walk_dps_rts * D2R, 3) * math.sqrt(dt)
        temperature = self.temperature_sign * m.temperature_dps * D2R * max(stamp, 0.0) / m.temperature_s  # as V-G0: linear
        white = self.rng.normal(0, 1, 3) * (m.noise_dps_rthz * D2R / math.sqrt(dt))
        measured = (1 + m.scale) * w_imu + self.b0 + temperature + self.walk + white
        lsb = m.lsb_dps * D2R
        measured = np.round(measured / lsb) * lsb
        if self.fault_at_s is not None and stamp >= self.fault_at_s:
            measured = np.full(3, np.nan)
        self.seq += 1
        return stamp, [float(v) for v in measured], self.seq

    def record(self) -> dict:
        return {"b0_rad_s": self.b0.tolist(), "temperature_sign": self.temperature_sign.tolist(),
                "mount_error_deg": self.model.mount_error_deg, "fault_at_s": self.fault_at_s}


def new_heading_state() -> dict:
    """The heading's plain state (watched by the boundary replay)."""
    return {"bias": None, "bias_end_s": None, "updates": 0, "block": [], "last_stamp": None, "last_seq": None,
            "history": []}


def heading_summary(state: dict) -> dict:
    """The run's heading record (design D7): the updates, the last bias and every [block end s, bias rad/s]."""
    return {"updates": state["updates"], "bias_rad_s": state["bias"], "bias_end_s": state["bias_end_s"],
            "history": [list(h) for h in state["history"]]}


def vertical_rate(known_mount_rotation, w_imu) -> float:
    """The body-vertical rate of an IMU sample, through the mount (vehicle <- IMU, 3x3) the algorithm knows."""
    return float((np.asarray(known_mount_rotation, float).reshape(3, 3) @ np.asarray(w_imu, float))[2])


def heading_step(state: dict, model: GyroModel, *, stamp_s: float, seq: int, raw_rate: float, wheels_still: bool,
                 command_zero: bool, dt: float, w_imu=None) -> float | None:
    """One physics tick: fault checks, the standstill block, and the heading increment (None while no bias exists).
    ``w_imu``: the sample's three sensor axes, each range-checked before the projection hides one (Codex G2 (A) P2)."""
    max_rate = model.range_dps * D2R
    if w_imu is not None:
        axes = np.asarray(w_imu, float)
        if axes.shape != (3,) or not np.isfinite(axes).all() or np.any(np.abs(axes) > max_rate):
            raise GyroFault(f"slam_gyro_fault: sample {seq} axes {axes.tolist()!r}")
    if not math.isfinite(raw_rate) or abs(raw_rate) > max_rate:
        raise GyroFault(f"slam_gyro_fault: sample {seq} rate {raw_rate!r}")
    if state["last_stamp"] is not None:
        if not stamp_s > state["last_stamp"] or seq == state["last_seq"]:
            raise GyroFault(f"slam_gyro_fault: sample {seq} at {stamp_s} does not advance on {state['last_stamp']}")
        if stamp_s - state["last_stamp"] > model.gap_s:
            raise GyroFault(f"slam_gyro_fault: {stamp_s - state['last_stamp']:.3f} s without a sample")
    state["last_stamp"], state["last_seq"] = float(stamp_s), int(seq)
    if wheels_still and command_zero:
        state["block"].append(float(raw_rate))
        block = max(1, int(round(model.zupt_s / dt)))
        if len(state["block"]) >= block:
            window = np.asarray(state["block"][:block])
            white_var = (model.noise_dps_rthz * D2R) ** 2 / dt
            if float(np.var(window)) <= model.zupt_k * white_var:
                state["bias"], state["bias_end_s"] = float(window.mean()), float(stamp_s)
                state["updates"] += 1
                state["history"].append([float(stamp_s), state["bias"]])
            state["block"] = []  # non-overlapping: the next block starts afresh
    else:
        state["block"] = []
    if state["bias"] is None:
        return None
    return (raw_rate - state["bias"]) * dt


__all__ = ["FIRST_BIAS_LIMIT_S", "FORMAT", "GyroFault", "GyroModel", "GyroSampler", "heading_step", "heading_summary",
           "new_heading_state", "quat_matrix", "rot_y", "vertical_rate"]
