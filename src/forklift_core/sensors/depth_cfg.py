"""The real D435i depth stream's configuration and calibration identity (PR 7 7e, docs/plans/2026-11-26-7e-depth-pocket-check.md
1-4). ``load_depth_config`` reads ``config/real/depth.yaml``; ``calib_id`` is the canonical sha256 of a depth stream's
intrinsics, the identity a record row carries and a replay recomputes (plan 4.3)."""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

DEPTH_YAML = Path(__file__).resolve().parents[3] / "config" / "real" / "depth.yaml"
SCHEMA = "depth_source/1"
BASES = ("measured", "engineering")
APPLIES_KEYS = ("serial", "firmware", "usb", "backend", "profile", "settings_sha256")
# written in this order: the preset first (it resets the others), then the rest
SENSOR_KEYS = ("visual_preset", "global_time_enabled", "emitter_enabled", "laser_power", "enable_auto_exposure",
               "auto_exposure_limit_toggle", "auto_exposure_limit")


@dataclass(frozen=True)
class DepthConfig:
    width: int
    height: int
    fps: int
    sensor: dict  # option -> value to write (None: left as the device has it, still read back)
    delay_bound_s: float | None  # None: no bound -> diagnostic only
    measured: bool
    provisional_delay_bound_s: float
    receive_gap_s: float
    queue_max: int
    chunk_frames: int
    space_margin: float
    timing_basis: str | None = None
    timing_decision: str | None = None
    timing_doc: str | None = None
    applies_to: tuple = ()
    mount: dict = field(default_factory=dict)
    distortion_max_px: float = 0.5

    @property
    def interval_bound_s(self) -> float:
        """The interval width the source uses: the bound when entered, else the provisional one (diagnostic)."""
        return self.delay_bound_s if self.delay_bound_s is not None else self.provisional_delay_bound_s

    def bytes_per_s(self) -> float:
        return float(self.width * self.height * 2 * self.fps)


def _pos(v, name, *, allow_none=False):
    if v is None and allow_none:
        return None
    if isinstance(v, bool):
        raise ValueError(f"depth config: {name}")
    try:
        f = float(v)
    except (TypeError, ValueError):
        raise ValueError(f"depth config: {name}") from None
    if not math.isfinite(f) or f <= 0:
        raise ValueError(f"depth config: {name}")
    return f


def _int(v, name):
    if isinstance(v, bool) or not isinstance(v, int) or v <= 0:
        raise ValueError(f"depth config: {name}")
    return v


def load_depth_config(path: str | Path = DEPTH_YAML) -> DepthConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema") != SCHEMA:
        raise ValueError("depth config: schema")
    s, t, g, r = raw["stream"], raw["timing"], raw["generation"], raw["record"]
    sensor = raw.get("sensor") or {}
    unknown = set(sensor) - set(SENSOR_KEYS)
    if unknown:
        raise ValueError(f"depth config: unknown sensor options {sorted(unknown)}")
    for k, v in sensor.items():
        if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(float(v))):
            raise ValueError(f"depth config: sensor.{k}")
    if sensor.get("auto_exposure_limit") is not None and (sensor.get("auto_exposure_limit_toggle") != 1
                                                           or sensor.get("enable_auto_exposure") != 1):
        # the D400 applies the ceiling only with auto exposure and its own toggle on (Codex 7e-1 round 2 P2-4)
        raise ValueError("depth config: sensor.auto_exposure_limit needs enable_auto_exposure 1 and "
                         "auto_exposure_limit_toggle 1")
    bound = _pos(t.get("delay_bound_s"), "timing.delay_bound_s", allow_none=True)
    basis = t.get("basis")
    if basis is not None and basis not in BASES:
        raise ValueError(f"depth config: timing.basis must be one of {BASES} or null")
    if (basis is None) != (bound is None):
        raise ValueError("depth config: timing.delay_bound_s and timing.basis go together")
    applies = t.get("applies_to")
    if basis is not None:
        if not isinstance(applies, dict) or set(applies) != set(APPLIES_KEYS):
            raise ValueError(f"depth config: timing.applies_to must give exactly {APPLIES_KEYS}")
        if basis == "engineering" and not (t.get("decision_id") and t.get("basis_doc")):
            raise ValueError("depth config: an engineering basis needs timing.decision_id and timing.basis_doc")
    measured = t.get("measured") is True
    if measured and bound is None:
        raise ValueError("depth config: measured without timing.delay_bound_s")
    return DepthConfig(
        width=_int(s["width"], "stream.width"), height=_int(s["height"], "stream.height"),
        fps=_int(s["fps"], "stream.fps"), sensor=dict(sensor), delay_bound_s=bound, measured=measured,
        provisional_delay_bound_s=_pos(t["provisional_delay_bound_s"], "timing.provisional_delay_bound_s"),
        receive_gap_s=_pos(g["receive_gap_s"], "generation.receive_gap_s"),
        queue_max=_int(raw["queue_max"], "queue_max"), chunk_frames=_int(r["chunk_frames"], "record.chunk_frames"),
        space_margin=_pos(r["space_margin"], "record.space_margin"), timing_basis=basis,
        timing_decision=t.get("decision_id"), timing_doc=t.get("basis_doc"),
        applies_to=() if applies is None else tuple(sorted(applies.items())),
        mount=dict(raw.get("mount") or {}),
        distortion_max_px=_pos(raw.get("distortion_max_px", 0.5), "distortion_max_px"))


def calib_id(intr: dict) -> str:
    """The sha256 of a depth stream's identity: device serial and firmware, stream kind and size, focal lengths and
    principal point, distortion model and its coefficients. Canonical JSON (sorted keys, floats as ``repr`` -- which
    ``json`` uses, so a value round-trips exactly). A key missing raises: a row without its intrinsics has no id."""
    keys = ("serial", "firmware", "stream", "width", "height", "fx", "fy", "ppx", "ppy", "model", "coeffs")
    doc = {k: intr[k] for k in keys}
    doc["width"], doc["height"] = int(doc["width"]), int(doc["height"])
    for k in ("fx", "fy", "ppx", "ppy"):
        doc[k] = float(doc[k])
    doc["coeffs"] = [float(c) for c in doc["coeffs"]]
    doc["serial"], doc["firmware"], doc["stream"], doc["model"] = (str(doc[k]) for k in ("serial", "firmware",
                                                                                          "stream", "model"))
    text = json.dumps(doc, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def frame_sha256(z16) -> str:
    """The sha256 of a Z16 frame's bytes as uint16 little-endian, row-major (the record's and the replay's)."""
    import numpy as np

    a = np.ascontiguousarray(np.asarray(z16), dtype="<u2")
    return hashlib.sha256(a.tobytes()).hexdigest()


__all__ = ["APPLIES_KEYS", "DEPTH_YAML", "DepthConfig", "SCHEMA", "SENSOR_KEYS", "calib_id", "frame_sha256",
           "load_depth_config"]
