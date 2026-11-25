"""The real A2M12's one source for mount, self mask and timing bounds, and its scan -> layer conversion (PR 7 design
4, 7a-1; config/real/lidar.yaml).

``to_layer_scan`` turns a ``scan_feed.RealScan`` into the obstacle layer's ``real_layer.RealScan`` only when the
scan's acquisition can be bounded: the SDK's first-sample time (on the host monotonic clock, carried by the adapter
as ``first_beam_mono_ns``) with an error bound ``frame_bound`` allows for this very frame. Without both the scan stays
diagnostic -- the driver stamp is taken before a buffered scan is fetched and is not an acquisition time (2026-11-12
design 1.1).

``frame_bound`` (7a-3, plan 2026-11-26 3 and 9) is the one judgement the layer and the SLAM path share: the bound
applies to a frame only when the frame is stamped at the SDK's byte arrival (``sdk_first_sample_rx``, patch 0002),
``lidar.yaml``'s ``timing.basis`` is ``measured`` or ``engineering`` with its decision id and basis document (never
called measured: the lead's 2026-11-26 decision), and the facts of the source that actually ran -- carried by the
adapter in the frame itself -- equal ``timing.applies_to``. With the bound:

- first beam ``[Flo, Fhi]`` and last beam ``[Elo, Ehi]`` (the beam spacing ``time_increment``);
- the layer gets ``first_s = Flo`` (pose and FREE age, conservative), ``last_s = Ehi`` (the odometry history must
  reach it) and ``deadline_last_s = Elo`` (the layer deadline ``+ B``);
- a scan with ``Ehi - Flo`` over the nominal period is refused for the layer: the stop model counts
  ``scan_period_s + B`` from the first beam (Codex PR 7 design round 4 P2-1);
- an invalid range (NaN: the driver's zero, out of range) is never free (``valid`` False) -- the HIL rule that a
  no-return is a valid far reading does not apply here (Codex round 1 P2-6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

__all__ = ["APPLIES_KEYS", "BASES", "LidarConfig", "LIDAR_YAML", "RX_STAMP", "band_limit_m", "frame_bound",
           "load_lidar_config", "self_hit_mask", "source_facts", "to_layer_scan"]

LIDAR_YAML = Path(__file__).resolve().parents[3] / "config" / "real" / "lidar.yaml"

RX_STAMP = "sdk_first_sample_rx"  # patch 0002: the SDK's sample times from the receive thread's byte arrival
BASES = ("measured", "engineering")  # timing.basis: what the bound rests on (null: no bound)
# the source facts a bound is tied to (plan 3): the image that ran (its ID, not its tag), the USB adapter (udev
# vendor:product:serial), the host, and what the SDK chose (scan_sdk_info: answer type, sample period, baud rate)
APPLIES_KEYS = ("image_id", "usb", "host", "ans_type", "sample_period_us", "baudrate")


@dataclass(frozen=True)
class LidarConfig:
    mount: tuple[float, float, float, float]  # x, y, z (m, rear-axle frame), yaw (rad)
    mount_measured: bool
    mask_rad: tuple[tuple[float, float], ...]  # sensor-frame intervals the truck's body hides
    limit_m: float  # per-beam height-band range limit (band_half_m / sin(max_tilt))
    scan_period_s: float
    first_beam_err_s: float | None  # None: no bound yet -> every scan diagnostic
    timing_basis: str | None = None  # None, "measured" or "engineering" (never written as measured: lead 2026-11-26)
    timing_decision: str | None = None  # engineering: the lead's exception decision id
    timing_doc: str | None = None  # engineering: the document that states the assumptions
    applies_to: tuple = ()  # ((key, value), ...) sorted: the source facts the bound was taken on

    @property
    def bound_usable(self) -> bool:
        """The bound exists and rests on a measurement, or on an engineering estimate with its decision id and basis
        document; and it names the source it holds for."""
        if self.first_beam_err_s is None or self.timing_basis not in BASES:
            return False
        if self.timing_basis == "engineering" and not (self.timing_decision and self.timing_doc):
            return False
        return {k for k, _ in self.applies_to} == set(APPLIES_KEYS)


def band_limit_m(max_tilt_deg: float, band_half_m: float) -> float:
    """The range within which a beam tilted at most ``max_tilt_deg`` stays inside +-``band_half_m`` of the mount
    height (the teleop bridge's rule)."""
    tilt = math.radians(float(max_tilt_deg))
    return float(band_half_m) / math.sin(tilt) if tilt > 0 else math.inf


def self_hit_mask(angles: np.ndarray, mask_rad) -> np.ndarray:
    """Beams whose (wrapped) angle falls in any masked interval -- the truck's own returns."""
    a = np.asarray(angles, dtype=float)
    hit = np.zeros(a.shape, dtype=bool)
    wrapped = np.arctan2(np.sin(a), np.cos(a))
    for lo, hi in mask_rad:
        hit |= (wrapped >= lo) & (wrapped <= hi)
    return hit


def _finite(v, name):
    f = float(v)
    if not math.isfinite(f):
        raise ValueError(f"lidar config: {name} must be finite")
    return f


def _fact_ok(key: str, v) -> bool:
    if key in ("ans_type", "baudrate"):
        return isinstance(v, int) and not isinstance(v, bool) and v > 0
    if key == "sample_period_us":
        return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v)) and v > 0
    return isinstance(v, str) and bool(v)


def _timing_basis(t: dict, err) -> dict:
    """``timing.basis`` and what it needs (lead 2026-11-26): no bound without a basis, no basis without a bound; an
    engineering basis names its decision and its document; every basis names the source facts it holds for. The old
    ``timing.measured`` is refused, never read as a basis."""
    if "measured" in t:
        raise ValueError("lidar config: timing.measured is replaced by timing.basis (measured | engineering)")
    basis = t.get("basis")
    if basis is None:
        if err is not None:
            raise ValueError("lidar config: timing.first_beam_err_s needs timing.basis")
        return {"timing_basis": None}
    if basis not in BASES:
        raise ValueError(f"lidar config: timing.basis must be one of {BASES} or null")
    if err is None:
        raise ValueError("lidar config: timing.basis needs timing.first_beam_err_s")
    decision, doc = t.get("decision_id"), t.get("basis_doc")
    if basis == "engineering" and not (isinstance(decision, str) and decision and isinstance(doc, str) and doc):
        raise ValueError("lidar config: an engineering basis needs timing.decision_id and timing.basis_doc")
    applies = t.get("applies_to")
    if not isinstance(applies, dict) or set(applies) != set(APPLIES_KEYS):
        raise ValueError(f"lidar config: timing.applies_to must give exactly {APPLIES_KEYS}")
    for k, v in applies.items():
        if not _fact_ok(k, v):
            raise ValueError(f"lidar config: timing.applies_to.{k} is malformed")
    return {"timing_basis": basis, "timing_decision": decision if isinstance(decision, str) else None,
            "timing_doc": doc if isinstance(doc, str) else None, "applies_to": tuple(sorted(applies.items()))}


def load_lidar_config(path: str | Path = LIDAR_YAML) -> LidarConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    m = raw["mount"]
    xyz = [_finite(v, "mount.xyz") for v in m["xyz"]]
    if len(xyz) != 3:
        raise ValueError("lidar config: mount.xyz takes three numbers")
    mask = []
    for pair in raw.get("self_mask_deg") or []:
        a, b = (_finite(v, "self_mask_deg") for v in pair)
        if not -180.0 <= a <= b <= 180.0:
            raise ValueError("lidar config: self_mask_deg intervals are [a, b] with -180 <= a <= b <= 180")
        mask.append((math.radians(a), math.radians(b)))
    band = raw["height_band"]
    t = raw["timing"]
    err = t.get("first_beam_err_s")
    if err is not None:
        err = _finite(err, "timing.first_beam_err_s")
        if err < 0:
            raise ValueError("lidar config: timing.first_beam_err_s must be >= 0")
    period = _finite(t["scan_period_s"], "timing.scan_period_s")
    if period <= 0:
        raise ValueError("lidar config: timing.scan_period_s must be > 0")
    return LidarConfig(
        mount=(xyz[0], xyz[1], xyz[2], math.radians(_finite(m["yaw_deg"], "mount.yaw_deg"))),
        mount_measured=bool(m.get("measured", False)),
        mask_rad=tuple(mask),
        limit_m=band_limit_m(_finite(band["max_tilt_deg"], "max_tilt_deg"), _finite(band["band_half_m"], "band_half_m")),
        scan_period_s=period,
        first_beam_err_s=err,
        **_timing_basis(t, err),
    )


def source_facts(meta: dict) -> dict | None:
    """The facts of the source a frame came from, flat (the adapter's ``source`` head: launcher facts and, when the
    node's scan_sdk_info for this very start was received, its ``sdk_info``); None without the SDK's part."""
    src = meta.get("source")
    if not isinstance(src, dict) or not isinstance(src.get("sdk_info"), dict):
        return None
    facts = {k: src.get(k) for k in ("image_id", "usb", "host")}
    facts.update({k: src["sdk_info"].get(k) for k in ("ans_type", "sample_period_us", "baudrate")})
    return facts


def frame_bound(scan, cfg: LidarConfig) -> tuple[float | None, str | None]:
    """``(first_beam_err_s, None)`` when ``cfg``'s bound holds for this frame, else ``(None, why)``: ``no_time_bound``
    (no usable bound in the config), ``not_arrival_stamped`` (not a patch-0002 frame), ``no_source_facts`` (the frame
    does not say what ran, or not with its run, start and producer generation), ``source_mismatch`` (it ran on something the bound was not taken on). The layer and the
    SLAM path both ask this (plan 3, Codex P2-1)."""
    if not cfg.bound_usable:
        return None, "no_time_bound"
    if scan.meta.get("stamp_kind") != RX_STAMP:
        return None, "not_arrival_stamped"
    facts = source_facts(scan.meta)
    src = scan.meta.get("source")
    if facts is None or any(facts.get(k) is None for k in APPLIES_KEYS):
        return None, "no_source_facts"
    if not (isinstance(src.get("run"), str) and isinstance(src.get("start"), str)
            and getattr(scan, "producer_gen", None) is not None):
        return None, "no_source_facts"  # a bound needs the frame's whole identity: run, start, generation (round 1 P3-2)
    if any(facts[k] != v for k, v in cfg.applies_to):
        return None, "source_mismatch"
    return cfg.first_beam_err_s, None


def to_layer_scan(scan, cfg: LidarConfig, start_ns: int, *, sensor: str = "lidar"):
    """``(real_layer.RealScan, None)`` or ``(None, why)`` -- why one of ``no_acquisition_time``, ``frame_bound``'s
    reasons, ``no_beam_timing``, ``scan_too_long``, ``after_receipt`` (the last beam's earliest time after the adapter
    received the scan: impossible, the time is wrong). Core seconds are ``(ns - start_ns) / 1e9``."""
    from forklift_core.live.real_layer import RealScan as LayerScan

    first = scan.meta.get("first_beam_mono_ns")
    if not isinstance(first, int) or isinstance(first, bool):
        return None, "no_acquisition_time"
    bound, why = frame_bound(scan, cfg)
    if why is not None:
        return None, why  # no bound for this frame keeps it diagnostic (Codex 7a-1 P2-3, 7a-3 plan 3)
    n = len(scan.ranges)
    end = scan.meta.get("last_beam_mono_ns")
    if not isinstance(end, int) or isinstance(end, bool) or end <= first or n < 2:
        # the last sample of the scan's own buffer (forklift patch 0001): never derived from a period or a beam step
        # (Codex 7a-2 P1-1: the previous rotation's length is not this scan's end)
        return None, "no_beam_timing"
    err_ns = int(round(bound * 1e9))
    flo, elo, ehi = first - err_ns, end - err_ns, end + err_ns
    if (ehi - flo) > cfg.scan_period_s * 1e9:
        return None, "scan_too_long"
    if elo > int(scan.recv_mono_ns):
        return None, "after_receipt"
    ranges = np.asarray(scan.ranges, dtype=float)
    valid = np.isfinite(ranges)
    out = LayerScan(sensor=sensor, first_s=(flo - start_ns) / 1e9, last_s=(ehi - start_ns) / 1e9,
                    angles_rad=np.asarray(scan.angles, dtype=float), ranges_m=np.where(valid, ranges, np.inf),
                    valid=valid, self_hit=self_hit_mask(scan.angles, cfg.mask_rad),
                    limit_m=np.full(n, cfg.limit_m), deadline_last_s=(elo - start_ns) / 1e9,
                    origin="synthetic" if scan.meta.get("synthetic") else "real")
    return out, None
