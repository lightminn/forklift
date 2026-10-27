"""Fit a ``gyro_model/1`` from a static D435i IMU record (T2's C1 arm, docs/plans/2026-10-27-insertion-lateral-drift.md).

The record is the 2026-10-23 desk run (``record_imu.py``: gyro 400 Hz raw, motion correction off, one npz per minute,
columns host ms, device frame_timestamp us, unix s, x, y, z rad/s; no temperature reading). Following the desk review
(docs/design/2026-10-22-d435i-imu-desk-review.md 5.2): the cold warm-up gives the temperature response, the settled
part gives the Allan deviation (angle random walk -> noise density, bias instability, an upper bound on the bias walk).

What ``gyro_model/1`` can hold (src/forklift_core/localization/gyro.py): a uniform initial bias, a linear temperature
ramp (unbounded), a bias random walk, white noise, an LSB, a range, a scale error, a mount error and the standstill-
update settings. A flicker (bias instability) term has no field: the walk's upper bound stands in for it at long tau.

    python -m tools.fit_gyro_static <run dir> --out-json fit.json --out-model config/gyro_model_x.yaml
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import sys
from pathlib import Path

import numpy as np

R2D = 180.0 / math.pi
AXES = ("x", "y", "z")
SETTLED_FROM_S = 1800.0  # the desk review's "first 30 minutes": warm-up before, Allan after
BI_FACTOR = 0.664  # flicker floor of the Allan deviation = 0.664 x bias instability (IEEE 952)


def allan_deviation(rate: np.ndarray, fs: float, points: int = 60) -> tuple[np.ndarray, np.ndarray]:
    """Overlapping Allan deviation of a rate (any unit) sampled at ``fs`` Hz, on log-spaced cluster sizes."""
    rate = np.asarray(rate, float)
    n = len(rate)
    theta = np.concatenate(([0.0], np.cumsum(rate))) / fs  # integrated angle
    ms = np.unique(np.logspace(0, math.log10(max(n // 9, 2)), points).astype(int))
    taus, adev = [], []
    for m in ms:
        tau = m / fs
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        if len(d) < 2:
            continue
        taus.append(tau)
        adev.append(math.sqrt(np.mean(d * d) / (2 * tau * tau)))
    return np.asarray(taus), np.asarray(adev)


def noise_density(taus: np.ndarray, adev: np.ndarray, band=(0.5, 5.0)) -> float:
    """The white-noise density (unit/sqrt(Hz)): sigma(tau) sqrt(tau) on the slope -1/2 band, its median."""
    sel = (taus >= band[0]) & (taus <= band[1])
    if not sel.any():
        raise ValueError("no tau inside the white-noise band")
    return float(np.median(adev[sel] * np.sqrt(taus[sel])))


def bias_instability(taus: np.ndarray, adev: np.ndarray) -> tuple[float, float]:
    """(bias instability, tau at the Allan minimum)."""
    i = int(np.argmin(adev))
    return float(adev[i] / BI_FACTOR), float(taus[i])


def random_walk_bound(taus: np.ndarray, adev: np.ndarray) -> float:
    """A conservative rate random walk K (unit/sqrt(s)): a walk alone gives sigma = K sqrt(tau / 3), so at every tau
    the walk's share is at most sigma(tau) sqrt(3 / tau); the largest of these over tau >= the Allan minimum is taken.
    It overstates K where white noise or bias instability still contribute (by about sqrt(2) where they cross) --
    the minimum would be tighter but the long-tau estimates scatter low, so it can fall under the true K."""
    i = int(np.argmin(adev))
    t, a = taus[i:], adev[i:]
    return float(np.max(a * np.sqrt(3.0 / t)))


def segment_bias(rate: np.ndarray) -> float:
    return float(np.mean(rate))


def lsb(rate: np.ndarray) -> float:
    """The smallest step between distinct sample values."""
    v = np.unique(np.round(np.asarray(rate, float), 9))
    steps = np.diff(v)
    steps = steps[steps > 1e-9]
    return float(np.min(steps)) if len(steps) else 0.0


def warm_up(t: np.ndarray, rate: np.ndarray, *, settled_from_s: float = SETTLED_FROM_S, minute_s: float = 60.0,
            tolerance_dps: float = 0.002) -> dict:
    """The warm-up against the settled mean: per-minute means; ``change_dps`` the largest |minute mean - settled
    mean| before ``settled_from_s``, ``settle_s`` the first minute end after which every minute stays within
    ``tolerance_dps`` of the settled mean."""
    t = np.asarray(t, float) - float(t[0])
    rate = np.asarray(rate, float)
    settled = float(np.mean(rate[t >= settled_from_s]))
    edges = np.arange(0.0, t[-1] + minute_s, minute_s)
    means = []
    for a, b in zip(edges[:-1], edges[1:], strict=False):
        sel = (t >= a) & (t < b)
        if sel.any():
            means.append((b, float(np.mean(rate[sel]))))
    dev = [(end, abs(m - settled)) for end, m in means]
    change = max((d for end, d in dev if end <= settled_from_s), default=0.0)
    # The first minute from which every later minute stays within the tolerance; a bias still outside it at the end
    # never settled (settle_s = the record's end, settled False).
    settle, settled_flag = float(dev[-1][0]), False
    for k in range(len(dev)):
        if all(dd <= tolerance_dps for _, dd in dev[k:]):
            settle, settled_flag = (dev[k - 1][0] if k > 0 else 0.0), True
            break
    return {"settled_mean_dps": settled, "change_dps": float(change), "settle_s": float(settle),
            "settled": settled_flag, "tolerance_dps": tolerance_dps,
            "minute_means_dps": [[e, m] for e, m in means]}


def warm_up_exponential(t: np.ndarray, rate: np.ndarray, *, minute_s: float = 60.0) -> dict:
    """The warm-up as b(t) = settled + A exp(-t / tau), least squares on the minute means: a grid over tau (30 s to
    the record length), the linear pair (settled, A) solved at each tau. The steepest change of the bias is at the
    cold start, |A| / tau deg/s per s."""
    t = np.asarray(t, float) - float(t[0])
    rate = np.asarray(rate, float)
    edges = np.arange(0.0, t[-1] + minute_s, minute_s)
    mids, means = [], []
    for a, b in zip(edges[:-1], edges[1:], strict=False):
        sel = (t >= a) & (t < b)
        if sel.any():
            mids.append(float(np.mean(t[sel])))
            means.append(float(np.mean(rate[sel])))
    mids, means = np.asarray(mids), np.asarray(means)
    best = None
    for tau in np.geomspace(30.0, max(float(t[-1]), 60.0), 400):
        design = np.column_stack((np.ones_like(mids), np.exp(-mids / tau)))
        coef, *_ = np.linalg.lstsq(design, means, rcond=None)
        sse = float(np.sum((design @ coef - means) ** 2))
        if best is None or sse < best[0]:
            best = (sse, float(tau), float(coef[0]), float(coef[1]))
    sse, tau, settled, amplitude = best
    return {"settled_dps": settled, "amplitude_dps": amplitude, "tau_s": tau,
            "initial_slope_dps_per_s": abs(amplitude) / tau, "rms_residual_dps": math.sqrt(sse / len(means))}


def model_text(values: dict, provenance: dict, *, header: str) -> str:
    """A gyro_model/1 file with a provenance comment above every value."""
    lines = [f"# {line}" if line else "#" for line in header.splitlines()]
    lines.append("format: gyro_model/1")
    for key, value in values.items():
        lines.append(f"# {key}: {provenance[key]}")
        lines.append(f"{key}: {value!r}")
    return "\n".join(lines) + "\n"


def device_seconds(frame_timestamp_us: np.ndarray) -> np.ndarray:
    """Seconds from the first sample on the device's frame_timestamp, a 32-bit microsecond counter that wraps every
    2^32 us (4294.97 s): every backward jump of more than half the range adds one wrap."""
    us = np.asarray(frame_timestamp_us, float)
    wraps = np.concatenate(([0.0], np.cumsum(np.diff(us) < -(2.0**31)))) * 2.0**32
    return (us + wraps - us[0]) * 1e-6


def load_run(run_dir: Path) -> tuple[np.ndarray, np.ndarray]:
    """(time s from the device frame timestamps, unwrapped; rates deg/s [n, 3]) over every chunk in order."""
    rows = [np.load(f)["gyro"] for f in sorted(glob.glob(str(Path(run_dir) / "chunk_*.npz")))]
    g = np.concatenate([r for r in rows if len(r)])
    return device_seconds(g[:, 1]), g[:, 3:6] * R2D


def analyse(t: np.ndarray, rates: np.ndarray) -> dict:
    dt = np.diff(t)
    fs = 1.0 / float(np.median(dt))
    out = {"samples": int(len(t)), "duration_s": float(t[-1] - t[0]), "fs_hz": fs,
           "gaps_over_10ms": int(np.sum(dt > 0.01)), "settled_from_s": SETTLED_FROM_S, "axes": {}}
    for k, axis in enumerate(AXES):
        r = rates[:, k]
        settled = r[t >= SETTLED_FROM_S]
        taus, adev = allan_deviation(settled, fs)
        bi, tau_min = bias_instability(taus, adev)
        density = noise_density(taus, adev)
        # A settled minute mean still scatters by the white noise over 60 s and the flicker floor: 3 x their sum
        # in quadrature is the warm-up's "settled" tolerance (at least 0.002 deg/s).
        tolerance = max(0.002, 3.0 * math.hypot(density / math.sqrt(60.0), BI_FACTOR * bi))
        out["axes"][axis] = {
            "settled_bias_dps": segment_bias(settled),
            "noise_density_dps_rthz": density,
            "bias_instability_dps": bi, "bias_instability_dph": bi * 3600.0, "allan_min_tau_s": tau_min,
            "random_walk_bound_dps_rts": random_walk_bound(taus, adev),
            "lsb_dps": lsb(r),
            "warm_up": warm_up(t, r, tolerance_dps=tolerance),
            "warm_up_exponential": warm_up_exponential(t, r),
            "allan": [[float(a), float(b)] for a, b in zip(taus, adev, strict=True)],
        }
    return out


def model_from(analysis: dict, record: str) -> tuple[dict, dict]:
    axes = analysis["axes"]
    worst = lambda key: max(axes[a][key] for a in AXES)  # noqa: E731 -- the model is one value for all axes
    worst_axis = lambda key: max(AXES, key=lambda a: axes[a][key])  # noqa: E731
    warm = {a: axes[a]["warm_up_exponential"] for a in AXES}
    warm_axis = max(AXES, key=lambda a: warm[a]["initial_slope_dps_per_s"])
    values = {
        "scale": 0.01,
        "initial_bias_dps": round(max(abs(axes[a]["settled_bias_dps"]) for a in AXES), 4),
        "temperature_dps": round(abs(warm[warm_axis]["amplitude_dps"]), 4),
        "temperature_s": round(warm[warm_axis]["tau_s"], 1),
        "random_walk_dps_rts": float(f"{worst('random_walk_bound_dps_rts'):.3g}"),
        "noise_dps_rthz": round(worst("noise_density_dps_rthz"), 5),
        "lsb_dps": round(min(axes[a]["lsb_dps"] for a in AXES), 4),
        "range_dps": 1000.0,
        "mount_error_deg": 1.0,
        "zupt_s": 0.5,
        "zupt_k": 4.0,
        "gap_s": 0.05,
    }
    provenance = {
        "scale": "not measured (a static record has no rotation); kept from gyro_model_spec_typical",
        "initial_bias_dps": f"measured: the largest |settled mean| over the axes ({record}); each run draws U(-x, x)",
        "temperature_dps": (f"measured: |A| of the warm-up fit b(t) = settled + A exp(-t/tau) on the minute means, "
                            f"the axis with the steepest cold-start slope |A|/tau ({warm_axis}); the record has no "
                            "temperature, so the warm-up stands in for it"),
        "temperature_s": (f"measured: tau of that fit (axis {warm_axis}). The model's ramp temperature_dps x t / "
                          "temperature_s starts at 0 and grows without bound at |A|/tau, the measured cold start's "
                          "steepest slope: over a mission of minutes it bounds the measured change, past tau it "
                          "overstates it; the shape (a ramp, not a settling step) is the model's, not the record's"),
        "random_walk_dps_rts": (f"measured conservative bound: max over tau >= the Allan minimum of sigma sqrt(3/tau), the "
                                f"worst axis ({worst_axis('random_walk_bound_dps_rts')}), settled part; it also stands in"
                                " for the bias instability (no flicker field)"),
        "noise_dps_rthz": (f"measured: sigma(tau) sqrt(tau) on tau 0.5-5 s, the worst axis "
                           f"({worst_axis('noise_density_dps_rthz')}), settled part"),
        "lsb_dps": "measured: the smallest step between sample values (motion_xyz32f output)",
        "range_dps": "datasheet (D400 series 337029-017 4.11), not measured",
        "mount_error_deg": "assumption kept from gyro_model_spec_typical (no mount measurement)",
        "zupt_s": "algorithm setting kept from gyro_model_spec_typical",
        "zupt_k": "algorithm setting kept from gyro_model_spec_typical",
        "gap_s": "algorithm setting kept from gyro_model_spec_typical",
    }
    return values, provenance


HEADER = """The D435i gyro fitted to the team unit's static desk record (2026-10-23, serial {serial}, firmware {firmware},
2.2 h, gyro 400 Hz, motion correction off, no factory IMU calibration; ws1 {path}) by tools/fit_gyro_static.py.
A static desk measurement: no driving vibration, mast lift or temperature reading, so it does not complete the gyro
plan's G4 list (docs/design/2026-10-22-d435i-imu-desk-review.md 5). For T2's exploratory C1 comparison only
(docs/plans/2026-10-27-insertion-lateral-drift.md) -- not an adoption of gyro heading."""


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", type=Path)
    ap.add_argument("--out-json", type=Path, required=True)
    ap.add_argument("--out-model", type=Path, required=True)
    args = ap.parse_args(argv)
    meta = json.loads((args.run / "meta.json").read_text())
    t, rates = load_run(args.run)
    analysis = analyse(t, rates)
    analysis["meta"] = {k: v for k, v in meta.items() if not k.startswith("intrinsics")}
    values, provenance = model_from(analysis, str(args.run))
    header = HEADER.format(serial=meta.get("serial_number"), firmware=meta.get("firmware_version"), path=args.run)
    args.out_model.write_text(model_text(values, provenance, header=header))
    analysis["model"] = values
    args.out_json.write_text(json.dumps(analysis, indent=1, allow_nan=False))
    for axis in AXES:
        a = analysis["axes"][axis]
        print(axis, {k: a[k] for k in a if k not in ("allan", "warm_up")},
              {k: a["warm_up"][k] for k in ("change_dps", "settle_s", "settled_mean_dps")})
    print("model", values)
    return 0


if __name__ == "__main__":
    sys.exit(main())
