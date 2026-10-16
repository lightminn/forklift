"""A depth-noise table from team D435i captures (plan docs/plans/2026-10-17-recognition-measured-noise.md).

Each capture is a folder of 30 raw Z16 frames with capture.json (tools/capture_d435i_noise.py). Every
frame is dithered by +-0.5 mm (a seeded stream per frame) and turned to inverse depth, then fitted two
ways with depth_noise_fit.fit_noise_components on the robust variance of inverse-depth second
differences at steps 1-32:

- temporal: the per-pixel mean over the frames is taken out (pixels valid in every frame), scaled by
  sqrt(n/(n-1)) and placed back on a constant inverse depth -- the part of the frame-to-frame noise
  that second differences identify (a whole-frame tilt or shift cancels in them);
- frame: the frame as it is -- the observed wall's one-frame spatial variation, a scenario, not an
  upper bound on the fixed error (a plane in inverse depth cancels in second differences too).

Levels are the medians over the wall captures in the chosen distance range, one pair per resolution
(the 640 and 848 amplitudes differ); widths are kept as angles and written at one source_fx. Usage:
python tools/fit_capture_noise.py --captures <dir> --report <json> --config <yaml>
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from forklift_core.perception import depth_noise_fit as fit  # noqa: E402
from forklift_core.perception.depth_noise import correlated_field  # noqa: E402

DITHER_M = 0.0005
PROVISIONAL_DROPOUT = 0.05028932548774274   # config/depth_noise_d435i.yaml, kept so only the components change


def load_capture(folder: Path):
    """(depth stack in metres inside the ROI, NaN where invalid, capture.json)."""
    from PIL import Image

    meta = json.loads((folder / "capture.json").read_text())
    scale = float(meta["device"]["depth_scale_m"])
    x0, y0, x1, y1 = meta["setup"]["roi"]
    frames = [np.asarray(Image.open(folder / f["file"]), dtype=np.float64)[y0:y1, x0:x1] * scale for f in meta["frames"]]
    stack = np.stack(frames)
    stack[stack <= 0] = np.nan
    return stack, meta


def dither(stack, seed: int):
    out = np.empty_like(stack)
    for i, frame in enumerate(stack):
        out[i] = frame + np.random.default_rng([seed, i]).uniform(-DITHER_M, DITHER_M, frame.shape)
    return out


def temporal_frames(stack, distance_m: float):
    """Frames of the temporal residual in inverse depth, placed on 1/distance_m (NaN where any frame is invalid)."""
    w = 1.0 / stack
    valid = np.all(np.isfinite(w), axis=0)
    n = stack.shape[0]
    resid = (w - np.where(valid, np.nanmean(w, axis=0), np.nan)) * math.sqrt(n / (n - 1))
    return 1.0 / (1.0 / distance_m + np.where(valid, resid, np.nan))


def fit_frame(depth, z_range) -> dict:
    """{'short': [k, [gu, gv]], 'long': [k, [gu, gv]], 'residual': r} for one frame."""
    valid = np.isfinite(depth)
    var = {axis: {s: fit.robust_variance(fit.second_differences(depth, axis=axis, step=s, z_range=z_range, valid=valid))
                  for s in fit.STEPS} for axis in (0, 1)}
    found = fit.fit_noise_components(var[1], var[0])
    comps = [[float(k), [float(g) for g in w]] for k, w in found.components]
    while len(comps) < 2:
        comps.insert(0, [0.0, [0.0, 0.0]])
    return {"short": comps[0], "long": comps[-1], "residual": float(found.rel_residual_rms), "var_u": var[1], "var_v": var[0]}


def _median_components(rows):
    def med(key):
        return [float(np.median([r[key][0] for r in rows])),
                [float(np.median([r[key][1][0] for r in rows])), float(np.median([r[key][1][1] for r in rows]))]]
    return {"short": med("short"), "long": med("long"), "residual_p50": float(np.median([r["residual"] for r in rows]))}


def capture_fit(folder: Path, *, seed: int = 0, frames=None) -> dict:
    stack, meta = load_capture(folder)
    if frames is not None:
        stack = stack[:frames]
    dist = float(meta["setup"]["distance_m"])
    z_range = (0.5 * dist, 1.5 * dist)
    d = dither(stack, seed)
    temporal = temporal_frames(d, dist)
    t_rows = [fit_frame(f, z_range) for f in temporal]
    f_rows = [fit_frame(f, z_range) for f in d]
    st = meta["device"]["stream"]
    return {"label": meta["setup"]["label"], "surface": meta["setup"]["surface"], "distance_m": dist,
            "fx": float(meta["device"]["intrinsics"]["fx"]), "mode": int(st["width"]),
            "temporal": _median_components(t_rows), "frame": _median_components(f_rows),
            "temporal_tail_4sigma": tail_fraction(1.0 / temporal - 1.0 / dist),
            "frames": int(stack.shape[0])}


def tail_fraction(values, k: float = 4.0) -> float:
    x = np.asarray(values, float)
    x = x[np.isfinite(x)]
    s = 1.4826 * np.median(np.abs(x - np.median(x)))
    return float(np.mean(np.abs(x - np.median(x)) > k * s)) if s > 0 else 0.0


def face_pixels(stack, band_m: float = 0.030):
    """Pixels on the nearest surface: the mean depth (pixels valid in every frame) within band_m of its
    10th percentile. Returns (mask, face depth)."""
    valid = np.all(np.isfinite(stack), axis=0)
    mean = np.where(valid, np.nanmean(stack, axis=0), np.nan)
    face = float(np.nanpercentile(mean, 10))
    return valid & (np.abs(mean - face) <= band_m), face


def surface_check(stack, *, min_pixels: int = 500) -> dict:
    """Temporal sigma (depth units, via the inverse-depth MAD) on the observed face, or unverified."""
    mask, face = face_pixels(stack)
    n = int(mask.sum())
    if n < min_pixels:
        return {"status": "unverified", "face_m": face, "pixels": n}
    w = 1.0 / stack[:, mask]
    r = (w - w.mean(axis=0)) * math.sqrt(stack.shape[0] / (stack.shape[0] - 1))
    sigma_w = 1.4826 * float(np.median(np.abs(r - np.median(r))))
    return {"status": "measured", "face_m": face, "pixels": n, "sigma_m": sigma_w * face ** 2}


def build_levels(fits, *, source_fx: float, distances=(2.0, 3.0)) -> dict:
    """Median components of the wall captures at the given distances, per resolution and kind."""
    levels = {}
    for mode in sorted({f["mode"] for f in fits}):
        rows = [f for f in fits if f["mode"] == mode and f["surface"] == "wall" and f["distance_m"] in distances]
        if not rows:
            continue
        for kind, name in (("temporal", "temporal"), ("frame", "frame")):
            comps = []
            for part in ("short", "long"):
                ratio = [source_fx / r["fx"] for r in rows]
                comps.append({"k": float(np.median([r[kind][part][0] for r in rows])),
                              "correlation_px_source": [float(np.median([r[kind][part][1][j] * q for r, q in zip(rows, ratio)]))
                                                        for j in (0, 1)]})
            levels[f"{name}_{mode}"] = {"k_iid": 0.0, "components": comps,
                                        "from": [r["label"] for r in rows]}
    return levels


def reproduction(level: dict, fit_row: dict, *, source_fx: float, frames: int = 5, seed: int = 0) -> dict:
    """Regenerate a flat plane at the capture's distance, size and fx with the level's components and
    compare the second-difference variances and the temporal tail with the capture's measurements."""
    z = fit_row["distance_m"]
    shape = fit_row["shape"]
    ratio = fit_row["fx"] / source_fx
    ratios_u, ratios_v, tails = [], [], []
    for i in range(frames):
        w = np.full(shape, 1.0 / z)
        for j, c in enumerate(level["components"]):
            g = [v * ratio for v in c["correlation_px_source"]]
            w = w + c["k"] * correlated_field(shape, g, np.random.default_rng([seed, i, j]))
        depth = 1.0 / w
        for axis, store, measured in ((1, ratios_u, fit_row["var_u"]), (0, ratios_v, fit_row["var_v"])):
            store.append([fit.robust_variance(fit.second_differences(depth, axis=axis, step=s, z_range=(0.5 * z, 1.5 * z)))
                          / measured[s] for s in fit.STEPS])
        tails.append(tail_fraction(w - 1.0 / z))
    return {"ratio_u": np.median(ratios_u, axis=0).tolist(), "ratio_v": np.median(ratios_v, axis=0).tolist(),
            "steps": list(fit.STEPS), "model_tail_4sigma": float(np.median(tails)),
            "measured_tail_4sigma": fit_row.get("temporal_tail_4sigma")}


def _files_sha256(folders) -> str:
    h = hashlib.sha256()
    for folder in sorted(folders):
        h.update((folder / "capture.json").read_bytes())
    return h.hexdigest()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--captures", type=Path, required=True)
    ap.add_argument("--report", type=Path, required=True)
    ap.add_argument("--config", type=Path, required=True)
    ap.add_argument("--source-fx", type=float, default=None, help="default: the 640 mode's fx")
    a = ap.parse_args(argv)
    import yaml

    folders = sorted(p for p in a.captures.iterdir() if (p / "capture.json").is_file())
    walls = [p for p in folders if json.loads((p / "capture.json").read_text())["setup"]["surface"] == "wall"]
    fits = []
    for p in walls:
        row = capture_fit(p)
        stack, _ = load_capture(p)
        row["shape"] = list(stack.shape[1:])
        # per-step variances of the median frame fit, for the reproduction check
        d = dither(stack, 0)
        dist = row["distance_m"]
        t = temporal_frames(d, dist)
        f0 = fit_frame(t[0], (0.5 * dist, 1.5 * dist))
        row["var_u"], row["var_v"] = f0["var_u"], f0["var_v"]
        fits.append(row)
    source_fx = a.source_fx or next(f["fx"] for f in fits if f["mode"] == 640)
    levels = build_levels(fits, source_fx=source_fx)
    repro = {}
    for name, level in levels.items():
        mode = int(name.split("_")[1])
        for row in (f for f in fits if f["mode"] == mode and f["distance_m"] in (2.0, 3.0)):
            repro[f"{name}@{row['label']}"] = reproduction(level, row, source_fx=source_fx)
    surfaces = {}
    for p in folders:
        meta = json.loads((p / "capture.json").read_text())
        stack, _ = load_capture(p)
        surfaces[meta["setup"]["label"]] = surface_check(stack)
    report = {"captures": [{k: v for k, v in f.items() if k not in ("var_u", "var_v")} for f in fits],
              "levels": levels, "reproduction": repro, "surfaces": surfaces, "source_fx": source_fx}
    a.report.write_text(json.dumps(report, indent=1) + "\n")
    config = {
        "source": f"team RealSense D435i capture {a.captures.name} (tools/capture_d435i_noise.py)",
        "license": "internal", "revision": a.captures.name, "files_sha256": _files_sha256(folders),
        "role": "provisional",
        "method": __doc__.split("\n\n")[1].replace("\n", " "),
        "source_fx": float(source_fx), "dropout_fraction": PROVISIONAL_DROPOUT,
        "provenance": {"components": "measured (wall captures at 2 and 3 m)",
                       "dropout_fraction": "provisional table (MR6D), kept so only the components change",
                       "registration_shift_px, flying_fraction, quantize_step_m": "model defaults, not measured"},
        "levels": {k: {kk: vv for kk, vv in v.items() if kk != "from"} for k, v in levels.items()},
        "levels_from": {k: v["from"] for k, v in levels.items()},
        "limitations": [
            "no reference flat: fixed spatial error and absolute bias are not separated; the frame levels are "
            "a scenario of one observed wall frame, not an upper bound",
            "a plane in inverse depth cancels in second differences and slow warps are attenuated: whole-frame "
            "tilt or shift and lens warps are not guaranteed to be represented",
            "one k for 2-3 m: the measured sigma grows slightly faster than z^2",
            "Gaussian components: the measured heavy tails are not reproduced (see the reproduction report)",
            "random dropout at the provisional rate: persistent invalid pixels are not reproduced",
            "static tripod, indoor fluorescent light, one team D435i",
        ],
    }
    a.config.write_text(yaml.safe_dump(config, sort_keys=False, allow_unicode=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
