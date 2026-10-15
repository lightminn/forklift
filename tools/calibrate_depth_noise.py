#!/usr/bin/env python3
"""Measure D435i depth-noise structure on MR6D (CC-BY-4.0) and write the noise config.

The calibration uses MR6D's `dynamic` split, which the project does not evaluate on;
two val scenes with pallets give a reported (not adopted) pallet-surface comparison
and are listed as touched. Frames are chosen from a pinned listing, downloaded from
the pinned revision into a cache outside the repository, and treated as untrusted
data (Pillow for PNG, json for JSON).
"""

import argparse
import datetime
import hashlib
import json
import sys
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import yaml
from PIL import Image

from forklift_core.perception import depth_noise_fit as fit

DATASET = "anas-gouda/mr6d"
REVISION = "3c4ed8453312655499c3d3a3cf38b4929fae77ee"
BASE = f"https://huggingface.co/datasets/{DATASET}/resolve/{REVISION}/"
API = f"https://huggingface.co/api/datasets/{DATASET}/revision/{REVISION}"
CALIBRATION_SCENES = tuple(f"dynamic/{i:06d}" for i in range(40))
PALLET_SCENES = ("val/000000", "val/000005")
BANDS = ((1.5, 2.5), (2.5, 4.0), (4.0, 6.0))
PALLET_STEPS = (1, 2, 3, 4, 6, 8)
MIN_PATCHES = 20
DIAGNOSTIC_STEPS = (48, 64)
DITHER_STEP_M = 0.001  # MR6D depth is whole millimetres (depth_scale 1.0)
METHOD = ("per-frame two-component fit (depth_noise_fit.fit_noise_components) of the robust variogram of "
          "inverse-depth second differences, steps 1-32, on background pixels (margin_px from every visible object "
          "mask and invalid pixel) after a seeded +-0.5 mm dither of the whole-millimetre depth; components sorted "
          "by width into short and long; no acceptance gate -- the distribution over usable frames is reported and "
          "the levels are its percentiles")
HISTORY = [
    "2026-10-15 plan v2 method (single correlated component, frames accepted only at fit residual <= 0.15): UBAI "
    "job 1012969 stopped on a frame whose MAD was 0 (over half the 1 px second differences of whole-millimetre "
    "depth are exactly 0); run 20261015T144823Z (zero-variance frames rejected) accepted 0/240 frames; with the "
    "dither the single-component residual was 0.50-0.55 on every probed frame",
    "Codex 2026-10-16: a gate on steps 4-32 passed a model that underpredicts vertical step 1 by 91 %; patch "
    "plane residuals of the same image cannot separate sensor error from surface shape; dither plus MAD is not "
    "additive -- so these values are a model of MR6D background inverse-depth variation, not a sensor measurement",
    "user decision 2026-10-16: provisional low/mid/high levels for the stage 1B baseline; the official noise "
    "population waits for a team D435i capture",
]
LIMITATIONS = [
    "a model of MR6D background inverse-depth variation: second differences and patch residuals cannot tell "
    "sensor error from surface shape, so the long component may include real surface relief",
    "MR6D depth is registered to the 1280x720 colour grid; its native depth mode and RealSense post-processing "
    "(spatial, temporal, hole-filling filters) are not recorded, and a spatial filter alone produces correlated error",
    "the robot's depth mode is not chosen and the target K (fx 465.74) is synthetic; widths are converted by the "
    "focal-length ratio, assuming the correlation is fixed in angle",
    "the +-0.5 mm dither adds its own small variance and the MAD scale assumes a Gaussian; neither is corrected",
    "a fresh field is drawn per scene: any fixed-pattern part of the error is not reproduced, so gains from "
    "multi-frame fusion would be overstated",
    "dropout is modelled as independent pixels at the background rate; structured holes on pallets (grazing top "
    "boards, occlusion shadows) are reported in pallet_surface, not modelled",
    "edge mixing (flying pixels), quantisation and RGB registration keep the model's assumed defaults",
]


def _get(url: str) -> bytes:
    with urllib.request.urlopen(url, timeout=120) as response:
        return response.read()


def read_listing(cache: Path) -> list:
    obj = json.loads((cache / "listing.json").read_text())
    if not isinstance(obj, dict) or obj.get("revision") != REVISION:
        raise ValueError(f"cache listing is not from revision {REVISION}")
    return obj["files"]


def snapshot_listing(cache: Path) -> list:
    path = cache / "listing.json"
    if not path.exists():
        info = json.loads(_get(API))
        if info.get("sha") != REVISION:
            raise ValueError(f"dataset API returned revision {info.get('sha')}, expected {REVISION}")
        cache.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps({"revision": REVISION,
                                    "files": sorted(s["rfilename"] for s in info["siblings"])}))
    return read_listing(cache)


def select_frames(listing, scenes, stride: int) -> list:
    frames = []
    for scene in scenes:
        depths = sorted(n for n in listing if n.startswith(f"{scene}/depth/") and n.endswith(".png"))
        for rel in depths[::stride]:
            stem = Path(rel).stem
            frames.append({"scene": scene, "frame": int(stem), "depth": rel,
                           "masks": sorted(n for n in listing if n.startswith(f"{scene}/mask_visib/{stem}_"))})
    return frames


def download(cache: Path, frames, listing) -> None:
    wanted = {"models_names.json"}
    for f in frames:
        wanted |= {f"{f['scene']}/scene_camera.json", f["depth"], *f["masks"]}
        if f"{f['scene']}/scene_gt.json" in listing:
            wanted.add(f"{f['scene']}/scene_gt.json")
    for rel in sorted(wanted):
        target = cache / rel
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(_get(BASE + rel))


def _load(cache: Path, f):
    cams = json.loads((cache / f["scene"] / "scene_camera.json").read_text())
    cam = cams[str(f["frame"])]
    raw = np.asarray(Image.open(cache / f["depth"]))
    if raw.dtype != np.uint16 or raw.ndim != 2:
        raise ValueError(f"{f['depth']} is not a single-channel uint16 depth image")
    depth = np.where(raw > 0, raw * float(cam.get("depth_scale", 1.0)) / 1000.0, np.nan)
    masks = [np.asarray(Image.open(cache / m)) > 0 for m in f["masks"]]
    return depth, masks, np.asarray(cam["cam_K"], dtype=float).reshape(3, 3)


def _variances(depth, valid, z_range, steps=fit.STEPS):
    return {axis: {s: fit.robust_variance(fit.second_differences(depth, axis=axis, step=s, z_range=z_range,
                                                                 valid=valid)) for s in steps}
            for axis in (1, 0)}


def dither(depth, rng, step_m: float = DITHER_STEP_M):
    """Untie whole-millimetre depth (otherwise over half the 1 px second differences are exactly 0)."""
    return np.where(np.isfinite(depth), depth + rng.uniform(-step_m / 2, step_m / 2, depth.shape), np.nan)


def frame_structure(depth, masks, *, z_range, margin_px, seed) -> dict:
    background = fit.background_mask(depth, masks, margin_px=margin_px)
    objects = np.any(masks, axis=0) if masks else np.zeros(depth.shape, bool)
    record = {"dropout": fit.dropout_fraction(depth, ~objects),
              "depth_m": [float(v) for v in np.nanpercentile(np.where(background, depth, np.nan), [5, 50, 95])]}
    depth = dither(depth, np.random.default_rng(seed))
    try:
        var = _variances(depth, background, z_range)
    except ValueError as exc:
        return {**record, "reject": "samples", "detail": str(exc)}
    if any(v <= 0 for axis in var.values() for v in axis.values()):
        return {**record, "reject": "degenerate"}
    patches = fit.patch_plane_rms(depth, background, z_range=z_range)
    if patches.size < MIN_PATCHES:
        return {**record, "reject": "patches"}
    s = fit.fit_noise_components(var[1], var[0])
    components = [[k, list(g)] for k, g in s.components]
    if len(components) == 1:
        components.insert(0, [0.0, [0.0, 0.0]])
    model = fit.model_patch_rms_components(0.0, s.components, size=depth.shape)
    record.update(components=components, rel_residual_rms=s.rel_residual_rms, ratio_u=s.ratio_u, ratio_v=s.ratio_v,
                  patch_ratio=float(np.median(patches)) / model, patches=int(patches.size))
    diag = {}
    for axis, name in ((1, "u"), (0, "v")):
        for step in DIAGNOSTIC_STEPS:
            d = fit.second_differences(depth, axis=axis, step=step, z_range=z_range, valid=background)
            if d.size >= 1000:
                predicted = sum(k * k * fit.component_response(step, g[0 if name == "u" else 1])
                                for k, g in s.components)
                diag[f"{name}{step}"] = predicted / fit.robust_variance(d)
    record["diagnostic_ratio"] = diag  # model / measured variance at steps the fit did not use
    bands = {}
    for lo, hi in BANDS:
        try:
            b = _variances(depth, background, (lo, hi))
        except ValueError:
            continue
        if any(v <= 0 for axis in b.values() for v in axis.values()):
            continue
        bs = fit.fit_noise_components(b[1], b[0])
        bands[f"{lo}-{hi}"] = [[k, list(g)] for k, g in bs.components]
    record["bands"] = bands
    return record


def _digest(cache: Path, files) -> str:
    digest = hashlib.sha256()
    for rel in files:
        digest.update(rel.encode())
        digest.update(hashlib.sha256((cache / rel).read_bytes()).digest())
    return digest.hexdigest()


def _dist(values):
    v = np.asarray(values, dtype=float)
    if not v.size:
        return {"n": 0}
    return {"n": int(v.size), "p10": float(np.percentile(v, 10)), "p50": float(np.median(v)),
            "p90": float(np.percentile(v, 90))}


def _component_dist(rows, j):
    return {"k": _dist([r["components"][j][0] for r in rows]), "g_u": _dist([r["components"][j][1][0] for r in rows]),
            "g_v": _dist([r["components"][j][1][1] for r in rows])}


def calibrate(cache: Path, frames, *, z_range=(1.5, 4.0), margin_px: int = 8) -> dict:
    records, files, grid = [], [], None
    for f in frames:
        depth, masks, k = _load(cache, f)
        seed = int(hashlib.sha256(f["depth"].encode()).hexdigest()[:8], 16)
        records.append(frame_structure(depth, masks, z_range=z_range, margin_px=margin_px, seed=seed))
        files += [f["depth"], *f["masks"]]
        grid = {"width": int(depth.shape[1]), "height": int(depth.shape[0]), "fx": float(k[0, 0]),
                "fy": float(k[1, 1])}
    usable = [r for r in records if "reject" not in r]
    if not usable:
        raise ValueError(f"no usable frame among {len(records)}")
    short, long = _component_dist(usable, 0), _component_dist(usable, 1)

    def component(dist, k_key):
        return {"k": dist["k"][k_key], "correlation_px_source": [dist["g_u"]["p50"], dist["g_v"]["p50"]]}

    levels = {name: {"k_iid": 0.0, "components": [component(short, "p50"), component(long, pct)]}
              for name, pct in (("low", "p10"), ("mid", "p50"), ("high", "p90"))}
    bands = {}
    for lo, hi in BANDS:
        key = f"{lo}-{hi}"
        rows = [{"components": r["bands"][key]} for r in usable if key in r["bands"] and len(r["bands"][key]) == 2]
        bands[key] = {"short": _component_dist(rows, 0), "long": _component_dist(rows, 1)} if rows else {"n": 0}
    steps = list(fit.STEPS)
    scenes = sorted({f["scene"] for f in frames})
    files = ["models_names.json", *[f"{s}/scene_camera.json" for s in scenes], *files]
    return {
        "source": f"Hugging Face {DATASET} (MR6D, Intel RealSense D435i)", "license": "CC-BY-4.0",
        "revision": REVISION, "role": "provisional", "retrieved": datetime.date.today().isoformat(),
        "method": METHOD, "history": HISTORY, "calibration_split": scenes, "files": files,
        "files_sha256": _digest(cache, files), "source_grid": grid, "source_fx": grid["fx"],
        "z_range_m": [float(v) for v in z_range], "margin_px": margin_px, "steps": steps,
        "levels": levels,
        "levels_note": ("artificial scenarios: the long component's k at the p10/p50/p90 of the usable frames "
                        "(every frame not rejected for samples, degenerate variance or patches) with the median "
                        "widths and the median short component; components are ordered by g_u + g_v. Not an 80 % "
                        "coverage, confidence interval or worst case: the joint variation of widths and amplitudes, "
                        "the short component's spread and the dither/MAD bias are not covered, and detection need "
                        "not change monotonically across the levels. k_iid is 0: the two components are the whole "
                        "correlated-plus-per-pixel model (a zero width is per-pixel noise)"),
        "frames": len(records), "usable": len(usable),
        "rejected": {reason: sum(r.get("reject") == reason for r in records)
                     for reason in ("samples", "degenerate", "patches")},
        "survey": {"short": short, "long": long, "rel_residual_rms": _dist([r["rel_residual_rms"] for r in usable]),
                   "ratio_u": {s: _dist([r["ratio_u"][i] for r in usable]) for i, s in enumerate(steps)},
                   "ratio_v": {s: _dist([r["ratio_v"][i] for r in usable]) for i, s in enumerate(steps)},
                   "diagnostic_ratio": {f"{a}{s}": _dist([r["diagnostic_ratio"][f"{a}{s}"] for r in usable
                                                          if f"{a}{s}" in r["diagnostic_ratio"]])
                                        for a in "uv" for s in DIAGNOSTIC_STEPS}},
        "band_fits": bands,
        "patch_check": {"ratio": _dist([r["patch_ratio"] for r in usable]),
                        "note": "observed / model median 48 px plane residual per frame; a consistency figure, "
                                "not an identification of sensor error"},
        "dropout_fraction": float(np.median([r["dropout"] for r in records])),
        "background_depth_m": _dist([r["depth_m"][1] for r in records]),
        "limitations": LIMITATIONS,
    }


def _pallet_ids(cache: Path) -> set:
    names = json.loads((cache / "models_names.json").read_text())
    return {int(k) for k, v in names.items() if "pallet" in str(v).lower()}


def pallet_surface_report(cache: Path, frames, *, z_range=(0.5, 2.5), margin_px: int = 8) -> dict:
    ids = _pallet_ids(cache)
    ratios_u, ratios_v = {s: [] for s in PALLET_STEPS}, {s: [] for s in PALLET_STEPS}
    drops, touched, files, used = [], set(), ["models_names.json"], 0
    for f in frames:
        gts = json.loads((cache / f["scene"] / "scene_gt.json").read_text())[str(f["frame"])]
        files += [f"{f['scene']}/scene_camera.json", f"{f['scene']}/scene_gt.json", f["depth"], *f["masks"]]
        depth, masks, _ = _load(cache, f)
        pallets = [m for m, gt in zip(masks, gts) if int(gt["obj_id"]) in ids]
        if not pallets:
            continue
        touched.add(f["scene"])
        region = np.any(pallets, axis=0)
        if not region.any():  # the pallet is listed for the frame but not visible in it
            continue
        used += 1
        drops.append(fit.dropout_fraction(depth, region))
        try:
            pal = _variances(depth, fit.erode(region, 4), z_range, PALLET_STEPS)
            bg = _variances(depth, fit.background_mask(depth, masks, margin_px=margin_px), z_range, PALLET_STEPS)
        except ValueError:
            continue
        if any(v <= 0 for d in (pal, bg) for axis in d.values() for v in axis.values()):
            continue
        for s in PALLET_STEPS:
            ratios_u[s].append(float(np.sqrt(pal[1][s] / bg[1][s])))
            ratios_v[s].append(float(np.sqrt(pal[0][s] / bg[0][s])))
    files = list(dict.fromkeys(files))
    return {"scenes": sorted(touched), "files": files, "files_sha256": _digest(cache, files),
            "pallet_frames": used, "pallet_dropout_fraction": float(np.median(drops)) if drops else None,
            "ratio_u": {s: _dist(v) for s, v in ratios_u.items()},
            "ratio_v": {s: _dist(v) for s, v in ratios_v.items()},
            "note": "pallet / background noise std at small steps; larger steps include gaps between boards"}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--cache", type=Path, default=Path.home() / ".cache/forklift/mr6d" / REVISION)
    p.add_argument("--calibration-scenes", default=",".join(CALIBRATION_SCENES))
    p.add_argument("--pallet-scenes", default=",".join(PALLET_SCENES))
    p.add_argument("--stride", type=int, default=50)
    p.add_argument("--pallet-stride", type=int, default=8)
    p.add_argument("--z-range", default="1.5,4.0")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--no-download", action="store_true")
    a = p.parse_args(argv)
    listing = read_listing(a.cache) if a.no_download else snapshot_listing(a.cache)
    frames = select_frames(listing, [s for s in a.calibration_scenes.split(",") if s], a.stride)
    pallet_frames = select_frames(listing, [s for s in a.pallet_scenes.split(",") if s], a.pallet_stride)
    if not a.no_download:
        download(a.cache, frames + pallet_frames, listing)
    result = calibrate(a.cache, frames, z_range=tuple(float(v) for v in a.z_range.split(",")))
    pallet = pallet_surface_report(a.cache, pallet_frames) if pallet_frames else {"scenes": []}
    result["pallet_surface"] = pallet
    result["calibration_touched"] = pallet["scenes"]
    a.out.write_text(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
    print(json.dumps({"levels": result["levels"], "frames": result["frames"], "usable": result["usable"],
                      "rejected": result["rejected"], "dropout_fraction": result["dropout_fraction"]}))


if __name__ == "__main__":
    main()
