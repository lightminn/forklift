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
METHOD = ("per-frame robust variogram of inverse-depth second differences on background pixels (margin_px from "
          "every visible object mask and invalid pixel); V(s) = 6A + B(6 - 8 rho(s) + 2 rho(2s)), "
          "rho = exp(-s^2/(4 g^2)); adopted values are medians over accepted frames; cross-checked against "
          "48 px patch plane residuals")
LIMITATIONS = [
    "MR6D depth is registered to the 1280x720 colour grid; its native depth mode and RealSense post-processing "
    "(spatial, temporal, hole-filling filters) are not recorded, and a spatial filter alone produces correlated error",
    "the robot's depth mode is not chosen and the target K (fx 465.74) is synthetic; the x1.66 sensitivity variant "
    "scales k and g for a 640x480 native mode and is an approximation, not a bound",
    "fitted on indoor background surfaces; the patch check ties the model to planar patches, and pallet surfaces "
    "are compared in pallet_surface (reported only)",
    "beyond the measured depth bands the k z^2 law is assumed",
    "a fresh correlated field is drawn per scene; any fixed-pattern part of the error is not reproduced, so gains "
    "from multi-frame fusion would be overstated",
    "dropout is modelled as independent pixels at the background rate; structured holes on pallets (grazing top "
    "boards, occlusion shadows) are reported in pallet_surface, not modelled",
    "edge mixing (flying pixels), quantisation and RGB registration keep the model's assumed defaults",
]


def _get(url: str) -> bytes:
    with urllib.request.urlopen(url, timeout=120) as response:
        return response.read()


def snapshot_listing(cache: Path) -> list:
    path = cache / "listing.json"
    if not path.exists():
        info = json.loads(_get(API))
        if info.get("sha") != REVISION:
            raise ValueError(f"dataset API returned revision {info.get('sha')}, expected {REVISION}")
        cache.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(sorted(s["rfilename"] for s in info["siblings"])))
    return json.loads(path.read_text())


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


def frame_structure(depth, masks, *, z_range, margin_px) -> dict:
    background = fit.background_mask(depth, masks, margin_px=margin_px)
    objects = np.any(masks, axis=0) if masks else np.zeros(depth.shape, bool)
    record = {"dropout": fit.dropout_fraction(depth, ~objects),
              "depth_m": [float(v) for v in np.nanpercentile(np.where(background, depth, np.nan), [5, 50, 95])]}
    try:
        var = _variances(depth, background, z_range)
    except ValueError as exc:
        return {**record, "reject": "samples", "detail": str(exc)}
    s = fit.fit_noise_structure(var[1], var[0])
    patches = fit.patch_plane_rms(depth, background, z_range=z_range)
    record.update(k_iid=s.k_iid, k_corr=s.k_corr, g=list(s.correlation_px), rel_residual_rms=s.rel_residual_rms,
                  patch_median=float(np.median(patches)) if patches.size else None, patches=int(patches.size))
    bands = {}
    for lo, hi in BANDS:
        try:
            b = _variances(depth, background, (lo, hi))
        except ValueError:
            continue
        bs = fit.fit_noise_structure(b[1], b[0])
        bands[f"{lo}-{hi}"] = {"k_iid": bs.k_iid, "k_corr": bs.k_corr, "g": list(bs.correlation_px)}
    record["bands"] = bands
    if patches.size < MIN_PATCHES:
        record["reject"] = "patches"
    elif s.at_grid_edge:
        record["reject"] = "grid_edge"
    elif s.rel_residual_rms > 0.15:
        record["reject"] = "residual"
    return record


def _dist(values):
    v = np.asarray(values, dtype=float)
    if not v.size:
        return {"n": 0}
    return {"n": int(v.size), "p10": float(np.percentile(v, 10)), "p50": float(np.median(v)),
            "p90": float(np.percentile(v, 90))}


def calibrate(cache: Path, frames, *, z_range=(1.5, 4.0), margin_px: int = 8, accept_fraction: float = 0.75) -> dict:
    records, files, grid = [], [], None
    for f in frames:
        depth, masks, k = _load(cache, f)
        records.append(frame_structure(depth, masks, z_range=z_range, margin_px=margin_px))
        files += [f["depth"], *f["masks"]]
        grid = {"width": int(depth.shape[1]), "height": int(depth.shape[0]), "fx": float(k[0, 0]),
                "fy": float(k[1, 1])}
    accepted = [r for r in records if "reject" not in r]
    if len(accepted) < accept_fraction * len(records):
        raise ValueError(f"only {len(accepted)}/{len(records)} frames accepted")
    k_iid = float(np.median([r["k_iid"] for r in accepted]))
    k_corr = float(np.median([r["k_corr"] for r in accepted]))
    g = [float(np.median([r["g"][j] for r in accepted])) for j in (0, 1)]
    model = fit.model_patch_rms(fit.NoiseStructure(k_iid, k_corr, tuple(g), 0.0, False),
                                size=(grid["height"], grid["width"]))
    ratios = [r["patch_median"] / model for r in accepted if r["patch_median"] is not None]
    if not ratios or not 0.67 <= float(np.median(ratios)) <= 1.5:
        raise ValueError(f"patch check failed: observed/model ratios {_dist(ratios)}")
    bands = {}
    for lo, hi in BANDS:
        key = f"{lo}-{hi}"
        rows = [r["bands"][key] for r in accepted if key in r["bands"]]
        bands[key] = {"k_iid": _dist([b["k_iid"] for b in rows]), "k_corr": _dist([b["k_corr"] for b in rows]),
                      "g_u": _dist([b["g"][0] for b in rows])}
    scenes = sorted({f["scene"] for f in frames})
    files = ["models_names.json", *[f"{s}/scene_camera.json" for s in scenes], *files]
    digest = hashlib.sha256()
    for rel in files:
        digest.update(rel.encode())
        digest.update(hashlib.sha256((cache / rel).read_bytes()).digest())
    return {
        "source": f"Hugging Face {DATASET} (MR6D, Intel RealSense D435i)", "license": "CC-BY-4.0",
        "revision": REVISION, "retrieved": datetime.date.today().isoformat(), "method": METHOD,
        "calibration_split": scenes, "files": files, "files_sha256": digest.hexdigest(),
        "source_grid": grid, "source_fx": grid["fx"], "z_range_m": [float(v) for v in z_range],
        "margin_px": margin_px, "steps": list(fit.STEPS),
        "k_iid": k_iid, "k_corr": k_corr, "correlation_px_source": g,
        "per_frame": {"accepted": len(accepted), "total": len(records),
                      "k_iid": _dist([r["k_iid"] for r in accepted]), "k_corr": _dist([r["k_corr"] for r in accepted]),
                      "g_u": _dist([r["g"][0] for r in accepted]), "g_v": _dist([r["g"][1] for r in accepted])},
        "rejected": {reason: sum(r.get("reject") == reason for r in records)
                     for reason in ("samples", "patches", "grid_edge", "residual")},
        "band_fits": bands,
        "patch_check": {"model_median": model, "ratio": _dist(ratios), "ratio_median": float(np.median(ratios))},
        "dropout_fraction": float(np.median([r["dropout"] for r in records])),
        "background_depth_m": _dist([r["depth_m"][1] for r in records]),
        "frames": len(records), "limitations": LIMITATIONS,
    }


def _pallet_ids(cache: Path) -> set:
    names = json.loads((cache / "models_names.json").read_text())
    return {int(k) for k, v in names.items() if "pallet" in str(v).lower()}


def pallet_surface_report(cache: Path, frames, *, z_range=(0.5, 2.5), margin_px: int = 8) -> dict:
    ids = _pallet_ids(cache)
    ratios_u, ratios_v = {s: [] for s in PALLET_STEPS}, {s: [] for s in PALLET_STEPS}
    drops, touched = [], set()
    for f in frames:
        gts = json.loads((cache / f["scene"] / "scene_gt.json").read_text())[str(f["frame"])]
        depth, masks, _ = _load(cache, f)
        pallets = [m for m, gt in zip(masks, gts) if int(gt["obj_id"]) in ids]
        if not pallets:
            continue
        touched.add(f["scene"])
        region = np.any(pallets, axis=0)
        drops.append(fit.dropout_fraction(depth, region))
        try:
            pal = _variances(depth, fit.erode(region, 4), z_range, PALLET_STEPS)
            bg = _variances(depth, fit.background_mask(depth, masks, margin_px=margin_px), z_range, PALLET_STEPS)
        except ValueError:
            continue
        for s in PALLET_STEPS:
            ratios_u[s].append(float(np.sqrt(pal[1][s] / bg[1][s])))
            ratios_v[s].append(float(np.sqrt(pal[0][s] / bg[0][s])))
    return {"scenes": sorted(touched), "pallet_dropout_fraction": float(np.median(drops)) if drops else None,
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
    listing = json.loads((a.cache / "listing.json").read_text()) if a.no_download else snapshot_listing(a.cache)
    frames = select_frames(listing, [s for s in a.calibration_scenes.split(",") if s], a.stride)
    pallet_frames = select_frames(listing, [s for s in a.pallet_scenes.split(",") if s], a.pallet_stride)
    if not a.no_download:
        download(a.cache, frames + pallet_frames, listing)
    result = calibrate(a.cache, frames, z_range=tuple(float(v) for v in a.z_range.split(",")))
    pallet = pallet_surface_report(a.cache, pallet_frames) if pallet_frames else {"scenes": []}
    result["pallet_surface"] = pallet
    result["calibration_touched"] = pallet["scenes"]
    a.out.write_text(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
    print(json.dumps({k: result[k] for k in ("k_iid", "k_corr", "correlation_px_source", "dropout_fraction",
                                             "frames")} | {"accepted": result["per_frame"]["accepted"]}))


if __name__ == "__main__":
    main()
