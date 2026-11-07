#!/usr/bin/env python3
"""Shape or sensor: the pallet's front measured the same way at every distance, in the levelled 30-frame median.

If an edge is the object's, it stays at the same height or width in metres as the camera backs off; if it is the depth's
matching window, it stays at about the same size in pixels, so it grows in metres with distance. For each capture:
  deck_top_z_m        p99 of the front's height
  legs                the runs of occupied 1 cm columns of the front layer at z 15-40 mm, in metres and in pixels at that
                      distance (fx / x); the gaps between them; the middle of the outer legs (the pallet's centre)
  opening_rays        for the pixels whose ray crosses the front plane in the openings' middle columns, per 5 mm of
                      height on that plane, the share that returned a point in the front layer: ``share`` counts every
                      such ray, a ray that went through to the floor or behind included. ``first_half_z_m`` is where
                      that share first reaches one half going up (the apparent underside); ``first_tail_z_m`` where it
                      first reaches 2 % (a column needs a single point to count as occupied)
  front_x_m           the median x of the deck edge (z >= 72 mm) and of the legs (z 15-55 mm, leg columns); the difference
                      is the legs' setback, to compare with the plane inlier
Also the first frame's plane candidates of toy_1p00m_848 under E4's call.

  PYTHONPATH=<repo>/src python e4_profile.py DATA_DIR --json e4_profile.json
"""

import argparse
import json
import sys
import warnings
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
import e4_diagnose as E  # noqa: E402

N, det = E.N, E.det
CELL_M = 0.01
LAYER_M = 0.04


def median_scene(capture: Path, meta: dict):
    frames = [np.asarray(Image.open(q)) for q in sorted(capture.glob("z16_*.png"))]
    scenes = [N.toy_input(meta, f)[0] for f in frames]
    roi = meta["setup"]["roi"]
    fits = []
    for scene in scenes[::5]:
        grid = E.base_grid(scene).copy()
        grid[roi[1]:roi[3], roi[0]:roi[2]] = np.nan
        q = grid.reshape(-1, 3)
        fits.append(E.floor_fit(q[np.isfinite(q).all(axis=1)]))
    fit = tuple(float(np.mean([f[i] for f in fits])) for i in range(3))
    stack = np.stack(frames).astype(float)
    stack[stack == 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        median = np.nan_to_num(np.nanmedian(stack, axis=0)).astype(np.uint16)
    return E.levelled(N.toy_input(meta, median)[0], fit), scenes[0]


def runs(columns: np.ndarray, edges: np.ndarray) -> list:
    out, start = [], None
    for i, on in enumerate(columns):
        if on and start is None:
            start = i
        if not on and start is not None:
            out.append((float(edges[start]), float(edges[i])))
            start = None
    if start is not None:
        out.append((float(edges[start]), float(edges[len(columns)])))
    return out


def profile(scene, meta: dict, name: str) -> dict:
    distance = float(meta["setup"]["distance_m"])
    x0, y0, x1, y1 = meta["setup"]["roi"]
    p = E.base_grid(scene)[y0:y1, x0:x1].reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1)]
    p = p[(p[:, 0] < distance + 0.5) & (p[:, 2] > 0.002)]
    front = float(np.percentile(p[p[:, 2] > 0.006, 0], 2))
    layer = p[p[:, 0] < front + LAYER_M]
    fx = float(scene.intrinsics.fx)
    out = {"capture": name, "distance_m": distance, "front_m": front, "deck_top_z_m": float(np.percentile(layer[:, 2], 99)),
           "metres_per_pixel": front / fx}
    edges = np.arange(-0.34, 0.3401, CELL_M)
    low = layer[(layer[:, 2] >= 0.015) & (layer[:, 2] < 0.04)]
    counts = np.histogram(low[:, 1], bins=edges)[0]
    legs = [r for r in runs(counts >= 2, edges) if r[1] - r[0] >= 0.02]
    out["legs_m"] = legs
    if len(legs) < 3:
        return out
    legs = legs[:3] if len(legs) == 3 else sorted(legs, key=lambda r: r[0] - r[1])[:3]
    legs = sorted(legs)
    out["leg_widths_m"] = [r[1] - r[0] for r in legs]
    out["leg_widths_px"] = [(r[1] - r[0]) * fx / front for r in legs]
    out["gaps_m"] = [legs[1][0] - legs[0][1], legs[2][0] - legs[1][1]]
    out["leg_spacing_m"] = [(legs[i + 1][0] + legs[i + 1][1] - legs[i][0] - legs[i][1]) / 2 for i in range(2)]
    out["centre_y_m"] = (legs[0][0] + legs[2][1]) / 2
    # the apparent underside: going up in the openings' middle, where the front layer starts
    middles = [(legs[0][1] + legs[1][0]) / 2, (legs[1][1] + legs[2][0]) / 2]
    half = min(out["gaps_m"]) * 0.25
    points, rays = det._base_points(scene)
    camera = scene.base_from_optical.translation_m
    h, w = scene.depth_m.shape
    index = np.zeros((h, w), bool)
    index[y0:y1, x0:x1] = True
    pts, ray = points[index.ravel()], rays[index.ravel()]
    plane_x = front + 0.01  # the front layer's middle ground: the deck edge at 0.5 m
    t = (plane_x - camera[0]) / ray[:, 0]
    crossing = camera + t[:, None] * ray
    in_opening = (np.abs(crossing[:, 1] - middles[0]) < half) | (np.abs(crossing[:, 1] - middles[1]) < half)
    on_front = np.isfinite(pts).all(axis=1) & (pts[:, 0] < front + LAYER_M)
    share, rays_by_z = {}, {}
    for low_z in np.arange(0.02, 0.095, 0.005):
        band = in_opening & (crossing[:, 2] >= low_z) & (crossing[:, 2] < low_z + 0.005)
        if band.sum() >= 5:
            share[round(float(low_z), 3)] = float(on_front[band].mean())
            rays_by_z[round(float(low_z), 3)] = int(band.sum())
    out["opening_rays"] = {"share": share, "rays": rays_by_z}
    out["first_half_z_m"] = next((z for z in sorted(share) if share[z] >= 0.5), None)
    out["first_tail_z_m"] = next((z for z in sorted(share) if share[z] >= 0.02), None)
    deck = layer[layer[:, 2] >= 0.072]
    in_leg = np.zeros(len(layer), bool)
    for a, b in legs:
        in_leg |= (layer[:, 1] >= a + 0.01) & (layer[:, 1] < b - 0.01)
    leg = layer[in_leg & (layer[:, 2] >= 0.015) & (layer[:, 2] < 0.055)]
    out["front_x_m"] = {"deck_edge": float(np.median(deck[:, 0])), "legs": float(np.median(leg[:, 0])) if len(leg) else None}
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("data", type=Path)
    p.add_argument("--json", type=Path, required=True)
    args = p.parse_args()
    rows, planes = [], []
    for capture in sorted(q for q in args.data.iterdir() if q.is_dir() and q.name.startswith("toy")):
        meta = json.loads((capture / "capture.json").read_text())
        scene, first = median_scene(capture, meta)
        rows.append(profile(scene, meta, capture.name))
        if capture.name == "toy_1p00m_848":
            prior = N.TOY_PRIORS["toy"]
            params = E.DetectorParams.derived_for(prior)
            camera = first.base_from_optical.translation_m
            pts, _ = det._base_points(first)
            for plane in det._vertical_plane_candidates(det._filter_workspace(pts, camera, prior, params), camera, params):
                planes.append({"distance_m": float(abs((camera - plane.point) @ plane.normal)), "points": len(plane.points)})
    args.json.write_text(json.dumps({"captures": rows, "first_frame_planes_toy_1p00m_848": planes}, indent=1) + "\n")


if __name__ == "__main__":
    main()
