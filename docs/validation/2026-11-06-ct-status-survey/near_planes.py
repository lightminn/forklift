"""Where did the right block's evidence go? For the failed near capture: label workspace points by pallet part and by
named box, then for the face planes 0 and 5 count, per named front-face box, the points inside the plane's inlier band
and inside the opening-height band, their signed distance to the plane, and the plane's column grid. Exploratory."""
import json, math, sys
from pathlib import Path
import numpy as np

SNAP, RUN = Path(sys.argv[1]), Path(sys.argv[2])
sys.path.insert(0, str(SNAP / "src")); sys.path.insert(0, str(SNAP))
from tools import diagnose_detection as DD
from forklift_core.perception import pocket_detector as det
from forklift_core.perception.pallet_prior import PalletPrior
from forklift_core.perception.pocket_detector import DetectorParams

result = json.loads((RUN / "result.json").read_text())
attempt = json.loads((RUN / "missions/m2/records.json").read_text())["state"]["observation_attempts"][1]
depth = np.load(RUN / "perception_capture_2_depth_m.npy")
prior = PalletPrior(**result["arguments"]["pallet_prior_loaded"])
params = DetectorParams(**result["detector_params"])
scene = DD.attempt_scene(result, attempt, depth)
geometry = result["arguments"]["pallet_geometry_loaded"]
truth = DD.PalletTruth(0.953999936580658, -1.4620000123977661, -1.043e-07, -0.10472070767372293,
                       geometry["overall_depth_m"], DD.pallet_parts(SNAP / "sim/models/epal6_pallet/pallet.urdf"))
boxes, names, front_point, front_normal = DD.truth_in_base(truth, attempt["capture_diagnostics"]["accepted_pose"])
points, rays = det._base_points(scene)
camera = np.asarray(scene.base_from_optical.translation_m)
ws = det._filter_workspace(points, camera, prior, params)
planes = det._vertical_plane_candidates(ws, camera, params)
print("prior band z:", prior.deck_bottom_m + params.band_margin_m, prior.height_m - prior.deck_top_m - params.band_margin_m,
      "cell", params.cell_m, "inlier", params.plane_inlier_m, "min_band", params.min_band_points)
# per named box membership of workspace points
member = {n: DD._inside(ws, b, DD.PART_MARGIN_M) for b, n in zip(boxes, names)}
front_boxes = [n for n in names if "block" in n or "bottom" in n or "top" in n]
print("named boxes:", names)
for k in (0, 5):
    p = planes[k]
    d = (ws - p.point) @ p.normal
    inl = np.abs(d) <= params.plane_inlier_m
    band = (ws[:, 2] >= prior.deck_bottom_m + params.band_margin_m) & (ws[:, 2] <= prior.height_m - prior.deck_top_m - params.band_margin_m)
    lat = ws @ p.left_axis
    print(f"== plane {k}: normal {np.round(p.normal,4)} point {np.round(p.point,3)} inliers {inl.sum()} residual_p95 {p.residual_p95_m:.4f}")
    for n in names:
        m = member[n]
        if m.sum() == 0:
            continue
        dm = d[m]
        print(f"  {n:28s} ws {m.sum():4d} | in-band {np.count_nonzero(m & band):4d} | inlier {np.count_nonzero(m & inl):4d} | inlier&band {np.count_nonzero(m & inl & band):4d} "
              f"| dist median {np.median(dm):+.4f} p05 {np.percentile(dm,5):+.4f} p95 {np.percentile(dm,95):+.4f} | lateral {np.min(lat[m]):+.3f}..{np.max(lat[m]):+.3f}")
    # column grid
    local = np.column_stack((np.zeros(inl.sum()), lat[inl], ws[inl][:, 2]))
    counts, origin = det._column_grid(local, prior, params)
    occ = [(round((origin + i) * params.cell_m, 2), int(c)) for i, c in enumerate(counts)]
    print("  columns (lateral m, count) nonzero:", [o for o in occ if o[1] > 0][:12], "...", [o for o in occ if o[1] > 0][-12:])
    print("  columns lateral range", occ[0][0], occ[-1][0])
