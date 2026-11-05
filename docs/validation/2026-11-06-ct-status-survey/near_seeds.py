"""The failed near capture's frame re-detected with the RANSAC seed varied (everything else as recorded, expected site
and lower evidence as the mission used them, and the lower-waived narrow site too): how often does the same frame give
a valid pocket, and how far off the face are the planes? Exploratory, Isaac-free."""
import dataclasses, json, sys
from pathlib import Path
import numpy as np

SNAP, RUN, N = Path(sys.argv[1]), Path(sys.argv[2]), int(sys.argv[3])
sys.path.insert(0, str(SNAP / "src")); sys.path.insert(0, str(SNAP))
from tools import diagnose_detection as DD
from forklift_core.perception import pocket_detector as det
from forklift_core.perception.pallet_prior import PalletPrior
from forklift_core.perception.pocket_detector import DetectorParams

result = json.loads((RUN / "result.json").read_text())
attempt = json.loads((RUN / "missions/m2/records.json").read_text())["state"]["observation_attempts"][1]
depth = np.load(RUN / "perception_capture_2_depth_m.npy")
prior = PalletPrior(**result["arguments"]["pallet_prior_loaded"])
base = DetectorParams(**result["detector_params"])
scene = DD.attempt_scene(result, attempt, depth)
expected = DD.attempt_expected(attempt)
narrow = det.ExpectedSite(expected.x_m, expected.y_m, expected.yaw_rad, 0.15, expected.axis_tolerance_rad)
print("recorded seed", base.seed, "iterations", base.ransac_iterations, "inlier", base.plane_inlier_m)
geometry = result["arguments"]["pallet_geometry_loaded"]
truth = DD.PalletTruth(0.953999936580658, -1.4620000123977661, -1.043e-07, -0.10472070767372293,
                       geometry["overall_depth_m"], DD.pallet_parts(SNAP / "sim/models/epal6_pallet/pallet.urdf"))
_, _, front_point, front_normal = DD.truth_in_base(truth, attempt["capture_diagnostics"]["accepted_pose"])
import math
face_yaw = math.atan2(-front_normal[1], -front_normal[0])  # the pallet's +x (insertion) direction in base

def pose_error(obs):
    """Face centre (mean of the two pocket centres) against the truth front-face centre: along, lateral (in the face
    frame) and insertion yaw error."""
    c = (np.asarray(obs.left.center_m) + np.asarray(obs.right.center_m)) / 2
    d = c[:2] - np.asarray(front_point[:2])
    ca, sa = math.cos(face_yaw), math.sin(face_yaw)
    along, lateral = ca * d[0] + sa * d[1], -sa * d[0] + ca * d[1]
    yaw = math.atan2(math.sin(obs.insertion_yaw_rad - face_yaw), math.cos(obs.insertion_yaw_rad - face_yaw))
    return along, lateral, yaw
errs = []
ok, ok_w, reasons = 0, 0, {}
for s in range(N):
    p = dataclasses.replace(base, seed=base.seed + s)
    r, _ = det.detect_pockets_expected(scene, prior, p, expected, lower_evidence=True)
    v = r.observation.status == "valid"
    ok += v
    if v:
        errs.append(pose_error(r.observation))
    if not v:
        r2, _ = det.detect_pockets_expected(scene, prior, p, narrow, lower_evidence=False)
        ok_w += r2.observation.status == "valid"
    reasons[r.observation.reason or "valid"] = reasons.get(r.observation.reason or "valid", 0) + 1
print(f"seeds {N}: valid {ok}, rescued by the lower waiver {ok_w}, outcomes {reasons}")
e = np.array(errs)
if len(e):
    print("valid seeds' face-centre error (m, m, rad): along p50/p95/max", np.round(np.percentile(np.abs(e[:, 0]), [50, 95, 100]), 4),
          "lateral", np.round(np.percentile(np.abs(e[:, 1]), [50, 95, 100]), 4), "yaw", np.round(np.percentile(np.abs(e[:, 2]), [50, 95, 100]), 4))
for it in (400, 800):
    p = dataclasses.replace(base, ransac_iterations=it)
    r, _ = det.detect_pockets_expected(scene, prior, p, expected, lower_evidence=True)
    print(f"recorded seed with {it} iterations: {r.observation.status} {r.observation.reason}",
          np.round(pose_error(r.observation), 4) if r.observation.status == "valid" else "")
