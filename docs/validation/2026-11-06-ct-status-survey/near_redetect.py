"""CPU re-detection of the failed near capture (O6_random_m_s0 m2, attempt 2) with tools.diagnose_detection: replay the
detector on the saved depth, check it reproduces the recorded diagnostics, then join the truth pallet (P-5) to say
where its evidence went. Exploratory (lead 2026-11-06), Isaac-free."""
import json, sys
from pathlib import Path
import numpy as np

SNAP, RUN = Path(sys.argv[1]), Path(sys.argv[2])
sys.path.insert(0, str(SNAP / "src")); sys.path.insert(0, str(SNAP))
from tools import diagnose_detection as DD
from forklift_core.perception.pallet_prior import PalletPrior
from forklift_core.perception.pocket_detector import DetectorParams

result = json.loads((RUN / "result.json").read_text())
state = json.loads((RUN / "missions/m2/records.json").read_text())["state"]
attempt = state["observation_attempts"][1]
depth = np.load(RUN / "perception_capture_2_depth_m.npy")
assert DD.depth_array_sha256(depth) == attempt["depth_sha256"], "not the failed capture"
prior = PalletPrior(**result["arguments"]["pallet_prior_loaded"])
params = DetectorParams(**result["detector_params"])
scene = DD.attempt_scene(result, attempt, depth)
urdf = SNAP / "sim/models/epal6_pallet/pallet.urdf"
geometry = result["arguments"]["pallet_geometry_loaded"]
truth = DD.PalletTruth(0.953999936580658, -1.4620000123977661, -1.043e-07, -0.10472070767372293,
                       geometry["overall_depth_m"], DD.pallet_parts(urdf))
expected = DD.attempt_expected(attempt)
out = DD.diagnose(scene, prior, params, truth, attempt["capture_diagnostics"]["accepted_pose"],
                  lower_evidence=True, expected=expected)
rec = out["detection"]
print("REPLAY status", rec.get("observation", {}).get("status"), rec.get("observation", {}).get("reason"))
diag = rec.get("diagnostics") or {}
print("REPLAY reasons", diag.get("rejected_plane_reasons"), "recorded", attempt["detection_diagnostics"]["rejected_plane_reasons"])
print("REPLAY same as recorded:", diag.get("rejected_plane_reasons") == attempt["detection_diagnostics"]["rejected_plane_reasons"]
      and diag.get("candidate_plane_count") == attempt["detection_diagnostics"]["candidate_plane_count"])
print("FAMILY", json.dumps(out.get("family"))[:800])
print("VIS", json.dumps(out.get("visibility"))[:1500])
print("PRE", json.dumps(out.get("preprocessing"))[:1200])
for p in out.get("planes") or []:
    print("PLANE", json.dumps(p)[:1400])
print("SELECTION", json.dumps(out.get("selection"))[:600])
Path(sys.argv[3]).write_text(json.dumps(out, indent=1, default=str))
