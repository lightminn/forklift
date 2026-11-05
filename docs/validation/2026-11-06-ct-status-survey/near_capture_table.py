"""The near capture attempt of every random_m second mission (P-5:D-2) across the survey's sessions, and every stall
replan event, from the mission records (docs/validation/2026-11-06-ct-status-survey.md). Isaac-free.

The near attempt is the mission's last entry in ``observation_attempts`` (the first is the far observation);
``state["perception"]`` is updated only by a successful observation, so it is not read here.

    python near_capture_table.py SURVEY_OUT > near_capture.txt"""
import glob
import json
import sys
from collections import Counter

out = sys.argv[1]
print("session | near_capture | attempt | arrived_xy_yaw | pocket_status | reason | plane_inliers | candidate_planes | "
      "rejected_plane_reasons | far_error_m")
for f in sorted(glob.glob(f"{out}/*random_m*/live/epoch_1/run/missions/m2/records.json")):
    st = json.load(open(f))["state"]
    attempts = st.get("observation_attempts") or []
    a = attempts[-1] if attempts else {}
    po = a.get("pocket_observation") or {}
    dd = a.get("detection_diagnostics") or {}
    ap = a.get("arrived_pose") or [None, None, None]
    nc = st.get("near_capture") or {}
    print(f"{f.split('/')[-7]} | {nc.get('status')} | {len(attempts) - 1} | ({ap[0]:.3f}, {ap[1]:.3f}, {ap[2]:.3f}) | "
          f"{po.get('status')} | {po.get('reason')} | {dd.get('plane_inlier_count')} | {dd.get('candidate_plane_count')} | "
          f"{dict(Counter(dd.get('rejected_plane_reasons') or []))} | {(nc.get('far_error') or {}).get('position_m'):.4f}")
print()
print("session | mission | phase | position_error_m | yaw_error_rad | status")
for f in sorted(glob.glob(f"{out}/*/live/epoch_1/run/missions/*/records.json")):
    st = json.load(open(f))["state"]
    for e in st.get("stall_replans") or []:
        print(f"{f.split('/')[-7]} | {f.split('/')[-2]} | {e.get('phase')} | {e.get('position_error_m'):.4f} | "
              f"{e.get('yaw_error_rad'):.4f} | {e.get('status')}")
