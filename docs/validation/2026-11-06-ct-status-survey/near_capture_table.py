"""The near capture of every random_m second mission (P-5:D-2) across the survey's sessions, and every stall replan
event, from the mission records (docs/validation/2026-11-06-ct-status-survey.md). Isaac-free.

    python near_capture_table.py SURVEY_OUT > near_capture.txt"""
import glob
import json
import math
import sys
from collections import Counter

out = sys.argv[1]
print("session | near_capture | far_error_m | capture_stand_xy | pallet_xy | stand_to_pallet_m | candidate_planes | "
      "rejected_plane_reasons")
for f in sorted(glob.glob(f"{out}/*random_m*/live/epoch_1/run/missions/m2/records.json")):
    st = json.load(open(f))["state"]
    pe = st.get("perception") or {}
    cd = pe.get("capture_diagnostics") or {}
    dd = pe.get("detection_diagnostics") or {}
    stand = ((cd.get("accepted_pose") or [[None, None]])[0] or [None, None])[:2]
    pick = (st.get("mission_scenario") or {}).get("pickup") or {}
    dist = (math.hypot(pick["x_m"] - stand[0], pick["y_m"] - stand[1])
            if stand[0] is not None and "x_m" in pick else None)
    nc = st.get("near_capture") or {}
    print(f"{f.split('/')[-7]} | {nc.get('status')} | {(nc.get('far_error') or {}).get('position_m'):.4f} | "
          f"({stand[0]:.3f}, {stand[1]:.3f}) | ({pick.get('x_m'):.3f}, {pick.get('y_m'):.3f}) | {dist:.3f} | "
          f"{dd.get('candidate_plane_count')} | {dict(Counter(dd.get('rejected_plane_reasons') or []))}")
print()
print("session | mission | phase | position_error_m | yaw_error_rad | status")
for f in sorted(glob.glob(f"{out}/*/live/epoch_1/run/missions/*/records.json")):
    st = json.load(open(f))["state"]
    for e in st.get("stall_replans") or []:
        print(f"{f.split('/')[-7]} | {f.split('/')[-2]} | {e.get('phase')} | {e.get('position_error_m'):.4f} | "
              f"{e.get('yaw_error_rad'):.4f} | {e.get('status')}")
