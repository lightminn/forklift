"""A stand-in result.json for tools/low_clearance_report.py when a run lost its own (diagnosis only, labelled): scene,
geometry and chassis from a sibling run of the same scene and seed; samples (10 Hz) and phase transitions from the
run's frames.jsonl; held steering from its slam_log.npz. The frames' time is mapped to the mission time by the offset
of the first transition into "observe" (measured on the sibling with its real result).
    python rebuild_result.py RUN_DIR SIBLING_RESULT OUT_DIR
OUT_DIR gets result.json and a link to RUN_DIR/slam_control.npy."""
import json, math, os, sys

import numpy as np

run, sib_path, out = sys.argv[1], sys.argv[2], sys.argv[3]
sib = json.load(open(sib_path))
frames = [json.loads(l) for l in open(os.path.join(run, "frames.jsonl"))]


def yaw_of(wxyz):
    w, x, y, z = wxyz
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def first_phase_time(rows, phase, key):
    return next(r[key] for r in rows if r["phase"] == phase)


# frames time against the mission time: the median gap over the transitions the sibling's result and frames share
sib_frames = [json.loads(l) for l in open(os.path.join(os.path.dirname(sib_path), "frames.jsonl"))]
gaps, prev_ = [], None
changes = {}
for r in sib_frames:
    if prev_ is not None and r["phase"] != prev_ and r["phase"] not in changes:
        changes[r["phase"]] = r["simulation_time_s"]
    prev_ = r["phase"]
for t in sib["transitions"]:
    if t["to"] in changes:
        gaps.append(changes[t["to"]] - t["time_s"])
offset = float(np.median(gaps))
log = np.load(os.path.join(run, "slam_log.npz"))
js, steer = log["joint_stamps_s"], log["steering_measured_rad"]
samples, transitions, prev = [], [], None
for i, r in enumerate(frames):
    t = r["simulation_time_s"] - offset
    if prev is not None and r["phase"] != prev:
        transitions.append({"from": prev, "to": r["phase"], "time_s": t})
    prev = r["phase"]
    if i % 6:
        continue
    k = int(np.clip(np.searchsorted(js, t), 0, len(js) - 1))
    samples.append({"time_s": t, "phase": r["phase"], "pallet_position_m": r["pallet_position_m"],
                    "pallet_yaw_rad": yaw_of(r["pallet_orientation_wxyz"]),
                    "steering_actual_rad": [float(v) for v in steer[k]], "rear_pose": [0.0, 0.0, 0.0],
                    "signed_speed_mps": 0.0, "curvature_inv_m": 0.0})
result = {k: sib[k] for k in ("scenario", "scene_layout", "geometry", "chassis_model") if k in sib}
result.update(samples=samples, transitions=transitions, rebuilt_from_frames=True, frames_offset_s=offset)
os.makedirs(out, exist_ok=True)
json.dump(result, open(os.path.join(out, "result.json"), "w"))
link = os.path.join(out, "slam_control.npy")
if not os.path.exists(link):
    os.symlink(os.path.join(run, "slam_control.npy"), link)
print(out, len(samples), "samples", len(transitions), "transitions", "offset", round(offset, 3))
