"""Near-capture frequency batch: the run lists and the frozen manifest (runs on ws1, from BATCH)."""
import hashlib
import json
import os
import sys

P = "/home/projects/forklift"
SHA = "46d8929d2f8119eee7f9cb6dcd242f9b51366aeb"
NAME = "nf_" + SHA[:8]
SNAP = f"{P}/snapshots/{NAME}"
B = f"{P}/test_runs/near_freq/{NAME}"
SCENES = f"{P}/test_runs/riibotics/scenes_ct1_a770031"
WRAP = f"{P}/test_runs/live/ct1_queue.sbatch"
CYCLES = {"line_m": "P-1:D-1,P-2:D-2,P-3:D-3", "random_m": "P-4:D-1,P-5:D-2,P-3:D-3"}


def row(scene, seed):
    out = f"{B}/O0_{scene}_s{seed}"
    return {"name": f"O0_{scene}_s{seed}", "cwd": SNAP, "argv": ["bash", WRAP, out],
            "env": {"SCENE_FILE": f"{SCENES}/{scene}_session.json", "SCENARIO": "queue", "CYCLE": CYCLES[scene],
                    "SEED": str(seed), "MAX_EPOCHS": "1", "MAP_BUNDLE": "registry"},
            "retry_move": [out], "scan": [f"{out}/live.log"], "pace_log": f"{out}/live/epoch_*/run/live_log.jsonl"}


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


rows = [row(s, seed) for seed in range(100, 110) for s in ("line_m", "random_m")]
# nine and eleven: three and four full turns of the three slots; the node runs one bundle at a time with the VM on
lists = {"RUNS_A.jsonl": rows[:9], "RUNS_B.jsonl": rows[9:]}
for name, part in lists.items():
    with open(f"{B}/{name}", "w") as f:
        f.writelines(json.dumps(r) + "\n" for r in part)
files = {k: f"{B}/{k}" for k in lists}
files.update({"wrapper": WRAP, "live_launcher": f"{SNAP}/deploy/console/isaac_live.sbatch", "monitor": f"{B}/nf_monitor.py",
              "onnx": f"{P}/test_runs/s3/yolo_runs/yolo11s-seg_p15/weights/best.onnx",
              "scene_line_m": f"{SCENES}/line_m_session.json", "scene_random_m": f"{SCENES}/random_m_session.json",
              "registry": f"{SNAP}/config/slam_map_bundles.yaml"})
manifest = {"plan": "docs/plans/2026-11-07-near-capture-frequency.md", "commit": SHA, "snapshot": SNAP,
            "sha256": {k: sha(v) for k, v in files.items()}}
json.dump(manifest, open(f"{B}/MANIFEST.frozen.json", "w"), indent=1, sort_keys=True)
print(json.dumps(manifest["sha256"], indent=0)[:600], len(rows))
