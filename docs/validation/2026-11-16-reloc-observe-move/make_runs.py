#!/usr/bin/env python3
"""The observation move's isaac_bundle run list (docs/plans/2026-11-16-reloc-observe-move.md 3): one unattended
start relocalization session per start (deploy/console/reloc_session.sh), the B-P2d shape -- the seed-5 hall's survey
map hall_s5b_G0 (map 3edf24b0), the same scene, URDF, launcher factor and odometry/obstacle-layer arguments as its
survey (bp2d_survey/submit_s5b_gyro.sh, its noise seed 0) -- with --reloc-observe-move D.

    make_runs.py --starts val_s5.json --snap SNAP --out OUT --move 0.3 [--ids val4_000,val4_001] > RUNS.jsonl
"""

import argparse
import json
from pathlib import Path

P = "/home/projects/forklift"
A = f"{P}/artifacts/20261013_fork_spacing"
MAP_BUNDLE = f"{P}/test_runs/live/bp2d_survey/hall_s5b_G0/bundle"
# the survey's own EXTRA_ARGS word for word: localization refuses a run whose SLAM noise seed differs from the survey's
EXTRA = ("--slam-noise-seed 0 --obstacle-layer config/obstacle_layer_single_team_spec_gyro.yaml --odometry-heading gyro "
         "--gyro-model config/gyro_model_spec_typical.yaml")


def runs(starts: list, snap: str, out: str, move: float, ids=None) -> list:
    rows = []
    for s in starts:
        if ids is not None and s["id"] not in ids:
            continue
        x, y, yaw = s["base_world"]
        d = f"{out}/s/{s['id']}"
        extra = EXTRA + (f" --reloc-observe-move {move:g}" if move > 0 else "")
        rows.append({
            "name": s["id"], "argv": ["bash", "deploy/console/reloc_session.sh", d], "cwd": snap,
            "env": {"BASE_SCENE": f"{A}/base_scene/scene.usda",
                    "FORKLIFT_URDF": f"{A}/dd_cfg/old/dls08_measured/forklift.urdf",
                    "VENV_PY": f"{P}/.venv/bin/python", "SEED": "5", "FACTOR": "0.3", "MAP_BUNDLE": MAP_BUNDLE,
                    "START_POSE": f"{x!r},{y!r},{yaw!r}", "RELOC_UNATTENDED": "1", "MAX_EPOCHS": "1",
                    # the launcher asks a map folder's start pose; with START_POSE the bridge takes the runner's
                    # estimate instead (isaac_live.sbatch bridge.sh), so this value is not used
                    "MAP_START_POSE": f"{x!r},{y!r},{yaw!r}",
                    "EXTRA_ARGS": extra},
            "retry_move": [d], "pace_log": f"{d}/epoch_*/run/live_log.jsonl"})
    if ids is not None and len(rows) != len(ids):
        raise SystemExit(f"ids not in the start list: {sorted(set(ids) - {r['name'] for r in rows})}")
    return rows


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--starts", type=Path, required=True)
    ap.add_argument("--snap", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--move", type=float, required=True)
    ap.add_argument("--ids", default=None)
    a = ap.parse_args()
    starts = json.loads(a.starts.read_text())["poses"]
    for r in runs(starts, a.snap, a.out, a.move, None if a.ids is None else a.ids.split(",")):
        print(json.dumps(r))


if __name__ == "__main__":
    main()
