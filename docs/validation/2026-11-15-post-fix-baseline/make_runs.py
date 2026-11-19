"""Run lists of the post-fix baseline (plan docs/plans/2026-11-15-post-fix-baseline.md 2.2, 2.3, 3.3, revision 4).

  python make_runs.py SNAP_NAME OUT_DIR
writes OUT_DIR/RUNS_pfb_R_<snap>.jsonl (74), RUNS_pfb_L_<snap>.jsonl (32), RUNS_pfb_S_<snap>.jsonl (40).
"""
import json
import sys
from pathlib import Path

P = "/home/projects/forklift"
RIIB_ENV = {  # the comparators' input files (curvature B0 and K1 re-check run lists)
    "SCENE_DIR": f"{P}/test_runs/riibotics/scenes_p1c_81a007b1",
    "BASE_SCENE": f"{P}/artifacts/20261013_fork_spacing/base_scene/scene.usda",
    "FORKLIFT_URDF": f"{P}/artifacts/20261013_fork_spacing/dd_cfg/old/dls08_measured/forklift.urdf",
    "VENV_PY": f"{P}/.venv/bin/python",
    "BOTTOM_DECK": "off",  # the riib default since 2026-11-19 is auto (structure on line / random): these records ran without it
    "ONNX": f"{P}/test_runs/s3/yolo_runs/yolo11s-seg_p15/weights/best.onnx",
    "LEARNED_PY": f"{P}/venvs/learned/bin/python",
    "VIDEO": "off",
}
CYCLES = {"line_m": "P-1:D-1,P-2:D-2,P-3:D-3", "random_m": "P-4:D-1,P-5:D-2,P-3:D-3"}
R_SEEDS = [("random_1", s) for s in range(60)] + [("line_2", s) for s in range(5)] + [("aisle_rack", s) for s in range(5)]
R_REPEATS = [("random_1", 0), ("random_1", 1), ("line_2", 0), ("aisle_rack", 0)]
S_SEEDS = [("random_1", s) for s in range(30)] + [("line_2", s) for s in range(5)] + [("aisle_rack", s) for s in range(5)]


def riib(snap, root, name, scene, seed, extra_env=None):
    out = f"{root}/{name}"
    env = {**RIIB_ENV, "SNAP": f"{P}/snapshots/{snap}", "SCENE": scene, "SEED": str(seed),
           "EXTRA_ARGS": "--deterministic", **(extra_env or {})}
    return {"name": name, "cwd": f"{P}/snapshots/{snap}",
            "argv": ["bash", f"{P}/snapshots/{snap}/deploy/riibotics/riib_scene.sbatch", out],
            "env": env, "retry_move": [out], "scan": [f"{out}/run.log"]}


def live(snap, root, name, scene, seed, deterministic):
    out = f"{root}/{name}"
    env = {"SCENE_FILE": f"{P}/test_runs/riibotics/scenes_ct1_a770031/{scene}_session.json", "SCENARIO": "queue",
           "CYCLE": CYCLES[scene], "SEED": str(seed), "MAP_BUNDLE": "registry",
           "SLAM_BUNDLE_ROOT": f"{P}/artifacts/slam_bundles", "MAX_EPOCHS": "1"}
    if deterministic:
        env["MORE"] = "--deterministic"
    return {"name": name, "cwd": f"{P}/snapshots/{snap}",
            "argv": ["bash", f"{P}/test_runs/live/ct1_queue.sbatch", out], "env": env, "retry_move": [out],
            "scan": [f"{out}/live.log", f"{out}/live/epoch_*/run.log"],
            "pace_log": f"{out}/live/epoch_*/run/live_log.jsonl",
            "first_record": f"{out}/live/epoch_1/run/scenario.json"}


def main(snap: str, out_dir: str) -> None:
    o = Path(out_dir)
    root = f"{P}/test_runs/pfb"
    r = [riib(snap, f"{root}/R_{snap}", f"pfb_R_{sc}_s{s}", sc, s) for sc, s in R_SEEDS]
    r += [riib(snap, f"{root}/Rrep_{snap}", f"pfb_Rrep_{sc}_s{s}", sc, s) for sc, s in R_REPEATS]
    lst = [live(snap, f"{root}/L_{snap}", f"{arm}_{sc}_s{s}", sc, s, arm == "LD")  # ct_survey reads the arm from the first token
           for s in range(8) for sc in ("line_m", "random_m") for arm in ("LD", "LO")]
    st = [riib(snap, f"{root}/S_{snap}", f"pfb_S_{sc}_s{s}", sc, s,
               {"FORKLIFT_PATH_DISPLAY_DIAGNOSTIC": "stage"})  # stage comes from the variable; the launcher refuses it in EXTRA_ARGS
          for sc, s in S_SEEDS]
    for tag, rows, n in (("R", r, 74), ("L", lst, 32), ("S", st, 40)):
        assert len(rows) == n and len({x["name"] for x in rows}) == n, tag
        (o / f"RUNS_pfb_{tag}_{snap}.jsonl").write_text("".join(json.dumps(x) + "\n" for x in rows))
        print(tag, len(rows))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
