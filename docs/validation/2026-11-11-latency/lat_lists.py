"""The latency-tolerance test's run lists (docs/plans/2026-11-11-latency-tolerance.md 3, 6, 8).

    python lat_lists.py smoke SNAP OUT        # the implementation PR's Isaac smoke: A rows (deterministic batch) in
                                              # OUT/RUNS_smoke.jsonl and the live B row in OUT/lat_sm_B50.jsonl
    python lat_lists.py a SNAP OUT            # A: 80 riib rows in OUT/RUNS_A.jsonl
    python lat_lists.py b SNAP OUT            # B: 15 single-slot live jobs, one OUT/<name>.jsonl each, in order
    python lat_lists.py rerun SNAP OUT NAME.. # one row per named A run under lat_r_<name> (the first record stays),
                                              # in OUT/RUNS_rerun.jsonl (B names: one OUT/lat_r_<name>.jsonl each)

A runs riib_scene.sbatch with --deterministic, VIDEO off, RETURN_START=boxed, NEAR_END_STAND=on and --lidar-cast loop
(the batch defaults, set explicitly) and no G3 kernel variables; the latency seed is the scene seed (the runner's
default). B runs r1c_session.sh line_m far K f 1.0 confirm, seed 0, INSTR off, G3 1/1/fast, LATENCY_PLAN_MS = d."""

import json
import sys
from pathlib import Path

P = "/home/projects/forklift"
RIIB_ENV = {  # ws1 paths (the riib batches' template, e.g. test_runs/riibotics/epc7_f7c08464)
    "SCENE_DIR": f"{P}/test_runs/riibotics/scenes_p1c_81a007b1",
    "BASE_SCENE": f"{P}/artifacts/20261013_fork_spacing/base_scene/scene.usda",
    "FORKLIFT_URDF": f"{P}/artifacts/20261013_fork_spacing/dd_cfg/old/dls08_measured/forklift.urdf",
    "VENV_PY": f"{P}/.venv/bin/python",
    "ONNX": f"{P}/test_runs/s3/yolo_runs/yolo11s-seg_p15/weights/best.onnx",
    "LEARNED_PY": f"{P}/venvs/learned/bin/python",
    "VIDEO": "off", "RETURN_START": "boxed", "NEAR_END_STAND": "on",
}
BASE_ARGS = "--deterministic --lidar-cast loop"
ARMS = {  # plan 3.1
    "B0": "", "S0": "--latency-slam ticks:0", "L0": "--latency-layer undeclared:0",
    "S3": "--latency-slam ticks:3", "S6": "--latency-slam ticks:6", "S12": "--latency-slam ticks:12",
    "U3": "--latency-layer undeclared:3", "U6": "--latency-layer undeclared:6", "U12": "--latency-layer undeclared:12",
    "D12": "--latency-layer declared:12",
    "C6": "--latency-slam ticks:6 --latency-layer undeclared:6",
    "J": "--latency-slam jitter:12 --latency-layer jitter:12",
}
MAIN_ARMS = ("B0", "S3", "S6", "S12", "U3", "U6", "U12", "D12", "C6", "J")
AISLE_ARMS = ("S0", "S12", "U12", "C6")
B_WRAPPER = "docs/validation/2026-11-08-r1-confirmation/r1c_session.sh"
B_BASE = {"SET": "far", "ARM": "K", "FACTOR": "1.0", "SEED": "0", "INSTR": "off", "FORKLIFT_GRID_CLEAR_KERNEL": "1",
          "FORKLIFT_SHADOW_KERNEL": "1", "LIDAR_CAST": "fast"}
B_DELAYS = (0, 25, 50, 100, 500)


def riib_row(snap: str, out: str, name: str, arm: str, scene: str, seed: int) -> dict:
    d = f"{out}/{name}"
    extra = f"{BASE_ARGS} {ARMS[arm]}".strip()
    env = dict(RIIB_ENV, SNAP=snap, SCENE=scene, SEED=str(seed), EXTRA_ARGS=extra, LAT_ARM=arm)  # LAT_*: the list's own
    return {"name": name, "cwd": snap, "argv": ["bash", f"{snap}/deploy/riibotics/riib_scene.sbatch", d], "env": env,
            "retry_move": [d], "scan": [f"{d}/run.log"]}


def a_rows(snap: str, out: str) -> list[dict]:
    rows = []
    for seed in (0, 1, 2):
        for arm in AISLE_ARMS:  # the aisle and random_1 first (the longer runs first in a bundle)
            rows.append(riib_row(snap, out, f"lat_{arm}_aisle_rack_s{seed}", arm, "aisle_rack", seed))
        for scene in ("random_1", "line_2"):
            for arm in MAIN_ARMS:
                rows.append(riib_row(snap, out, f"lat_{arm}_{scene}_s{seed}", arm, scene, seed))
    for scene in ("random_1", "line_2"):  # seed 0 only: the equivalence arms and the determinism repeats
        for arm in ("S0", "L0"):
            rows.append(riib_row(snap, out, f"lat_{arm}_{scene}_s0", arm, scene, 0))
        for arm in ("B0", "C6"):
            rows.append(riib_row(snap, out, f"lat_{arm}_{scene}_s0_rep", arm, scene, 0))
    assert len(rows) == 80 and len({r["name"] for r in rows}) == 80
    return rows


def smoke_rows(snap: str, out: str) -> list[dict]:
    rows = []
    for scene in ("random_1", "line_2"):
        for arm in ("B0", "S0", "L0"):
            rows.append(riib_row(snap, out, f"lat_sm_{arm}_{scene}_s0", arm, scene, 0))
    for arm in ("B0", "S0"):
        rows.append(riib_row(snap, out, f"lat_sm_{arm}_aisle_rack_s0", arm, "aisle_rack", 0))
    for arm in ("S6", "U6"):
        rows.append(riib_row(snap, out, f"lat_sm_{arm}_line_2_s0", arm, "line_2", 0))
    return rows


SOCK_TAIL = "/live/epoch_999/sock/slam.sock"  # the live launcher's longest socket path below the run (AF_UNIX: 107)


def b_entry(snap: str, out: str, name: str, d: int) -> dict:
    run = f"{out}/{name}/{name}"
    if len((run + SOCK_TAIL).encode()) > 107:
        raise SystemExit(f"{run}: the SLAM socket path would pass 107 bytes -- a shorter OUT")
    return {"name": name, "argv": ["bash", f"{snap}/{B_WRAPPER}", run], "cwd": snap,
            "env": {**B_BASE, **({"LATENCY_PLAN_MS": str(d)} if d else {}), "LAT_DELAY_MS": str(d)},
            "retry_move": [run], "pace_log": f"{run}/live/epoch_*/run/live_log.jsonl",
            "first_record": f"{run}/live/epoch_1/run/scenario.json", "scan": [f"{run}/live/epoch_1/run.log"]}


def b_jobs(snap: str, out: str) -> list[dict]:
    order = list(B_DELAYS) + list(reversed(B_DELAYS)) + list(B_DELAYS)  # three rounds, the second reversed
    seen: dict = {}
    jobs = []
    for n, d in enumerate(order, start=1):
        seen[d] = seen.get(d, 0) + 1
        jobs.append(b_entry(snap, out, f"lat_b{n:02d}_d{d}_r{seen[d]}", d))
    assert len(jobs) == 15 and all(v == 3 for v in seen.values())
    return jobs


def write(path: Path, rows: list[dict]) -> None:
    text = "".join(json.dumps(r) + "\n" for r in rows)
    if path.exists() and path.read_text() != text:
        raise SystemExit(f"{path} exists with other content")
    path.write_text(text)


def main(argv: list) -> int:
    if len(argv) < 3 or argv[0] not in ("smoke", "a", "b", "rerun"):
        raise SystemExit(__doc__)
    mode, snap, out = argv[0], argv[1], argv[2].rstrip("/")
    Path(out).mkdir(parents=True, exist_ok=True)
    if mode == "smoke":
        write(Path(out) / "RUNS_smoke.jsonl", smoke_rows(snap, out))
        write(Path(out) / "lat_sm_B50.jsonl", [b_entry(snap, out, "lat_sm_B50", 50)])
    elif mode == "a":
        write(Path(out) / "RUNS_A.jsonl", a_rows(snap, out))
    elif mode == "b":
        for j in b_jobs(snap, out):
            write(Path(out) / f"{j['name']}.jsonl", [j])
            print(j["name"], j["env"]["LAT_DELAY_MS"])
    else:
        names = argv[3:]
        a = {r["name"]: r for r in a_rows(snap, out)}
        b = {j["name"]: j for j in b_jobs(snap, out)}
        rows = []
        for name in names:
            if name in a:
                e = a[name]["env"]
                rows.append(riib_row(snap, out, f"lat_r_{name}", e["LAT_ARM"], e["SCENE"], int(e["SEED"])))
            elif name in b:
                write(Path(out) / f"lat_r_{name}.jsonl",
                      [b_entry(snap, out, f"lat_r_{name}", int(b[name]["env"]["LAT_DELAY_MS"]))])
            else:
                raise SystemExit(f"{name}: not a registered run")
        if rows:
            write(Path(out) / "RUNS_rerun.jsonl", rows)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
