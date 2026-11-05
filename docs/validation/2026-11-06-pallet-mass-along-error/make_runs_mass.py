"""Pallet mass -> delivery round-1 SLAM along-track error (docs/plans/2026-11-05-pallet-mass-along-error-probe.md):
the run list. Arm A (10 kg) and B (4.93 kg) differ only in their snapshot (two settings files' epal6 value); both run
the riib launcher defaults (video on) with --deterministic --tyre-contact-log, alternating per seed, plus a
determinism repeat of seed 0 per scene and arm (new process).
    python make_runs_mass.py SNAP_A SNAP_B OUT     line_2 and random_1 x seeds 0-9 x 2 arms + 4 repeats -> OUT/RUNS.jsonl"""
import hashlib, json, sys
from pathlib import Path

SNAP_A, SNAP_B, OUT = sys.argv[1], sys.argv[2], sys.argv[3]
TMPL_LINE = Path("/home/projects/forklift/test_runs/riibotics/f2b_22eac21d/RUNS.jsonl").read_text().splitlines()[0]
assert hashlib.md5((TMPL_LINE + "\n").encode()).hexdigest() == "1ed7ab28b82787fce84ba476699e0e2c", "template row changed"
tmpl = json.loads(TMPL_LINE)
KEEP = ("VENV_PY", "ONNX", "LEARNED_PY", "SCENE_DIR", "BASE_SCENE", "FORKLIFT_URDF")  # paths only, from the template
base_env = {k: tmpl["env"][k] for k in KEEP if k in tmpl["env"]}
base_env.update(EXTRA_ARGS="--deterministic --tyre-contact-log")  # video on: the launcher default (plan amendment 2)
SNAPS = {"A": SNAP_A, "B": SNAP_B}


def run(name, arm, scene, seed):
    snap = SNAPS[arm]
    env = dict(base_env, SNAP=snap, SCENE=scene, SEED=str(seed))
    d = f"{OUT}/{name}"
    return {"name": name, "cwd": snap, "argv": ["bash", f"{snap}/deploy/riibotics/riib_scene.sbatch", d], "env": env,
            "retry_move": [d], "scan": [f"{d}/run.log"]}


rows = []
for s in range(10):
    for scene in ("random_1", "line_2"):  # the longer scene first in each seed
        for arm in (("A", "B") if s % 2 == 0 else ("B", "A")):
            rows.append(run(f"mae_{arm}_{scene}_s{s}", arm, scene, s))
for scene in ("random_1", "line_2"):
    for arm in ("A", "B"):
        rows.append(run(f"mae_{arm}_{scene}_s0_rep", arm, scene, 0))
assert len(rows) == 44 and len({r["name"] for r in rows}) == 44
assert not any("record-boundary" in r["env"]["EXTRA_ARGS"] or "near-tracking" in r["env"]["EXTRA_ARGS"] for r in rows)
Path(OUT).mkdir(parents=True, exist_ok=True)
text = "".join(json.dumps(r) + "\n" for r in rows)
fn = Path(OUT) / "RUNS.jsonl"
assert not fn.exists() or fn.read_text() == text, "RUNS.jsonl exists with other content"
fn.write_text(text)
print(fn, len(rows), "runs", hashlib.md5(text.encode()).hexdigest()[:8])
