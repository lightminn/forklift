"""line_2 delivery robustness V3 (docs/plans/2026-11-04-line2-delivery-robustness.md §4 V3, §8): the run list.
Arms off (riib default) and on (--docking-degenerate-continue on), alternating per seed; the parts' injections go to
both arms alike. VIDEO=off, RETURN_START=boxed, NEAR_END_STAND=on and RETURN_CUSP_REPLANS=2 set explicitly (the riib
defaults now), no boundary record (the recorder changes timing).
    python make_runs_v3.py SNAP OUT        V3a line_2 0-11 (prior error along +0.08), V3b aisle_rack 0-7 (round-1
                                           injection), V3c line_2 12-21 (natural): 60 runs -> OUT/RUNS.jsonl"""
import hashlib, json, sys
from pathlib import Path

SNAP, OUT = sys.argv[1], sys.argv[2]
TMPL_LINE = Path("/home/projects/forklift/test_runs/riibotics/f2b_22eac21d/RUNS.jsonl").read_text().splitlines()[0]
assert hashlib.md5((TMPL_LINE + "\n").encode()).hexdigest() == "1ed7ab28b82787fce84ba476699e0e2c", "template row changed"
tmpl = json.loads(TMPL_LINE)
KEEP = ("VENV_PY", "ONNX", "LEARNED_PY", "SCENE_DIR", "BASE_SCENE", "FORKLIFT_URDF")  # paths only, from the template
base_env = {k: tmpl["env"][k] for k in KEEP if k in tmpl["env"]}
base_env.update(SNAP=SNAP, VIDEO="off", RETURN_START="boxed", NEAR_END_STAND="on", RETURN_CUSP_REPLANS="2")
ARMS = {"off": "", "on": "--docking-degenerate-continue on"}
PARTS = (("v3a", "line_2", range(0, 12), "--destination-prior-error-along-m 0.08"),
         ("v3b", "aisle_rack", range(0, 8), "--inject-round1-degenerate"),
         ("v3c", "line_2", range(12, 22), ""))


def run(name, scene, seed, extra):
    env = dict(base_env, SCENE=scene, SEED=str(seed))
    if extra.strip():
        env["EXTRA_ARGS"] = extra.strip()
    d = f"{OUT}/{name}"
    return {"name": name, "cwd": SNAP, "argv": ["bash", f"{SNAP}/deploy/riibotics/riib_scene.sbatch", d], "env": env,
            "retry_move": [d], "scan": [f"{d}/run.log"]}


rows = []
for part, scene, seeds, inject in PARTS:
    for s in seeds:
        order = ("off", "on") if s % 2 == 0 else ("on", "off")
        for arm in order:
            rows.append(run(f"ddc_{arm}_{part}_{scene}_s{s}", scene, s, " ".join(x for x in (inject, ARMS[arm]) if x)))
assert len(rows) == 60 and not any("record-boundary" in r["env"].get("EXTRA_ARGS", "") for r in rows)
Path(OUT).mkdir(parents=True, exist_ok=True)
text = "".join(json.dumps(r) + "\n" for r in rows)
fn = Path(OUT) / "RUNS.jsonl"
assert not fn.exists() or fn.read_text() == text, "RUNS.jsonl exists with other content"
fn.write_text(text)
print(fn, len(rows), "runs")
