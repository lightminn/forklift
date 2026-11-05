"""line_2 robustness V2/§8: one on-path run recorded at its boundary (line_2 seed 0, the round-1 injection so the new
branch runs), to be replayed on CPU before V3. -> OUT/RUNS_rec.jsonl"""
import hashlib, json, sys
from pathlib import Path

SNAP, OUT = sys.argv[1], sys.argv[2]
TMPL_LINE = Path("/home/projects/forklift/test_runs/riibotics/f2b_22eac21d/RUNS.jsonl").read_text().splitlines()[0]
assert hashlib.md5((TMPL_LINE + "\n").encode()).hexdigest() == "1ed7ab28b82787fce84ba476699e0e2c", "template row changed"
tmpl = json.loads(TMPL_LINE)
KEEP = ("VENV_PY", "ONNX", "LEARNED_PY", "SCENE_DIR", "BASE_SCENE", "FORKLIFT_URDF")
env = {k: tmpl["env"][k] for k in KEEP if k in tmpl["env"]}
env.update(SNAP=SNAP, VIDEO="off", RETURN_START="boxed", NEAR_END_STAND="on", RETURN_CUSP_REPLANS="2", SCENE="line_2",
           SEED="0", EXTRA_ARGS="--docking-degenerate-continue on --inject-round1-degenerate --record-boundary "
                                "--record-boundary-format 2")
d = f"{OUT}/ddc_rec_on_line_2_s0"
row = {"name": "ddc_rec_on_line_2_s0", "cwd": SNAP, "argv": ["bash", f"{SNAP}/deploy/riibotics/riib_scene.sbatch", d],
       "env": env, "retry_move": [d], "scan": [f"{d}/run.log"]}
Path(OUT).mkdir(parents=True, exist_ok=True)
(Path(OUT) / "RUNS_rec.jsonl").write_text(json.dumps(row) + "\n")
print(Path(OUT) / "RUNS_rec.jsonl")
