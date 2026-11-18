"""DRY_RUN both launchers (riib: line_2, aisle, trailer; live: default) and print the runner flags that differ
(docs/plans/2026-11-18-live-vs-riib-defaults.md 1). Run from the repository root with PYTHONPATH=src."""
import sys, tempfile, shutil, json
from pathlib import Path
ROOT = Path.cwd()
sys.path.insert(0, str(ROOT / "tests/unit/tools"))
import test_riib_scene_launcher as RB
import test_live_riib_45_defaults as LV

def flags(args):
    out, i = {}, 0
    while i < len(args):
        a = args[i]
        if a.startswith("--"):
            if "=" in a:
                k, v = a.split("=", 1); out[k] = v
            elif i + 1 < len(args) and not args[i + 1].startswith("--"):
                out[a] = args[i + 1]; i += 1
            else:
                out[a] = True
        i += 1
    return out

res = {}
for scene in ("line_2", "aisle", "trailer"):
    t = Path(tempfile.mkdtemp(dir="/tmp", prefix="ld"))
    o = RB.dry_run(t, scene=scene)
    assert o.returncode == 0, o.stdout + o.stderr
    res["riib_" + scene] = flags(RB.args_of(o))
    shutil.rmtree(t, ignore_errors=True)
t = Path(tempfile.mkdtemp(dir="/tmp", prefix="ld"))
p = LV._launch(t)
assert p.returncode == 0, p.stdout + p.stderr
res["live"] = flags((t / "runner_args").read_text().splitlines())
shutil.rmtree(t, ignore_errors=True)
keys = sorted(set().union(*[set(v) for v in res.values()]))
skip = ("--output", "--seed", "--scene-file", "--live-port", "--live-port-file", "--slam-feedback", "--slam-map-dir",
        "--forklift-urdf", "--base-scene", "--live-web-root", "--live-epoch", "--slam-choice", "--quarter-eye",
        "--quarter-target", "--frame-log", "--recognition-onnx", "--recognition-python", "--max-sim-seconds")
cols = list(res)
print("flag\t" + "\t".join(cols))
for k in keys:
    if k in skip: continue
    vals = [str(res[c].get(k, "-")) for c in cols]
    if len(set(vals)) > 1:
        print(k + "\t" + "\t".join(vals))
