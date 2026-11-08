"""Each run's DRY_RUN (wrapper env, EXTRA_ARGS, the runner's --check-args) kept in R/pa/<name>/ (short: the SLAM socket path under it must stay within AF_UNIX 107 bytes)."""
import json
import os
import subprocess
import sys

R = sys.argv[1]
bad = 0
for line in open(f"{R}/RUNS.jsonl"):
    r = json.loads(line)
    d = os.path.join(R, "pa", r["name"])
    os.makedirs(d, exist_ok=True)
    env = dict(os.environ, **r["env"], DRY_RUN="1")
    p = subprocess.run(["bash", r["argv"][1], d], cwd=r["cwd"], env=env, capture_output=True, text=True, timeout=300)
    with open(os.path.join(d, "dry_run.txt"), "w") as fh:
        fh.write(p.stdout + p.stderr)
    ok = p.returncode == 0 and "ARGS_OK" in p.stdout
    bad += not ok
    print(r["name"], p.returncode, "ARGS_OK" if ok else "FAIL")
print("failed", bad)
