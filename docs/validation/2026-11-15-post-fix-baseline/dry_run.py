"""DRY_RUN of chosen run-list entries (plan 2.1): the launcher writes riib_defaults.txt / prints RUNNER_ARGS."""
import json, os, subprocess, sys
from pathlib import Path
lists, out = sys.argv[1:-1], Path(sys.argv[-1])
want = {"pfb_R_random_1_s0", "pfb_R_line_2_s0", "pfb_R_aisle_rack_s0", "LD_line_m_s0", "LO_line_m_s0",
        "LD_random_m_s0", "LO_random_m_s0", "pfb_S_aisle_rack_s0", "pfb_S_line_2_s0"}
for lst in lists:
    for line in open(lst):
        r = json.loads(line)
        if r["name"] not in want:
            continue
        d = out / r["name"]
        d.mkdir(parents=True, exist_ok=True)
        argv = list(r["argv"][:-1]) + [str(d)]
        env = {**os.environ, **r["env"], "DRY_RUN": "1"}
        p = subprocess.run(argv, cwd=r["cwd"], env=env, capture_output=True, text=True, timeout=300)
        (d / "dry_stdout.txt").write_text(p.stdout)
        (d / "dry_stderr.txt").write_text(p.stderr)
        print(r["name"], "rc", p.returncode, "ARGS_OK" in p.stdout, len(p.stdout.splitlines()))
