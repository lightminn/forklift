"""Real-time plan 5.25: the R1 mark/wall re-run's environment validity, beside r1c_analyze.py's own session rules --
(1) another process on the GPU, as the session's monitor saw it (host.jsonl), (2) other users' CPU over one core in
more than 5 % of the CPU sampler's samples in the session's monitor window (or no sample) -- the rules of the set-F
supplement (5.22 correction), from g2m_analyze.py.

    python r1c_validity.py OUT_DIR --list LIST.jsonl --ps PS.jsonl [--json out.json]
"""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

HERE = Path(__file__).resolve()
_spec = importlib.util.spec_from_file_location("g2m_analyze", HERE.parents[1] / "2026-11-09-g2-measure" / "g2m_analyze.py")
g2m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(g2m)


def session_validity(out: Path, samples) -> list[str]:
    f = out / "monitor" / "host.jsonl"
    lines = [json.loads(x) for x in f.read_text().splitlines()] if f.exists() else []
    if not lines:
        return ["no monitor record"]
    bad = []
    for r in lines:
        if r.get("gpu_status") != "ok" or r.get("gpu_procs") is None:
            bad.append(f"monitor query not ok ({r.get('gpu_status')})")
            break
        foreign = [p for p in r["gpu_procs"] if len(p) < 4 or p[3] is not True]
        if foreign:
            bad.append(f"a GPU process not known to be this job's: {foreign[0]}")
            break
    walls = [r.get("wall") for r in lines if r.get("wall") is not None]
    rule = g2m.ps_rule(samples, min(walls), max(walls)) if walls else {"invalid": True, "samples": 0}
    if rule["invalid"]:
        bad.append(f"other users' CPU over 1 core in more than 5 % of samples (or no sample): {rule}")
    return bad


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", type=Path)
    ap.add_argument("--list", type=Path, required=True)
    ap.add_argument("--ps", type=Path, required=True)
    ap.add_argument("--json", type=Path)
    a = ap.parse_args()
    samples = g2m.read_ps(a.ps.read_text())
    rows = []
    for line in a.list.read_text().splitlines():
        if line.strip():
            name = json.loads(line)["name"]
            d = a.out_dir / name
            rows.append({"name": name, "invalid": session_validity(d, samples) if d.exists() else ["missing"]})
    for r in rows:
        print(json.dumps(r))
    if a.json:
        a.json.write_text(json.dumps(rows, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
