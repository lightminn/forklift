"""Run lists of the short cusp arc comparison (docs/plans/2026-11-16-aisle-short-cusp-arc.md 2, 3): the riib
deterministic rows of the post-fix baseline (its make_runs.riib), aisle_rack.

  python make_runs.py SNAP_NAME OUT_DIR smoke|compare
smoke: one B run (aisle_rack s0). compare: A (off) and B (--short-cusp-arc 0.6,1.0) x seeds 0-9 = 20, A then B by seed.
"""
import importlib.util
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("pfb_runs", HERE.parent / "2026-11-15-post-fix-baseline" / "make_runs.py")
pfb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pfb)

P = pfb.P
B_ARGS = "--deterministic --short-cusp-arc 0.6,1.0"


def rows(snap: str, kind: str) -> list:
    root = f"{P}/test_runs/acs/{kind}_{snap}"
    if kind == "smoke":
        return [pfb.riib(snap, root, "acsB_aisle_rack_s0", "aisle_rack", 0, {"EXTRA_ARGS": B_ARGS})]
    out = []
    for s in range(10):
        out.append(pfb.riib(snap, f"{root}/A", f"acsA_aisle_rack_s{s}", "aisle_rack", s))
        out.append(pfb.riib(snap, f"{root}/B", f"acsB_aisle_rack_s{s}", "aisle_rack", s, {"EXTRA_ARGS": B_ARGS}))
    return out


def main(snap: str, out_dir: str, kind: str) -> None:
    r = rows(snap, kind)
    assert len({x["name"] for x in r}) == len(r) == (1 if kind == "smoke" else 20)
    path = Path(out_dir) / f"RUNS_acs_{kind}_{snap}.jsonl"
    path.write_text("".join(json.dumps(x) + "\n" for x in r))
    print(path, len(r))


if __name__ == "__main__":
    main(*sys.argv[1:4])
