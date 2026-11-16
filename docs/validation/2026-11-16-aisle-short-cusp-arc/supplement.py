"""Post-hoc supplement to the short cusp arc comparison (written after the results, Codex recompute review P2; not a
pre-registered measure and not part of the verdict):

1. the short curved cusp legs of the paths the runner set aside in ``paths_replaced`` (the original transport path is
   kept there and ``paths.transport`` is overwritten by the final straight, so compare.py's count missed them);
2. the #522 judgements of the runs that failed finally, kept as data (``cusp_heading.analyse`` per run).

    python supplement.py --a DIR_A --b DIR_B --ch CUSP_HEADING_PY --out OUT.json
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def replaced_counts(result: dict, short_curved_cusps) -> dict:
    out = {"all": 0, "transport": 0, "n_paths": 0}
    for e in result.get("paths_replaced") or []:
        p = e.get("path") if isinstance(e, dict) else None
        if not isinstance(p, dict):
            continue
        k = short_curved_cusps(p)
        out["n_paths"] += 1
        out["all"] += k
        if e.get("leg") == "transport":
            out["transport"] += k
    return out


def main(argv=None) -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--a", type=Path, required=True)
    ap.add_argument("--b", type=Path, required=True)
    ap.add_argument("--ch", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    x = ap.parse_args(argv)
    cmp_ = _load("compare", HERE / "compare.py")
    ch = _load("cusp_heading", x.ch)
    out = {"note": "post-hoc, not pre-registered, not part of the verdict", "arms": {}, "failed_runs": {}}
    for arm, root in (("A", x.a), ("B", x.b)):
        rows = {}
        for run in ch.runs_from_raw([root], "aisle_rack"):
            seed = int(re.search(r"_s(\d+)$", run.name).group(1))
            rows[seed] = replaced_counts(run.result, cmp_.short_curved_cusps)
            if not run.result.get("success"):
                res = ch.analyse([run], "postfix", None)
                out["failed_runs"][run.name] = {
                    "failure": str(run.result.get("failure"))[:300],
                    "classification": res["counts"]["classification"], "outcome": res["counts"].get("outcome"),
                    "judgements": [{k: j.get(k) for k in ("kind", "leg_length_m", "classification", "time_s")}
                                   for j in (res.get("judgements") or [])]}
        out["arms"][arm] = {"per_seed": rows, "sum_all": sum(r["all"] for r in rows.values()),
                            "sum_transport": sum(r["transport"] for r in rows.values())}
    x.out.write_text(json.dumps(out, indent=1, sort_keys=True, default=float) + "\n")
    print(json.dumps({a: {k: v for k, v in d.items() if k != "per_seed"} for a, d in out["arms"].items()}, indent=1))
    print(json.dumps({n: (f["classification"], f["outcome"]) for n, f in out["failed_runs"].items()}))


if __name__ == "__main__":
    main()
