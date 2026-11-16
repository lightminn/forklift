#!/usr/bin/env python3
"""The measurement day's M1-M3 table (registration revision 4, 11.3): several ``m_analyze/1`` outputs in, one
``m_summary/1`` JSON line out -- per value the median over the valid trials, its band (good / watch / action against
L and H) and the trial values at or over H; the M2 left/right gain ratio per q step.

usage: m_summary.py OUT1.json [OUT2.json ...]     (each file: the JSON line m_analyze.py printed)
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "src"))

from forklift_core.plant import m_rev4  # noqa: E402


def main(argv=None) -> int:
    paths = [Path(p) for p in (sys.argv[1:] if argv is None else argv)]
    if not paths:
        print(__doc__, file=sys.stderr)
        return 2
    results = []
    for p in paths:
        res = json.loads(p.read_text(encoding="utf-8").strip().splitlines()[-1])
        if res.get("result") != "ok":
            print(json.dumps({"result": "refused", "why": f"{p}: not an ok m_analyze output"}))
            return 2
        results.append(res)
    print(json.dumps({"result": "ok", **m_rev4.summary(results), "inputs": [str(p) for p in paths]}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
