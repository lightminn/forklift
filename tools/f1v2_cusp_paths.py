#!/usr/bin/env python3
"""F1 v2 cusp arrival probe (docs/plans/2026-11-04-f1-v2-registration.md section 3): the path registry, fixed before any
result -- every registered transport path version of the v1 aisle_rack runs with its hash, the versions the rule leaves
out, and the runs' transport and observe tracker settings (which must agree).

  python tools/f1v2_cusp_paths.py --run NAME RESULT.json [--run ...] --json REGISTRY.json
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim" / "isaac"))

import f1v2_probe_core as core  # noqa: E402

CONFIGS = ("transport", "observe")


def build(results: dict) -> dict:
    configs = {name: {k: r["tracker_configs"][k] for k in CONFIGS} for name, r in results.items()}
    distinct = {json.dumps(c, sort_keys=True) for c in configs.values()}
    if len(distinct) != 1:
        raise ValueError(f"the runs' tracker settings differ: {sorted(configs)}")
    registry = core.registered_paths(results)
    registry["tracker_configs"] = next(iter(configs.values()))
    registry["runs"] = sorted(results)
    registry["cusp_legs"] = sum(v["cusp_legs"] for v in registry["versions"])
    return registry


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--run", nargs=2, action="append", required=True, metavar=("NAME", "RESULT_JSON"))
    p.add_argument("--json", type=Path, required=True)
    args = p.parse_args()
    results = {name: json.loads(Path(path).read_text()) for name, path in args.run}
    registry = build(results)
    args.json.write_text(json.dumps(registry) + "\n")
    print(json.dumps({"versions": [(v["id"], v["source"], v["cusp_legs"], v["sha256"][:12])
                                   for v in registry["versions"]],
                      "omitted": [(o["id"], o["why"]) for o in registry["omitted"]],
                      "cusp_legs": registry["cusp_legs"]}, indent=1))


if __name__ == "__main__":
    main()
