"""Summarise the P0b sensor study over seeds (priority-5 plan D1).

    python tools/summarise_p0b.py <dir with s<seed>_<candidate>.json> [--markdown]

Per candidate, over every seed: unpermitted entries (must be 0), distinct
objects that produced a stopping-volume event (>= 30 wanted), FREE cells on
any prop's floor projection (must be 0), and per motion class the coverage
(stop volume never UNKNOWN / expired / off-grid) and the permission ratio
(allowed >= recorded speed), pooled over seeds by instant counts.
"""

from __future__ import annotations

import argparse
import json
import re
from collections import defaultdict
from pathlib import Path

CLASSES = [
    "unloaded_forward_straight", "unloaded_forward_curve", "unloaded_reverse_straight", "unloaded_reverse_curve",
    "loaded_forward_straight", "loaded_forward_curve", "loaded_reverse_straight", "loaded_reverse_curve",
]


def summarise(directory: Path) -> dict:
    pooled: dict = defaultdict(lambda: {
        "seeds": [], "unpermitted": 0, "event_objects": 0, "free_inside": 0,
        "moving": defaultdict(int), "covered": defaultdict(float), "permitted": defaultdict(float),
    })
    for path in sorted(directory.glob("s*_*.json")):
        m = re.match(r"s(\d+)_(.+)\.json", path.name)
        seed, name = int(m.group(1)), m.group(2)
        c = json.loads(path.read_text())["candidates"][name]
        p = pooled[name]
        p["seeds"].append(seed)
        p["unpermitted"] += c["unpermitted_entries"]
        p["event_objects"] += c["event_objects"]
        p["free_inside"] += sum(c["free_cells_inside_obstacles"].values())
        for cls, n in c["moving_by_class"].items():
            p["moving"][cls] += n
            p["covered"][cls] += c["coverage_ratio"][cls] * n
            p["permitted"][cls] += c["permission_ratio"][cls] * n
    out = {}
    for name, p in pooled.items():
        cov = {k: p["covered"][k] / p["moving"][k] for k in p["moving"]}
        perm = {k: p["permitted"][k] / p["moving"][k] for k in p["moving"]}
        out[name] = {
            "seeds": sorted(p["seeds"]),
            "unpermitted_entries": p["unpermitted"],
            "event_objects": p["event_objects"],
            "free_cells_on_prop_projection": p["free_inside"],
            "coverage": cov,
            "permission": perm,
            "moving": dict(p["moving"]),
            "coverage_min": min(cov.values()) if cov else None,
            "missing_classes": [k for k in CLASSES if k not in p["moving"]],
        }
    return out


def markdown(summary: dict) -> str:
    head = "| 후보 | seed | 미허가 | 사건 물체 | 투영 안 FREE | " + " | ".join(
        k.replace("unloaded", "빈").replace("loaded", "적재").replace("_forward", "·전진").replace("_reverse", "·후진")
        .replace("_straight", "·직선").replace("_curve", "·곡선") for k in CLASSES) + " |"
    rows = [head, "|" + "---|" * (5 + len(CLASSES))]
    for name, s in sorted(summary.items()):
        cells = []
        for k in CLASSES:
            if k in s["coverage"]:
                cells.append(f"{100 * s['coverage'][k]:.0f}/{100 * s['permission'][k]:.0f}")
            else:
                cells.append("—")
        rows.append(f"| {name} | {','.join(map(str, s['seeds']))} | {s['unpermitted_entries']} | {s['event_objects']} | "
                    f"{s['free_cells_on_prop_projection']} | " + " | ".join(cells) + " |")
    return "\n".join(rows) + "\n\n(칸: 커버리지 % / 허가율 %)\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("directory", type=Path)
    parser.add_argument("--markdown", action="store_true")
    args = parser.parse_args()
    s = summarise(args.directory)
    print(markdown(s) if args.markdown else json.dumps(s, indent=2))


if __name__ == "__main__":
    main()
