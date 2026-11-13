#!/usr/bin/env python3
"""Was the slam_toolbox a replay ran the one it asked for? (docs/design/2026-11-13-slam-patch-build.md 2)

Run inside the replay container after the bridge returns (tools/slam_cost_container.sh):
- the image kind (SLAM_IMAGE_KIND: '' the deployed apt image, 'u0' the unpatched source build, 'patched') against the
  image's build record (/opt/slam_patched/series: 'none' for U0, the series id for a patch image);
- the libkartoSlamToolbox the node really loaded (slam_libs.txt, from /proc/<pid>/maps): apt for the deployed image,
  the overlay for a source image, never both;
- the patch identification line in the node's log: none from apt or U0; from a patch image exactly one, its series the
  image's and its switches those the replay passed (FORKLIFT_SLAM_* in this environment) -- a misspelt switch would
  otherwise run an arm under another's name.

    python3 tools/slam_patch_check.py --kind KIND [--series-file F] --libs slam_libs.txt --log slam_toolbox.log --out J

Writes {"ok": ..., "problems": [...]} to J; exits 2 when there is a problem.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path

APT_PREFIX = "/opt/ros/jazzy/"
OVERLAY_PREFIX = "/opt/slam_patched/"
IDENT = re.compile(r"forklift_slam_patch series=(\S+) trace=(\d) cwm_s=(\S+) delete_s=(\S+) s1=(\d) s1_buffer=(\d+)")


def _wanted(env: dict) -> dict:
    def num(name: str, default: float) -> float:
        v = env.get(name, "")
        return float(v) if v.strip() else default

    return {"trace": 1 if env.get("FORKLIFT_SLAM_TRACE", "") else 0, "cwm_s": num("FORKLIFT_SLAM_CWM_S", 0.0),
            "delete_s": num("FORKLIFT_SLAM_DELETE_OLDER_S", 0.0), "s1": 1 if num("FORKLIFT_SLAM_S1", 0.0) else 0,
            "s1_buffer": int(num("FORKLIFT_SLAM_S1_BUFFER", 10.0))}


def check(*, kind: str, series: str | None, libs: str, log: str, env: dict) -> list:
    problems = []
    if kind not in ("", "u0", "patched"):
        return [f"unknown SLAM_IMAGE_KIND {kind!r}"]
    if kind == "u0" and series != "none":
        problems.append(f"kind u0 but the image's series is {series!r}")
    if kind == "patched" and (series is None or series == "none"):
        problems.append(f"kind patched but the image's series is {series!r}")
    karto = sorted({line.split()[-1] for line in libs.splitlines() if "libkartoSlamToolbox" in line})
    if not karto:
        problems.append("no libkartoSlamToolbox in the node's maps")
    for path in karto:
        if kind == "" and not path.startswith(APT_PREFIX):
            problems.append(f"deployed image loaded a non-apt {path}")
        if kind in ("u0", "patched") and not path.startswith(OVERLAY_PREFIX):
            problems.append(f"source image loaded the apt (or another) {path}")
    lines = IDENT.findall(log)
    if kind in ("", "u0"):
        if lines:
            problems.append(f"an identification line from a {'deployed' if not kind else 'u0'} image")
        return problems
    if len(set(lines)) != 1:
        problems.append(f"{len(set(lines))} distinct identification lines (one expected)")
        return problems
    got_series, trace, cwm_s, delete_s, s1, s1_buffer = lines[0]
    if got_series != series:
        problems.append(f"identification series {got_series} is not the image's {series}")
    want = _wanted(env)
    got = {"trace": int(trace), "cwm_s": float(cwm_s), "delete_s": float(delete_s), "s1": int(s1),
           "s1_buffer": int(s1_buffer)}
    for key, value in want.items():
        if got[key] != value:
            problems.append(f"{key}: the node ran {got[key]}, the replay asked {value}")
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--kind", default="")
    ap.add_argument("--series-file", type=Path, default=Path("/opt/slam_patched/series"))
    ap.add_argument("--libs", type=Path, required=True)
    ap.add_argument("--log", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    series = a.series_file.read_text().strip() if a.series_file.is_file() else None
    libs = a.libs.read_text() if a.libs.is_file() else ""
    log = a.log.read_text(errors="replace") if a.log.is_file() else ""
    problems = check(kind=a.kind, series=series, libs=libs, log=log, env=dict(os.environ))
    a.out.write_text(json.dumps({"ok": not problems, "kind": a.kind, "series": series, "problems": problems},
                                indent=1) + "\n")
    if problems:
        raise SystemExit("slam patch check: " + "; ".join(problems))
    return 0


if __name__ == "__main__":
    sys.exit(main())
