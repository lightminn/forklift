"""The localization params of a survey bundle (SLAM localization plan L3), after checking the bundle (L4).

    python3 tools/slam_localization_params.py MAPPING_YAML OUT_YAML BUNDLE_DIR EXPECTED_MAP_ID "x,y,yaw"

Exits 1 (and writes nothing) when the bundle cannot be used -- slam_map.load_bundle's reason on stderr. Otherwise the
mapping params with ``mode: localization``, ``map_file_name`` (the bundle's graph), ``map_start_pose`` (map <-
base_link) and the localization variant: buffer and chain 3 with loop closure, links up to 3.0 m and a loop search of
2.0 m over a 3.0 m window (variant J, 2026-10-21 V5 diagnosis: S1's adopted 3/3 without loop closure drifted off the
r7 survey graph in the Isaac V5 run -- 1.30 m in 28 min -- and the same-keyframe replay with J held 0.072 m over noise
seeds 0-2, processing max 0.19 s; a diagnostic candidate, not a deployment pass: docs/plans/
2026-10-20-slam-localization-session.md). Run inside the bridge container (python3 with yaml, forklift_core on the
path).
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

from forklift_core.localization.slam_map import MapBundleError, load_bundle

LOCALIZATION_KEYS = {"scan_buffer_size": 3, "loop_match_minimum_chain_size": 3, "do_loop_closing": True,
                     "link_scan_maximum_distance": 3.0, "loop_search_space_dimension": 3.0,
                     "loop_search_maximum_distance": 2.0,
                     # pinned, not inherited from the mapping config: the values the bounded-graph plan's S2 replays
                     # measured J with (they ran on the slam_link_failed mitigation's online config)
                     "loop_search_space_resolution": 0.10, "loop_search_space_smear_deviation": 0.05}


def localization_params(mapping: dict, prefix: str, start: list[float]) -> dict:
    out = dict(mapping)
    p = dict(out["slam_toolbox"]["ros__parameters"])
    p.update(mode="localization", map_file_name=prefix, map_start_pose=[float(v) for v in start], **LOCALIZATION_KEYS)
    out["slam_toolbox"] = {**out["slam_toolbox"], "ros__parameters": p}
    return out


def main(argv=None) -> int:
    src, dst, folder, expected, start = argv or sys.argv[1:]
    try:
        bundle = load_bundle(Path(folder), expected)
    except MapBundleError as exc:
        print(exc, file=sys.stderr)
        return 1
    params = localization_params(yaml.safe_load(Path(src).read_text()), str(bundle.graph_prefix),
                                 [float(v) for v in start.split(",")])
    Path(dst).write_text(yaml.safe_dump(params))
    print(f"map_id {bundle.map_id}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
