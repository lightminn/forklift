"""The localization params of a survey bundle (SLAM localization plan L3), after checking the bundle (L4).

    python3 tools/slam_localization_params.py MAPPING_YAML OUT_YAML BUNDLE_DIR EXPECTED_MAP_ID "x,y,yaw"

Exits 1 (and writes nothing) when the bundle cannot be used -- slam_map.load_bundle's reason on stderr. Otherwise the
mapping params with ``mode: localization``, ``map_file_name`` (the bundle's graph), ``map_start_pose`` (map <-
base_link) and the variant S1 adopted: buffer and chain 3, no loop closure. Run inside the bridge container (python3
with yaml, forklift_core on the path).
"""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

from forklift_core.localization.slam_map import MapBundleError, load_bundle

LOCALIZATION_KEYS = {"scan_buffer_size": 3, "loop_match_minimum_chain_size": 3, "do_loop_closing": False}


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
