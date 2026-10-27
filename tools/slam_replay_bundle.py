#!/usr/bin/env python3
"""A SLAM map bundle from a cost replay's graph dump (slam-bounded-graph plan candidate A: a map built by replaying one
run, then other runs localized on it).

The replay's bridge folder holds ``bundle/map.posegraph`` and ``bundle/map.data`` (slam_toolbox's serialized graph)
and ``dump/grid.u8`` + ``dump/graph.json`` (that graph's karto grid, tools/slam_graph_dump.cpp: 0 unknown, 100
occupied, 255 free, row-major from the origin). This writes the bundle forklift_core.localization.slam_map reads:
the two graph files, ``grid.npz`` (int8 -1/0/100, origin x y yaw, resolution) and ``manifest.json`` (format, file
hashes, map_id). A replay bundle is a diagnostic: it carries ``"replay": true`` and the source it was built from, and
none of the survey's checks (tools/slam_survey_bundle.py) -- the plan's V2' condition for a deployed bundle stays.

    python tools/slam_replay_bundle.py BRIDGE_DIR OUT_DIR --source-run RUN [--units 0]
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from forklift_core.localization.slam_map import BUNDLE_FILES, FORMAT, file_sha256, map_id_of  # noqa: E402

KARTO_TO_BUNDLE = {0: -1, 100: 100, 255: 0}


def make_bundle(bridge: Path, out: Path, *, source: dict) -> dict:
    bridge, out = Path(bridge), Path(out)
    for name in ("map.posegraph", "map.data"):
        if not (bridge / "bundle" / name).is_file():
            raise SystemExit(f"{bridge}: bundle/{name} missing (run the replay with DUMP_GRAPH=1)")
    meta = json.loads((bridge / "dump" / "graph.json").read_text())
    raw = np.frombuffer((bridge / "dump" / "grid.u8").read_bytes(), np.uint8)
    width, height = int(meta["width"]), int(meta["height"])
    if raw.size != width * height:
        raise SystemExit(f"{bridge}: grid.u8 holds {raw.size} cells, the graph says {width} x {height}")
    data = np.full(raw.shape, -1, np.int8)
    for value, mapped in KARTO_TO_BUNDLE.items():
        data[raw == value] = mapped
    out.mkdir(parents=True, exist_ok=False)
    for name in ("map.posegraph", "map.data"):
        shutil.copyfile(bridge / "bundle" / name, out / name)
    np.savez(out / "grid.npz", data=data.reshape(height, width),
             origin=np.array([float(meta["origin"][0]), float(meta["origin"][1]), 0.0]),
             resolution_m=float(meta["resolution_m"]))
    hashes = {name: file_sha256(out / name) for name in BUNDLE_FILES}
    manifest = {"format": FORMAT, "files": hashes, "map_id": map_id_of(hashes), "replay": True, "source": source,
                "graph": {k: meta.get(k) for k in ("scans", "vertices", "edges")}}
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1) + "\n")
    return manifest


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("bridge", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument("--source-run", required=True)
    ap.add_argument("--units", type=int, nargs="+", default=[0])
    a = ap.parse_args(argv)
    manifest = make_bundle(a.bridge, a.out, source={"run": a.source_run, "units": a.units})
    print(manifest["map_id"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
