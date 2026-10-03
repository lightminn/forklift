"""Replay the near-field mount study on stored Isaac render-check depth.

B1a (docs/plans/2026-10-03-carriage-mount-adoption.md): swap each cell's CPU
depth for the Isaac depth from ``sim/isaac/render_mount_check.py``, 1 mm
quantised with the study's min range, and run the same near-field pipeline
(provisional chassis, base (0.559, 0, 0.27), tilt 0.10). ``cpu`` runs the
unmodified study for comparison.

    python tools/replay_isaac_nearfield.py <run>/cells <run>/result.json {isaac,cpu}
"""

import dataclasses
import json
import sys

import numpy as np

from tools import nearfield_mount_study as study
from tools import scene_rig


def main(cells_dir: str, record_path: str, mode: str) -> dict:
    record = json.load(open(record_path))
    by_x = {
        round(c["pallet_x_m"], 4): f"{cells_dir}/cell_{c['index']:03d}.npz"
        for c in record["cells"]
    }
    current = {}
    original_place, original_render = scene_rig.place, scene_rig.render

    def place(boxes, x_m=0.0, **kw):
        current["x"] = round(x_m, 4)
        return original_place(boxes, x_m=x_m, **kw)

    def render(boxes, **kw):
        scene = original_render(boxes, **kw)
        if mode == "cpu":
            return scene
        depth = np.load(by_x[current["x"]])["isaac_full"].astype(np.float64)
        depth = np.where(np.isfinite(depth), np.round(depth * 1000.0) / 1000.0, depth)
        depth = np.where(depth < study.MIN_RANGE_M, np.nan, depth)
        return dataclasses.replace(scene, depth_m=depth)

    scene_rig.place, scene_rig.render = place, render
    try:
        return study.near("provisional", 0.27, 0.10)
    finally:
        scene_rig.place, scene_rig.render = original_place, original_render


if __name__ == "__main__":
    cells, record, mode = sys.argv[1:4]
    result = main(cells, record, mode)
    keys = ("handoff_cells", "worst_max_gap_m", "worst_terminal_gap_m",
            "seeds_without_handoff", "post_entry_min_roof_seeds")
    print(json.dumps({k: result[k] for k in keys}), "mode", mode)
    json.dump(result, open(f"replay_{mode}.json", "w"))
