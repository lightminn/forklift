"""V2' of a remapped graph against its run's truth (tools/slam_survey_bundle.world_check) plus the best rigid
alignment's residual and the graph's counts; JSON on stdout.

    python tools/slam_remap_eval.py RUN_DIR DUMP_DIR
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from forklift_core.live.spool import load_slam_log  # noqa: E402
from slam_survey_bundle import read_nodes, rows_at, world_check  # noqa: E402


def main(argv=None) -> int:
    run, dump = (Path(v) for v in (argv or sys.argv[1:]))
    nodes, log = read_nodes(dump / "nodes.csv"), load_slam_log(run)
    joints, base = np.asarray(log["joint_stamps_s"], float), np.asarray(log["base_pose_world"], float)
    tolerance = 0.5 * float(np.median(np.diff(joints)))
    out = world_check(nodes, joints, base, tolerance)
    rows = rows_at(nodes["stamp_s"], joints, tolerance)
    truth, est = base[rows][:, :2], np.column_stack((nodes["x"], nodes["y"]))
    a, b = truth - truth.mean(0), est - est.mean(0)
    u, _, vt = np.linalg.svd(b.T @ a)
    r = u @ vt
    residual = np.hypot(*(b - a @ r.T).T)
    out["rigid_aligned"] = {"max_m": float(residual.max()), "rmse_m": float(np.sqrt(np.mean(residual ** 2))),
                            "theta_rad": float(np.arctan2(r[1, 0], r[0, 0]))}
    graph = json.loads((dump / "graph.json").read_text())
    out["graph"] = {k: graph[k] for k in ("scans", "vertices", "edges")}
    print(json.dumps(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
