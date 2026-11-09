"""Real-time plan 5.26 1b-0: the dump analysis replays real grid_snapshot dumps -- made here by a process with
FORKLIFT_PERF_DUMP set, as the dump run makes them -- reproduces each dumped result byte for byte and reports the
raw cells and acting bands of every _clear_group call."""

import importlib.util
import os
import subprocess
import sys
import textwrap
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]

MAKE = textwrap.dedent('''
    import math, sys
    import numpy as np
    sys.path.insert(0, sys.argv[1])
    from forklift_core.perception.obstacle_grid import AgeErrorTable, GridConfig, ObstacleGrid, ObstacleScan
    table = AgeErrorTable((0.1, 0.2, 0.3, 1.0, 3.0), (0.024, 0.030, 0.039, 0.091, 0.141),
                          (0.014, 0.022, 0.026, 0.039, 0.059))
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    grid = ObstacleGrid(GridConfig(x_min_m=-1, x_max_m=6, y_min_m=-3, y_max_m=3, error=table))
    for k in range(4):
        ranges = np.full(len(angles), np.inf)
        front = np.abs(angles) < math.radians(40)
        ranges[front] = 3.0 / np.cos(angles[front])
        grid.add_scan(ObstacleScan(0.1 * k, "low", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), angles, ranges,
                                   np.zeros(len(angles), bool), True))
        grid.snapshot(0.1 * k + 0.05, (0.0, 0.0, 0.0))
''')


def test_the_analysis_reproduces_real_dumps_and_counts_raw_and_bands(tmp_path):
    env = {**os.environ, "FORKLIFT_PERF_DUMP": str(tmp_path / "dump"), "FORKLIFT_PERF_DUMP_EVERY": "grid_snapshot=1",
           "PYTHONPATH": str(ROOT / "src")}
    subprocess.run([sys.executable, "-c", MAKE, str(ROOT / "src")], check=True, env=env, timeout=120)
    assert len(list((tmp_path / "dump" / "grid_snapshot").glob("*.pkl.gz"))) >= 4
    spec = importlib.util.spec_from_file_location(
        "g3_1b0_analyze", ROOT / "docs" / "validation" / "2026-11-10-g3-1b0" / "g3_1b0_analyze.py")
    an = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(an)
    paths = sorted((tmp_path / "dump" / "grid_snapshot").glob("*.pkl.gz"))
    out = an.analyse(paths, ["reference"])
    s = out["summary"]
    assert s["dumps"] == s["self_reproduced"] >= 4
    assert s["clear_group_calls"] >= 4 and s["raw_cells"]["max"] > 0
    assert s["acting_bands"]["max"] >= 1 and all(n > 0 for n in s["offsets"])
