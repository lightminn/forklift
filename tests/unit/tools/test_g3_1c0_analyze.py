"""Real-time plan 5.27 1c-0: the analyzer replays shadow_apply dumps switch off and on, checks the self-reproduction
and the equality, counts the calls the array path ran and the moves, and weights the move calls to a run's share."""

import copy
import gzip
import importlib.util
import json
import pickle
import sys
from pathlib import Path

import numpy as np

from forklift_core.control.shadow_memory import ShadowMemory
from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, AgeErrorTable, GridSnapshot
from forklift_core.planning.geometry import Footprint

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "g3_1c0_analyze", ROOT / "docs" / "validation" / "2026-11-10-g3-1c0" / "g3_1c0_analyze.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)
ERROR = AgeErrorTable((0.1, 1.0, 10.0), (0.005, 0.01, 0.02), (0.001, 0.002, 0.004))
BODY = Footprint(0.944, 0.17, 0.36)


def snapshot(rng, stamp, correction):
    u = rng.random((120, 120))
    state = np.full((120, 120), UNKNOWN, dtype=np.uint8)
    state[u < 0.8] = FREE
    state[u < 0.005] = OCCUPIED
    stamps = np.where(state == FREE, stamp - rng.uniform(0.0, 0.1, state.shape), np.nan)
    return GridSnapshot(state, stamps, stamp, correction, 0, 1, {"high": stamp}, -3.0, -3.0, 0.05)


def write_dumps(folder: Path) -> None:
    """Four recorded calls of one receiver, the third moving the memory, as perf.dumped writes them."""
    rng = np.random.default_rng(0)
    mem = ShadowMemory(0.15, ERROR)
    out = folder / "shadow_apply"
    out.mkdir(parents=True)
    corrections = [(0.0, 0.0, 0.0)] * 2 + [(0.013, -0.02, 0.006)] * 2
    for n, corr in enumerate(corrections):
        args = (snapshot(rng, 0.1 * (n + 1), corr), (0.01 * n, 0.0, 0.0), BODY)
        kwargs = {"extra_support": None}
        before = copy.deepcopy((mem, args, kwargs))
        result = mem.apply(*args, **kwargs)
        record = {"name": "shadow_apply", "call": n, "stratum": None, "pure": False, "stream": None,
                  "before": before, "after": copy.deepcopy((mem, args, kwargs)), "result": result}
        with gzip.open(out / f"1_{n:07d}.pkl.gz", "wb") as f:
            pickle.dump(record, f)


def test_the_analyzer_checks_equality_and_weights_the_moves(tmp_path, monkeypatch):
    from forklift_core import kernels

    before_mode = kernels.mode()
    write_dumps(tmp_path / "d")
    modes = ["numpy"] + (["numba"] if importlib.util.find_spec("numba") is not None else [])
    monkeypatch.setattr(sys, "argv", ["g3_1c0_analyze.py", str(tmp_path / "d"), "--json", str(tmp_path / "o.json"),
                                      "--modes", *modes, "--move-share", "0.5"])
    assert tool.main() == 0
    j = json.loads((tmp_path / "o.json").read_text())
    s = j["summary"]
    assert (s["dumps"], s["moved"], s["extra_support"]) == (4, 1, 0)
    for m in modes:
        assert s[m]["self_reproduced"] == s[m]["equal"] == 4
        assert s[m]["arrays_ran"] == 3  # the start-up call keeps the original path
        e = j["expected"][m]
        assert e["off_ms"] == 0.5 * s[m]["off_ms_still"]["mean"] + 0.5 * s[m]["off_ms_moved"]["mean"]
        assert e["expected_ms_per_sim_s"] == -e["saving_share"] * tool.LIVE_MS_PER_SIM_S
    assert j["conditions"]["modes"] == modes
    from forklift_core.control import shadow_memory as sm

    assert sm.SHADOW_KERNEL is False and sm.ShadowMemory._apply_arrays is tool._ARRAYS  # nothing leaks out
    assert kernels.mode() == before_mode


def test_a_dump_the_replay_does_not_reproduce_fails_the_run(tmp_path, monkeypatch):
    write_dumps(tmp_path / "d")
    path = sorted((tmp_path / "d" / "shadow_apply").glob("*.pkl.gz"))[1]
    with gzip.open(path, "rb") as f:
        record = pickle.load(f)
    record["result"].state[0, 0] = OCCUPIED if record["result"].state[0, 0] != OCCUPIED else FREE
    with gzip.open(path, "wb") as f:
        pickle.dump(record, f)
    monkeypatch.setattr(sys, "argv", ["g3_1c0_analyze.py", str(tmp_path / "d"), "--json", str(tmp_path / "o.json"),
                                      "--modes", "numpy"])
    assert tool.main() == 1
    assert json.loads((tmp_path / "o.json").read_text())["summary"]["numpy"]["self_reproduced"] == 3
