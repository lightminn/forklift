"""Real-time plan 5.27: the shadow-level event replay. A stream recorded by the dump module (in a child process with
FORKLIFT_PERF_DUMP set) and its completeness marker; the replay accepts a whole stream and refuses a prefix, a gap, a
missing last call or a missing marker (Codex plan 5.27 P2-1: a call that changes no state passes the continuity
check, so only the counts tell a missing one)."""

import importlib.util
import json
import os
import subprocess
import sys
import textwrap
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "g3_1c_stream", ROOT / "docs" / "validation" / "2026-11-10-g3-1c0" / "g3_1c_stream.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)
MODES = ["numpy"] + (["numba"] if importlib.util.find_spec("numba") is not None else [])

SESSION = textwrap.dedent("""
    import numpy as np
    from forklift_core import kernels, perf
    from forklift_core.control.shadow_memory import ShadowMemory
    from forklift_core.perception.obstacle_grid import FREE, OCCUPIED, UNKNOWN, AgeErrorTable, GridSnapshot
    from forklift_core.planning.geometry import Footprint

    kernels.set_mode("numpy")
    rng = np.random.default_rng(0)
    mem = ShadowMemory(0.15, AgeErrorTable((0.1, 1.0, 10.0), (0.005, 0.01, 0.02), (0.001, 0.002, 0.004)))
    for n in range(6):
        u = rng.random((100, 100))
        state = np.full((100, 100), UNKNOWN, dtype=np.uint8)
        state[u < 0.8] = FREE
        state[u < 0.005] = OCCUPIED
        stamp = 0.1 * (n + 1)
        corr = (0.0, 0.0, 0.0) if n < 3 else (0.012, -0.018, 0.005)
        snap = GridSnapshot(state, np.where(state == FREE, stamp - rng.uniform(0, 0.1, state.shape), np.nan), stamp,
                            corr, 0, 1, {"high": stamp}, -2.5, -2.5, 0.05)
        mem.apply(snap, (0.01 * n, 0.0, 0.0), Footprint(0.944, 0.17, 0.36),
                  extra_support={(55, 52): stamp - 0.05} if n == 4 else None)
    perf.stream_report()
""")


def record(folder: Path, limit: int) -> Path:
    env = {**os.environ, "PYTHONPATH": str(ROOT / "src"), "FORKLIFT_PERF_DUMP": str(folder),
           "FORKLIFT_PERF_DUMP_STREAM": f"shadow_apply={limit}", "FORKLIFT_PERF_DUMP_EVERY": "0",
           "FORKLIFT_PERF_DUMP_STRATA_K": "0"}
    env.pop("FORKLIFT_SHADOW_KERNEL", None)
    subprocess.run([sys.executable, "-c", SESSION], env=env, check=True, timeout=120)
    return folder


def run(folder: Path, tmp_path: Path, monkeypatch) -> dict:
    out = tmp_path / "out.json"
    monkeypatch.setattr(sys, "argv", ["g3_1c_stream.py", str(folder), "--json", str(out), "--modes", *MODES])
    rc = tool.main()
    j = json.loads(out.read_text())
    assert rc == (0 if j["valid"] else 1)
    return j


@pytest.fixture(scope="module")
def whole(tmp_path_factory):
    return record(tmp_path_factory.mktemp("s") / "dump", 100)


def test_the_marker_counts_every_call_and_the_records(tmp_path):
    folder = record(tmp_path / "dump", 4)
    (marker,) = folder.glob("stream_totals_*.json")
    (row,) = json.loads(marker.read_text())
    assert (row["name"], row["calls"], row["records"], row["limit"], row["exhausted"]) == ("shadow_apply", 6, 4, 4,
                                                                                           True)
    assert len(list((folder / "shadow_apply").glob("*.pkl.gz"))) == 4
    assert not (folder / "grid_snapshot").exists()


def test_a_whole_stream_replays_equal_in_every_mode(whole, tmp_path, monkeypatch):
    j = run(whole, tmp_path, monkeypatch)
    assert j["valid"] and j["unstreamed_records"] == 0 and j["missing_receivers"] == []
    (r,) = j["receivers"]
    assert (r["replayed"], r["invalid"], r["mismatches"]) == (6, None, 0)
    cover = r["coverage"]
    assert cover["startup"] == 1 and cover["move"] == 1 and cover["extra_support"] == 1
    assert set(r["time_s"]) == {"off", *(f"on_{m}" for m in MODES)}


def test_a_prefix_a_gap_a_lost_last_call_or_no_marker_is_refused(whole, tmp_path, monkeypatch):
    import shutil

    prefix = record(tmp_path / "prefix", 4)
    assert "limit reached" in run(prefix, tmp_path, monkeypatch)["receivers"][0]["invalid"]
    files = sorted((whole / "shadow_apply").glob("*.pkl.gz"))
    for name, drop in (("gap", files[2]), ("last", files[-1])):
        copy = tmp_path / name
        shutil.copytree(whole, copy)
        (copy / "shadow_apply" / drop.name).unlink()
        j = run(copy, tmp_path, monkeypatch)
        assert not j["valid"]
        reason = j["receivers"][0]["invalid"]
        assert ("seq 3 where 2 was due" in reason) if name == "gap" else ("5 replayed" in reason)
    bare = tmp_path / "bare"
    shutil.copytree(whole, bare)
    for marker in bare.glob("stream_totals_*.json"):
        marker.unlink()
    assert run(bare, tmp_path, monkeypatch)["receivers"][0]["invalid"] == "no completeness marker for this receiver"


def test_the_replay_leaves_the_switch_and_the_mode_as_they_were(whole, tmp_path, monkeypatch):
    from forklift_core import kernels
    from forklift_core.control import shadow_memory as sm

    mode = kernels.mode()
    run(whole, tmp_path, monkeypatch)
    assert sm.SHADOW_KERNEL is False and kernels.mode() == mode
    assert sm.ShadowMemory._apply_arrays is tool._ARRAYS and sm.ShadowMemory._move_arrays is tool._MOVE_ARRAYS
