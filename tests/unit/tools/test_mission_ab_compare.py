"""Normalised run-to-run comparison for the M-a equivalence checks (docs/plans/2026-10-15-m-a-mission-core.md D2)."""

import importlib.util
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("mission_ab_compare", ROOT / "tools/mission_ab_compare.py")
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def run_dir(base: Path, name: str, *, wall=1.0, map_index=3, x=0.5, snapshot="snap_a") -> Path:
    run = base / name / "seed_0" / "run"
    run.mkdir(parents=True)
    result = {
        "success": True,
        "phase": "complete",
        "wall_time_s": wall,
        "simulation_wall_s": wall * 2,
        "arguments": {"output": str(run), "slam_feedback": str(run.parent / "sock/slam.sock"),
                      "record_boundary": name == "b",
                      "near_bounds": f"/home/projects/forklift/snapshots/{snapshot}/config/near.json"},
        "source_sha256": {"sim/isaac/run_transport.py": snapshot},
        "observation_attempts": [{"detection_diagnostics": {"elapsed_s": wall / 10},
                                  "capture_diagnostics": {"rejection_counts": {"sensor_frame_not_ready": int(wall)}}}],
        "obstacle_layer": {"grid_plans": [{"slam_map": {"index": map_index, "after_scan_id": 10 + map_index},
                                           "memory_revision": 100 + map_index}],
                           "wall_s_per_scan": {"max": wall, "mean": wall / 2}},
        "final_pallet_m": [x, 1.0, 0.0],
    }
    (run / "result.json").write_text(json.dumps(result))
    np.savez(run / "slam_log.npz", base_pose_world=np.array([[x, 0.0, np.nan]]))
    (run / "paths.json").write_text(json.dumps({"transport": [[0, 0, 0], [1, 0, 0]]}))
    (run / "pocket_frames").mkdir()
    (run / "pocket_frames" / "index.json").write_text(json.dumps({"reads": [{"rendering_frame": [200 + map_index, 1]}]}))
    (run / "pallet.usd").write_bytes(name.encode())  # scene exports differ by path: not compared
    if name == "b":
        (run / "boundary").mkdir()
        (run / "boundary" / "boundary_00000.jsonl").write_text("{}\n")  # a record of the run: not compared
    return run


def test_runs_that_differ_only_in_wall_time_paths_snapshot_and_nondeterministic_reads_compare_equal(tmp_path):
    a = run_dir(tmp_path, "a", wall=1.0, map_index=3, snapshot="snap_a")
    b = run_dir(tmp_path, "b", wall=7.0, map_index=5, snapshot="snap_b")
    rep = M.compare(a, b)
    assert rep["equal"], rep["differences"]
    assert any("slam_map.index" in d for d in rep["nondeterministic"])
    assert any("sensor_frame_not_ready" in d for d in rep["nondeterministic"])
    assert any("memory_revision" in d for d in rep["nondeterministic"])
    assert any("rendering_frame" in d for d in rep["nondeterministic"])
    assert rep["provenance"] == ["result.json: source_sha256.sim/isaac/run_transport.py differs"]
    assert "slam_log.npz" in rep["identical"] and "paths.json" in rep["identical"]


def test_a_decision_difference_is_reported_with_its_path(tmp_path):
    a = run_dir(tmp_path, "a", x=0.5)
    b = run_dir(tmp_path, "b", x=0.5000000001)
    rep = M.compare(a, b)
    assert not rep["equal"]
    assert "result.json: final_pallet_m[0]" in " ".join(rep["differences"])
    assert any(d.startswith("slam_log.npz: base_pose_world") for d in rep["differences"])


def test_arrays_compare_bit_for_bit_and_missing_files_count(tmp_path):
    a = run_dir(tmp_path, "a")
    b = run_dir(tmp_path, "b")
    np.save(a / "tyre_contacts.npy", np.array([0.0]))
    np.save(b / "tyre_contacts.npy", np.array([-0.0]))  # equal as numbers, different bits
    rep = M.compare(a, b)
    assert "tyre_contacts.npy: bits differ" in rep["differences"]
    (b / "paths.json").unlink()
    rep = M.compare(a, b)
    assert "paths.json: only in A" in rep["differences"]
