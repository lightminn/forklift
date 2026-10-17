"""tools/d5_v11_replay.py on a synthetic run directory (D5 v11 plan, check 1).

The truck stands at the held-frame origin with the pallet face 2 m ahead (its boxes stay outside
the check's region, so every tick is permitted); the truth records 0.1 s of observe then
approach at 120 Hz, the near tracker one episode built at 0.05 s with an accepted 10 Hz frame
every 0.1 s and a tick record for every approach tick. Each test breaks one input and reads the
episode's status.
"""

import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("d5_v11_replay", ROOT / "tools/d5_v11_replay.py")
REPLAY = importlib.util.module_from_spec(SPEC)
sys.modules["d5_v11_replay"] = REPLAY
SPEC.loader.exec_module(REPLAY)

TICK = 1.0 / 120
END_S = 0.6
BOUND = {"lateral_m": 0.006, "along_m": 0.001, "yaw_rad": 0.001, "wall_m": 0.01, "width_m": 0.01}
SHAPE = (24, 32)
ROWS = len(np.arange(0.0, END_S + 1e-9, TICK))
APPROACH = [k for k in range(ROWS) if k * TICK >= 0.1 - 1e-9]  # truth rows in approach


def write_run(root: Path, *, drop_ticks=(), nan_rows=(), drop_reads=(), events=(), drop_latch=None,
              inf_steer_rows=(), inf_tick_yaw=(), inf_frame_pose=(), built_s=0.05, nan_read=(), face_x=2.0,
              uncovered=False, moving_rows=(), urdf_sha=None, shift_rows=()) -> Path:
    run = root / "run"
    (run / "pocket_frames").mkdir(parents=True)
    t = np.round(np.arange(0.0, END_S + 1e-9, TICK), 9)
    phase = np.where(t < 0.1 - 1e-9, 1, 2)  # observe, then approach
    base = np.tile([0.0, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0], (len(t), 1))
    for k in nan_rows:
        base[k, 0] = np.nan
    for k in shift_rows:  # the truck creeping 1 mm under a zero command
        base[k, 0] += 0.001
    steer = np.zeros((len(t), 2))
    for k in inf_steer_rows:
        steer[k, 0] = np.inf
    np.savez(run / "pocket_frames" / "truth.npz", stamp_s=t, phase=phase,
             phases=np.array(["other", "observe", "approach", "insert"]), base_pose=base,
             lift_m=np.zeros(len(t)), applied_speed_mps=np.array([0.05 if k in moving_rows else 0.0 for k in range(len(t))]),
             applied_steering_rad=steer)
    stamps = [0.15, 0.25, 0.35, 0.45, 0.55]
    reads = []
    for k, s in enumerate(stamps):
        name = f"frame_{k:05d}.npz"
        np.savez(run / "pocket_frames" / name, depth_m=np.full(SHAPE, np.nan, dtype=np.float32))
        if k not in drop_reads:
            reads.append({"index": len(reads), "stamp_s": s, "file": name})
    (run / "pocket_frames" / "index.json").write_text(json.dumps({"reads": reads}))
    capture = np.full(SHAPE, np.nan)
    np.save(run / "perception_capture_1_depth_m.npy", capture)
    capture_s = 0.06
    if uncovered:  # the tracker's first frame, skipped as uncovered (no pose), read before the first accepted one
        # (as recorded: build 0.11 s = the capture window start, capture stamp 0.13 s, the frame at 0.14 s)
        built_s, capture_s = 0.11, 0.13
        np.savez(run / "pocket_frames" / "frame_uncovered.npz", depth_m=np.full(SHAPE, np.nan, dtype=np.float32))
        reads.insert(0, {"index": 0, "stamp_s": 0.14, "file": "frame_uncovered.npz"})
        for k, r in enumerate(reads):
            r["index"] = k
        (run / "pocket_frames" / "index.json").write_text(json.dumps({"reads": reads}))
    held = {"left": [face_x, 0.186, 0.061], "right": [face_x, -0.186, 0.061], "yaw": 0.0}
    obs = {"left": {"center_m": held["left"]}, "right": {"center_m": held["right"]}, "insertion_yaw_rad": 0.0}
    frames = [{"read_s": np.nan if k in nan_read else s + TICK, "episode": 1, "stamp_s": s,
               "pose": [0.0, 0.0, np.inf if k in inf_frame_pose else 0.0], "lift_m": 0.0,
               "accepted": True, "latest_held": held, "latest_bound": BOUND} for k, s in enumerate(stamps)]
    # The runner's skipped rows carry no stamp: a re-read (not_new) and, after a drop latch, dropped reads.
    frames.insert(2, {"read_s": stamps[1] + 2 * TICK, "episode": 1, "skipped": "not_new"})
    if uncovered:
        frames.insert(0, {"read_s": 0.14 + TICK, "episode": 1, "stamp_s": 0.14, "aligned_s": 0.14 - 0.0917,
                          "skipped": "uncovered"})
    if drop_latch is not None:
        frames = [f if f["read_s"] < drop_latch else {"read_s": f["read_s"], "episode": 1, "skipped": "dropped",
                                                       "latch_s": drop_latch} for f in frames]
    ticks = [{"t": float(x), "base": [0.0, 0.0, np.inf if k in inf_tick_yaw else 0.0], "episode": 1}
             for k, x in enumerate(t) if phase[k] == 2 and k not in drop_ticks]
    (run / "near_tracking.json").write_text(json.dumps({"frames": frames, "ticks": ticks, "events": list(events)}))
    terms = {"condition": {"input_files": {"argument obstacle_layer": sha(ROOT / "config/obstacle_layer_single.yaml")}}}
    (root / "drive_terms.json").write_text(json.dumps(terms))
    near = {"starts": [{"episode": 1, "built_s": built_s, "pose": [0.0, 0.0, 0.0], "attempt": 1,
                        "capture_stamp_s": capture_s, "observation": obs}],
            "bounds_sha256": sha(BOUNDS), "drive_terms": str(root / "drive_terms.json"),
            "drive_terms_sha256": sha(root / "drive_terms.json")}
    if drop_latch is not None:
        near["drop_latch"] = {"time_s": drop_latch}
    result = {
        "success": True, "failure_reason": None, "near_tracking": near,
        "arguments": {"rear_axle_offset_m": -0.34, "obstacle_layer": "config/obstacle_layer_single.yaml",
                      "pallet_geometry": "config/pallet_geometry_epal6.yaml",
                      "forklift_urdf": "sim/models/dls08_measured/forklift.urdf", "seed": 0},
        "geometry": {"axle_to_fork_tip_m": 1.29, "carriage_limit_m": 0.346, "pallet_depth_m": 0.6,
                     "inserted_offset_m": 1.26, "unloaded_footprint": {"front_m": 1.29, "rear_m": 0.17,
                                                                       "half_width_m": 0.36}},
        "pocket_camera": {"intrinsics": {"width": SHAPE[1], "height": SHAPE[0], "fx": 23.3, "fy": 23.3,
                                         "cx": 15.5, "cy": 11.5}},
        "perception": {"base_from_optical": {"translation_m": [0.619, 0.0, 0.27], "rotation": [
            [0.0, -0.09983341664682815, 0.9950041652780258], [-1.0, 0.0, 0.0],
            [0.0, -0.9950041652780258, -0.09983341664682815]]}},
        "chassis_model": {"drive_geometry": {"wheelbase_m": 0.66}},
        "forklift_urdf_sha256": urdf_sha or sha(ROOT / "sim/models/dls08_measured/forklift.urdf"),
        "pallet_geometry_sha256": sha(ROOT / "config/pallet_geometry_epal6.yaml"),
        "observation_attempts": [{"depth_sha256": REPLAY.depth_sha256(capture)}],
    }
    (run / "result.json").write_text(json.dumps(result))
    return run


def sha(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


BOUNDS = ROOT / "config/near_field_bounds_measured.json"


def replay(run: Path, **kwargs) -> dict:
    data = json.loads(BOUNDS.read_text())
    bounds = REPLAY.NearFieldBounds.from_dict(data)
    return REPLAY.replay(REPLAY.Run(run), bounds, tuple(data["latency"]["band_s"]), ROOT, {},
                         bounds_sha=sha(BOUNDS), **kwargs)


def episode(run: Path, **kwargs) -> dict:
    out = replay(run, **kwargs)
    assert len(out["episodes"]) == 1
    return out["episodes"][0]


def test_a_complete_standing_record_passes(tmp_path):
    ep = episode(write_run(tmp_path))
    assert ep["status"] == "pass" and ep["unchecked_ticks"] == 0 and ep["ticks"] == len(APPROACH)


def test_approach_ticks_missing_before_the_first_record_leave_the_episode_unverified(tmp_path):
    ep = episode(write_run(tmp_path, drop_ticks=(12, 13, 14)))
    assert ep["status"] == "unverified" and ep["unchecked_ticks"] == 3


def test_a_non_finite_next_tick_is_unverified_not_standing(tmp_path):
    ep = episode(write_run(tmp_path, nan_rows=(31,)))
    assert ep["status"] == "unverified" and ep["nonfinite_ticks"] >= 1


def test_a_recorded_read_without_its_depth_is_unverified(tmp_path):
    ep = episode(write_run(tmp_path, drop_reads=(2,)))
    assert ep["status"] == "unverified" and ep["missing_depth"] == 1


def test_a_loss_end_recorded_as_a_stop_drops_the_store(tmp_path):
    ep = episode(write_run(tmp_path, events=[{"time_s": 0.3, "event": "stop", "reason": "lost"}]))
    assert [e["time_s"] for e in ep["summary"]["events"] if e["event"] == "invalidated"] == [0.3]


def test_a_loss_injection_run_is_checked_up_to_its_drop_latch(tmp_path):
    ep = episode(write_run(tmp_path, drop_latch=0.3))
    assert ep["cut_at_injection_s"] == 0.3 and ep["status"] == "pass"
    assert ep["ticks"] == sum(1 for k in APPROACH if k * TICK < 0.3 - 1e-9)


def test_infinite_steering_tick_pose_or_frame_pose_is_unverified_not_an_error(tmp_path):
    for k, kwargs in enumerate(({"inf_steer_rows": (20,)}, {"inf_tick_yaw": (20,)}, {"inf_frame_pose": (1,)})):
        ep = episode(write_run(tmp_path / str(k), **kwargs))
        assert ep["status"] == "unverified", kwargs
        assert ep["nonfinite_ticks"] + ep["nonfinite_frames"] >= 1, kwargs


def test_a_non_finite_start_or_read_time_leaves_the_run_unverified(tmp_path):
    for k, kwargs in enumerate(({"built_s": float("nan"), "drop_ticks": (12, 13, 14)}, {"nan_read": (0,)})):
        out = replay(write_run(tmp_path / str(k), **kwargs))
        assert out["status"] == "unverified" and out["record_problems"], kwargs


def test_a_coverage_failure_while_standing_blocks_the_episode(tmp_path):
    # The face 1 m ahead puts the standing blades inside the check's region; empty depth certifies
    # nothing, so the standing arc is never covered (check 1: no coverage failure to the end).
    ep = episode(write_run(tmp_path, face_x=1.0))
    assert ep["status"] == "blocked" and ep["blocks_count"] == 0
    assert ep["standing_coverage_count"] == len(APPROACH)


def test_a_run_whose_inputs_are_not_the_replays_files_is_unverified(tmp_path):
    out = replay(write_run(tmp_path / "urdf", urdf_sha="0" * 64))
    assert out["status"] == "unverified" and "forklift_urdf is not the file the run used" in out["record_problems"]
    run = write_run(tmp_path / "capture")
    np.save(run / "perception_capture_1_depth_m.npy", np.full(SHAPE, 10.0))  # not the array the runner hashed
    out = replay(run)
    assert out["status"] == "unverified" and any("near-capture depth" in p for p in out["record_problems"])
    assert replay(write_run(tmp_path / "noman"), require_manifest=True)["record_problems"] == ["no batch manifest"]
    run = write_run(tmp_path / "man")
    good = {"runs": {run.parent.name: REPLAY.run_files_digest(run)}}
    assert replay(run, manifest=good)["status"] == "pass"
    bad = {"runs": {run.parent.name: "0" * 64}}
    assert replay(run, manifest=bad)["record_problems"] == ["run files do not match the batch manifest"]


def test_the_manifest_digest_is_the_judges():
    spec = importlib.util.spec_from_file_location("s4_judge_for_replay", ROOT / "tools/s4_judge.py")
    judge = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(judge)
    assert judge.run_files_digest.__doc__  # the function the batch manifest was written with
    run = ROOT / "tests/fixtures/delivery_match"  # any directory: the digest names missing files with "-"
    assert REPLAY.run_files_digest(run) == judge.run_files_digest(run)


def test_the_uncovered_first_frame_is_placed_by_the_anchor_only_while_standing(tmp_path):
    standing = (0.0005, 0.0001)
    ep = episode(write_run(tmp_path / "a", uncovered=True), standing=standing)
    assert ep["anchor_placed"] == 1 and ep["status"] == "pass"
    ep = episode(write_run(tmp_path / "b", uncovered=True))
    assert ep["anchor_placed"] == 0 and ep["anchor_refused"] == {"no_standing_bound": 1}
    # anchor 0.13 - L_hi = 0.0311 s to the pixel end 0.14 - L_lo = 0.0556 s: the record at 0.0583 s
    # holds the step that contains the pixel end (Codex S0b P2) -- a command there is motion.
    ep = episode(write_run(tmp_path / "c", uncovered=True, moving_rows=(7,)), standing=standing)
    assert ep["anchor_placed"] == 0 and ep["anchor_refused"] == {"moving": 1}
    ep = episode(write_run(tmp_path / "d", uncovered=True, shift_rows=(5,)), standing=standing)
    assert ep["anchor_refused"] == {"standing_bound_exceeded": 1}  # 1 mm of true creep > d0 0.5 mm
    # Creep on the record that closes the step holding the pixel end (0.0583 s) counts too, and a
    # non-finite pose in the span never passes (Codex S0b round 2).
    ep = episode(write_run(tmp_path / "d2", uncovered=True, shift_rows=(7,)), standing=standing)
    assert ep["anchor_refused"] == {"standing_bound_exceeded": 1}
    run = REPLAY.Run(write_run(tmp_path / "d3", uncovered=True, nan_rows=(5,)))
    assert run.standing_motion(0.0311, 0.0556, 0.34) == (math.inf, math.inf)
    # An anchor before the truth record (the default fixture's capture at 0.06 s: 0.06 - L_hi < 0)
    # is not known to be standing.
    run = REPLAY.Run(write_run(tmp_path / "e"))
    assert not run.command_zero(-0.039, 0.0156) and run.command_zero(0.0, 0.0156)


def test_the_truck_split_covers_the_measured_urdf_over_the_steering_range():
    sys.path.insert(0, str(ROOT / "sim" / "isaac"))
    import pocket_check_v11 as V11

    urdf = ROOT / "sim/models/dls08_measured/forklift.urdf"
    split = V11.truck_split_from_urdf(urdf, 0.34, 0.17, 0.36, 0.944, 0.235)
    # Steered 15 degrees the front wheel reaches 0.451093 m in base_link (the rim polygon is
    # circumscribed, so a little more); everything ahead is within the carriage half-width.
    assert 0.791093 <= split["chassis_front_m"] < 0.7915
    assert split["carriage_half_width_m"] == 0.235
    with pytest.raises(ValueError):
        V11.truck_split_from_urdf(urdf, 0.34, 0.17, 0.30, 0.944, 0.235)  # a body box narrower than the wheels
