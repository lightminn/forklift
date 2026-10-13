"""tools/d5_v11_replay.py on a synthetic run directory (D5 v11 plan, check 1).

The truck stands at the held-frame origin with the pallet face 2 m ahead (its boxes stay outside
the check's region, so every tick is permitted); the truth records 0.1 s of observe then
approach at 120 Hz, the near tracker one episode built at 0.05 s with an accepted 10 Hz frame
every 0.1 s and a tick record for every approach tick. Each test breaks one input and reads the
episode's status.
"""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np

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
              inf_steer_rows=(), inf_tick_yaw=(), inf_frame_pose=(), built_s=0.05, nan_read=(), face_x=2.0) -> Path:
    run = root / "run"
    (run / "pocket_frames").mkdir(parents=True)
    t = np.round(np.arange(0.0, END_S + 1e-9, TICK), 9)
    phase = np.where(t < 0.1 - 1e-9, 1, 2)  # observe, then approach
    base = np.tile([0.0, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0], (len(t), 1))
    for k in nan_rows:
        base[k, 0] = np.nan
    steer = np.zeros((len(t), 2))
    for k in inf_steer_rows:
        steer[k, 0] = np.inf
    np.savez(run / "pocket_frames" / "truth.npz", stamp_s=t, phase=phase,
             phases=np.array(["other", "observe", "approach", "insert"]), base_pose=base,
             lift_m=np.zeros(len(t)), applied_speed_mps=np.zeros(len(t)), applied_steering_rad=steer)
    stamps = [0.15, 0.25, 0.35, 0.45, 0.55]
    reads = []
    for k, s in enumerate(stamps):
        name = f"frame_{k:05d}.npz"
        np.savez(run / "pocket_frames" / name, depth_m=np.full(SHAPE, np.nan, dtype=np.float32))
        if k not in drop_reads:
            reads.append({"index": len(reads), "stamp_s": s, "file": name})
    (run / "pocket_frames" / "index.json").write_text(json.dumps({"reads": reads}))
    np.save(run / "perception_capture_1_depth_m.npy", np.full(SHAPE, np.nan))
    held = {"left": [face_x, 0.186, 0.061], "right": [face_x, -0.186, 0.061], "yaw": 0.0}
    obs = {"left": {"center_m": held["left"]}, "right": {"center_m": held["right"]}, "insertion_yaw_rad": 0.0}
    frames = [{"read_s": np.nan if k in nan_read else s + TICK, "episode": 1, "stamp_s": s,
               "pose": [0.0, 0.0, np.inf if k in inf_frame_pose else 0.0], "lift_m": 0.0,
               "accepted": True, "latest_held": held, "latest_bound": BOUND} for k, s in enumerate(stamps)]
    # The runner's skipped rows carry no stamp: a re-read (not_new) and, after a drop latch, dropped reads.
    frames.insert(2, {"read_s": stamps[1] + 2 * TICK, "episode": 1, "skipped": "not_new"})
    if drop_latch is not None:
        frames = [f if f["read_s"] < drop_latch else {"read_s": f["read_s"], "episode": 1, "skipped": "dropped",
                                                       "latch_s": drop_latch} for f in frames]
    ticks = [{"t": float(x), "base": [0.0, 0.0, np.inf if k in inf_tick_yaw else 0.0], "episode": 1}
             for k, x in enumerate(t) if phase[k] == 2 and k not in drop_ticks]
    (run / "near_tracking.json").write_text(json.dumps({"frames": frames, "ticks": ticks, "events": list(events)}))
    near = {"starts": [{"episode": 1, "built_s": built_s, "pose": [0.0, 0.0, 0.0], "attempt": 1,
                        "capture_stamp_s": 0.06, "observation": obs}]}
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
    }
    (run / "result.json").write_text(json.dumps(result))
    return run


def replay(run: Path) -> dict:
    data = json.loads((ROOT / "config/near_field_bounds_measured.json").read_text())
    bounds = REPLAY.NearFieldBounds.from_dict(data)
    return REPLAY.replay(REPLAY.Run(run), bounds, tuple(data["latency"]["band_s"]), ROOT, {})


def episode(run: Path) -> dict:
    out = replay(run)
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
