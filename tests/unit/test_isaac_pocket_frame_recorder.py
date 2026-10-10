"""Plan D8b recording bookkeeping (no Isaac)."""

import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("pocket_frame_recorder", ROOT / "sim/isaac/pocket_frame_recorder.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["pocket_frame_recorder"] = MODULE
SPEC.loader.exec_module(MODULE)


def test_gaps_parse_as_decreasing_positive_distances():
    assert MODULE.parse_gaps("2.25,2.0,0.75") == (2.25, 2.0, 0.75)
    for bad in ("", "1.0,1.0", "0.5,1.0", "1.0,-0.5", "1.0,nan", "a"):
        with pytest.raises(ValueError):
            MODULE.parse_gaps(bad)


def test_an_unusable_fabric_frame_is_kept_as_text_not_a_crash():
    assert MODULE.fabric_id(np.int64(5)) == 5 and MODULE.fabric_id("frame-a") == "frame-a"
    assert MODULE.fabric_id(None) is None


def test_stamps_are_labelled_in_the_runners_order():
    assert MODULE.classify_stamp(None, 1.0, None) == "no_time"
    assert MODULE.classify_stamp(math.nan, 1.0, None) == "no_time"
    assert MODULE.classify_stamp(1.1, 1.0, None) == "future"
    assert MODULE.classify_stamp(0.4, 1.0, 0.5) == "backwards"
    assert MODULE.classify_stamp(0.5, 1.0, 0.5) == "same_stamp"
    assert MODULE.classify_stamp(0.6, 1.0, 0.5) == "ok"


def test_every_read_is_kept_with_float32_depth_and_its_label(tmp_path):
    recorder = MODULE.PocketFrameRecorder(tmp_path)
    recorder.phase = "approach"
    depth = np.array([[1.0001234, np.nan], [0.5, 2.0]])
    first = recorder.record_read(read_s=1.0, stamp_s=0.98, rendering_frame={"referenceTimeNumerator": 98,
                                 "referenceTimeDenominator": 100}, depth_m=depth, lift_m=0.0,
                                 control_rear=(1.0, 2.0, 0.1), odom_speed_mps=0.05)
    same = recorder.record_read(read_s=1.1, stamp_s=1.08, rendering_frame=7, depth_m=depth, lift_m=0.0)
    late = recorder.record_read(read_s=1.2, stamp_s=1.0, rendering_frame=8, depth_m=depth * 2, lift_m=0.0)
    timeless = recorder.record_read(read_s=1.3, stamp_s=None, rendering_frame=None, depth_m=None, lift_m=0.0)
    assert [r["result"] for r in (first, same, late, timeless)] == ["ok", "ok", "backwards", "no_depth"]
    # A standing frame keeps its depth: the duplicate is labelled, not dropped.
    assert (first["duplicate"], same["duplicate"], late["duplicate"], timeless["file"]) == (False, True, False, None)
    assert first["rendering_frame"] == [98, 100] and first["control_rear"] == [1.0, 2.0, 0.1]
    stored = np.load(tmp_path / "pocket_frames" / first["file"])["depth_m"]
    assert stored.dtype == np.float32
    np.testing.assert_array_equal(stored, depth.astype(np.float32))
    # The order check runs against the last ok stamp, not the backwards one.
    assert recorder.record_read(read_s=1.4, stamp_s=1.05, rendering_frame=9, depth_m=depth,
                                lift_m=0.0)["result"] == "backwards"
    assert recorder.record_read(read_s=1.5, stamp_s=None, rendering_frame=None, depth_m=depth,
                                lift_m=0.0)["result"] == "no_time"
    assert recorder.counts()["no_depth"] == 1 and recorder.counts()["backwards"] == 2


def test_close_writes_the_index_and_the_truth_once(tmp_path):
    recorder = MODULE.PocketFrameRecorder(tmp_path)
    recorder.phase = "insert"
    recorder.record_tick(stamp_s=0.0, rendered=True, base_pose=[0.0] * 3 + [1.0, 0, 0, 0], lift_m=1e-5,
                         pallet_pose=[3.0, 0, 0, 1.0, 0, 0, 0])
    recorder.record_read(read_s=0.0, stamp_s=0.0, rendering_frame=1, depth_m=np.ones((2, 2)), lift_m=0.0)
    summary = recorder.close({"camera": "pocket"}, [{"kind": "start"}])
    assert summary["reads"] == 1 and summary["ok"] == 1 and summary["ticks"] == 1 and summary["dwells"] == 1
    index = json.loads((tmp_path / "pocket_frames" / "index.json").read_text())
    assert index["meta"] == {"camera": "pocket"} and index["reads"][0]["phase"] == "insert"
    truth = np.load(tmp_path / "pocket_frames" / "truth.npz")
    assert truth["base_pose"].shape == (1, 7) and np.isnan(truth["camera_prim_pose"]).all()
    assert list(truth["phases"])[truth["phase"][0]] == "insert"
    assert recorder.close({}, []) == recorder.counts()  # a second close writes nothing new


def drive(schedule, ticks):
    """ticks: (t, d_est, still, arrived) -> list of (hold, released)."""
    return [schedule.update(t, d_est_m=d, still=s, arrived=a) for t, d, s, a in ticks]


def test_the_start_dwell_holds_until_the_truck_has_stood_two_seconds():
    schedule = MODULE.DwellSchedule((1.0,))
    out = drive(schedule, [(0.0, 2.5, True, False), (1.0, 2.5, True, False), (1.999, 2.5, True, False),
                           (2.0, 2.5, True, False), (2.1, 2.5, False, False)])
    assert out == [(True, False), (True, False), (True, False), (False, True), (False, False)]
    assert schedule.records[0]["kind"] == "start" and schedule.records[0]["end_s"] == 2.0


def test_a_gap_dwell_waits_for_the_stop_and_fires_once():
    schedule = MODULE.DwellSchedule((1.0, 0.5), hold_s=1.0)
    schedule.started = True
    out = drive(schedule, [(0.0, 1.2, False, False), (0.1, 0.99, False, False), (0.3, 0.98, True, False),
                           (0.8, 0.98, False, False), (1.2, 0.98, True, False), (1.3, 0.98, True, False),
                           (1.4, 0.97, False, False)])
    # The hold counts from the first stop (0.3 s); a flicker of the stop detector at 0.8 s
    # does not restart it (smoke run 1856).
    assert out == [(False, False), (True, False), (True, False), (True, False), (True, False), (False, True),
                   (False, False)]
    assert [r["kind"] for r in schedule.records] == ["gap"] and schedule.pending == [0.5]
    assert schedule.records[0]["still_since_s"] == 0.3 and schedule.held_s(2.0) == pytest.approx(1.3 - 0.1)


def test_held_time_counts_the_dwell_in_progress():
    schedule = MODULE.DwellSchedule((), hold_s=2.0)
    assert schedule.held_s(0.0) == 0.0
    schedule.update(5.0, d_est_m=2.0, still=True, arrived=False)  # the start dwell begins
    assert schedule.held_s(6.5) == pytest.approx(1.5)


def test_the_arrival_dwell_holds_the_switch_and_does_not_restart_the_slew():
    schedule = MODULE.DwellSchedule((), hold_s=1.0)
    schedule.started = True
    out = drive(schedule, [(10.0, 0.44, True, True), (11.0, 0.44, True, True), (11.1, 0.44, True, True)])
    assert out == [(True, False), (False, False), (False, False)]
    assert schedule.records[0]["kind"] == "arrival" and schedule.arrival_done


def test_a_dwell_needs_a_positive_hold():
    with pytest.raises(ValueError):
        MODULE.DwellSchedule((1.0,), hold_s=0.0)


def test_a_read_without_depth_keeps_its_time_frame_and_control_pose(tmp_path):
    recorder = MODULE.PocketFrameRecorder(tmp_path)
    row = recorder.record_read(read_s=2.0, stamp_s=1.98, rendering_frame=4, depth_m=None, lift_m=0.0,
                               control_rear=(1.0, 0.0, 0.0), control_stamp_s=1.9917, odom_speed_mps=0.3,
                               odom_yaw_rate_rps=0.01, no_depth=True)
    assert (row["result"], row["time_label"], row["stamp_s"], row["rendering_frame"]) == ("no_depth", "ok", 1.98, 4)
    assert row["control_stamp_s"] == 1.9917 and row["odom_yaw_rate_rps"] == 0.01 and row["file"] is None
    bad = recorder.record_read(read_s=2.1, stamp_s=2.08, rendering_frame=5, depth_m=None, lift_m=0.0,
                               error="ValueError('cannot reshape')")
    assert bad["result"] == "bad_depth" and bad["error"].startswith("ValueError")
    # Neither moved the order reference: the next frame at 2.0 s is not "backwards".
    assert recorder.record_read(read_s=2.2, stamp_s=2.0, rendering_frame=6, depth_m=np.ones((2, 2)),
                                lift_m=0.0)["result"] == "ok"


def test_a_failed_close_stays_open_and_a_retry_writes(tmp_path, monkeypatch):
    recorder = MODULE.PocketFrameRecorder(tmp_path)
    recorder.record_tick(stamp_s=0.0, rendered=False, base_pose=[0.0] * 3 + [1.0, 0, 0, 0], lift_m=0.0,
                         pallet_pose=[0.0] * 3 + [1.0, 0, 0, 0], stop_now=True, odom_speed_mps=0.0)
    real = MODULE.np.savez_compressed

    def broken(*args, **kwargs):
        raise OSError("disk full")

    monkeypatch.setattr(MODULE.np, "savez_compressed", broken)
    with pytest.raises(OSError):
        recorder.close({}, [])
    assert not recorder.closed
    monkeypatch.setattr(MODULE.np, "savez_compressed", real)
    recorder.close({"near_capture": {"offset": np.float64(0.02), "pose": np.zeros(3)}}, [])
    truth = np.load(tmp_path / "pocket_frames" / "truth.npz")
    assert truth["stop_now"].tolist() == [1] and np.isnan(truth["odom_yaw_rate_rps"]).all()
    meta = json.loads((tmp_path / "pocket_frames" / "index.json").read_text())["meta"]
    assert meta["near_capture"] == {"offset": 0.02, "pose": [0.0, 0.0, 0.0]}


def test_camera_pose_failures_are_counted_with_the_first_cause(tmp_path):
    recorder = MODULE.PocketFrameRecorder(tmp_path)
    recorder.camera_pose_failed(RuntimeError("prim gone"))
    recorder.camera_pose_failed(RuntimeError("again"))
    counts = recorder.counts()
    assert counts["camera_pose_errors"] == 2 and "prim gone" in counts["camera_pose_first_error"]


def test_safety_stops_hold_from_their_gap_until_after_the_first_stand():
    SafetyStopSchedule = MODULE.SafetyStopSchedule

    s = SafetyStopSchedule((1.5, 1.0), hold_s=0.5)
    assert s.update(0.0, d_est_m=2.0, still=False) == (False, False)
    assert s.update(1.0, d_est_m=1.49, still=False) == (True, False)  # fires, braking
    assert s.update(1.2, d_est_m=1.48, still=True) == (True, False)  # first stand at 1.2 s
    assert s.update(1.5, d_est_m=1.48, still=False) == (True, False)  # a flicker does not restart it
    assert s.update(1.71, d_est_m=1.48, still=True) == (False, True)  # 0.51 s after the stand
    assert s.update(1.8, d_est_m=1.4, still=False) == (False, False)
    assert s.update(3.0, d_est_m=0.99, still=False) == (True, False)
    assert [r["gap_m"] for r in s.records] == [1.5]
    assert s.held_s(3.0) == pytest.approx(0.71)


def test_safety_stop_gaps_must_decrease():
    SafetyStopSchedule = MODULE.SafetyStopSchedule

    with pytest.raises(ValueError):
        SafetyStopSchedule((1.0, 1.5))
    with pytest.raises(ValueError):
        SafetyStopSchedule(())


def test_the_applied_command_is_kept_per_tick(tmp_path):
    rec = MODULE.PocketFrameRecorder(tmp_path)
    pose = [0.0] * 7
    rec.record_tick(stamp_s=0.0, rendered=False, base_pose=pose, lift_m=0.0, pallet_pose=pose,
                    applied_speed_mps=0.055, applied_steering_rad=(0.01, -0.01))
    rec.record_tick(stamp_s=1 / 120, rendered=False, base_pose=pose, lift_m=0.0, pallet_pose=pose)
    rec.close({}, [])
    truth = np.load(tmp_path / "pocket_frames/truth.npz")
    assert truth["applied_speed_mps"][0] == pytest.approx(0.055) and np.isnan(truth["applied_speed_mps"][1])
    assert truth["applied_steering_rad"].shape == (2, 2)
