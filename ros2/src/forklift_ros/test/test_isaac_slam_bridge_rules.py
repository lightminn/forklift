"""The bridge's reply-status rule without ROS (plan v3 warm-up)."""

import importlib.util
import sys
import types
from pathlib import Path


def _rules():
    # Import only the rule class: stub the ROS modules the file imports.
    for name in (
        "rclpy", "rclpy.node", "rclpy.executors", "rclpy.qos", "rclpy.callback_groups", "builtin_interfaces.msg",
        "geometry_msgs.msg", "nav_msgs.msg", "rosgraph_msgs.msg", "sensor_msgs.msg", "tf2_msgs.msg",
    ):
        module = types.ModuleType(name)
        for attr in ("Node", "MultiThreadedExecutor", "DurabilityPolicy", "QoSProfile", "ReliabilityPolicy",
                     "Time", "PoseWithCovarianceStamped", "TransformStamped", "OccupancyGrid", "Odometry",
                     "Clock", "LaserScan", "TFMessage", "MutuallyExclusiveCallbackGroup"):
            setattr(module, attr, type(attr, (), {}))
        sys.modules.setdefault(name, module)
    path = Path(__file__).resolve().parents[1] / "forklift_ros/isaac_slam_bridge.py"
    spec = importlib.util.spec_from_file_location("bridge_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_bridge_sends_keyframes_to_a_thresholds_zero_slam_toolbox():
    """Plan v3.4: the gate is the bridge's; slam_toolbox answers every scan sent."""
    module = _rules()
    assert not hasattr(module, "WarmupState")
    assert module.slam_link.KeyframeGate().min_interval_ns == 500_000_000
    import yaml

    config = Path(__file__).resolve().parents[2] / (
        "forklift_bringup/config/slam_toolbox_isaac_online.yaml"
    )
    params = yaml.safe_load(config.read_text())["slam_toolbox"]["ros__parameters"]
    assert params["minimum_travel_distance"] == 0.0
    assert params["minimum_travel_heading"] == 0.0
    assert params["minimum_time_interval"] == 0.0


def _grid(value=0, shape=(40, 50)):
    import numpy as np

    return np.full(shape, value, dtype=np.int8)


def _add(store, data, *, stamp_ns=1, after=0, origin=(0.0, 0.0, 0.0), frame_id="map"):
    store.add(stamp_ns=stamp_ns, resolution_m=0.05, width=data.shape[1], height=data.shape[0], origin=list(origin),
              data=data, after_scan_id=after, received_wall_s=float(stamp_ns), frame_id=frame_id)


def test_a_batch_bridge_keeps_every_map_and_saves_them_as_before(tmp_path):
    import json

    import numpy as np

    store = _rules().MapStore(tmp_path, persistent=False)
    _add(store, _grid(0), stamp_ns=1)
    _add(store, _grid(0), stamp_ns=2)
    store.save()
    maps = json.loads((tmp_path / "maps.json").read_text())
    assert [m["index"] for m in maps] == [0, 1]
    with np.load(tmp_path / "maps.npz") as arrays:
        assert sorted(arrays.files) == ["map_0000", "map_0001"]
    with np.load(tmp_path / "latest_map.npz") as latest:
        assert int(latest["index"]) == 1 and int(latest["stamp_ns"]) == 2


def test_a_persistent_bridge_keeps_no_map_in_memory_and_a_repeat_costs_nothing(tmp_path):
    """Plan D8b: slam_toolbox publishes the map every second; a session left standing got the same grid again each
    time and kept them all (10.6 MB a minute). A repeat bumps a count; a changed map goes to disk."""
    import json

    import numpy as np

    store = _rules().MapStore(tmp_path, persistent=True)
    for k in range(100):
        _add(store, _grid(0), stamp_ns=k + 1, after=3)
    _add(store, _grid(100), stamp_ns=200, after=9)  # the truck moved: a new map
    _add(store, _grid(100), stamp_ns=201, after=9)
    assert store.map_arrays == {} and store.maps == []
    with np.load(tmp_path / "latest_map.npz") as latest:  # the planner's file as before: every map, its own index
        assert int(latest["index"]) == 101 and int(latest["stamp_ns"]) == 201 and int(latest["after_scan_id"]) == 9
    store.save()
    lines = [json.loads(line) for line in (tmp_path / "maps.jsonl").read_text().splitlines()]
    kept = [line for line in lines if "repeat_of" not in line]
    assert [m["index"] for m in kept] == [0, 100]
    repeats = {line["repeat_of"]: line for line in lines if "repeat_of" in line}
    assert repeats[0]["repeats"] == 99 and repeats[0]["last_index"] == 99 and repeats[100]["repeats"] == 1
    assert sorted(p.name for p in (tmp_path / "maps").iterdir()) == ["map_000000.npz", "map_000100.npz"]
    with np.load(tmp_path / "maps" / "map_000100.npz") as m:
        assert (m["data"] == 100).all()
    assert not (tmp_path / "maps.json").exists() and not (tmp_path / "maps.npz").exists()


def test_a_map_with_the_same_cells_but_a_new_origin_is_a_new_map(tmp_path):
    store = _rules().MapStore(tmp_path, persistent=True)
    _add(store, _grid(0), stamp_ns=1)
    _add(store, _grid(0), stamp_ns=2, origin=(0.05, 0.0, 0.0))
    _add(store, _grid(0, shape=(41, 50)), stamp_ns=3, origin=(0.05, 0.0, 0.0))
    store.save()
    assert len(list((tmp_path / "maps").iterdir())) == 3


def test_persistent_reply_records_go_to_a_file_and_the_verdict_still_counts_failures(tmp_path):
    import json

    records = _rules().ReplyRecords(tmp_path, persistent=True)
    for i in range(50):
        records.append({"scan_id": i, "status": "processed", "stray_pose_stamps_ns": []})
    records.append({"scan_id": 50, "status": "failed", "stray_pose_stamps_ns": [7]})
    assert records.kept == [] and records.failed == 1 and records.any_stray
    records.save()
    lines = (tmp_path / "records.jsonl").read_text().splitlines()
    assert len(lines) == 51 and json.loads(lines[-1])["status"] == "failed"
    assert not (tmp_path / "bridge_records.json").exists()
    (tmp_path / "b").mkdir()  # save() makes the output folder first
    batch = _rules().ReplyRecords(tmp_path / "b", persistent=False)
    batch.append({"scan_id": 0, "status": "skipped", "stray_pose_stamps_ns": []})
    batch.save()
    assert json.loads((tmp_path / "b" / "bridge_records.json").read_text())[0]["status"] == "skipped"


def test_the_container_and_the_live_job_turn_persistence_on():
    root = Path(__file__).resolve().parents[4]
    script = (root / "tools" / "slam_bridge_container.sh").read_text()
    assert '-p persistent:=$PERSIST' in script and '[ "$4" = persistent ] && PERSIST=true' in script
    job = (root / "deploy" / "console" / "isaac_live.sbatch").read_text()
    assert '/run/slam/slam.sock /out "-0.12,0.0,1.05,0.0" persistent' in job


def test_a_repeat_run_keeps_the_first_maps_scan_id_and_stamp_and_a_new_frame_is_a_new_map(tmp_path):
    """Codex D8b round 1 P3: the video reads a map as present after its first after_scan_id; a repeat does not move
    it, and a repeated message keeps its own stamp."""
    import json

    store = _rules().MapStore(tmp_path, persistent=True)
    _add(store, _grid(0), stamp_ns=5, after=4)
    _add(store, _grid(0), stamp_ns=5, after=14)  # TRANSIENT_LOCAL redelivered: same message
    _add(store, _grid(0), stamp_ns=6, after=14, frame_id="odom")
    store.save()
    lines = [json.loads(line) for line in (tmp_path / "maps.jsonl").read_text().splitlines()]
    first = next(line for line in lines if line.get("index") == 0)
    assert first["after_scan_id"] == 4 and first["stamp_ns"] == 5
    run = next(line for line in lines if line.get("repeat_of") == 0)
    assert run["repeats"] == 1 and run["last_after_scan_id"] == 14 and run["last_stamp_ns"] == 5
    assert any(line.get("index") == 2 and line["frame_id"] == "odom" for line in lines)


def test_the_map_callback_cannot_hold_up_the_pose_and_the_reply_goes_before_its_record():
    """Codex D8b round 1 P2: the subscriptions shared the default mutually exclusive group, so compressing a map
    held the /pose callback the reply waits on; and the record was written before sendall."""
    src = (Path(__file__).resolve().parents[1] / "forklift_ros/isaac_slam_bridge.py").read_text()
    assert "callback_group=self.map_group" in src and "self.map_group = MutuallyExclusiveCallbackGroup()" in src
    assert src.index("conn.sendall(slam_link.encode_reply(reply))") < src.index("self.replies.append(record_)")


def test_a_slow_disk_does_not_hold_up_the_next_reply(tmp_path):
    """Codex D8b round 2 P2: the serve thread reads the next scan only after the record is appended; the runner's
    reply limit is 1 s. The file is written by a thread of its own; a full queue drops and counts."""
    import json
    import threading
    import time

    module = _rules()
    gate = threading.Event()
    records = module.ReplyRecords(tmp_path, persistent=True, queue_records=5)
    real = records._write_line

    def slow(line):
        gate.wait(5.0)
        real(line)

    records._write_line = slow
    started = time.monotonic()
    for i in range(8):
        records.append({"scan_id": i, "status": "processed", "stray_pose_stamps_ns": []})
    assert time.monotonic() - started < 0.2  # not one append waited on the disk
    assert records.failed == 0
    gate.set()
    records.save()
    lines = (tmp_path / "records.jsonl").read_text().splitlines()
    assert records.dropped >= 2 and len(lines) + records.dropped == 8
    assert [json.loads(line)["scan_id"] for line in lines] == sorted(json.loads(line)["scan_id"] for line in lines)


def test_a_line_being_written_at_the_deadline_is_counted_and_the_count_stays(tmp_path):
    """Codex D8b round 3 P2-1: the writer has taken a line it has not written when the close gives up."""
    import threading

    module = _rules()
    entered, gate = threading.Event(), threading.Event()
    records = module.ReplyRecords(tmp_path, persistent=True)

    def stuck(line):
        entered.set()
        gate.wait(5.0)

    records._write_line = stuck
    for i in range(3):
        records.append({"scan_id": i, "status": "processed", "stray_pose_stamps_ns": []})
    assert entered.wait(5.0)
    records.save(timeout_s=0.05)
    assert records.unsettled == 3 and records.dropped == 0
    gate.set()
    records._writer.join(5.0)
    assert records.unsettled == 3 and records.dropped == 0  # a late finish does not change what was reported


def test_the_status_says_pending_until_the_records_are_settled(tmp_path):
    """Codex D8b round 3 P2-2: the supervisor removes the container right after the runner ends; a status written
    before the records drained would read as final and clean."""
    import json
    import threading

    module = _rules()
    bridge = module.IsaacSlamBridge.__new__(module.IsaacSlamBridge)
    bridge.output_dir, bridge.cond, bridge.stray = tmp_path, threading.Condition(), []
    bridge.map_lock = threading.Lock()
    bridge.replies = module.ReplyRecords(tmp_path, persistent=True)
    bridge.map_store = module.MapStore(tmp_path, persistent=True)
    seen = {}
    real = bridge.replies.save

    def save(*a, **k):
        seen["during"] = json.loads((tmp_path / "bridge_status.json").read_text())
        real(*a, **k)

    bridge.replies.save = save
    bridge.save()
    assert seen["during"] == {"pending": True, "ok": False}
    final = json.loads((tmp_path / "bridge_status.json").read_text())
    assert final["ok"] is True and "pending" not in final and final["records_dropped"] == 0


def test_the_live_jobs_stop_waits_for_the_bridge_to_finish_before_removing_it():
    root = Path(__file__).resolve().parents[4]
    job = (root / "deploy" / "console" / "isaac_live.sbatch").read_text()
    stop = job[job.index('cat > "$OUT/stop.sh"'):]
    assert stop.index('timeout 45 docker wait "slam_bridge_${JOB}_e$1"') < stop.index('docker rm -f "slam_bridge_${JOB}_e$1"')
