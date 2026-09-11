"""Validate actual ROS sensor messages live, directly from bag, or fresh replay."""

import argparse
import json
import os
import time
import traceback
from pathlib import Path

from .observation import TOPICS, ExperimentWindow, validate_bag_inventory


def write_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def stored_bag(args, state):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"),
    )
    types = {
        item.name: get_message(item.type) for item in reader.get_all_topics_and_types()
    }
    reverse = {topic: kind for kind, topic in TOPICS.items()}
    if set(types) != set(reverse):
        raise ValueError(f"Bag topic set mismatch: {sorted(types)}")
    metadata = reader.get_metadata()
    declared_counts = {
        item.topic_metadata.name: item.message_count
        for item in metadata.topics_with_message_count
    }
    observed_counts = dict.fromkeys(types, 0)
    rows = 0
    first_record = None
    last_record = None
    deadline = time.monotonic() + args.wall_timeout
    while reader.has_next():
        if time.monotonic() > deadline:
            raise TimeoutError("Stored bag validation wall timeout")
        topic, data, record_stamp = reader.read_next()
        observed_counts[topic] += 1
        if first_record is None:
            first_record = record_stamp
        if last_record is not None and record_stamp < last_record:
            raise ValueError("Bag storage timestamp order regressed")
        last_record = record_stamp
        state.accept(reverse[topic], deserialize_message(data, types[topic]))
        rows += 1
    validate_bag_inventory(declared_counts, observed_counts, metadata.message_count)
    result = state.finish()
    result["storage"] = {
        "declared_rows": metadata.message_count,
        "declared_topic_counts": declared_counts,
        "observed_topic_counts": observed_counts,
        "rows": rows,
        "first_record_ns": first_record,
        "last_record_ns": last_record,
        "span_s": (last_record - first_record) / 1e9,
    }
    if result["storage"]["span_s"] < args.duration:
        raise ValueError("Bag storage duration is truncated")
    return result


def observe_ros(args, state):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from tf2_msgs.msg import TFMessage

    rclpy.init()
    node = rclpy.create_node(
        f"sensor_validator_{args.phase}",
        parameter_overrides=[Parameter("use_sim_time", value=True)],
    )
    # All recorded bridge publishers offer RELIABLE. Request retransmission for
    # complete large-image replay instead of silently losing UDP fragments.
    qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
    static_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    subscriptions = []
    errors = []

    def receive(kind, msg):
        if not errors:
            try:
                state.accept(kind, msg)
            except Exception as exc:
                errors.append(f"{type(exc).__name__}: {exc}")

    for kind, msg_type in [
        ("rgb", Image),
        ("depth", Image),
        ("camera_info", CameraInfo),
        ("scan", LaserScan),
        ("clock", Clock),
        ("tf_static", TFMessage),
    ]:
        subscriptions.append(
            node.create_subscription(
                msg_type,
                TOPICS[kind],
                lambda msg, kind=kind: receive(kind, msg),
                static_qos if kind == "tf_static" else qos,
            )
        )
    before = {}
    start = time.monotonic()
    try:
        if args.phase == "replay":
            # Fresh DDS graph: original publishers must already be gone.
            until = time.monotonic() + 2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=0.05)
            before = {topic: node.count_publishers(topic) for topic in TOPICS.values()}
            if any(before.values()) or any(state.stamps.values()) or state.transforms:
                raise ValueError(
                    f"Replay is contaminated by original publishers/state: {before}"
                )
        write_json(
            args.output / "ready.json",
            {
                "pid": os.getpid(),
                "phase": args.phase,
                "publishers_before_replay": before,
            },
        )
        last_progress = 0
        result = None
        while rclpy.ok() and time.monotonic() - start < args.wall_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if errors:
                raise ValueError(errors[0])
            now = time.monotonic()
            if now - last_progress >= 1:
                write_json(
                    args.output / "progress.json",
                    {
                        "counts": {
                            name: len(stamps) for name, stamps in state.stamps.items()
                        },
                        "spans_s": {
                            name: (stamps[-1] - stamps[0]) / 1e9 if stamps else 0
                            for name, stamps in state.stamps.items()
                        },
                        "tf": list(state.transforms),
                        "wall_elapsed_s": now - start,
                    },
                )
                last_progress = now
            if args.phase == "replay" and not (args.output / "playback_done").exists():
                continue
            try:
                result = state.finish()
            except ValueError:
                if args.phase == "replay" and (args.output / "playback_done").exists():
                    raise
                continue
            if args.phase == "live" and any(
                item["span_s"] < args.duration + 2
                for item in result["streams"].values()
            ):
                continue
            result["publishers_before_replay"] = before
            return result
        raise TimeoutError(
            f"{args.phase} did not complete within {args.wall_timeout}s; last counts="
            + str({k: len(v) for k, v in state.stamps.items()})
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=["live", "bag", "replay"], required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--wall-timeout", type=float, default=300.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    state = ExperimentWindow(args.duration)
    result = {
        "passed": False,
        "phase": args.phase,
        "source_provenance": "synthetic",
        "pid": os.getpid(),
    }
    try:
        observed = (
            stored_bag(args, state) if args.phase == "bag" else observe_ros(args, state)
        )
        result.update(observed)
        state.export_pngs(args.output)
    except Exception as exc:
        result["passed"] = False
        result["error"] = f"{type(exc).__name__}: {exc}"
        result["traceback"] = traceback.format_exc()
        result["observed_counts"] = {
            name: len(stamps) for name, stamps in state.stamps.items()
        }
    write_json(args.output / "result.json", result)
    print(json.dumps(result, allow_nan=False), flush=True)
    raise SystemExit(0 if result["passed"] else 1)
