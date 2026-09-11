"""Capture one synchronized synthetic RGB-D frame into the v1 scene format."""

import argparse
import json
import math
import os
import time
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--entry", type=Path, required=True)
    parser.add_argument("--image-id", required=True)
    parser.add_argument("--source-sha256", required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--warmup-s", type=float, default=2.0)
    parser.add_argument("--deadline-s", type=float, default=120.0)
    args = parser.parse_args()
    if (
        not math.isfinite(args.warmup_s)
        or args.warmup_s < 0
        or not math.isfinite(args.deadline_s)
        or args.deadline_s <= 0
    ):
        parser.error("warmup must be nonnegative; deadline must be finite and positive")

    # Importing this module on a host does not require a ROS installation.
    import rclpy
    import yaml
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_msgs.msg import TFMessage

    from .scene_files import (
        CaptureState,
        build_scene_json,
        ground_truth_for_capture,
        write_json,
        write_scene_files,
    )

    start = time.monotonic()
    deadline = start + args.deadline_s
    args.output.mkdir(parents=True, exist_ok=True)
    state = CaptureState(round(args.warmup_s * 1_000_000_000))
    result = {
        "passed": False,
        "stamp_ns": None,
        "counts": state.counts,
        "wall_times_s": {},
        "files": {},
    }
    wall_times = result["wall_times_s"]
    errors = []
    node = None
    initialized = False
    try:
        entry = yaml.safe_load(args.entry.read_text())
        rclpy.init()
        initialized = True
        node = rclpy.create_node(
            "scene_capture", parameter_overrides=[Parameter("use_sim_time", value=True)]
        )
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        static_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        def receive(kind, msg):
            if errors:
                return
            try:
                state.accept(kind, msg)
                if kind == "clock" and "first_clock" not in wall_times:
                    wall_times["first_clock"] = time.monotonic() - start
            except Exception as exc:
                errors.append(f"{type(exc).__name__}: {exc}")

        subscriptions = []
        for kind, msg_type, topic in (
            ("rgb", Image, "/camera/image"),
            ("depth", Image, "/camera/depth_image"),
            ("info", CameraInfo, "/camera/camera_info"),
            ("clock", Clock, "/clock"),
            ("tf_static", TFMessage, "/tf_static"),
        ):
            subscriptions.append(
                node.create_subscription(
                    msg_type,
                    topic,
                    lambda msg, kind=kind: receive(kind, msg),
                    static_qos if kind == "tf_static" else qos,
                )
            )
        wall_times["ready"] = time.monotonic() - start
        write_json(args.output / "ready.json", {"pid": os.getpid()})
        last_progress = float("-inf")
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("scene capture exceeded wall deadline")
            rclpy.spin_once(node, timeout_sec=min(0.05, remaining))
            if errors:
                raise ValueError(errors[0])
            now = time.monotonic()
            if now >= deadline:
                raise TimeoutError("scene capture exceeded wall deadline")
            if now - last_progress >= 1:
                write_json(
                    args.output / "progress.json",
                    {"counts": state.counts, "wall_elapsed_s": now - start},
                )
                last_progress = now
            stamp_ns = state.selected_stamp()
            if stamp_ns is None:
                continue
            wall_times["captured"] = now - start
            result["stamp_ns"] = stamp_ns
            result["files"] = write_scene_files(
                args.output,
                rgb=state.buffers["rgb"][stamp_ns],
                depth_m=state.buffers["depth"][stamp_ns],
                camera_info=state.buffers["info"][stamp_ns],
                tf=state.tf,
                ground_truth=ground_truth_for_capture(entry, stamp_ns),
                scene=build_scene_json(
                    entry,
                    stamp_ns,
                    args.image_id,
                    args.source_sha256,
                    args.run_id,
                    wall_times,
                ),
            )
            if time.monotonic() >= deadline:
                raise TimeoutError("scene file saving exceeded wall deadline")
            result["passed"] = True
            break
        if not result["passed"]:
            raise RuntimeError("ROS shut down before capture completed")
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if initialized and rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            result["passed"] = False
            result["error"] = f"ROS cleanup failed: {type(exc).__name__}: {exc}"
        result["wall_elapsed_s"] = time.monotonic() - start
        write_json(args.output / "result.json", result)
    print(json.dumps(result, allow_nan=False), flush=True)
    raise SystemExit(0 if result["passed"] else 1)


if __name__ == "__main__":
    main()
