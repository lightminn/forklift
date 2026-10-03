"""The bridge's reply-status rule without ROS (plan v3 warm-up)."""

import importlib.util
import sys
import types
from pathlib import Path


def _rules():
    # Import only the rule class: stub the ROS modules the file imports.
    for name in (
        "rclpy", "rclpy.node", "rclpy.executors", "rclpy.qos", "builtin_interfaces.msg",
        "geometry_msgs.msg", "nav_msgs.msg", "rosgraph_msgs.msg", "sensor_msgs.msg", "tf2_msgs.msg",
    ):
        module = types.ModuleType(name)
        for attr in ("Node", "MultiThreadedExecutor", "DurabilityPolicy", "QoSProfile", "ReliabilityPolicy",
                     "Time", "PoseWithCovarianceStamped", "TransformStamped", "OccupancyGrid", "Odometry",
                     "Clock", "LaserScan", "TFMessage"):
            setattr(module, attr, type(attr, (), {}))
        sys.modules.setdefault(name, module)
    path = Path(__file__).resolve().parents[1] / "forklift_ros/isaac_slam_bridge.py"
    spec = importlib.util.spec_from_file_location("bridge_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.WarmupState


def test_missing_poses_after_the_first_are_warmup_three_times_then_failure():
    state = _rules()()
    assert state.on_missing() == "failed"  # before any processed scan
    state = _rules()()
    state.on_processed(0)
    assert [state.on_missing() for _ in range(4)] == ["warmup", "warmup", "warmup", "failed"]


def test_warmup_ends_with_a_processed_scan_four_or_later():
    state = _rules()()
    state.on_processed(0)
    state.on_missing()
    state.on_processed(4)
    assert not state.warming
    assert state.on_missing() == "failed"
    assert state.wait_s(1.0, 10.0) == 10.0
