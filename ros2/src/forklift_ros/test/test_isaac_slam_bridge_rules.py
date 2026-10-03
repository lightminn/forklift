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
