"""Publish only the explicitly synthetic static mounting transforms."""

import argparse
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())
    if config["source_provenance"] != "synthetic":
        raise ValueError("Static mounting must be explicitly synthetic")
    rclpy.init()
    node = rclpy.create_node("synthetic_mounting")
    broadcaster = StaticTransformBroadcaster(node)
    transforms = []
    for item in config["transforms"]:
        msg = TransformStamped()
        msg.header.frame_id = item["parent"]
        msg.child_frame_id = item["child"]
        (
            msg.transform.translation.x,
            msg.transform.translation.y,
            msg.transform.translation.z,
        ) = map(float, item["translation_m"])
        (
            msg.transform.rotation.x,
            msg.transform.rotation.y,
            msg.transform.rotation.z,
            msg.transform.rotation.w,
        ) = map(float, item["quaternion_xyzw"])
        transforms.append(msg)
    broadcaster.sendTransform(transforms)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
