"""RTAB-Map front and back ends for one rig_slam_bridge mode.

    ros2 launch <this file> mode:=fusion output:=/out

Plan: docs/plans/2026-10-07-visual-slam-and-fusion.md D3. The bridge
(forklift_ros.rig_slam_bridge, started separately with the same mode) feeds
/rig/rgbd_images and /scan frame by frame and the keyframes on /rig/key/*.

| mode          | front ends (each predicts from the wheels, no TF) | back end registration |
|---------------|-----------------------------------------------|-----------------------|
| lidar         | icp_odometry                                  | ICP                   |
| vision        | rgbd_odometry                                 | visual                |
| vision_noodom | rgbd_odometry, no prediction, no wheels       | visual                |
| fusion        | rgbd_odometry and icp_odometry                | visual + ICP          |

The bridge fuses wheel and front end motion into the ``fused`` frame
(forklift_core odometry_fusion) and publishes fused->odom; rtabmap uses it as
its odometry frame.
"""

import json
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CONFIG = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config",
    "rtabmap_rig.yaml",
)


def _nodes(context):
    mode = LaunchConfiguration("mode").perform(context)
    output = LaunchConfiguration("output").perform(context)
    # Development overrides (seed 0 only, plan D3), JSON objects of RTAB-Map
    # parameters; the frozen configuration leaves them empty.
    override = {
        name: json.loads(LaunchConfiguration(f"{name}_params").perform(context))
        for name in ("vo", "icp", "slam")
    }
    vision = mode in ("vision", "vision_noodom", "fusion")
    lidar = mode in ("lidar", "fusion")
    nodes = []
    if vision:
        guess = {} if mode == "vision_noodom" else {"guess_frame_id": "odom"}
        nodes.append(
            Node(
                package="rtabmap_odom",
                executable="rgbd_odometry",
                name="rgbd_odometry",
                namespace="rig/vo",
                parameters=[
                    CONFIG,
                    {"frame_id": "base_link", "odom_frame_id": "vo", **guess},
                    override["vo"],
                ],
                remappings=[("rgbd_images", "/rig/rgbd_images")],
                arguments=["--ros-args", "--log-level", "warn"],
                output="screen",
            )
        )
    if lidar:
        nodes.append(
            Node(
                package="rtabmap_odom",
                executable="icp_odometry",
                name="icp_odometry",
                namespace="rig/icp",
                parameters=[
                    CONFIG,
                    {
                        "frame_id": "base_link",
                        "odom_frame_id": "icp_odom",
                        "guess_frame_id": "odom",
                    },
                    override["icp"],
                ],
                remappings=[("scan", "/scan")],
                arguments=["--ros-args", "--log-level", "warn"],
                output="screen",
            )
        )
    registration = {"lidar": "1", "fusion": "2"}.get(mode, "0")
    back = {
        "frame_id": "base_link",
        "odom_frame_id": "fused",
        "map_frame_id": "map",
        "subscribe_rgbd": vision,
        "rgbd_cameras": 0,
        "subscribe_scan": lidar,
        "database_path": os.path.join(output, "rtabmap.db"),
        "Reg/Strategy": registration,
        # Odometry links are refined by the back end's registration when the
        # LiDAR is there (the usual 2D-LiDAR setup); vision odometry is
        # already visual.
        "RGBD/NeighborLinkRefining": "true" if lidar else "false",
        "RGBD/ProximityPathMaxNeighbors": "10" if lidar else "0",
        "Grid/Sensor": "0" if lidar else "1",
    }
    nodes.append(
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            namespace="rtabmap",
            parameters=[CONFIG, back, override["slam"]],
            remappings=[
                ("rgbd_images", "/rig/key/rgbd_images"),
                ("scan", "/rig/key/scan"),
            ],
            arguments=["-d", "--ros-args", "--log-level", "warn"],
            output="screen",
        )
    )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode"),
            DeclareLaunchArgument("output"),
            DeclareLaunchArgument("vo_params", default_value="{}"),
            DeclareLaunchArgument("icp_params", default_value="{}"),
            DeclareLaunchArgument("slam_params", default_value="{}"),
            OpaqueFunction(function=_nodes),
        ]
    )
