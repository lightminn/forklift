"""Replay an Isaac SLAM bag into slam_toolbox and record what it produced.

    ros2 launch forklift_bringup isaac_slam_replay.launch.py \
        bag:=<replay>/bag output:=<replay>/slam

The bag (forklift_ros slam_replay) carries /scan, /tf and /odom stamped in
simulation time; `ros2 bag play --clock` turns that into /clock. slam_toolbox
runs its stock synchronous launch with this package's parameters. When the
bag ends, a short grace period lets the last map update land, then everything
shuts down and slam_recorder writes its files. With serialize:=<path> the pose
graph is first saved there (``<path>.posegraph``/``.data``) for a later
localization replay (docs/plans/2026-10-08-slam-operating-conditions.md O1).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _after_bag(context):
    """Grace period, then (optionally) serialize the pose graph, then shut down."""
    target = LaunchConfiguration("serialize").perform(context)
    shutdown = EmitEvent(event=Shutdown())
    if not target:
        return [TimerAction(period=8.0, actions=[shutdown])]
    save = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/slam_toolbox/serialize_map",
            "slam_toolbox/srv/SerializePoseGraph",
            f"{{filename: '{target}'}}",
        ],
        output="screen",
    )
    return [
        TimerAction(
            period=8.0,
            actions=[
                save,
                RegisterEventHandler(
                    OnProcessExit(target_action=save, on_exit=[shutdown])
                ),
            ],
        )
    ]


def generate_launch_description():
    bringup = get_package_share_directory("forklift_bringup")
    slam = get_package_share_directory("slam_toolbox")
    bag = LaunchConfiguration("bag")
    output = LaunchConfiguration("output")
    play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            bag,
            "--clock",
            "100",
            "--rate",
            LaunchConfiguration("rate"),
            "--delay",
            "3",
            "--topics",
            "/scan",
            "/tf",
            "/odom",
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag"),
            DeclareLaunchArgument("output"),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("map_history", default_value="false"),
            DeclareLaunchArgument("serialize", default_value=""),
            DeclareLaunchArgument(
                "params",
                default_value=os.path.join(
                    bringup, "config", "slam_toolbox_isaac_replay.yaml"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam, "launch", "online_sync_launch.py")
                ),
                launch_arguments={
                    "slam_params_file": LaunchConfiguration("params"),
                    "use_sim_time": "true",
                    "autostart": "true",
                }.items(),
            ),
            Node(
                package="forklift_ros",
                executable="slam_recorder",
                parameters=[
                    {
                        "use_sim_time": True,
                        "output_dir": output,
                        "map_history": LaunchConfiguration("map_history"),
                    }
                ],
                output="screen",
            ),
            play,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=play,
                    on_exit=[OpaqueFunction(function=_after_bag)],
                )
            ),
        ]
    )
