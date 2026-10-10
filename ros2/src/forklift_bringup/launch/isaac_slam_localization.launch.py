"""Replay an Isaac SLAM bag into slam_toolbox localization on a saved map.

    ros2 launch forklift_bringup isaac_slam_localization.launch.py \
        bag:=<replay>/bag output:=<replay>/slam params:=<localization yaml> \
        start_offset:=<s>

Plan: docs/plans/2026-10-08-slam-operating-conditions.md (O2-O4). Same bag and
recorder as isaac_slam_replay.launch.py, but slam_toolbox runs its stock
localization launch: the params file carries mode: localization, the pose
graph (map_file_name) and the first scan's pose in that map (map_start_pose).
start_offset skips that many seconds of the bag, which is how a restart part
way along the route is replayed. delay gives the node time to load the graph
before the first message plays.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam = get_package_share_directory("slam_toolbox")
    output = LaunchConfiguration("output")
    play = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag"),
            "--clock",
            "100",
            "--rate",
            LaunchConfiguration("rate"),
            "--delay",
            LaunchConfiguration("delay"),
            "--start-offset",
            LaunchConfiguration("start_offset"),
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
            DeclareLaunchArgument("params"),
            DeclareLaunchArgument("start_offset", default_value="0.0"),
            DeclareLaunchArgument("rate", default_value="1.0"),
            DeclareLaunchArgument("delay", default_value="10"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(slam, "launch", "localization_launch.py")
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
                    {"use_sim_time": True, "output_dir": output, "map_history": False}
                ],
                output="screen",
            ),
            play,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=play,
                    on_exit=[
                        TimerAction(period=8.0, actions=[EmitEvent(event=Shutdown())])
                    ],
                )
            ),
        ]
    )
