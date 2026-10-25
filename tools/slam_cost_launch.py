"""slam_toolbox's online_sync_launch.py (2.8.5) with the node executable as a launch argument, so a cost replay can run
lifelong_slam_toolbox_node under the same lifecycle start-up as the sync node (slam-bounded-graph plan candidate B;
slam_toolbox ships no lifelong launch file).

    ros2 launch tools/slam_cost_launch.py executable:=lifelong_slam_toolbox_node slam_params_file:=P use_sim_time:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    node = LifecycleNode(
        parameters=[LaunchConfiguration("slam_params_file"),
                    {"use_lifecycle_manager": False, "use_sim_time": LaunchConfiguration("use_sim_time")}],
        package="slam_toolbox",
        executable=LaunchConfiguration("executable"),
        name="slam_toolbox",
        output="screen",
        namespace="",
    )
    configure = EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                            transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, start_state="configuring", goal_state="inactive",
        entities=[LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                  EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(node),
                                              transition_id=Transition.TRANSITION_ACTIVATE))]))
    return LaunchDescription([
        DeclareLaunchArgument("executable", default_value="sync_slam_toolbox_node"),
        DeclareLaunchArgument("slam_params_file"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        node, configure, activate,
    ])
