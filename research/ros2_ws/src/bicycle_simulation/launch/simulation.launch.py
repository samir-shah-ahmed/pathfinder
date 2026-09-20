from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("bicycle_simulation"))
    config = LaunchConfiguration("config")
    parameters = [{"config": config, "use_sim_time": True}]
    recorder = Node(
        package="bicycle_simulation",
        executable="recorder",
        namespace="bicycle",
        parameters=parameters + [{"output": LaunchConfiguration("output")}],
        output="screen",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(share / "config/simulation.json")),
            DeclareLaunchArgument("output", default_value="runs/ros-latest"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("controller", default_value="true"),
            DeclareLaunchArgument("real_time_factor", default_value="1.0"),
            DeclareLaunchArgument("watchdog_seconds", default_value="2.0"),
            Node(
                package="bicycle_simulation",
                executable="plant",
                namespace="bicycle",
                parameters=parameters
                + [
                    {
                        "real_time_factor": ParameterValue(
                            LaunchConfiguration("real_time_factor"), value_type=float
                        ),
                        "watchdog_seconds": ParameterValue(
                            LaunchConfiguration("watchdog_seconds"), value_type=float
                        ),
                    }
                ],
                output="screen",
            ),
            Node(
                package="bicycle_simulation",
                executable="estimator",
                namespace="bicycle",
                parameters=parameters,
                output="screen",
            ),
            Node(
                package="bicycle_simulation",
                executable="controller",
                namespace="bicycle",
                condition=IfCondition(LaunchConfiguration("controller")),
                parameters=parameters,
                output="screen",
            ),
            recorder,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace="bicycle",
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": (share / "urdf/bicycle.urdf").read_text(),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                condition=IfCondition(LaunchConfiguration("rviz")),
                arguments=["-d", str(share / "config/bicycle.rviz")],
                parameters=[{"use_sim_time": True}],
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=recorder,
                    on_exit=[EmitEvent(event=Shutdown(reason="Recording finished"))],
                )
            ),
        ]
    )
