from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    user_name = LaunchConfiguration("user_name")
    config = PathJoinSubstitution(
        [FindPackageShare("manus_system"), "config", "manus_calibration.yaml"]
    )

    calibration_node = Node(
        package="manus_system",
        executable="manus_calibration_node",
        name="manus_calibration_node",
        output="screen",
        parameters=[config, {"user_name": user_name}],
    )

    workflow_node = Node(
        package="manus_system",
        executable="manus_calibration_workflow.py",
        name="manus_calibration_workflow_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "user_name",
                default_value="",
                description="User name for calibration storage under bringup/calibrations/<user_name>.",
            ),
            calibration_node,
            workflow_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=workflow_node,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="manus_calibration_workflow_node exited"
                            )
                        )
                    ],
                )
            ),
        ]
    )
