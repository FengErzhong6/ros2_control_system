from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("recipe_id", default_value="marvin_tracker_collection"),
            DeclareLaunchArgument("recipe_directory", default_value=""),
            DeclareLaunchArgument("startup_policy_config", default_value=""),
            DeclareLaunchArgument("fault_policy_config", default_value=""),
            Node(
                package="data_collection_orchestrator",
                executable="supervisor",
                name="data_collection_supervisor",
                output="screen",
                parameters=[
                    {
                        "recipe_id": LaunchConfiguration("recipe_id"),
                        "recipe_directory": LaunchConfiguration("recipe_directory"),
                        "startup_policy_config": LaunchConfiguration("startup_policy_config"),
                        "fault_policy_config": LaunchConfiguration("fault_policy_config"),
                    }
                ],
            ),
        ]
    )
