from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("data_collection_bringup")
    default_recipe_directory = PathJoinSubstitution([bringup_share, "config", "recipes"])
    default_startup_policy = PathJoinSubstitution(
        [bringup_share, "config", "policies", "startup.yaml"]
    )
    default_fault_policy = PathJoinSubstitution([bringup_share, "config", "policies", "fault.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("recipe_id", default_value="marvin_tracker_collection"),
            DeclareLaunchArgument("recipe_directory", default_value=default_recipe_directory),
            DeclareLaunchArgument("startup_policy_config", default_value=default_startup_policy),
            DeclareLaunchArgument("fault_policy_config", default_value=default_fault_policy),
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
