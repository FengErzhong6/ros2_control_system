from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("data_collection_bringup")

    default_recipe_directory = PathJoinSubstitution([bringup_share, "config", "recipes"])
    default_site_config_root = PathJoinSubstitution(
        [bringup_share, "config", "site", LaunchConfiguration("site_name")]
    )
    default_operator_config = PathJoinSubstitution(
        [bringup_share, "config", "operators", LaunchConfiguration("operator_profile")]
    )
    default_startup_policy = PathJoinSubstitution(
        [bringup_share, "config", "policies", "startup.yaml"]
    )
    default_fault_policy = PathJoinSubstitution([bringup_share, "config", "policies", "fault.yaml"])
    default_ui_config = PathJoinSubstitution([bringup_share, "config", "session", "ui.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("recipe_id", default_value="marvin_tracker_collection"),
            DeclareLaunchArgument("site_name", default_value="default_lab"),
            DeclareLaunchArgument("operator_profile", default_value="default.yaml"),
            DeclareLaunchArgument("use_ui", default_value="false"),
            DeclareLaunchArgument("recipe_directory", default_value=default_recipe_directory),
            DeclareLaunchArgument("site_config_root", default_value=default_site_config_root),
            DeclareLaunchArgument("operator_config", default_value=default_operator_config),
            DeclareLaunchArgument("startup_policy_config", default_value=default_startup_policy),
            DeclareLaunchArgument("fault_policy_config", default_value=default_fault_policy),
            DeclareLaunchArgument("ui_config", default_value=default_ui_config),
            Node(
                package="data_collection_orchestrator",
                executable="supervisor",
                name="data_collection_supervisor",
                output="screen",
                parameters=[
                    {
                        "recipe_id": LaunchConfiguration("recipe_id"),
                        "recipe_directory": LaunchConfiguration("recipe_directory"),
                        "site_config_root": LaunchConfiguration("site_config_root"),
                        "operator_config": LaunchConfiguration("operator_config"),
                        "startup_policy_config": LaunchConfiguration("startup_policy_config"),
                        "fault_policy_config": LaunchConfiguration("fault_policy_config"),
                    }
                ],
            ),
            Node(
                package="data_collection_ui",
                executable="ui_stub",
                name="data_collection_ui_stub",
                output="screen",
                parameters=[
                    {
                        "recipe_id": LaunchConfiguration("recipe_id"),
                        "recipe_directory": LaunchConfiguration("recipe_directory"),
                        "site_config_root": LaunchConfiguration("site_config_root"),
                        "ui_config": LaunchConfiguration("ui_config"),
                    }
                ],
                condition=IfCondition(LaunchConfiguration("use_ui")),
            ),
        ]
    )
