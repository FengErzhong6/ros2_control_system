from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("recipe_id", default_value="default_policy_deployment"),
            DeclareLaunchArgument("recipe_directory", default_value=""),
            DeclareLaunchArgument("policy_profiles_config", default_value=""),
            DeclareLaunchArgument("default_policy_profile_id", default_value=""),
            DeclareLaunchArgument("operator_id", default_value=""),
            DeclareLaunchArgument("site_name", default_value=""),
            DeclareLaunchArgument("connect_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("policy_step_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("observation_max_image_age_sec", default_value="1.0"),
            DeclareLaunchArgument("observation_max_joint_state_age_sec", default_value="1.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            Node(
                package="policy_deployment_orchestrator",
                executable="supervisor",
                name="policy_deployment_supervisor",
                output="screen",
                parameters=[
                    {
                        "recipe_id": LaunchConfiguration("recipe_id"),
                        "recipe_directory": LaunchConfiguration("recipe_directory"),
                        "policy_profiles_config": LaunchConfiguration("policy_profiles_config"),
                        "default_policy_profile_id": LaunchConfiguration("default_policy_profile_id"),
                        "operator_id": LaunchConfiguration("operator_id"),
                        "site_name": LaunchConfiguration("site_name"),
                        "connect_timeout_sec": LaunchConfiguration("connect_timeout_sec"),
                        "policy_step_timeout_sec": LaunchConfiguration("policy_step_timeout_sec"),
                        "observation_max_image_age_sec": LaunchConfiguration("observation_max_image_age_sec"),
                        "observation_max_joint_state_age_sec": LaunchConfiguration(
                            "observation_max_joint_state_age_sec"
                        ),
                        "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                        "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
                    }
                ],
            ),
        ]
    )
