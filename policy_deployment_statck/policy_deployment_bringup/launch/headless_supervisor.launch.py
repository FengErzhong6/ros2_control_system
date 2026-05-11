from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("policy_deployment_bringup")
    default_recipe_directory = PathJoinSubstitution([bringup_share, "config", "recipes"])
    default_policy_profiles = PathJoinSubstitution([bringup_share, "config", "policies", "policy_profiles.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("recipe_id", default_value="default_policy_deployment"),
            DeclareLaunchArgument("recipe_directory", default_value=default_recipe_directory),
            DeclareLaunchArgument("policy_profiles_config", default_value=default_policy_profiles),
            DeclareLaunchArgument("default_policy_profile_id", default_value="pi05_tianji_wuji_pick_place"),
            DeclareLaunchArgument("operator_id", default_value=""),
            DeclareLaunchArgument("site_name", default_value=""),
            DeclareLaunchArgument("connect_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("policy_step_timeout_sec", default_value="5.0"),
            DeclareLaunchArgument("observation_max_image_age_sec", default_value="1.0"),
            DeclareLaunchArgument("observation_max_joint_state_age_sec", default_value="1.0"),
            DeclareLaunchArgument("publish_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("supervisor_sigterm_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("supervisor_sigkill_timeout_sec", default_value="20.0"),
            Node(
                package="policy_deployment_orchestrator",
                executable="supervisor",
                name="policy_deployment_supervisor",
                output="screen",
                sigterm_timeout=LaunchConfiguration("supervisor_sigterm_timeout_sec"),
                sigkill_timeout=LaunchConfiguration("supervisor_sigkill_timeout_sec"),
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
                        "use_mock_hardware": ParameterValue(
                            LaunchConfiguration("use_mock_hardware"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
