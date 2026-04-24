from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from data_collection_bringup.relay_launch import relay_setup


def generate_launch_description():
    bringup_share = FindPackageShare("data_collection_bringup")
    default_recipe_directory = PathJoinSubstitution([bringup_share, "config", "recipes"])
    default_startup_policy = PathJoinSubstitution(
        [bringup_share, "config", "policies", "startup.yaml"]
    )
    default_fault_policy = PathJoinSubstitution([bringup_share, "config", "policies", "fault.yaml"])
    default_recording_config = PathJoinSubstitution([bringup_share, "config", "session", "recording.yaml"])
    default_cameras_config = PathJoinSubstitution(
        [FindPackageShare("camera_system"), "bringup", "config", "cameras.yaml"]
    )
    default_trackers_config = PathJoinSubstitution(
        [FindPackageShare("htc_system"), "bringup", "config", "trackers.yaml"]
    )
    default_manus_config = PathJoinSubstitution(
        [FindPackageShare("manus_system"), "config", "manus_raw_publisher.yaml"]
    )
    default_wujihand_identity_file = PathJoinSubstitution(
        [FindPackageShare("wujihand_system"), "config", "wujihand_identities.yaml"]
    )
    default_wujihand_teleop_config = PathJoinSubstitution(
        [FindPackageShare("wujihand_system"), "config", "manus_input.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "recipe_id",
                default_value="marvin_tracker_manus_camera_collection",
            ),
            DeclareLaunchArgument("recipe_directory", default_value=default_recipe_directory),
            DeclareLaunchArgument("startup_policy_config", default_value=default_startup_policy),
            DeclareLaunchArgument("fault_policy_config", default_value=default_fault_policy),
            DeclareLaunchArgument("recording_config", default_value=default_recording_config),
            DeclareLaunchArgument("cameras_config", default_value=default_cameras_config),
            DeclareLaunchArgument("trackers_config", default_value=default_trackers_config),
            DeclareLaunchArgument("manus_config", default_value=default_manus_config),
            DeclareLaunchArgument("manus_user_name", default_value=""),
            DeclareLaunchArgument("wujihand_identity_file", default_value=default_wujihand_identity_file),
            DeclareLaunchArgument("wujihand_teleop_config", default_value=default_wujihand_teleop_config),
            DeclareLaunchArgument("wujihand_use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("mock_manus", default_value="false"),
            DeclareLaunchArgument("marvin_mock_grippers", default_value="false"),
            DeclareLaunchArgument("supervisor_sigterm_timeout_sec", default_value="20.0"),
            DeclareLaunchArgument("supervisor_sigkill_timeout_sec", default_value="20.0"),
            OpaqueFunction(function=relay_setup),
            Node(
                package="data_collection_orchestrator",
                executable="supervisor",
                name="data_collection_supervisor",
                output="screen",
                sigterm_timeout=LaunchConfiguration("supervisor_sigterm_timeout_sec"),
                sigkill_timeout=LaunchConfiguration("supervisor_sigkill_timeout_sec"),
                parameters=[
                    {
                        "recipe_id": LaunchConfiguration("recipe_id"),
                        "recipe_directory": LaunchConfiguration("recipe_directory"),
                        "startup_policy_config": LaunchConfiguration("startup_policy_config"),
                        "fault_policy_config": LaunchConfiguration("fault_policy_config"),
                        "recording_config": LaunchConfiguration("recording_config"),
                        "cameras_config": LaunchConfiguration("cameras_config"),
                        "trackers_config": LaunchConfiguration("trackers_config"),
                        "manus_config": LaunchConfiguration("manus_config"),
                        "manus_user_name": LaunchConfiguration("manus_user_name"),
                        "wujihand_identity_file": LaunchConfiguration("wujihand_identity_file"),
                        "wujihand_teleop_config": LaunchConfiguration("wujihand_teleop_config"),
                        "wujihand_use_mock_hardware": ParameterValue(
                            LaunchConfiguration("wujihand_use_mock_hardware"),
                            value_type=bool,
                        ),
                        "mock_manus": ParameterValue(
                            LaunchConfiguration("mock_manus"),
                            value_type=bool,
                        ),
                        "marvin_mock_grippers": ParameterValue(
                            LaunchConfiguration("marvin_mock_grippers"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
