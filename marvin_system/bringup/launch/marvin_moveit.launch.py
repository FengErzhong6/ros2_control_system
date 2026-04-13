from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_file = PathJoinSubstitution(
        [FindPackageShare("marvin_system"), "bringup", "launch", "marvin_safe_operation.launch.py"]
    )

    passthrough_args = [
        "gui",
        "use_mock_hardware",
        "description_package",
        "description_file",
        "use_gripper_L",
        "use_gripper_R",
        "left_xyz",
        "left_rpy",
        "right_xyz",
        "right_rpy",
        "collision_guard_enabled",
        "collision_guard_check_rate_hz",
        "collision_guard_min_command_delta_deg",
        "collision_guard_near_distance_m",
        "collision_guard_hard_collision_distance_m",
        "collision_guard_escape_min_distance_improvement_m",
        "collision_guard_interpolation_steps",
        "collision_guard_binary_search_steps",
        "motion_allow_legacy_home_fallback",
    ]

    declared_args = [
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("description_package", default_value="marvin_system"),
        DeclareLaunchArgument("description_file", default_value="description/urdf/marvin_dual.urdf"),
        DeclareLaunchArgument("use_gripper_L", default_value="true"),
        DeclareLaunchArgument("use_gripper_R", default_value="true"),
        DeclareLaunchArgument("left_xyz", default_value="0 0.037 0.3618964"),
        DeclareLaunchArgument("left_rpy", default_value="-1.5707963 0 0"),
        DeclareLaunchArgument("right_xyz", default_value="0 -0.037 0.3618964"),
        DeclareLaunchArgument("right_rpy", default_value="1.5707963 0 0"),
        DeclareLaunchArgument("collision_guard_enabled", default_value="false"),
        DeclareLaunchArgument("collision_guard_check_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("collision_guard_min_command_delta_deg", default_value="0.0"),
        DeclareLaunchArgument("collision_guard_near_distance_m", default_value="0.10"),
        DeclareLaunchArgument("collision_guard_hard_collision_distance_m", default_value="0.05"),
        DeclareLaunchArgument("collision_guard_escape_min_distance_improvement_m", default_value="0.001"),
        DeclareLaunchArgument("collision_guard_interpolation_steps", default_value="6"),
        DeclareLaunchArgument("collision_guard_binary_search_steps", default_value="5"),
        DeclareLaunchArgument("motion_allow_legacy_home_fallback", default_value="false"),
    ]

    include_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={
            name: LaunchConfiguration(name) for name in passthrough_args
        }.items(),
    )

    return LaunchDescription(
        declared_args
        + [
            LogInfo(
                msg=(
                    "Phase-1 MoveIt launch scaffold: trajectory execution and motion service are enabled, "
                    "while the full MoveIt planning backend will be added in a later phase."
                )
            ),
            include_launch,
        ]
    )
