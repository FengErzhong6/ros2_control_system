from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    marvin_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("marvin_system"),
                            "bringup",
                            "launch",
                            "marvin_dual_joint_gui_control.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "gui": "false",
                    "use_jsp_gui": "false",
                    "shutdown_on_gui_exit": "false",
                    "use_gripper_L": "false",
                    "use_gripper_R": "false",
                    "mock_grippers": "false",
                    "collision_guard_enabled": LaunchConfiguration("collision_guard_enabled"),
                    "collision_guard_check_rate_hz": LaunchConfiguration("collision_guard_check_rate_hz"),
                    "collision_guard_near_distance_m": LaunchConfiguration("collision_guard_near_distance_m"),
                    "collision_guard_hard_collision_distance_m": LaunchConfiguration(
                        "collision_guard_hard_collision_distance_m"
                    ),
                    "collision_guard_escape_min_distance_improvement_m": LaunchConfiguration(
                        "collision_guard_escape_min_distance_improvement_m"
                    ),
                    "collision_guard_interpolation_steps": LaunchConfiguration(
                        "collision_guard_interpolation_steps"
                    ),
                    "collision_guard_binary_search_steps": LaunchConfiguration(
                        "collision_guard_binary_search_steps"
                    ),
                    "collision_description_package": "marvin_system",
                    "collision_description_file": "description/urdf/marvin_wuji_simplified_collision.urdf",
                    "collision_srdf_file": "description/srdf/marvin_wuji_simplified_collision.srdf",
                    "wujihand_joint_state_topics": "/joint_states",
                    "motion_start_delay_sec": "4.0",
                    "initial_motion_mode": "SAFE_HOLD",
                    "go_home_return_mode": "SAFE_HOLD",
                    "controller_state_settle_timeout_sec": "10.0",
                    "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("collision_guard_enabled", default_value="false"),
            DeclareLaunchArgument("collision_guard_check_rate_hz", default_value="30.0"),
            DeclareLaunchArgument("collision_guard_near_distance_m", default_value="0.04"),
            DeclareLaunchArgument("collision_guard_hard_collision_distance_m", default_value="0.01"),
            DeclareLaunchArgument(
                "collision_guard_escape_min_distance_improvement_m",
                default_value="0.0002",
            ),
            DeclareLaunchArgument("collision_guard_interpolation_steps", default_value="6"),
            DeclareLaunchArgument("collision_guard_binary_search_steps", default_value="5"),
            marvin_launch,
        ]
    )
