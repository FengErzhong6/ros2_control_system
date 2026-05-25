import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 禁用 FastDDS Shared Memory Transport，避免 RealSense 图像流耗尽 SHM 资源，
    # 导致 WujiHand ros2_control_node 无法初始化 DDS participant（进程无输出）。
    if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = os.path.join(
            get_package_share_directory("policy_deployment_bringup"),
            "config", "fastdds_disable_shm.xml"
        )
    if not os.environ.get("ROS_LOCALHOST_ONLY"):
        os.environ["ROS_LOCALHOST_ONLY"] = "1"
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
                    "collision_guard_enabled": "false",
                    "collision_guard_check_rate_hz": "30.0",
                    "collision_guard_near_distance_m": "0.04",
                    "collision_guard_hard_collision_distance_m": "0.01",
                    "collision_guard_escape_min_distance_improvement_m": "0.0002",
                    "collision_guard_interpolation_steps": "6",
                    "collision_guard_binary_search_steps": "5",
                    "collision_description_package": "marvin_system",
                    "collision_description_file": "description/urdf/marvin_wuji_simplified_collision.urdf",
                    "collision_srdf_file": "description/srdf/marvin_wuji_simplified_collision.srdf",
                    "wujihand_joint_state_topics": "/joint_states",
                    "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
                }.items(),
            )
        ],
    )

    wujihand_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("wujihand_system"),
                            "launch",
                            "wujihand_right_control.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "namespace": "right",
                    "gui": "false",
                    "use_jsp_gui": "false",
                    "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
                }.items(),
            )
        ],
    )

    high_camera_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("camera_system"),
                            "bringup",
                            "launch",
                            "single_realsense.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "camera_name": "cam_high",
                    "use_showimage": "false",
                }.items(),
            )
        ],
        condition=IfCondition(LaunchConfiguration("start_cameras")),
    )

    left_wrist_camera_launch = GroupAction(
        scoped=True,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("camera_system"),
                            "bringup",
                            "launch",
                            "single_orbbec.launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "camera_name": "cam_left_wrist",
                    "use_showimage": "false",
                }.items(),
            )
        ],
        condition=IfCondition(LaunchConfiguration("start_cameras")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("start_cameras", default_value="true"),
            high_camera_launch,
            left_wrist_camera_launch,
            wujihand_launch,
            TimerAction(period=3.0, actions=[marvin_launch]),
        ]
    )
