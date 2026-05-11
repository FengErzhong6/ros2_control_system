from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
            DeclareLaunchArgument("start_cameras", default_value="true"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            high_camera_launch,
            left_wrist_camera_launch,
        ]
    )
