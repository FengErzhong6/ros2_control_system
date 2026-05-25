from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("use_jsp_gui", default_value="false"),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("namespace", default_value="right"),
            DeclareLaunchArgument(
                "identity_file", default_value="config/wujihand_identities.yaml"
            ),
            DeclareLaunchArgument(
                "controllers_file",
                default_value="config/wujihand_right_controllers.yaml",
            ),
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
                    "gui": LaunchConfiguration("gui"),
                    "use_jsp_gui": LaunchConfiguration("use_jsp_gui"),
                    "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
                    "namespace": LaunchConfiguration("namespace"),
                    "identity_file": LaunchConfiguration("identity_file"),
                    "controllers_file": LaunchConfiguration("controllers_file"),
                }.items(),
            ),
        ]
    )
