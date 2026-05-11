from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    activate_forward_controller = LaunchConfiguration("activate_forward_controller")
    identity_file = LaunchConfiguration("identity_file")
    controllers_file = LaunchConfiguration("controllers_file")
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
                    "namespace": LaunchConfiguration("namespace"),
                    "gui": gui,
                    "use_jsp_gui": use_jsp_gui,
                    "activate_forward_controller": activate_forward_controller,
                    "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
                    "identity_file": identity_file,
                    "controllers_file": controllers_file,
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("use_jsp_gui", default_value="false"),
            DeclareLaunchArgument("activate_forward_controller", default_value="true"),
            DeclareLaunchArgument("identity_file", default_value="config/wujihand_identities.yaml"),
            DeclareLaunchArgument(
                "controllers_file",
                default_value="config/wujihand_right_controllers.yaml",
            ),
            DeclareLaunchArgument("use_mock_hardware", default_value="false"),
            DeclareLaunchArgument("namespace", default_value="right"),
            wujihand_launch,
        ]
    )
