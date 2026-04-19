from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    user_name = LaunchConfiguration("user_name")
    config_file = LaunchConfiguration("config_file")
    mock = LaunchConfiguration("mock")
    default_config = PathJoinSubstitution(
        [FindPackageShare("manus_system"), "config", "manus_raw_publisher.yaml"]
    )

    raw_publisher_node = Node(
        package="manus_system",
        executable="manus_raw_publisher_node",
        name="manus_raw_publisher_node",
        output="screen",
        parameters=[config_file, {"user_name": user_name}],
        condition=UnlessCondition(mock),
    )

    mock_raw_publisher_node = Node(
        package="manus_system",
        executable="manus_mock_raw_publisher_node",
        name="manus_raw_publisher_node",
        output="screen",
        condition=IfCondition(mock),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Path to the MANUS raw publisher parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "user_name",
                default_value="",
                description="User name whose calibration files will be loaded from bringup/calibrations/<user_name>.",
            ),
            DeclareLaunchArgument(
                "mock",
                default_value="false",
                description="Start a mock MANUS raw publisher instead of the hardware-backed publisher.",
            ),
            raw_publisher_node,
            mock_raw_publisher_node,
        ]
    )
