from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    trackers_config_default = PathJoinSubstitution(
        [FindPackageShare("htc_system"), "bringup", "config", "trackers.yaml"]
    )

    tracker_publisher_node = Node(
        package="htc_system",
        executable="tracker_publisher",
        name="tracker_publisher",
        output="screen",
        parameters=[LaunchConfiguration("trackers_config")],
        condition=UnlessCondition(LaunchConfiguration("use_mock_trackers")),
    )

    mock_tracker_publisher_node = Node(
        package="htc_system",
        executable="mock_tracker_publisher.py",
        name="mock_tracker_publisher",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("trackers_config"),
                "publish_rate": LaunchConfiguration("mock_publish_rate"),
            }
        ],
        condition=IfCondition(LaunchConfiguration("use_mock_trackers")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "trackers_config",
            default_value=trackers_config_default,
            description="Path to the tracker inventory YAML file.",
        ),
        DeclareLaunchArgument(
            "use_mock_trackers",
            default_value="false",
            description="Publish deterministic mock tracker TF instead of using OpenVR.",
        ),
        DeclareLaunchArgument(
            "mock_publish_rate",
            default_value="60.0",
            description="Publish rate in Hz for the mock tracker TF publisher.",
        ),
        tracker_publisher_node,
        mock_tracker_publisher_node,
    ])
