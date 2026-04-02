from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "trackers_config",
            default_value=trackers_config_default,
            description="Path to the tracker inventory YAML file.",
        ),
        tracker_publisher_node,
    ])
