from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("manus_system"), "config", "manus_raw_publisher.yaml"]
    )

    raw_publisher_node = Node(
        package="manus_system",
        executable="manus_raw_publisher_node",
        name="manus_raw_publisher_node",
        output="screen",
        parameters=[config],
    )

    return LaunchDescription([raw_publisher_node])
