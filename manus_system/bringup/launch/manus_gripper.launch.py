from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    user_name = LaunchConfiguration("user_name")
    raw_publisher_config = PathJoinSubstitution(
        [FindPackageShare("manus_system"), "config", "manus_raw_publisher.yaml"]
    )
    gripper_config = PathJoinSubstitution(
        [FindPackageShare("manus_system"), "config", "manus_gripper.yaml"]
    )

    raw_publisher_node = Node(
        package="manus_system",
        executable="manus_raw_publisher_node",
        name="manus_raw_publisher_node",
        output="screen",
        parameters=[raw_publisher_config, {"user_name": user_name}],
    )

    gripper_node = Node(
        package="manus_system",
        executable="manus_gripper_node",
        name="manus_gripper_node",
        output="screen",
        parameters=[gripper_config],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "user_name",
                default_value="",
                description=(
                    "User name whose calibration files will be loaded from "
                    "bringup/calibrations/<user_name>."
                ),
            ),
            raw_publisher_node,
            gripper_node,
        ]
    )
