import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


DEFAULT_TRAJECTORY_ROOT = (
    "/home/mmlab/codes/huangshzh/ros2_control_system/htc_system/trajectory"
)


def launch_setup(context, *args, **kwargs):
    del args
    del kwargs

    trackers_config = PathJoinSubstitution(
        [FindPackageShare("htc_system"), "bringup", "config", "trackers.yaml"]
    )

    trajectory_root = LaunchConfiguration("trajectory_root").perform(context)
    bag_prefix = LaunchConfiguration("bag_prefix").perform(context)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_output_dir = os.path.join(trajectory_root, f"{bag_prefix}_{timestamp}")

    os.makedirs(trajectory_root, exist_ok=True)

    tracker_publisher_node = Node(
        package="htc_system",
        executable="tracker_publisher",
        name="tracker_publisher",
        output="screen",
        parameters=[trackers_config],
    )

    record_tf_bag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            bag_output_dir,
            "/tf",
            "/tf_static",
        ],
        output="screen",
    )

    return [
        LogInfo(msg=f"Recording tracker topics to: {bag_output_dir}"),
        tracker_publisher_node,
        record_tf_bag,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "trajectory_root",
            default_value=DEFAULT_TRAJECTORY_ROOT,
            description="Directory used to store timestamped tracker rosbag data.",
        ),
        DeclareLaunchArgument(
            "bag_prefix",
            default_value="tracker_trajectory",
            description="Prefix for the timestamped rosbag output directory.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
