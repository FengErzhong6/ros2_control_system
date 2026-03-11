from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("ros2_control_marvin")

    return LaunchDescription([
        # ── Arguments (forwarded to IK launch) ───────────────────────────
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start RViz2 automatically.",
        ),
        DeclareLaunchArgument(
            "use_gripper_L", default_value="false",
            description="Enable OmniPicker gripper on left arm.",
        ),
        DeclareLaunchArgument(
            "use_gripper_R", default_value="false",
            description="Enable OmniPicker gripper on right arm.",
        ),

        # ── Joy-to-pose arguments ────────────────────────────────────────
        DeclareLaunchArgument(
            "linear_speed", default_value="0.1",
            description="End-effector translation speed (m/s).",
        ),
        DeclareLaunchArgument(
            "angular_speed", default_value="0.5",
            description="End-effector rotation speed from analog axes (rad/s).",
        ),
        DeclareLaunchArgument(
            "button_angular_speed", default_value="0.3",
            description="End-effector rotation speed from LB/RB buttons (rad/s).",
        ),
        DeclareLaunchArgument(
            "deadzone", default_value="0.15",
            description="Joystick deadzone [0, 1).",
        ),

        # ── Include the IK control launch ────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    pkg_share, "bringup", "launch",
                    "marvin_ik_control.launch.py",
                ])
            ),
            launch_arguments={
                "gui": LaunchConfiguration("gui"),
                "use_gripper_L": LaunchConfiguration("use_gripper_L"),
                "use_gripper_R": LaunchConfiguration("use_gripper_R"),
            }.items(),
        ),

        # ── Joystick driver ──────────────────────────────────────────────
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{
                "device_id": 0,
                "deadzone": 0.0,
                "autorepeat_rate": 50.0,
            }],
            output="screen",
        ),

        # ── Joy → Pose bridge ────────────────────────────────────────────
        Node(
            package="ros2_control_marvin",
            executable="joy_to_pose",
            name="joy_to_pose",
            output="screen",
            parameters=[{
                "linear_speed": LaunchConfiguration("linear_speed"),
                "angular_speed": LaunchConfiguration("angular_speed"),
                "button_angular_speed": LaunchConfiguration("button_angular_speed"),
                "deadzone": LaunchConfiguration("deadzone"),
            }],
        ),
    ])
