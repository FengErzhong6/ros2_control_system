from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
        DeclareLaunchArgument(
            "left_xyz", default_value="0 0.037 0.3618964",
            description="Mount pose (xyz) of Base_L in world.",
        ),
        DeclareLaunchArgument(
            "left_rpy", default_value="-1.5707963 0 0",
            description="Mount pose (rpy) of Base_L in world.",
        ),
        DeclareLaunchArgument(
            "right_xyz", default_value="0 -0.037 0.3618964",
            description="Mount pose (xyz) of Base_R in world.",
        ),
        DeclareLaunchArgument(
            "right_rpy", default_value="1.5707963 0 0",
            description="Mount pose (rpy) of Base_R in world.",
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

        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    pkg_share = FindPackageShare("ros2_control_marvin")

    grip_L = LaunchConfiguration("use_gripper_L").perform(context).lower() == "true"
    grip_R = LaunchConfiguration("use_gripper_R").perform(context).lower() == "true"
    use_gripper = grip_L or grip_R

    # ── Include the IK control launch ────────────────────────────────
    ik_launch = IncludeLaunchDescription(
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
            "left_xyz": LaunchConfiguration("left_xyz"),
            "left_rpy": LaunchConfiguration("left_rpy"),
            "right_xyz": LaunchConfiguration("right_xyz"),
            "right_rpy": LaunchConfiguration("right_rpy"),
        }.items(),
    )

    # ── Joystick driver ──────────────────────────────────────────────
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "device_id": 0,
            "deadzone": 0.0,
            "autorepeat_rate": 50.0,
        }],
        output="screen",
    )

    # ── Joy → Pose bridge ────────────────────────────────────────────
    joy_to_pose_params = {
        "linear_speed": LaunchConfiguration("linear_speed"),
        "angular_speed": LaunchConfiguration("angular_speed"),
        "button_angular_speed": LaunchConfiguration("button_angular_speed"),
        "deadzone": LaunchConfiguration("deadzone"),
        "use_gripper": use_gripper,
    }

    joy_to_pose_node = Node(
        package="ros2_control_marvin",
        executable="joy_to_pose",
        name="joy_to_pose",
        output="screen",
        parameters=[joy_to_pose_params],
    )

    return [ik_launch, joy_node, joy_to_pose_node]
