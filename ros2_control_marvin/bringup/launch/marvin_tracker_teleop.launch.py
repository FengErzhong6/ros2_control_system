"""
Launch HTC Tracker teleop: tracker_publisher (htc_system) + ros2_control with TrackerTeleopController.
Requires htc_system and ros2_control_marvin to be in the same workspace.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start RViz2 automatically.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
            description="Use mock hardware (GenericSystem) instead of real robot.",
        ),
        DeclareLaunchArgument(
            "use_keyboard_gate", default_value="true",
            description="Start keyboard gate helper (Space=start/pause teleop).",
        ),
        DeclareLaunchArgument(
            "description_package", default_value="ros2_control_marvin",
            description="Package with the composite URDF/XACRO file.",
        ),
        DeclareLaunchArgument(
            "description_file", default_value="description/urdf/marvin_dual.urdf",
            description="Composite URDF/XACRO file to load.",
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
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_keyboard_gate = LaunchConfiguration("use_keyboard_gate")

    pkg = LaunchConfiguration("description_package").perform(context)
    desc_file = LaunchConfiguration("description_file").perform(context)
    grip_L = LaunchConfiguration("use_gripper_L").perform(context).lower() == "true"
    grip_R = LaunchConfiguration("use_gripper_R").perform(context).lower() == "true"
    left_xyz = LaunchConfiguration("left_xyz")
    left_rpy = LaunchConfiguration("left_rpy")
    right_xyz = LaunchConfiguration("right_xyz")
    right_rpy = LaunchConfiguration("right_rpy")

    # ── Robot description (xacro) ─────────────────────────────────────────
    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(pkg), desc_file]),
        ' left_xyz:="', left_xyz, '"',
        ' left_rpy:="', left_rpy, '"',
        ' right_xyz:="', right_xyz, '"',
        ' right_rpy:="', right_rpy, '"',
        " use_mock_hardware:=", use_mock_hardware,
    ]
    if grip_L:
        xacro_cmd.append(" use_gripper_L:=true")
    if grip_R:
        xacro_cmd.append(" use_gripper_R:=true")

    robot_description = {
        "robot_description": ParameterValue(Command(xacro_cmd), value_type=str)
    }

    # ── Tracker teleop controller config ───────────────────────────────────
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), "bringup", "config",
         "marvin_tracker_teleop_controllers.yaml"]
    )

    # ── Kinematics config path for TrackerTeleopController ─────────────────
    pkg_share = get_package_share_directory("ros2_control_marvin")
    kine_config_path = os.path.join(pkg_share, "config", "marvinCfg", "ccs_m6_40.MvKDCfg")

    tracker_teleop_params_override = (
        f'tracker_teleop_controller:\n'
        f'  ros__parameters:\n'
        f'    kine_config_path: "{kine_config_path}"\n'
    )
    tracker_teleop_params_file = os.path.join(tempfile.mkdtemp(), "tracker_teleop_kine_params.yaml")
    with open(tracker_teleop_params_file, "w") as f:
        f.write(tracker_teleop_params_override)

    # ── HTC Tracker publisher config (htc_system) ──────────────────────────
    trackers_config = PathJoinSubstitution(
        [FindPackageShare("htc_system"), "bringup", "config", "trackers.yaml"]
    )

    # Gripper controller configs (generated per enabled gripper)
    gripper_params_files = []
    gripper_names = []
    for side, enabled in [("L", grip_L), ("R", grip_R)]:
        if not enabled:
            continue
        name = f"gripper_{side}_controller"
        gripper_names.append(name)
        content = (
            f'controller_manager:\n'
            f'  ros__parameters:\n'
            f'    {name}:\n'
            f'      type: forward_command_controller/ForwardCommandController\n'
            f'\n'
            f'{name}:\n'
            f'  ros__parameters:\n'
            f'    interface_name: position\n'
            f'    joints:\n'
            f'      - gripper_{side}\n'
        )
        path = os.path.join(tempfile.mkdtemp(), f"{name}.yaml")
        with open(path, "w") as f:
            f.write(content)
        gripper_params_files.append(path)

    # ── Core nodes ─────────────────────────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            controllers_yaml,
            tracker_teleop_params_file,
        ] + gripper_params_files,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # HTC Tracker publisher (broadcasts TF: world -> tracker_left_hand, tracker_torso, etc.)
    tracker_publisher_node = Node(
        package="htc_system",
        executable="tracker_publisher",
        name="tracker_publisher",
        output="screen",
        parameters=[trackers_config],
    )

    tracker_teleop_keyboard_node = Node(
        package="ros2_control_marvin",
        executable="tracker_teleop_keyboard.py",
        name="tracker_teleop_keyboard",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_keyboard_gate),
    )

    # ── Visualisation ─────────────────────────────────────────────────────
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), "description", "rviz", "marvin_dual.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[robot_description],
        condition=IfCondition(gui),
    )

    # ── Controller spawners ───────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    tracker_teleop_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tracker_teleop_controller",
            "--param-file",
            controllers_yaml,
            "--param-file",
            tracker_teleop_params_file,
        ],
        output="screen",
    )

    gripper_spawners = []
    for name in gripper_names:
        gripper_spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name],
            output="screen",
        ))

    return [
        ros2_control_node,
        robot_state_publisher_node,
        tracker_publisher_node,
        tracker_teleop_keyboard_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        tracker_teleop_controller_spawner,
    ] + gripper_spawners
