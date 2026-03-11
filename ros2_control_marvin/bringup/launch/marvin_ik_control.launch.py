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
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    gui = LaunchConfiguration("gui")

    pkg = LaunchConfiguration("description_package").perform(context)
    desc_file = LaunchConfiguration("description_file").perform(context)
    grip_L = LaunchConfiguration("use_gripper_L").perform(context).lower() == "true"
    grip_R = LaunchConfiguration("use_gripper_R").perform(context).lower() == "true"

    # ── Robot description (xacro) ─────────────────────────────────────────
    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(pkg), desc_file]),
    ]
    if grip_L:
        xacro_cmd.append(" use_gripper_L:=true")
    if grip_R:
        xacro_cmd.append(" use_gripper_R:=true")

    robot_description = {
        "robot_description": ParameterValue(Command(xacro_cmd), value_type=str)
    }

    # ── Controller config ─────────────────────────────────────────────────
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), "bringup", "config", "marvin_ik_controllers.yaml"]
    )

    # ── Kinematics config path (resolved at launch time) ──────────────────
    pkg_share = get_package_share_directory("ros2_control_marvin")
    kine_config_path = os.path.join(pkg_share, "config", "marvinCfg", "ccs_m6_40.MvKDCfg")

    ik_params_override = (
        f'ik_controller:\n'
        f'  ros__parameters:\n'
        f'    kine_config_path: "{kine_config_path}"\n'
    )
    ik_params_file = os.path.join(tempfile.mkdtemp(), "ik_kine_params.yaml")
    with open(ik_params_file, "w") as f:
        f.write(ik_params_override)

    # ── Core ──────────────────────────────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            controllers_yaml,
            ik_params_file,
        ],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
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

    ik_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ik_controller",
            "--param-file",
            controllers_yaml,
            "--param-file",
            ik_params_file,
        ],
        output="screen",
    )

    # ── Assemble ──────────────────────────────────────────────────────────
    return [
        ros2_control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        ik_controller_spawner,
    ]
