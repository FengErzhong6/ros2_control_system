import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml_file(path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def resolve_config_path(config_path):
    if os.path.isabs(config_path):
        return config_path
    return os.path.join(get_package_share_directory("wujihand_system"), config_path)


def load_hand_identity(config_path, side):
    data = load_yaml_file(config_path)
    hands = data.get("hands", {})
    if not isinstance(hands, dict):
        raise RuntimeError(f"Invalid hands mapping in identity file: {config_path}")

    identity = hands.get(side, {})
    if not isinstance(identity, dict):
        raise RuntimeError(f"Invalid identity entry for '{side}' in: {config_path}")

    return {
        key: "" if value is None else str(value).strip()
        for key, value in identity.items()
    }


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="wujihand_system",
            description="Package with the composite URDF/XACRO file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="urdf/wujihand-right.urdf.xacro",
            description="Composite URDF/XACRO file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock_components/GenericSystem instead of real Wuji hardware.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "identity_file",
            default_value="config/wujihand_identities.yaml",
            description="YAML file with left/right Wuji hand identity settings.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "usb_serial_number",
            default_value="",
            description="Override the USB serial number used to select the Wuji hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "expected_handedness",
            default_value="",
            description="Override handedness check (left/right/0/1).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="config/wujihand_right_controllers.yaml",
            description="YAML with controller_manager and controller parameters.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_jsp_gui",
            default_value="true",
            description="Start joint_state_publisher_gui for slider command input.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    gui = LaunchConfiguration("gui")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    controllers_file = LaunchConfiguration("controllers_file")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")

    identity_file_arg = LaunchConfiguration("identity_file").perform(context).strip()
    usb_serial_number = LaunchConfiguration("usb_serial_number").perform(context).strip()
    expected_handedness = LaunchConfiguration("expected_handedness").perform(context).strip()

    identity = {}
    if identity_file_arg:
        identity = load_hand_identity(resolve_config_path(identity_file_arg), "right")

    usb_serial_number = usb_serial_number or identity.get("usb_serial_number", "")
    expected_handedness = expected_handedness or identity.get("expected_handedness", "")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), description_file]),
            ' use_mock_hardware:="',
            use_mock_hardware,
            '"',
            f' usb_serial_number:="{usb_serial_number}"',
            f' expected_handedness:="{expected_handedness}"',
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("wujihand_system"), controllers_file]
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controllers_yaml],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("asset_description"), "wuji", "rviz", "wujihand_right.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_publisher_gui_node = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [FindPackageShare("wujihand_system"), "launch", "joint_gui_publisher.py"]
            ),
            "--ros-args",
            "-p",
            "publish_topic:=gui_joint_states",
        ],
        output="screen",
        condition=IfCondition(use_jsp_gui),
    )

    gui_to_forward_bridge = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [FindPackageShare("wujihand_system"), "launch", "gui_joint_state_to_forward_command.py"]
            ),
            "--ros-args",
            "-p",
            "input_topic:=gui_joint_states",
            "-p",
            "output_topic:=/forward_position_controller/commands",
        ],
        output="screen",
        condition=IfCondition(use_jsp_gui),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--inactive",
            "--param-file",
            controllers_yaml,
        ],
        output="screen",
    )

    activate_forward_controller = TimerAction(
        period=1.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "control",
                    "switch_controllers",
                    "--controller-manager",
                    "/controller_manager",
                    "--activate",
                    "forward_position_controller",
                ],
                output="screen",
                condition=IfCondition(use_jsp_gui),
            )
        ],
    )

    return [
        ros2_control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_gui_node,
        gui_to_forward_bridge,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
        activate_forward_controller,
    ]
