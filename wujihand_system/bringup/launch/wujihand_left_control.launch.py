import os
import tempfile

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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


def resolve_share_path(package_name, relative_path):
    if os.path.isabs(relative_path):
        return relative_path
    return os.path.join(get_package_share_directory(package_name), relative_path)


def write_namespaced_controller_params(source_path, namespace):
    data = load_yaml_file(source_path)
    namespaced = {str(namespace).strip("/"): data}
    handle = tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"wujihand_{str(namespace).strip('/')}_controllers_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    )
    try:
        yaml.safe_dump(namespaced, handle, sort_keys=False)
    finally:
        handle.close()
    return handle.name


def absolute_namespace(namespace):
    namespace = str(namespace).strip().strip("/")
    if not namespace:
        raise RuntimeError("Namespace must not be empty.")
    return f"/{namespace}"


def robot_description_remappings(namespace):
    topic = f"{absolute_namespace(namespace)}/robot_description"
    return [
        ("robot_description", topic),
        ("/robot_description", topic),
    ]


def launch_bool(value):
    return "true" if str(value).strip().lower() in {"1", "true", "yes", "on"} else "false"


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
            default_value="urdf/wujihand-left.urdf.xacro",
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
            default_value="config/wujihand_left_controllers.yaml",
            description="YAML with controller_manager and controller parameters.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "namespace",
            default_value="left",
            description="ROS namespace for the left hand stack.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_jsp_gui",
            default_value="true",
            description="Start joint GUI for slider command input.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_forward_controller",
            default_value="true",
            description="Activate forward_position_controller automatically.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    gui = LaunchConfiguration("gui")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    controllers_file = LaunchConfiguration("controllers_file")
    namespace = LaunchConfiguration("namespace")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    activate_forward_controller_arg = LaunchConfiguration("activate_forward_controller")

    description_package_arg = description_package.perform(context).strip() or "wujihand_system"
    description_file_arg = description_file.perform(context).strip() or "urdf/wujihand-left.urdf.xacro"
    controllers_file_arg = (
        controllers_file.perform(context).strip() or "config/wujihand_left_controllers.yaml"
    )
    namespace_arg = namespace.perform(context).strip() or "left"
    gui_enabled = launch_bool(gui.perform(context))
    use_jsp_gui_enabled = launch_bool(use_jsp_gui.perform(context))
    activate_forward_controller_enabled = launch_bool(
        activate_forward_controller_arg.perform(context)
    )
    use_mock_hardware_arg = launch_bool(use_mock_hardware.perform(context))

    identity_file_arg = LaunchConfiguration("identity_file").perform(context).strip()
    usb_serial_number = LaunchConfiguration("usb_serial_number").perform(context).strip()
    expected_handedness = LaunchConfiguration("expected_handedness").perform(context).strip()

    identity = {}
    if identity_file_arg:
        identity = load_hand_identity(resolve_config_path(identity_file_arg), "left")

    usb_serial_number = usb_serial_number or identity.get("usb_serial_number", "")
    expected_handedness = expected_handedness or identity.get("expected_handedness", "")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            resolve_share_path(description_package_arg, description_file_arg),
            ' use_mock_hardware:="',
            use_mock_hardware_arg,
            '"',
            f' usb_serial_number:="{usb_serial_number}"',
            f' expected_handedness:="{expected_handedness}"',
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = resolve_share_path("wujihand_system", controllers_file_arg)
    namespaced_controllers_yaml = write_namespaced_controller_params(
        controllers_yaml,
        namespace_arg,
    )
    remappings = robot_description_remappings(namespace_arg)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace_arg,
        output="both",
        parameters=[robot_description, namespaced_controllers_yaml],
        remappings=remappings,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace_arg,
        output="both",
        parameters=[robot_description],
        remappings=remappings,
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
        condition=IfCondition(gui_enabled),
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
        condition=IfCondition(use_jsp_gui_enabled),
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
            f"output_topic:=/{namespace_arg}/forward_position_controller/commands",
        ],
        output="screen",
        condition=IfCondition(use_jsp_gui_enabled),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        namespace=namespace_arg,
        output="screen",
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--inactive",
            "--param-file",
            namespaced_controllers_yaml,
        ],
        namespace=namespace_arg,
        output="screen",
    )

    activate_forward_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=forward_position_controller_spawner,
            on_exit=[
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "control",
                        "switch_controllers",
                        "--controller-manager",
                        f"/{namespace_arg}/controller_manager",
                        "--activate",
                        "forward_position_controller",
                    ],
                    output="screen",
                    condition=IfCondition(activate_forward_controller_enabled),
                )
            ],
        )
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
