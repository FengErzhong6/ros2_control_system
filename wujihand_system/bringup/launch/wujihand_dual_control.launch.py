import os
import tempfile

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


def launch_hand_stack(
    *,
    side,
    namespace,
    description_file,
    controllers_file,
    usb_serial_number,
    expected_handedness,
    use_mock_hardware,
    use_gui,
    activate_forward_controller,
    mount_xyz,
    mount_rpy,
):
    remappings = robot_description_remappings(namespace)
    abs_ns = absolute_namespace(namespace)
    namespaced_controllers_yaml = write_namespaced_controller_params(controllers_file, namespace)

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare("wujihand_system"), description_file]),
            ' use_mock_hardware:="',
            use_mock_hardware,
            '"',
            f' usb_serial_number:="{usb_serial_number}"',
            f' expected_handedness:="{expected_handedness}"',
            ' mount_xyz:="',
            mount_xyz,
            '"',
            ' mount_rpy:="',
            mount_rpy,
            '"',
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=namespace,
        output="both",
        parameters=[robot_description, namespaced_controllers_yaml],
        remappings=remappings,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=namespace,
        output="both",
        parameters=[robot_description],
        remappings=remappings,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            f"{abs_ns}/controller_manager",
        ],
        output="screen",
    )

    forward_position_controller_arguments = [
        "forward_position_controller",
        "--param-file",
        namespaced_controllers_yaml,
        "--controller-manager",
        f"{abs_ns}/controller_manager",
    ]
    if str(activate_forward_controller).lower() not in ("1", "true", "yes", "on"):
        forward_position_controller_arguments.insert(1, "--inactive")

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=namespace,
        arguments=forward_position_controller_arguments,
        output="screen",
    )

    joint_state_publisher_gui_node = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [FindPackageShare("wujihand_system"), "launch", "joint_gui_publisher.py"]
            ),
            "--ros-args",
            "-r",
            f"__ns:={abs_ns}",
            "-r",
            f"robot_description:={abs_ns}/robot_description",
            "-r",
            f"/robot_description:={abs_ns}/robot_description",
            "-p",
            "publish_topic:=gui_joint_states",
        ],
        output="screen",
        condition=IfCondition(use_gui),
    )

    gui_to_forward_bridge = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution(
                [FindPackageShare("wujihand_system"), "launch", "gui_joint_state_to_forward_command.py"]
            ),
            "--ros-args",
            "-r",
            f"__ns:={abs_ns}",
            "-r",
            f"robot_description:={abs_ns}/robot_description",
            "-r",
            f"/robot_description:={abs_ns}/robot_description",
            "-p",
            "input_topic:=gui_joint_states",
            "-p",
            "output_topic:=forward_position_controller/commands",
        ],
        output="screen",
        condition=IfCondition(use_gui),
    )

    return [
        ros2_control_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
        joint_state_publisher_gui_node,
        gui_to_forward_bridge,
    ]


def generate_launch_description():
    declared_arguments = []
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
            "left_namespace",
            default_value="left",
            description="ROS namespace for the left-hand stack.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_namespace",
            default_value="right",
            description="ROS namespace for the right-hand stack.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_controllers_file",
            default_value="config/wujihand_left_controllers.yaml",
            description="Controller YAML for the left hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_controllers_file",
            default_value="config/wujihand_right_controllers.yaml",
            description="Controller YAML for the right hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_left_gui",
            default_value="false",
            description="Start a dedicated slider GUI for the left hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_right_gui",
            default_value="false",
            description="Start a dedicated slider GUI for the right hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_mount_xyz",
            default_value="0 0.25 0",
            description="Mount pose xyz for the left hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_mount_rpy",
            default_value="0 0 0",
            description="Mount pose rpy for the left hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_mount_xyz",
            default_value="0 -0.25 0",
            description="Mount pose xyz for the right hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_mount_rpy",
            default_value="0 0 0",
            description="Mount pose rpy for the right hand.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_forward_controller",
            default_value="false",
            description="Activate forward_position_controller automatically for both hands.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    identity_file = resolve_config_path(
        LaunchConfiguration("identity_file").perform(context).strip()
    )
    left_namespace = LaunchConfiguration("left_namespace").perform(context).strip()
    right_namespace = LaunchConfiguration("right_namespace").perform(context).strip()
    left_controllers_file = resolve_config_path(
        LaunchConfiguration("left_controllers_file").perform(context).strip()
    )
    right_controllers_file = resolve_config_path(
        LaunchConfiguration("right_controllers_file").perform(context).strip()
    )
    use_left_gui = LaunchConfiguration("use_left_gui")
    use_right_gui = LaunchConfiguration("use_right_gui")
    left_mount_xyz = LaunchConfiguration("left_mount_xyz").perform(context).strip()
    left_mount_rpy = LaunchConfiguration("left_mount_rpy").perform(context).strip()
    right_mount_xyz = LaunchConfiguration("right_mount_xyz").perform(context).strip()
    right_mount_rpy = LaunchConfiguration("right_mount_rpy").perform(context).strip()
    activate_forward_controller = LaunchConfiguration("activate_forward_controller").perform(context).strip()

    left_identity = load_hand_identity(identity_file, "left")
    right_identity = load_hand_identity(identity_file, "right")

    actions = []
    actions.extend(
        launch_hand_stack(
            side="left",
            namespace=left_namespace,
            description_file="urdf/wujihand-left.urdf.xacro",
            controllers_file=left_controllers_file,
            usb_serial_number=left_identity.get("usb_serial_number", ""),
            expected_handedness=left_identity.get("expected_handedness", ""),
            use_mock_hardware=use_mock_hardware,
            use_gui=use_left_gui,
            activate_forward_controller=activate_forward_controller,
            mount_xyz=left_mount_xyz,
            mount_rpy=left_mount_rpy,
        )
    )
    actions.extend(
        launch_hand_stack(
            side="right",
            namespace=right_namespace,
            description_file="urdf/wujihand-right.urdf.xacro",
            controllers_file=right_controllers_file,
            usb_serial_number=right_identity.get("usb_serial_number", ""),
            expected_handedness=right_identity.get("expected_handedness", ""),
            use_mock_hardware=use_mock_hardware,
            use_gui=use_right_gui,
            activate_forward_controller=activate_forward_controller,
            mount_xyz=right_mount_xyz,
            mount_rpy=right_mount_rpy,
        )
    )
    return actions
