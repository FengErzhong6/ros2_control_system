import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_launch_config(config_path):
    with open(config_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return data.get("marvin_gripper_teleop", {})


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
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("marvin_system"), "bringup", "config", "marvin_gripper_teleop.yaml"]
            ),
            description="YAML config for Marvin gripper teleop defaults.",
        ),
        DeclareLaunchArgument(
            "description_package", default_value="marvin_system",
            description="Package with the composite URDF/XACRO file.",
        ),
        DeclareLaunchArgument(
            "description_file", default_value="description/urdf/marvin_dual.urdf",
            description="Composite URDF/XACRO file to load.",
        ),
        DeclareLaunchArgument(
            "use_gripper_L", default_value="true",
            description="Enable OmniPicker gripper on left arm.",
        ),
        DeclareLaunchArgument(
            "use_gripper_R", default_value="true",
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
    config_file = LaunchConfiguration("config_file").perform(context)
    config = load_launch_config(config_file)
    start_manus = bool(config.get("start_manus", False))
    manus_user_name = str(config.get("manus_user_name", "test"))
    gripper_velocity = int(config.get("gripper_velocity", 255))
    gripper_acceleration = int(config.get("gripper_acceleration", 255))

    pkg = LaunchConfiguration("description_package").perform(context)
    desc_file = LaunchConfiguration("description_file").perform(context)
    grip_L = LaunchConfiguration("use_gripper_L").perform(context).lower() == "true"
    grip_R = LaunchConfiguration("use_gripper_R").perform(context).lower() == "true"
    left_xyz = LaunchConfiguration("left_xyz")
    left_rpy = LaunchConfiguration("left_rpy")
    right_xyz = LaunchConfiguration("right_xyz")
    right_rpy = LaunchConfiguration("right_rpy")

    if not grip_L and not grip_R:
        raise RuntimeError(
            "marvin_gripper_teleop.launch.py requires at least one enabled gripper. "
            "Set use_gripper_L:=true or use_gripper_R:=true."
        )
    if gripper_velocity < 0 or gripper_velocity > 255:
        raise RuntimeError("gripper_velocity in config_file must be within [0, 255].")
    if gripper_acceleration < 0 or gripper_acceleration > 255:
        raise RuntimeError("gripper_acceleration in config_file must be within [0, 255].")

    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(pkg), desc_file]),
        ' left_xyz:="', left_xyz, '"',
        ' left_rpy:="', left_rpy, '"',
        ' right_xyz:="', right_xyz, '"',
        ' right_rpy:="', right_rpy, '"',
        " use_mock_hardware:=", use_mock_hardware,
        f" gripper_velocity:={gripper_velocity}",
        f" gripper_acceleration:={gripper_acceleration}",
    ]
    if grip_L:
        xacro_cmd.append(" use_gripper_L:=true")
    if grip_R:
        xacro_cmd.append(" use_gripper_R:=true")

    robot_description = {
        "robot_description": ParameterValue(Command(xacro_cmd), value_type=str)
    }

    controller_manager_content = [
        "controller_manager:\n",
        "  ros__parameters:\n",
        "    update_rate: 1000\n",
        "    joint_state_broadcaster:\n",
        "      type: joint_state_broadcaster/JointStateBroadcaster\n",
    ]

    gripper_names = []
    for side, enabled in [("L", grip_L), ("R", grip_R)]:
        if not enabled:
            continue
        name = f"gripper_{side}_controller"
        gripper_names.append(name)
        controller_manager_content.extend([
            f"    {name}:\n",
            "      type: forward_command_controller/ForwardCommandController\n",
        ])

    controller_manager_content.append("\n")
    for side, enabled in [("L", grip_L), ("R", grip_R)]:
        if not enabled:
            continue
        name = f"gripper_{side}_controller"
        controller_manager_content.extend([
            f"{name}:\n",
            "  ros__parameters:\n",
            "    interface_name: position\n",
            "    joints:\n",
            f"      - gripper_{side}\n",
            "\n",
        ])

    controllers_file = os.path.join(tempfile.mkdtemp(), "marvin_gripper_teleop_controllers.yaml")
    with open(controllers_file, "w") as f:
        f.writelines(controller_manager_content)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controllers_file],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("marvin_system"), "description", "rviz", "marvin_dual.rviz"]
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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "30.0",
            "--service-call-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
        ],
        output="screen",
    )

    gripper_spawners = []
    for name in gripper_names:
        gripper_spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--param-file",
                controllers_file,
                "--controller-manager-timeout",
                "30.0",
                "--service-call-timeout",
                "30.0",
                "--switch-timeout",
                "30.0",
            ],
            output="screen",
        ))

    manus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("manus_system"), "launch", "manus_gripper.launch.py"]
            )
        ),
        condition=IfCondition(str(start_manus).lower()),
        launch_arguments={
            "user_name": manus_user_name,
        }.items(),
    )

    launch_sequence = []

    first_action_after_feedback = gripper_spawners[0]
    start_first_gripper_controller_after_feedback_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[first_action_after_feedback],
        ),
    )
    launch_sequence.append(start_first_gripper_controller_after_feedback_ready)

    for current_spawner, next_spawner in zip(gripper_spawners, gripper_spawners[1:]):
        launch_sequence.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=current_spawner,
                    on_exit=[next_spawner],
                ),
            )
        )

    launch_sequence.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=gripper_spawners[-1],
                on_exit=[manus_launch],
            ),
        )
    )

    return [
        ros2_control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
    ] + launch_sequence
