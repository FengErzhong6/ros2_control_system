from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
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
            "description_package", default_value="ros2_control_marvin",
            description="Package with the composite URDF/XACRO file.",
        ),
        DeclareLaunchArgument(
            "description_file", default_value="description/urdf/marvin_dual.urdf",
            description="Composite URDF/XACRO file to load.",
        ),
        DeclareLaunchArgument(
            "controllers_file", default_value="",
            description="Controller YAML (auto-selected when empty).",
        ),
        DeclareLaunchArgument(
            "use_jsp_gui", default_value="true",
            description="Start the custom joint GUI publisher for slider command input.",
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
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")

    pkg = LaunchConfiguration("description_package").perform(context)
    desc_file = LaunchConfiguration("description_file").perform(context)
    ctrl_file_arg = LaunchConfiguration("controllers_file").perform(context)
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

    # ── Controller config (auto-select if not overridden) ─────────────────
    if ctrl_file_arg:
        ctrl_file = ctrl_file_arg
    elif grip_L or grip_R:
        ctrl_file = "bringup/config/marvin_dual_gripper_controllers.yaml"
    else:
        ctrl_file = "bringup/config/marvin_dual_controllers.yaml"

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), ctrl_file]
    )

    # ── Joint names for bridge (arm + optional grippers) ──────────────────
    joint_names = [
        "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L",
        "Joint5_L", "Joint6_L", "Joint7_L",
        "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R",
        "Joint5_R", "Joint6_R", "Joint7_R",
    ]
    if grip_L:
        joint_names.append("gripper_L")
    if grip_R:
        joint_names.append("gripper_R")

    # ── Core ──────────────────────────────────────────────────────────────

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

    # ── Visualisation ─────────────────────────────────────────────────────

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), "description", "rviz", "marvin_dual.rviz"]
    )
    home_joint_positions_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), "description", "config", "home_joint_positions.yaml"]
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

    # ── GUI slider → forward controller bridge ────────────────────────────

    joint_state_publisher_gui_node = Node(
        package="ros2_control_marvin",
        executable="joint_gui_publisher.py",
        output="both",
        parameters=[home_joint_positions_file],
        remappings=[("joint_states", "gui_joint_states")],
        condition=IfCondition(use_jsp_gui),
    )

    gui_to_forward_bridge = Node(
        package="ros2_control_marvin",
        executable="gui_joint_state_to_forward_command",
        output="screen",
        parameters=[
            {"input_topic": "gui_joint_states"},
            {"output_topic": "/forward_position_controller/commands"},
            {"joint_names": joint_names},
        ],
        condition=IfCondition(use_jsp_gui),
    )

    # ── Controller spawners ───────────────────────────────────────────────

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
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "control", "switch_controllers",
                    "--controller-manager", "/controller_manager",
                    "--activate", "forward_position_controller",
                ],
                output="screen",
                condition=IfCondition(use_jsp_gui),
            )
        ],
    )

    start_gui_after_feedback_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                joint_state_publisher_gui_node,
                gui_to_forward_bridge,
            ],
        ),
        condition=IfCondition(use_jsp_gui),
    )

    # ── Assemble ──────────────────────────────────────────────────────────

    return [
        ros2_control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
        activate_forward_controller,
        start_gui_after_feedback_ready,
    ]
