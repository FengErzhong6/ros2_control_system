from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="ros2_control_marvin",
            description="Package with the composite URDF/XACRO file.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="description/urdf/marvin_dual.urdf",
            description="Composite URDF/XACRO file to load.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="bringup/config/marvin_dual_controllers.yaml",
            description="YAML with controller_manager and controller parameters.",
        ),
        DeclareLaunchArgument(
            "use_jsp_gui",
            default_value="true",
            description="Start joint_state_publisher_gui for slider command input.",
        ),
    ]

    gui = LaunchConfiguration("gui")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), description_file]),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), controllers_file]
    )

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
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
        parameters=[robot_description],
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

    # ── Assemble ──────────────────────────────────────────────────────────

    return LaunchDescription(
        declared_arguments
        + [
            ros2_control_node,
            robot_state_publisher_node,
            rviz_node,
            joint_state_publisher_gui_node,
            gui_to_forward_bridge,
            joint_state_broadcaster_spawner,
            forward_position_controller_spawner,
            activate_forward_controller,
        ]
    )
