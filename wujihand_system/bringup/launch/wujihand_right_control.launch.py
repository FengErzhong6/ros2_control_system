from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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
            default_value="urdf/wujihand-right.urdf",
            description="Composite URDF/XACRO file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="config/wujihand_controllers.yaml",
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

    gui = LaunchConfiguration("gui")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")

    # robot_description from xacro
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
        [FindPackageShare("wujihand_system"), controllers_file]
    )

    # Control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controllers_yaml],
    )

    # TF publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Optional RViz
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

    # Slider GUI (publishes JointState). We remap its output so it does not conflict with
    # joint_state_broadcaster's /joint_states.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
        parameters=[robot_description],
        remappings=[("joint_states", "gui_joint_states")],
        condition=IfCondition(use_jsp_gui),
    )

    # Bridge GUI joint states -> forward controller Float64MultiArray commands.
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

    # Spawn controllers
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

    # Activate forward controller after the bridge publishes initial all-zeros.
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

    nodes = [
        ros2_control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_gui_node,
        gui_to_forward_bridge,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
        activate_forward_controller,
    ]

    return LaunchDescription(declared_arguments + nodes)
