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
            "use_jsp_gui",
            default_value="true",
            description="Start joint_state_publisher_gui for slider command input.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ros2_control_marvin",
            description="Package with the composite URDF/XACRO file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="description/urdf/marvin_dual.urdf",
            description="Composite URDF/XACRO file to load.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="config/marvin_dual_controllers.yaml",
            description="YAML with controller_manager and controller parameters.",
        )
    )

    # Keep these mount args consistent with the description launch.
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_xyz",
            default_value="0 -0.4375 0",
            description="Left arm mount xyz in meters.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_rpy",
            default_value="0 0 0",
            description="Left arm mount rpy in radians.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_xyz",
            default_value="0 0.4375 0",
            description="Right arm mount xyz in meters.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_rpy",
            default_value="0 0 0",
            description="Right arm mount rpy in radians.",
        )
    )

    gui = LaunchConfiguration("gui")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")

    left_xyz = LaunchConfiguration("left_xyz")
    left_rpy = LaunchConfiguration("left_rpy")
    right_xyz = LaunchConfiguration("right_xyz")
    right_rpy = LaunchConfiguration("right_rpy")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), description_file]),
            " ",
            "left_xyz:=\"",
            left_xyz,
            "\" ",
            "left_rpy:=\"",
            left_rpy,
            "\" ",
            "right_xyz:=\"",
            right_xyz,
            "\" ",
            "right_rpy:=\"",
            right_rpy,
            "\"",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), "bringup", controllers_file]
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

    # Optional RViz (use the description RViz config).
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), "description", "rviz", "marvin_dual.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    # Slider GUI (publishes JointState). Remap to avoid colliding with the real /joint_states.
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",
        parameters=[robot_description],
        remappings=[("joint_states", "gui_joint_states")],
        condition=IfCondition(use_jsp_gui),
    )

    # Bridge GUI joint states -> forward controller Float64MultiArray commands.
    # Implemented in C++ to avoid python dependency issues on some systems.
    gui_to_forward_bridge = Node(
        package="ros2_control_marvin",
        executable="gui_joint_state_to_forward_command",
        output="screen",
        parameters=[
            {
                "gui_topic": "gui_joint_states",
                "real_topic": "/joint_states",
                "output_topic": "/forward_position_controller/commands",
            }
        ],
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

    # Activate forward controller after the bridge has had time to capture initial poses.
    activate_forward_controller = TimerAction(
        period=2.0,
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
