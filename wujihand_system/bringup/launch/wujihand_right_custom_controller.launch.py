from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch.conditions import IfCondition
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
            "policy_file",
            default_value="config/rl_controller.yaml",
            description="YAML with RL policy parameters (applied to rl_controller node).",
        )
    )

    gui = LaunchConfiguration("gui")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    controllers_file = LaunchConfiguration("controllers_file")
    policy_file = LaunchConfiguration("policy_file")

    # robot_description from xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), description_file]),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    controllers_yaml = PathJoinSubstitution([FindPackageShare("wujihand_system"), controllers_file])
    policy_yaml = PathJoinSubstitution([FindPackageShare("wujihand_system"), policy_file])

    # Control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controllers_yaml, policy_yaml],
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

    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    rl_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "rl_controller",
            "--controller-manager",
            "/controller_manager",
            "--param-file",
            controllers_yaml,
        ],
        output="screen",
    )

    nodes = [
        ros2_control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        rl_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
