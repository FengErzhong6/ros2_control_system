from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="marvin_system",
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
            "gui",
            default_value="true",
            description="Start RViz2 and Joint State Publisher GUI.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_xyz",
            default_value="0 0.037 0.3618964",
            description="Mount pose (xyz) of Base_L in world",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_rpy",
            default_value="-1.5707963 0 0",
            description="Mount pose (rpy) of Base_L in world",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_xyz",
            default_value="0 -0.037 0.3618964",
            description="Mount pose (xyz) of Base_R in world",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_rpy",
            default_value="1.5707963 0 0",
            description="Mount pose (rpy) of Base_R in world",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")

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

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("marvin_system"), "description", "rviz", "marvin_dual.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[robot_description],
        condition=IfCondition(gui),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"publish_robot_description": True}],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
