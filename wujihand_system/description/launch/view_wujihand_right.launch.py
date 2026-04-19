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
            default_value="wujihand_system",
            description="Package with the composite URDF/XACRO file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="urdf/wujihand_view.urdf.xacro",
            description="URDF/Xacro file to load for visualization.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hand",
            default_value="both",
            choices=["left", "right", "both"],
            description="Which hand model to show: left, right, or both.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_xyz",
            default_value="0 0.18 0",
            description="Mount pose xyz for the left hand in the view wrapper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "left_rpy",
            default_value="0 0 0",
            description="Mount pose rpy for the left hand in the view wrapper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_xyz",
            default_value="0 -0.18 0",
            description="Mount pose xyz for the right hand in the view wrapper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "right_rpy",
            default_value="0 0 0",
            description="Mount pose rpy for the right hand in the view wrapper.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_jsp_gui",
            default_value="true",
            description="Start joint_state_publisher_gui for interactive joint sliders.",
        )
    )

    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    hand = LaunchConfiguration("hand")
    left_xyz = LaunchConfiguration("left_xyz")
    left_rpy = LaunchConfiguration("left_rpy")
    right_xyz = LaunchConfiguration("right_xyz")
    right_rpy = LaunchConfiguration("right_rpy")
    gui = LaunchConfiguration("gui")
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), description_file]),
            ' hand:="',
            hand,
            '"',
            ' left_xyz:="',
            left_xyz,
            '"',
            ' left_rpy:="',
            left_rpy,
            '"',
            ' right_xyz:="',
            right_xyz,
            '"',
            ' right_rpy:="',
            right_rpy,
            '"',
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("asset_description"), "wuji", "rviz", "wujihand_right.rviz"]
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[robot_description],
        condition=IfCondition(use_jsp_gui),
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
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
