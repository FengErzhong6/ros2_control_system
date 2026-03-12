from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    left_xyz = LaunchConfiguration("left_xyz")
    left_rpy = LaunchConfiguration("left_rpy")
    right_xyz = LaunchConfiguration("right_xyz")
    right_rpy = LaunchConfiguration("right_rpy")

    model = LaunchConfiguration("model")

    robot_description = Command(
        [
            "xacro ",
            model,
            " ",
            "left_xyz:=\"",
            left_xyz,
            "\" ",
            " ",
            "left_rpy:=\"",
            left_rpy,
            "\" ",
            " ",
            "right_xyz:=\"",
            right_xyz,
            "\" ",
            " ",
            "right_rpy:=\"",
            right_rpy,
            "\"",
        ]
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), "description/rviz/marvin_dual.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use /clock if true"
            ),
            DeclareLaunchArgument(
                "left_xyz",
                default_value="0 0.037 0.3618964",
                description="Mount pose (xyz) of Base_L in world",
            ),
            DeclareLaunchArgument(
                "left_rpy",
                default_value="-1.5707963 0 0",
                description="Mount pose (rpy) of Base_L in world",
            ),
            DeclareLaunchArgument(
                "right_xyz",
                default_value="0 -0.037 0.3618964",
                description="Mount pose (xyz) of Base_R in world",
            ),
            DeclareLaunchArgument(
                "right_rpy",
                default_value="1.5707963 0 0",
                description="Mount pose (rpy) of Base_R in world",
            ),
            DeclareLaunchArgument(
                "model",
                default_value=[
                    FindPackageShare("ros2_control_marvin"),
                    "/description/urdf/marvin_dual.urdf",
                ],
                description="Absolute path to the (xacro) URDF file",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "robot_description": ParameterValue(
                            robot_description, value_type=str
                        ),
                    }
                ],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
