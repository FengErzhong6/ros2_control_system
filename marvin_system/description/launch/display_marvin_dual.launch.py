from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _build_optional_viz_nodes(context, *args, **kwargs):
    """Launch optional visualisation helpers based on CLI arguments.

    - target_pose_L/R  -> static TF frames   (x y z qx qy qz qw)
    - target_vector_L/R -> ARROW markers      (vx vy vz  or  ox oy oz vx vy vz)
                           multiple vectors separated by ';'
    """
    nodes = []

    # --- target pose TF publishers ---
    for suffix, base_frame in [("L", "Base_L"), ("R", "Base_R")]:
        pose_str = LaunchConfiguration(f"target_pose_{suffix}").perform(context)
        if not pose_str.strip():
            continue
        parts = pose_str.split()
        if len(parts) != 7:
            continue
        x, y, z, qx, qy, qz, qw = parts
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=f"target_tf_{suffix}",
                arguments=[
                    "--x", x, "--y", y, "--z", z,
                    "--qx", qx, "--qy", qy, "--qz", qz, "--qw", qw,
                    "--frame-id", base_frame,
                    "--child-frame-id", f"target_{suffix}",
                ],
            )
        )

    # --- vector / plane marker publisher ---
    vec_L = LaunchConfiguration("target_vector_L").perform(context)
    vec_R = LaunchConfiguration("target_vector_R").perform(context)
    plane_L = LaunchConfiguration("target_plane_L").perform(context)
    plane_R = LaunchConfiguration("target_plane_R").perform(context)
    if any(s.strip() for s in (vec_L, vec_R, plane_L, plane_R)):
        nodes.append(
            Node(
                package="marvin_system",
                executable="vector_marker_publisher.py",
                parameters=[{
                    "vectors_L": vec_L, "vectors_R": vec_R,
                    "planes_L": plane_L, "planes_R": plane_R,
                }],
            )
        )

    return nodes


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    left_xyz = LaunchConfiguration("left_xyz")
    left_rpy = LaunchConfiguration("left_rpy")
    right_xyz = LaunchConfiguration("right_xyz")
    right_rpy = LaunchConfiguration("right_rpy")
    use_gripper_L = LaunchConfiguration("use_gripper_L")
    use_gripper_R = LaunchConfiguration("use_gripper_R")

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
            "\" ",
            " ",
            "use_gripper_L:=",
            use_gripper_L,
            " ",
            "use_gripper_R:=",
            use_gripper_R,
        ]
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("marvin_system"), "description/rviz/marvin_dual.rviz"]
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
                "use_gripper_L",
                default_value="true",
                description="Enable left gripper visualization",
            ),
            DeclareLaunchArgument(
                "use_gripper_R",
                default_value="true",
                description="Enable right gripper visualization",
            ),
            DeclareLaunchArgument(
                "model",
                default_value=[
                    FindPackageShare("marvin_system"),
                    "/description/urdf/marvin_dual.urdf",
                ],
                description="Absolute path to the (xacro) URDF file",
            ),
            DeclareLaunchArgument(
                "target_pose_L",
                default_value="",
                description="Target pose in Base_L frame: 'x y z qx qy qz qw'",
            ),
            DeclareLaunchArgument(
                "target_pose_R",
                default_value="",
                description="Target pose in Base_R frame: 'x y z qx qy qz qw'",
            ),
            DeclareLaunchArgument(
                "target_vector_L",
                default_value="",
                description="Vectors in Base_L frame: 'vx vy vz[;ox oy oz vx vy vz;...]'",
            ),
            DeclareLaunchArgument(
                "target_vector_R",
                default_value="",
                description="Vectors in Base_R frame: 'vx vy vz[;ox oy oz vx vy vz;...]'",
            ),
            DeclareLaunchArgument(
                "target_plane_L",
                default_value="",
                description="Plane in Base_L: 'v1x v1y v1z v2x v2y v2z[;ox oy oz ...]'",
            ),
            DeclareLaunchArgument(
                "target_plane_R",
                default_value="",
                description="Plane in Base_R: 'v1x v1y v1z v2x v2y v2z[;ox oy oz ...]'",
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
                package="marvin_system",
                executable="joint_gui_publisher.py",
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare("marvin_system"),
                        "description", "config", "home_joint_positions.yaml"
                    ]),
                ],
            ),
            OpaqueFunction(function=_build_optional_viz_nodes),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
