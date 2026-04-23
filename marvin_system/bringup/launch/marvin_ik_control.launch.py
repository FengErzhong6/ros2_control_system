import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
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
            "description_package", default_value="marvin_system",
            description="Package with the composite URDF/XACRO file.",
        ),
        DeclareLaunchArgument(
            "description_file", default_value="description/urdf/marvin_dual.urdf",
            description="Composite URDF/XACRO file to load.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
            description="Use mock hardware instead of the real Marvin robot.",
        ),
        DeclareLaunchArgument(
            "use_gripper_L", default_value="false",
            description="Enable OmniPicker gripper on left arm.",
        ),
        DeclareLaunchArgument(
            "marvin_ip", default_value="192.168.1.190",
            description="IP address for the real Marvin hardware connection.",
        ),
        DeclareLaunchArgument(
            "connect_timeout_ms", default_value="1500",
            description="Hardware connection timeout in milliseconds.",
        ),
        DeclareLaunchArgument(
            "state_timeout_ms", default_value="8000",
            description="Hardware state timeout in milliseconds.",
        ),
        DeclareLaunchArgument(
            "profile_switch_timeout_ms", default_value="5000",
            description="Profile switch timeout in milliseconds.",
        ),
        DeclareLaunchArgument(
            "use_gripper_R", default_value="false",
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
        DeclareLaunchArgument("collision_guard_enabled", default_value="false"),
        DeclareLaunchArgument("collision_guard_check_rate_hz", default_value="30.0"),
        DeclareLaunchArgument("collision_guard_min_command_delta_deg", default_value="0.0"),
        DeclareLaunchArgument("collision_guard_interpolation_steps", default_value="6"),
        DeclareLaunchArgument("collision_guard_binary_search_steps", default_value="5"),
        OpaqueFunction(function=launch_setup),
    ])


def launch_setup(context):
    gui = LaunchConfiguration("gui")

    pkg = LaunchConfiguration("description_package").perform(context)
    desc_file = LaunchConfiguration("description_file").perform(context)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    grip_L = LaunchConfiguration("use_gripper_L").perform(context).lower() == "true"
    grip_R = LaunchConfiguration("use_gripper_R").perform(context).lower() == "true"
    marvin_ip = LaunchConfiguration("marvin_ip")
    connect_timeout_ms = LaunchConfiguration("connect_timeout_ms")
    state_timeout_ms = LaunchConfiguration("state_timeout_ms")
    profile_switch_timeout_ms = LaunchConfiguration("profile_switch_timeout_ms")
    left_xyz = LaunchConfiguration("left_xyz")
    left_rpy = LaunchConfiguration("left_rpy")
    right_xyz = LaunchConfiguration("right_xyz")
    right_rpy = LaunchConfiguration("right_rpy")
    collision_guard_enabled = LaunchConfiguration("collision_guard_enabled")
    collision_guard_check_rate_hz = LaunchConfiguration("collision_guard_check_rate_hz")
    collision_guard_min_command_delta_deg = LaunchConfiguration("collision_guard_min_command_delta_deg")
    collision_guard_interpolation_steps = LaunchConfiguration("collision_guard_interpolation_steps")
    collision_guard_binary_search_steps = LaunchConfiguration("collision_guard_binary_search_steps")

    # ── Robot description (xacro) ─────────────────────────────────────────
    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(pkg), desc_file]),
        ' left_xyz:="', left_xyz, '"',
        ' left_rpy:="', left_rpy, '"',
        ' right_xyz:="', right_xyz, '"',
        ' right_rpy:="', right_rpy, '"',
        ' use_mock_hardware:="', use_mock_hardware, '"',
        ' marvin_ip:="', marvin_ip, '"',
        ' connect_timeout_ms:="', connect_timeout_ms, '"',
        ' state_timeout_ms:="', state_timeout_ms, '"',
        ' profile_switch_timeout_ms:="', profile_switch_timeout_ms, '"',
        f" use_gripper_L:={'true' if grip_L else 'false'}",
        f" use_gripper_R:={'true' if grip_R else 'false'}",
        ' collision_guard_enabled:="', collision_guard_enabled, '"',
        ' collision_guard_check_rate_hz:="', collision_guard_check_rate_hz, '"',
        ' collision_guard_min_command_delta_deg:="', collision_guard_min_command_delta_deg, '"',
        ' collision_guard_interpolation_steps:="', collision_guard_interpolation_steps, '"',
        ' collision_guard_binary_search_steps:="', collision_guard_binary_search_steps, '"',
    ]

    robot_description = {
        "robot_description": ParameterValue(Command(xacro_cmd), value_type=str)
    }

    # ── Controller config ─────────────────────────────────────────────────
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("marvin_system"), "bringup", "config", "marvin_ik_controllers.yaml"]
    )

    # ── Kinematics config path (resolved at launch time) ──────────────────
    pkg_share = get_package_share_directory("marvin_system")
    kine_config_path = os.path.join(pkg_share, "config", "marvinCfg", "ccs_m6_40.MvKDCfg")

    ik_params_override = (
        f'ik_controller:\n'
        f'  ros__parameters:\n'
        f'    kine_config_path: "{kine_config_path}"\n'
    )
    ik_params_file = os.path.join(tempfile.mkdtemp(), "ik_kine_params.yaml")
    with open(ik_params_file, "w") as f:
        f.write(ik_params_override)

    # Gripper controller configs (generated per enabled gripper)
    gripper_params_files = []
    gripper_names = []
    for side, enabled in [("L", grip_L), ("R", grip_R)]:
        if not enabled:
            continue
        name = f"gripper_{side}_controller"
        gripper_names.append(name)
        content = (
            f'controller_manager:\n'
            f'  ros__parameters:\n'
            f'    {name}:\n'
            f'      type: forward_command_controller/ForwardCommandController\n'
            f'\n'
            f'{name}:\n'
            f'  ros__parameters:\n'
            f'    interface_name: position\n'
            f'    joints:\n'
            f'      - gripper_{side}\n'
        )
        path = os.path.join(tempfile.mkdtemp(), f"{name}.yaml")
        with open(path, "w") as f:
            f.write(content)
        gripper_params_files.append(path)

    # ── Core ──────────────────────────────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            controllers_yaml,
            ik_params_file,
        ] + gripper_params_files,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ── Visualisation ─────────────────────────────────────────────────────
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

    # ── Controller spawners ───────────────────────────────────────────────
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    ik_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ik_controller",
            "--param-file",
            controllers_yaml,
            "--param-file",
            ik_params_file,
        ],
        output="screen",
    )

    # ── Gripper controller spawners ────────────────────────────────────────
    gripper_spawners = []
    for name in gripper_names:
        gripper_spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name],
            output="screen",
        ))

    # ── Assemble ──────────────────────────────────────────────────────────
    return [
        ros2_control_node,
        robot_state_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        ik_controller_spawner,
    ] + gripper_spawners
