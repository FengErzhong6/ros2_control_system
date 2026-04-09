import os
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def load_yaml_file(path: str | Path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def load_text_file(path: str | Path) -> str:
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "trajectory_bag",
            default_value="",
            description="Path to a recorded joint trajectory rosbag directory.",
        ),
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start RViz2 automatically.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
            description="Use mock hardware (GenericSystem) instead of real robot.",
        ),
        DeclareLaunchArgument(
            "rate_scale", default_value="1.0",
            description="Replay speed multiplier. 1.0 keeps original timing.",
        ),
        DeclareLaunchArgument(
            "start_delay_sec", default_value="1.0",
            description="Delay before replay starts after controllers are ready.",
        ),
        DeclareLaunchArgument(
            "use_gripper_L", default_value="false",
            description="Enable OmniPicker gripper on left arm.",
        ),
        DeclareLaunchArgument(
            "use_gripper_R", default_value="false",
            description="Enable OmniPicker gripper on right arm.",
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
            "controllers_file", default_value="",
            description="Controller YAML (auto-selected when empty).",
        ),
        DeclareLaunchArgument(
            "home_positions_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("marvin_system"), "motion", "config", "home_poses.yaml"]
            ),
            description="YAML file containing the target home joint positions.",
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
    use_mock_hardware_value = (
        LaunchConfiguration("use_mock_hardware").perform(context).lower() == "true"
    )
    workspace_guard_service_name = (
        "" if use_mock_hardware_value else "/marvin_dual/set_workspace_guard_enabled"
    )

    trajectory_bag = LaunchConfiguration("trajectory_bag").perform(context).strip()
    if not trajectory_bag:
        raise RuntimeError("Launch argument 'trajectory_bag' must point to a rosbag directory.")
    trajectory_bag = os.path.expanduser(trajectory_bag)

    pkg = LaunchConfiguration("description_package").perform(context)
    desc_file = LaunchConfiguration("description_file").perform(context)
    ctrl_file_arg = LaunchConfiguration("controllers_file").perform(context)
    home_positions_file = LaunchConfiguration("home_positions_file").perform(context)
    grip_L = LaunchConfiguration("use_gripper_L").perform(context).lower() == "true"
    grip_R = LaunchConfiguration("use_gripper_R").perform(context).lower() == "true"
    left_xyz = LaunchConfiguration("left_xyz")
    left_rpy = LaunchConfiguration("left_rpy")
    right_xyz = LaunchConfiguration("right_xyz")
    right_rpy = LaunchConfiguration("right_rpy")

    xacro_cmd = [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare(pkg), desc_file]),
        ' left_xyz:="', left_xyz, '"',
        ' left_rpy:="', left_rpy, '"',
        ' right_xyz:="', right_xyz, '"',
        ' right_rpy:="', right_rpy, '"',
        f" use_mock_hardware:={'true' if use_mock_hardware_value else 'false'}",
        f" use_gripper_L:={'true' if grip_L else 'false'}",
        f" use_gripper_R:={'true' if grip_R else 'false'}",
    ]

    robot_description = {
        "robot_description": ParameterValue(Command(xacro_cmd), value_type=str)
    }

    if ctrl_file_arg:
        ctrl_file = ctrl_file_arg
    elif grip_L or grip_R:
        ctrl_file = "bringup/config/marvin_dual_gripper_controllers.yaml"
    else:
        ctrl_file = "bringup/config/marvin_dual_controllers.yaml"

    controllers_yaml = PathJoinSubstitution([FindPackageShare("marvin_system"), ctrl_file])
    trajectory_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("marvin_system"), "bringup", "config", "marvin_dual_trajectory_controllers.yaml"]
    )

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

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_description, controllers_yaml, trajectory_controllers_yaml],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    pkg_share = get_package_share_directory("marvin_system")
    home_poses_file = PathJoinSubstitution(
        [FindPackageShare("marvin_system"), "motion", "config", "home_poses.yaml"]
    )
    cell_scene_file = PathJoinSubstitution(
        [FindPackageShare("marvin_system"), "motion", "config", "cell_scene.yaml"]
    )
    srdf_path = Path(pkg_share) / "description" / "srdf" / "marvin_dual.srdf"
    kinematics_path = Path(pkg_share) / "motion" / "config" / "kinematics.yaml"
    joint_limits_path = Path(pkg_share) / "motion" / "config" / "joint_limits.yaml"
    planning_pipelines_path = Path(pkg_share) / "motion" / "config" / "planning_pipelines.yaml"
    ompl_planning_path = Path(pkg_share) / "motion" / "config" / "ompl_planning.yaml"
    moveit_controllers_path = Path(pkg_share) / "motion" / "config" / "moveit_controllers.yaml"

    move_group_params = [
        robot_description,
        {"robot_description_semantic": load_text_file(srdf_path)},
        {"robot_description_kinematics": load_yaml_file(kinematics_path)},
        {"robot_description_planning": load_yaml_file(joint_limits_path)},
        load_yaml_file(planning_pipelines_path),
        {"ompl": load_yaml_file(ompl_planning_path)},
        load_yaml_file(moveit_controllers_path),
        {
            "moveit_manage_controllers": False,
            "publish_robot_description": True,
            "publish_robot_description_semantic": True,
            "publish_planning_scene": True,
            "publish_geometry_updates": True,
            "publish_state_updates": True,
            "publish_transforms_updates": True,
            "allow_trajectory_execution": True,
            "monitor_dynamics": False,
        },
    ]

    move_group_node = Node(
        package="marvin_system",
        executable="move_group_wrapper.py",
        output="screen",
        parameters=move_group_params,
        sigterm_timeout="15.0",
    )

    motion_server_node = Node(
        package="marvin_system",
        executable="motion_server",
        name="marvin_motion_server",
        output="screen",
        parameters=[
            robot_description,
            home_poses_file,
            cell_scene_file,
            {"robot_description_semantic": load_text_file(srdf_path)},
            {"robot_description_kinematics": load_yaml_file(kinematics_path)},
            {"robot_description_planning": load_yaml_file(joint_limits_path)},
            {
                "backend": "moveit",
                "go_home_service_name": "/marvin_motion/go_home",
                "set_mode_service_name": "/marvin_motion/set_mode",
                "get_mode_service_name": "/marvin_motion/get_mode",
                "get_status_service_name": "/marvin_motion/get_status",
                "set_enabled_service_name": "/marvin_motion/set_enabled",
                "legacy_go_home_service": "/tracker_teleop_controller/go_home",
                "tracker_set_armed_service": "",
                "tracker_set_enabled_service": "",
                "teleop_state_topic": "",
                "planning_group": "dual_arm",
                "home_pose_id": "home",
                "initial_mode": "TELEOP",
                "go_home_return_mode": "TELEOP",
                "planning_time_sec": 5.0,
                "move_group_wait_sec": 10.0,
                "num_planning_attempts": 3,
                "max_velocity_scaling": 0.2,
                "max_acceleration_scaling": 0.2,
                "execute_trajectory": True,
                "planning_pipeline_id": "ompl",
                "planner_id": "RRTConnect",
                "scene_frame_id": "world",
                "teleop_service_timeout_sec": 5.0,
                "legacy_go_home_timeout_sec": 10.0,
                "controller_switch_timeout_sec": 5.0,
                "controller_manager_switch_service": "/controller_manager/switch_controller",
                "controller_manager_list_service": "/controller_manager/list_controllers",
                "trajectory_controller_name": "dual_arm_trajectory_controller",
                "primary_controller_name": "forward_position_controller",
                "workspace_guard_service_name": workspace_guard_service_name,
                "workspace_z_min": "0.0",
                "workspace_safety_margin": "0.06",
                "mount_xyz_L": LaunchConfiguration("left_xyz"),
                "mount_rpy_L": LaunchConfiguration("left_rpy"),
                "mount_xyz_R": LaunchConfiguration("right_xyz"),
                "mount_rpy_R": LaunchConfiguration("right_rpy"),
                "tool_offset": "0 -0.245 0" if (grip_L or grip_R) else "",
                "recovery_enabled": True,
                "recovery_command_topic": "/forward_position_controller/commands",
                "allow_legacy_go_home_fallback": False,
            },
        ],
    )

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

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout", "30.0",
            "--service-call-timeout", "30.0",
            "--switch-timeout", "30.0",
        ],
        output="screen",
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "30.0",
            "--service-call-timeout", "30.0",
            "--switch-timeout", "30.0",
        ],
        output="screen",
    )

    dual_arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dual_arm_trajectory_controller",
            "--param-file", trajectory_controllers_yaml,
            "--inactive",
            "--controller-manager-timeout", "30.0",
            "--service-call-timeout", "30.0",
            "--switch-timeout", "30.0",
        ],
        output="screen",
    )

    replay_node = Node(
        package="marvin_system",
        executable="joint_trajectory_bag_replay.py",
        name="joint_trajectory_bag_replay",
        output="screen",
        parameters=[{
            "trajectory_bag": trajectory_bag,
            "joint_names": joint_names,
            "rate_scale": LaunchConfiguration("rate_scale"),
            "start_delay_sec": LaunchConfiguration("start_delay_sec"),
            "command_topic": "/forward_position_controller/commands",
            "go_home_service": "/marvin_motion/go_home",
            "use_motion_go_home": True,
            "fallback_local_home": False,
            "home_positions_file": home_positions_file,
        }],
    )

    start_replay_after_controllers_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=forward_position_controller_spawner,
            on_exit=[replay_node],
        ),
    )

    shutdown_after_replay_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=replay_node,
            on_exit=[EmitEvent(event=Shutdown(reason="Replay node exited"))],
        ),
    )

    start_forward_controller_after_feedback_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner, dual_arm_trajectory_controller_spawner],
        ),
    )

    return [
        ros2_control_node,
        robot_state_publisher_node,
        move_group_node,
        motion_server_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        start_forward_controller_after_feedback_ready,
        start_replay_after_controllers_ready,
        shutdown_after_replay_exit,
    ]
