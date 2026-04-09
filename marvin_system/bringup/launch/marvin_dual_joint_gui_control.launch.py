import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.events import Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml_file(path: str | Path):
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def load_text_file(path: str | Path) -> str:
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


def load_forward_controller_joint_names(path: str | Path) -> list[str]:
    data = load_yaml_file(path)
    params = data.get("forward_position_controller", {}).get("ros__parameters", {})
    joints = params.get("joints", [])
    if not isinstance(joints, list):
        raise RuntimeError(f"Invalid forward_position_controller.joints in {path}")
    names = [str(name) for name in joints if str(name).strip()]
    if not names:
        raise RuntimeError(f"No joints configured for forward_position_controller in {path}")
    return names


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gui", default_value="true",
            description="Start RViz2 automatically.",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="false",
            description="Use mock hardware (GenericSystem) instead of real robot.",
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
            "use_jsp_gui", default_value="true",
            description="Start the custom joint GUI publisher for slider command input.",
        ),
        DeclareLaunchArgument(
            "use_gripper_L", default_value="true",
            description="Enable OmniPicker gripper on left arm.",
        ),
        DeclareLaunchArgument(
            "use_gripper_R", default_value="true",
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
    use_jsp_gui = LaunchConfiguration("use_jsp_gui")

    pkg = LaunchConfiguration("description_package").perform(context)
    desc_file = LaunchConfiguration("description_file").perform(context)
    ctrl_file_arg = LaunchConfiguration("controllers_file").perform(context)
    grip_L = LaunchConfiguration("use_gripper_L").perform(context).lower() == "true"
    grip_R = LaunchConfiguration("use_gripper_R").perform(context).lower() == "true"
    left_xyz = LaunchConfiguration("left_xyz")
    left_rpy = LaunchConfiguration("left_rpy")
    right_xyz = LaunchConfiguration("right_xyz")
    right_rpy = LaunchConfiguration("right_rpy")

    # ── Robot description (xacro) ─────────────────────────────────────────
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

    # ── Controller config (auto-select if not overridden) ─────────────────
    if ctrl_file_arg:
        ctrl_file = ctrl_file_arg
    elif grip_L or grip_R:
        ctrl_file = "bringup/config/marvin_dual_gripper_controllers.yaml"
    else:
        ctrl_file = "bringup/config/marvin_dual_controllers.yaml"

    if os.path.isabs(ctrl_file):
        controllers_yaml_path = ctrl_file
    else:
        controllers_yaml_path = os.path.join(get_package_share_directory("marvin_system"), ctrl_file)
    controllers_yaml = controllers_yaml_path
    trajectory_controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("marvin_system"), "bringup", "config", "marvin_dual_trajectory_controllers.yaml"]
    )

    # ── Joint names for bridge (arm + optional grippers) ──────────────────
    joint_names = load_forward_controller_joint_names(controllers_yaml_path)

    # ── Core ──────────────────────────────────────────────────────────────

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
                "recovery_command_joint_names": joint_names,
                "allow_legacy_go_home_fallback": False,
            },
        ],
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

    # ── GUI slider → forward controller bridge ────────────────────────────

    joint_state_publisher_gui_node = Node(
        package="marvin_system",
        executable="joint_gui_publisher.py",
        output="both",
        parameters=[
            {
                "home_config_path": home_poses_file,
                "publish_topic": "gui_joint_states",
                "feedback_topic": "/joint_states",
                "go_home_service": "/marvin_motion/go_home",
                "go_home_timeout_sec": 20.0,
                "feedback_timeout_sec": 0.0,
                "sync_from_feedback_on_startup": not use_mock_hardware_value,
                "local_home_on_service_failure": False,
                "home_pose_id": "home",
            },
        ],
        condition=IfCondition(use_jsp_gui),
    )

    gui_to_forward_bridge = Node(
        package="marvin_system",
        executable="gui_joint_state_to_forward_command",
        output="screen",
        parameters=[
            {"input_topic": "gui_joint_states"},
            {"output_topic": "/forward_position_controller/commands"},
            {"joint_names": joint_names},
        ],
        condition=IfCondition(use_jsp_gui),
    )

    # ── Controller spawners ───────────────────────────────────────────────

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "30.0",
            "--service-call-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
        ],
        output="screen",
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller",
            "--param-file",
            controllers_yaml,
            "--controller-manager-timeout",
            "30.0",
            "--service-call-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
        ],
        output="screen",
    )

    dual_arm_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dual_arm_trajectory_controller",
            "--param-file",
            trajectory_controllers_yaml,
            "--inactive",
            "--controller-manager-timeout",
            "30.0",
            "--service-call-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
        ],
        output="screen",
    )

    start_forward_controller_after_feedback_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                forward_position_controller_spawner,
            ],
        ),
    )

    start_trajectory_and_bridge_after_forward_controller_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=forward_position_controller_spawner,
            on_exit=[
                dual_arm_trajectory_controller_spawner,
                gui_to_forward_bridge,
            ],
        ),
    )

    start_gui_after_trajectory_controller_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=dual_arm_trajectory_controller_spawner,
            on_exit=[
                joint_state_publisher_gui_node,
            ],
        ),
    )

    shutdown_waiter_node = Node(
        package="marvin_system",
        executable="ros_graph_waiter.py",
        name="ros_graph_waiter",
        output="screen",
        arguments=[
            "--timeout-sec",
            "8.0",
            "--poll-sec",
            "0.2",
            "--stable-count",
            "3",
            "--prefix",
            "/controller_manager",
            "--prefix",
            "/joint_state_broadcaster",
            "--prefix",
            "/forward_position_controller",
            "--prefix",
            "/dual_arm_trajectory_controller",
            "--prefix",
            "/marvin_dual",
            "--prefix",
            "/marvin_motion_server",
            "--prefix",
            "/move_group",
            "--prefix",
            "/move_group/moveit",
            "--prefix",
            "/move_group_private_",
            "--prefix",
            "/moveit_",
            "--prefix",
            "/moveit_simple_controller_manager",
            "--prefix",
            "/robot_state_publisher",
            "--prefix",
            "/gui_joint_state_to_forward_command",
            "--prefix",
            "/planning_scene_interface_",
            "--prefix",
            "/transform_listener_impl_",
        ],
    )

    shutdown_when_gui_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_publisher_gui_node,
            on_exit=[
                EmitEvent(
                    event=ShutdownProcess(
                        process_matcher=matches_action(gui_to_forward_bridge),
                    )
                ),
                EmitEvent(
                    event=ShutdownProcess(
                        process_matcher=matches_action(motion_server_node),
                    )
                ),
                EmitEvent(
                    event=ShutdownProcess(
                        process_matcher=matches_action(ros2_control_node),
                    )
                ),
                EmitEvent(
                    event=ShutdownProcess(
                        process_matcher=matches_action(robot_state_publisher_node),
                    )
                ),
                EmitEvent(
                    event=ShutdownProcess(
                        process_matcher=matches_action(rviz_node),
                    )
                ),
                EmitEvent(
                    event=ShutdownProcess(
                        process_matcher=matches_action(move_group_node),
                    )
                ),
                shutdown_waiter_node,
            ],
        ),
    )

    shutdown_when_waiter_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=shutdown_waiter_node,
            on_exit=[
                EmitEvent(event=Shutdown(reason="ros_graph_waiter completed")),
            ],
        ),
    )

    # ── Assemble ──────────────────────────────────────────────────────────

    return [
        ros2_control_node,
        robot_state_publisher_node,
        move_group_node,
        motion_server_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        start_forward_controller_after_feedback_ready,
        start_trajectory_and_bridge_after_forward_controller_ready,
        start_gui_after_trajectory_controller_ready,
        shutdown_when_gui_exits,
        shutdown_when_waiter_exits,
    ]
