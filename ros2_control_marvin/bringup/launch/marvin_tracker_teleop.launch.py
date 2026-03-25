"""
Launch HTC Tracker teleop: tracker_publisher (htc_system) + ros2_control with TrackerTeleopController.
Requires htc_system and ros2_control_marvin to be in the same workspace.
"""

import atexit
import os
import select
import sys
import tempfile
import termios
import threading
import time
import tty

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import rclpy
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger


_terminal_gate = None


class TrackerTeleopTerminalGate:
    def __init__(self) -> None:
        self._stop_event = threading.Event()
        self._thread = None
        self._node = None
        self._arm_client = None
        self._enable_client = None
        self._home_client = None
        self._input_file = None
        self._stdin_fd = None
        self._stdin_settings = None
        self._last_key_time = 0.0
        self._debounce_sec = 0.25
        self._started = False

    def start(self) -> None:
        try:
            input_file, input_label = self._open_input_tty()
        except OSError as exc:
            print(f"[tracker_teleop_gate] Failed to attach terminal input: {exc}", flush=True)
            return

        self._input_file = input_file
        self._stdin_fd = input_file.fileno()
        self._stdin_settings = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)

        if not rclpy.ok():
            rclpy.init(args=None)

        self._node = rclpy.create_node("tracker_teleop_terminal_gate")
        self._arm_client = self._node.create_client(SetBool, "/tracker_teleop_controller/set_armed")
        self._enable_client = self._node.create_client(SetBool, "/tracker_teleop_controller/set_enabled")
        self._home_client = self._node.create_client(Trigger, "/tracker_teleop_controller/go_home")

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        atexit.register(self.shutdown)
        print(
            f"[tracker_teleop_gate] Keyboard attached to {input_label}. "
            "[Space]=start/pause teleop, [n]=next sample (go home)",
            flush=True,
        )

    def shutdown(self) -> None:
        self._stop_event.set()
        self._restore_terminal()
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if rclpy.ok():
            rclpy.shutdown()

    def _open_input_tty(self):
        if sys.stdin.isatty():
            return sys.stdin, "stdin"
        return open("/dev/tty", "rb", buffering=0), "/dev/tty"

    def _restore_terminal(self) -> None:
        if self._stdin_fd is not None and self._stdin_settings is not None:
            termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._stdin_settings)
            self._stdin_fd = None
            self._stdin_settings = None
        if self._input_file is not None and self._input_file is not sys.stdin:
            self._input_file.close()
        self._input_file = None

    def _wait_for_service(self, client, label: str, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while not self._stop_event.is_set() and rclpy.ok():
            if client.service_is_ready() or client.wait_for_service(timeout_sec=0.2):
                return True
            if time.monotonic() >= deadline:
                break
        print(f"[tracker_teleop_gate] {label} service timeout", flush=True)
        return False

    def _call_set_bool(self, client, label: str, value: bool) -> bool:
        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)

        deadline = time.monotonic() + 2.0
        while not future.done() and not self._stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if time.monotonic() >= deadline:
                break

        if not future.done() or future.result() is None:
            print(f"[tracker_teleop_gate] {label} request timed out", flush=True)
            return False

        response = future.result()
        if not response.success:
            print(f"[tracker_teleop_gate] {label} rejected: {response.message}", flush=True)
            return False

        print(f"[tracker_teleop_gate] {label}: {response.message}", flush=True)
        return True

    def _call_trigger(self, client, label: str) -> bool:
        request = Trigger.Request()
        future = client.call_async(request)

        deadline = time.monotonic() + 2.0
        while not future.done() and not self._stop_event.is_set() and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if time.monotonic() >= deadline:
                break

        if not future.done() or future.result() is None:
            print(f"[tracker_teleop_gate] {label} request timed out", flush=True)
            return False

        response = future.result()
        if not response.success:
            print(f"[tracker_teleop_gate] {label} rejected: {response.message}", flush=True)
            return False

        print(f"[tracker_teleop_gate] {label}: {response.message}", flush=True)
        return True

    def _handle_space(self) -> None:
        if not self._wait_for_service(self._arm_client, "arm", 10.0):
            return
        if not self._wait_for_service(self._enable_client, "enable", 10.0):
            return

        if self._started:
            if self._call_set_bool(self._enable_client, "stop", False):
                self._started = False
            return

        if not self._call_set_bool(self._arm_client, "start-arm", True):
            return
        if self._call_set_bool(self._enable_client, "start-enable", True):
            self._started = True

    def _handle_next(self) -> None:
        if not self._wait_for_service(self._home_client, "go_home", 10.0):
            return
        if self._call_trigger(self._home_client, "next-sample"):
            self._started = False

    def _run(self) -> None:
        try:
            while not self._stop_event.is_set() and rclpy.ok():
                ready, _, _ = select.select([self._stdin_fd], [], [], 0.1)
                if not ready:
                    continue

                ch = os.read(self._stdin_fd, 1).decode(errors="ignore")
                now = time.monotonic()
                if now - self._last_key_time < self._debounce_sec:
                    continue
                self._last_key_time = now
                if ch == " ":
                    self._handle_space()
                elif ch in ("n", "N"):
                    self._handle_next()
        finally:
            self._restore_terminal()


def maybe_start_terminal_gate(context, *args, **kwargs):
    global _terminal_gate

    use_keyboard_gate = LaunchConfiguration("use_keyboard_gate").perform(context).lower() == "true"
    if not use_keyboard_gate:
        return []

    if _terminal_gate is None:
        _terminal_gate = TrackerTeleopTerminalGate()
        _terminal_gate.start()

    return []


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
            "use_keyboard_gate", default_value="true",
            description="Enable terminal Space key start/stop gate for tracker teleop.",
        ),
        DeclareLaunchArgument(
            "description_package", default_value="ros2_control_marvin",
            description="Package with the composite URDF/XACRO file.",
        ),
        DeclareLaunchArgument(
            "description_file", default_value="description/urdf/marvin_dual.urdf",
            description="Composite URDF/XACRO file to load.",
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
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    pkg = LaunchConfiguration("description_package").perform(context)
    desc_file = LaunchConfiguration("description_file").perform(context)
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
        " use_mock_hardware:=", use_mock_hardware,
    ]
    if grip_L:
        xacro_cmd.append(" use_gripper_L:=true")
    if grip_R:
        xacro_cmd.append(" use_gripper_R:=true")

    robot_description = {
        "robot_description": ParameterValue(Command(xacro_cmd), value_type=str)
    }

    # ── Tracker teleop controller config ───────────────────────────────────
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), "bringup", "config",
         "marvin_tracker_teleop_controllers.yaml"]
    )

    # ── Kinematics config path for TrackerTeleopController ─────────────────
    pkg_share = get_package_share_directory("ros2_control_marvin")
    kine_config_path = os.path.join(pkg_share, "config", "marvinCfg", "ccs_m6_40.MvKDCfg")

    tracker_teleop_params_override = (
        f'tracker_teleop_controller:\n'
        f'  ros__parameters:\n'
        f'    kine_config_path: "{kine_config_path}"\n'
    )
    tracker_teleop_params_file = os.path.join(tempfile.mkdtemp(), "tracker_teleop_kine_params.yaml")
    with open(tracker_teleop_params_file, "w") as f:
        f.write(tracker_teleop_params_override)

    # ── HTC Tracker publisher config (htc_system) ──────────────────────────
    trackers_config = PathJoinSubstitution(
        [FindPackageShare("htc_system"), "bringup", "config", "trackers.yaml"]
    )

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

    # ── Core nodes ─────────────────────────────────────────────────────────
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[
            robot_description,
            controllers_yaml,
            tracker_teleop_params_file,
        ] + gripper_params_files,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # HTC Tracker publisher (broadcasts TF: world -> tracker_left_hand, tracker_torso, etc.)
    tracker_publisher_node = Node(
        package="htc_system",
        executable="tracker_publisher",
        name="tracker_publisher",
        output="screen",
        parameters=[trackers_config],
    )

    # ── Visualisation ─────────────────────────────────────────────────────
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ros2_control_marvin"), "description", "rviz", "marvin_dual.rviz"]
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

    tracker_teleop_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "tracker_teleop_controller",
            "--param-file",
            controllers_yaml,
            "--param-file",
            tracker_teleop_params_file,
            "--controller-manager-timeout",
            "30.0",
            "--service-call-timeout",
            "30.0",
            "--switch-timeout",
            "30.0",
        ],
        output="screen",
    )

    gripper_spawners = []
    for name in gripper_names:
        gripper_spawners.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                name,
                "--controller-manager-timeout",
                "30.0",
                "--service-call-timeout",
                "30.0",
                "--switch-timeout",
                "30.0",
            ],
            output="screen",
        ))

    start_teleop_controllers_after_feedback_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[
                tracker_teleop_controller_spawner,
            ] + gripper_spawners,
        ),
    )

    start_keyboard_gate_after_teleop_controller_ready = RegisterEventHandler(
        OnProcessExit(
            target_action=tracker_teleop_controller_spawner,
            on_exit=[
                OpaqueFunction(function=maybe_start_terminal_gate),
            ],
        ),
    )

    return [
        ros2_control_node,
        robot_state_publisher_node,
        tracker_publisher_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        start_teleop_controllers_after_feedback_ready,
        start_keyboard_gate_after_teleop_controller_ready,
    ]
