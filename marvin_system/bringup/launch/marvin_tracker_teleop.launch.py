"""
Launch HTC Tracker teleop: tracker_publisher (htc_system) + ros2_control with TrackerTeleopController.
Requires htc_system and marvin_system to be in the same workspace.
"""

import atexit
from datetime import datetime
import os
import select
import shlex
import shutil
import subprocess
import sys
import tempfile
import termios
import threading
import time
import tty
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import rclpy
from rosbag2_interfaces.srv import Pause, Resume, Stop
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger


_terminal_gate = None
DEFAULT_TRAJECTORY_ROOT = os.path.expanduser("~/.ros/marvin_trajectory")
DEFAULT_TRAJECTORY_BAG_PREFIX = "marvin_joint_trajectory"
DEFAULT_TRAJECTORY_TOPICS = ["/joint_states"]


def load_launch_config(config_path):
    data = load_yaml_file(config_path)
    return data.get("marvin_tracker_teleop", {})


def load_yaml_file(path):
    with open(path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected mapping at YAML root: {path}")
    return data


def resolve_camera_namespace(cameras_cfg: dict, camera_name: str) -> str:
    cameras = cameras_cfg.get("cameras", {})
    camera_cfg = cameras.get(camera_name)
    if not isinstance(camera_cfg, dict):
        raise RuntimeError(f"Camera '{camera_name}' not found in cameras config")

    namespace = camera_cfg.get("namespace", camera_name)
    if not isinstance(namespace, str) or not namespace.strip():
        raise RuntimeError(f"Camera '{camera_name}' has an invalid namespace")
    return namespace


def _parse_bool(value, fallback: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered == "true":
            return True
        if lowered == "false":
            return False
    return fallback


def resolve_bool_arg(arg_value: str, config: dict, key: str, fallback: bool) -> bool:
    if arg_value.strip().lower() != "default":
        return _parse_bool(arg_value, fallback)
    return _parse_bool(config.get(key, fallback), fallback)


def resolve_string_arg(arg_value: str, config: dict, key: str, fallback: str) -> str:
    if arg_value.strip().lower() != "default":
        return arg_value

    config_value = config.get(key, fallback)
    if isinstance(config_value, str) and config_value.strip().lower() == "default":
        return fallback
    return str(config_value)


def resolve_topics_arg(arg_value: str, config: dict, key: str, fallback) -> list[str]:
    if arg_value.strip().lower() != "default":
        topics = shlex.split(arg_value)
        return topics or list(fallback)

    config_value = config.get(key, fallback)
    if isinstance(config_value, str):
        if config_value.strip().lower() == "default":
            return list(fallback)
        topics = shlex.split(config_value)
        return topics or list(fallback)

    if isinstance(config_value, (list, tuple)):
        topics = [str(topic) for topic in config_value if str(topic).strip()]
        return topics or list(fallback)

    return list(fallback)


class TrackerTeleopRosbagRecorder:
    def __init__(
        self,
        node,
        *,
        enabled: bool,
        trajectory_root: str,
        bag_prefix: str,
        bag_topics,
    ) -> None:
        self._node = node
        self._enabled = enabled
        self._trajectory_root = os.path.expanduser(trajectory_root)
        self._bag_prefix = bag_prefix
        self._bag_topics = list(bag_topics)
        self._recorder_node_name = "tracker_teleop_joint_recorder"
        self._resume_client = None
        self._pause_client = None
        self._stop_client = None
        self._process = None
        self._current_bag_dir = None
        self._recording_active = False
        self._session_started = False

        if not self._enabled:
            return

        recorder_ns = f"/{self._recorder_node_name}"
        self._resume_client = self._node.create_client(
            Resume, f"{recorder_ns}/resume")
        self._pause_client = self._node.create_client(
            Pause, f"{recorder_ns}/pause")
        self._stop_client = self._node.create_client(
            Stop, f"{recorder_ns}/stop")

    def start(self) -> None:
        if not self._enabled:
            return
        if not self._spawn_paused_session():
            print(
                "[tracker_teleop_gate] Joint trajectory recorder is disabled "
                "for this run because ros2 bag could not be prepared.",
                flush=True,
            )
            self._enabled = False

    def shutdown(self) -> None:
        if not self._enabled or self._process is None:
            return

        if self._recording_active:
            self.finish_session(prepare_next=False)
            return

        self._terminate_process(remove_bag_dir=not self._session_started)

    def begin_recording(self) -> bool:
        if not self._enabled:
            return True

        if self._process is None or self._process.poll() is not None:
            if not self._spawn_paused_session():
                return False

        if self._recording_active:
            return True

        if not self._call_empty_service(
            self._resume_client,
            Resume.Request(),
            "resume recorder",
            timeout_sec=5.0,
        ):
            return False

        self._recording_active = True
        self._session_started = True
        print(
            f"[tracker_teleop_gate] Recording joint trajectory to: {self._current_bag_dir}",
            flush=True,
        )
        return True

    def finish_session(self, prepare_next: bool = True) -> bool:
        if not self._enabled or self._process is None:
            return True

        if self._recording_active:
            if not self._call_empty_service(
                self._pause_client,
                Pause.Request(),
                "pause recorder",
                timeout_sec=5.0,
            ):
                return False
            self._recording_active = False

        if not self._call_empty_service(
            self._stop_client,
            Stop.Request(),
            "stop recorder",
            timeout_sec=5.0,
        ):
            self._terminate_process()
            return False

        bag_dir = self._current_bag_dir
        process = self._process
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            print(
                "[tracker_teleop_gate] Recorder stop timed out; terminating ros2 bag.",
                flush=True,
            )
            self._terminate_process()
            return False

        self._process = None
        self._current_bag_dir = None
        print(
            f"[tracker_teleop_gate] Joint trajectory bag saved: {bag_dir}",
            flush=True,
        )

        if prepare_next:
            return self._spawn_paused_session()
        return True

    def _spawn_paused_session(self) -> bool:
        os.makedirs(self._trajectory_root, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_dir = os.path.join(self._trajectory_root, f"{self._bag_prefix}_{timestamp}")
        cmd = [
            "ros2",
            "bag",
            "record",
            "--start-paused",
            "--disable-keyboard-controls",
            "--node-name",
            self._recorder_node_name,
            "-o",
            bag_dir,
            "--topics",
            *self._bag_topics,
        ]

        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=None,
                stderr=None,
                start_new_session=True,
            )
        except OSError as exc:
            print(
                f"[tracker_teleop_gate] Failed to start ros2 bag recorder: {exc}",
                flush=True,
            )
            self._process = None
            self._current_bag_dir = None
            return False

        self._current_bag_dir = bag_dir
        self._recording_active = False
        self._session_started = False

        if not self._wait_for_service(
            self._resume_client, "recorder resume", timeout_sec=10.0):
            self._terminate_process(remove_bag_dir=True)
            return False
        if not self._wait_for_service(
            self._pause_client, "recorder pause", timeout_sec=10.0):
            self._terminate_process(remove_bag_dir=True)
            return False
        if not self._wait_for_service(
            self._stop_client, "recorder stop", timeout_sec=10.0):
            self._terminate_process(remove_bag_dir=True)
            return False

        print(
            f"[tracker_teleop_gate] Joint trajectory recorder ready: {bag_dir}",
            flush=True,
        )
        return True

    def _wait_for_service(self, client, label: str, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok():
            if self._process is not None and self._process.poll() is not None:
                print(
                    f"[tracker_teleop_gate] {label} unavailable because ros2 bag exited.",
                    flush=True,
                )
                return False
            if client.service_is_ready() or client.wait_for_service(timeout_sec=0.2):
                return True
            if time.monotonic() >= deadline:
                break
        print(f"[tracker_teleop_gate] {label} service timeout", flush=True)
        return False

    def _call_empty_service(self, client, request, label: str, timeout_sec: float) -> bool:
        if not self._wait_for_service(client, label, timeout_sec):
            return False

        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and rclpy.ok():
            rclpy.spin_once(self._node, timeout_sec=0.05)
            if time.monotonic() >= deadline:
                break

        if not future.done():
            print(f"[tracker_teleop_gate] {label} request timed out", flush=True)
            return False

        if future.exception() is not None:
            print(
                f"[tracker_teleop_gate] {label} request failed: {future.exception()}",
                flush=True,
            )
            return False

        return True

    def _terminate_process(self, remove_bag_dir: bool = False) -> None:
        process = self._process
        bag_dir = self._current_bag_dir
        if process is None:
            return

        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)

        self._process = None
        self._current_bag_dir = None
        self._recording_active = False
        self._session_started = False
        if remove_bag_dir and bag_dir:
            shutil.rmtree(bag_dir, ignore_errors=True)


class TrackerTeleopTerminalGate:
    def __init__(
        self,
        *,
        enable_joint_recording: bool,
        trajectory_root: str,
        bag_prefix: str,
        bag_topics,
    ) -> None:
        self._stop_event = threading.Event()
        self._thread = None
        self._node = None
        self._arm_client = None
        self._enable_client = None
        self._home_client = None
        self._recorder = None
        self._input_file = None
        self._stdin_fd = None
        self._stdin_settings = None
        self._last_key_time = 0.0
        self._debounce_sec = 0.25
        self._started = False
        self._enable_joint_recording = enable_joint_recording
        self._trajectory_root = trajectory_root
        self._bag_prefix = bag_prefix
        self._bag_topics = list(bag_topics)

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
        self._recorder = TrackerTeleopRosbagRecorder(
            self._node,
            enabled=self._enable_joint_recording,
            trajectory_root=self._trajectory_root,
            bag_prefix=self._bag_prefix,
            bag_topics=self._bag_topics,
        )
        self._recorder.start()

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        atexit.register(self.shutdown)
        recorder_help = ""
        if self._enable_joint_recording:
            recorder_help = " + joint rosbag"
        print(
            f"[tracker_teleop_gate] Keyboard attached to {input_label}. "
            f"[Space]=start/pause teleop{recorder_help}, [n]=next sample (go home)",
            flush=True,
        )

    def shutdown(self) -> None:
        self._stop_event.set()
        self._restore_terminal()
        if self._recorder is not None:
            self._recorder.shutdown()
            self._recorder = None
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
                if self._recorder is not None:
                    self._recorder.finish_session()
                self._started = False
            return

        if self._recorder is not None and not self._recorder.begin_recording():
            print("[tracker_teleop_gate] Start aborted because ros2 bag is not ready", flush=True)
            return
        if not self._call_set_bool(self._arm_client, "start-arm", True):
            if self._recorder is not None:
                self._recorder.finish_session()
            return
        if self._call_set_bool(self._enable_client, "start-enable", True):
            self._started = True
            return

        if self._recorder is not None:
            self._recorder.finish_session()

    def _handle_next(self) -> None:
        if not self._wait_for_service(self._home_client, "go_home", 10.0):
            return
        if self._started and self._recorder is not None:
            self._recorder.finish_session()
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

    config_file = LaunchConfiguration("config_file").perform(context)
    config = load_launch_config(config_file)

    enable_joint_recording = resolve_bool_arg(
        LaunchConfiguration("enable_joint_recording").perform(context),
        config,
        "enable_joint_recording",
        True,
    )
    trajectory_root = os.path.expanduser(resolve_string_arg(
        LaunchConfiguration("trajectory_root").perform(context),
        config,
        "trajectory_root",
        DEFAULT_TRAJECTORY_ROOT,
    ))
    bag_prefix = resolve_string_arg(
        LaunchConfiguration("trajectory_bag_prefix").perform(context),
        config,
        "trajectory_bag_prefix",
        DEFAULT_TRAJECTORY_BAG_PREFIX,
    )
    bag_topics = resolve_topics_arg(
        LaunchConfiguration("trajectory_bag_topics").perform(context),
        config,
        "trajectory_bag_topics",
        DEFAULT_TRAJECTORY_TOPICS,
    )

    if _terminal_gate is None:
        _terminal_gate = TrackerTeleopTerminalGate(
            enable_joint_recording=enable_joint_recording,
            trajectory_root=trajectory_root,
            bag_prefix=bag_prefix,
            bag_topics=bag_topics,
        )
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
            "start_cameras", default_value="true",
            description="Launch the three cameras when using real hardware.",
        ),
        DeclareLaunchArgument(
            "show_camera_views", default_value="true",
            description="Open a single camera dashboard window when cameras are launched.",
        ),
        DeclareLaunchArgument(
            "high_camera_name", default_value="cam_high",
            description="Camera key in cameras.yaml for the RealSense top camera.",
        ),
        DeclareLaunchArgument(
            "cameras_config",
            default_value=PathJoinSubstitution(
                [FindPackageShare("camera_system"), "bringup", "config", "cameras.yaml"]
            ),
            description="Path to the shared camera inventory YAML file.",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("marvin_system"), "bringup", "config", "marvin_tracker_teleop.yaml"]
            ),
            description="YAML config for tracker teleop launch defaults.",
        ),
        DeclareLaunchArgument(
            "enable_joint_recording", default_value="default",
            description="Use true/false to override YAML; use default to keep YAML setting.",
        ),
        DeclareLaunchArgument(
            "trajectory_root", default_value="default",
            description="Override joint trajectory bag directory; use default to keep YAML setting.",
        ),
        DeclareLaunchArgument(
            "trajectory_bag_prefix", default_value="default",
            description="Override trajectory bag prefix; use default to keep YAML setting.",
        ),
        DeclareLaunchArgument(
            "trajectory_bag_topics", default_value="default",
            description="Override bag topics with a space-delimited list; use default to keep YAML setting.",
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
    use_mock_hardware_value = use_mock_hardware.perform(context).lower() == "true"
    start_cameras_value = LaunchConfiguration("start_cameras").perform(context).lower() == "true"
    show_camera_views_value = LaunchConfiguration("show_camera_views").perform(context).lower() == "true"
    high_camera_name_value = LaunchConfiguration("high_camera_name").perform(context)
    cameras_config_path = LaunchConfiguration("cameras_config").perform(context)

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
        [FindPackageShare("marvin_system"), "bringup", "config",
         "marvin_tracker_teleop_controllers.yaml"]
    )

    # ── Kinematics config path for TrackerTeleopController ─────────────────
    pkg_share = get_package_share_directory("marvin_system")
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

    camera_actions = []
    if start_cameras_value and not use_mock_hardware_value:
        cameras_cfg = load_yaml_file(cameras_config_path)
        left_namespace = resolve_camera_namespace(cameras_cfg, "cam_left_wrist")
        right_namespace = resolve_camera_namespace(cameras_cfg, "cam_right_wrist")
        high_namespace = resolve_camera_namespace(cameras_cfg, high_camera_name_value)

        camera_actions.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("camera_system"), "bringup", "launch", "dual_wrist_orbbec.launch.py"]
                    )
                ),
                launch_arguments={
                    "cameras_config": cameras_config_path,
                    "use_showimage": "false",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("camera_system"), "bringup", "launch", "single_realsense.launch.py"]
                    )
                ),
                launch_arguments={
                    "camera_name": high_camera_name_value,
                    "cameras_config": cameras_config_path,
                    "use_showimage": "false",
                }.items(),
            ),
        ])

        if show_camera_views_value:
            camera_actions.append(
                Node(
                    package="marvin_system",
                    executable="tracker_camera_dashboard.py",
                    name="tracker_camera_dashboard",
                    output="screen",
                    parameters=[{
                        "window_title": "Marvin Tracker Camera Dashboard",
                        "left_camera_title": "Left Wrist",
                        "right_camera_title": "Right Wrist",
                        "high_camera_title": "High Camera",
                        "left_image_topic": f"/{left_namespace}/image_raw",
                        "right_image_topic": f"/{right_namespace}/image_raw",
                        "high_image_topic": f"/{high_namespace}/{high_namespace}/color/image_raw",
                    }],
                )
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
        *camera_actions,
        rviz_node,
        joint_state_broadcaster_spawner,
        start_teleop_controllers_after_feedback_ready,
        start_keyboard_gate_after_teleop_controller_ready,
    ]
