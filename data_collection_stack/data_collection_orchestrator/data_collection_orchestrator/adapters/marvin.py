from __future__ import annotations

from controller_manager_msgs.srv import ListControllers
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from marvin_system.srv import GetMotionMode, GetMotionStatus, SetMotionMode
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
import yaml

from ..managed_launch import ManagedLaunchSession
from .base import AdapterBase, AdapterResult


class MarvinAdapter(AdapterBase):
    DEFAULT_LAUNCH_PACKAGE = "marvin_system"
    DEFAULT_LAUNCH_FILE = "marvin_tracker_teleop.launch.py"
    DEFAULT_HOME_POSES_RELATIVE_PATH = "motion/config/home_poses.yaml"
    DEFAULT_CONTROLLER_CONFIG_RELATIVE_PATH = "bringup/config/marvin_tracker_teleop_controllers.yaml"
    TRAJECTORY_CONTROLLER_NAME = "dual_arm_trajectory_controller"
    REQUIRED_CONTROLLER_STATES = {
        "joint_state_broadcaster": "active",
        "tracker_teleop_controller": "active",
    }
    REQUIRED_SERVICES = {
        "/marvin_motion/go_home": "std_srvs/srv/Trigger",
        "/marvin_motion/set_mode": "marvin_system/srv/SetMotionMode",
        "/marvin_motion/get_mode": "marvin_system/srv/GetMotionMode",
        "/marvin_motion/get_status": "marvin_system/srv/GetMotionStatus",
        "/marvin_motion/set_enabled": "std_srvs/srv/SetBool",
    }
    GRAPH_CLEANUP_SERVICES = (
        "/controller_manager/list_controllers",
        "/controller_manager/switch_controller",
        "/marvin_motion/go_home",
        "/marvin_motion/set_mode",
        "/marvin_motion/get_mode",
        "/marvin_motion/get_status",
        "/marvin_motion/set_enabled",
        "/marvin_dual/set_collision_guard_enabled",
        "/marvin_dual/set_control_profile",
    )
    GRAPH_CLEANUP_NODE_BASENAMES = (
        "controller_manager",
        "robot_state_publisher",
        "marvin_motion_server",
        "move_group",
        "manus_gripper_node",
    )
    EXTERNAL_PROCESS_MATCH_MARKERS = (
        "marvin_tracker_teleop.launch.py",
        "ros2 launch marvin_system",
        "marvin_system marvin_tracker_teleop.launch.py",
        "install/marvin_system/lib/marvin_system/motion_server",
        "install/marvin_system/lib/marvin_system/move_group_wrapper.py",
        "marvin_motion_server",
        "dual_arm_controller_manager.yaml",
        "marvin_tracker_teleop_controllers.yaml",
        "tracker_teleop_controller",
        "dual_arm_trajectory_controller",
        "gripper_L_controller",
        "gripper_R_controller",
    )

    def __init__(self, device, node=None) -> None:
        super().__init__(device, node=node)
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._managed_launch: ManagedLaunchSession | None = None
        self._log_path: Path | None = None
        self._launch_command: list[str] = []
        self._ready_relaunch_count = 0
        self._latest_teleop_state = ""
        self._teleop_state_subscription = None
        self._joint_state_subscription = None
        self._service_callback_group = None
        self._list_controllers_client = None
        self._motion_go_home_client = None
        self._motion_set_mode_client = None
        self._motion_get_mode_client = None
        self._motion_get_status_client = None
        self._motion_set_enabled_client = None
        self._joint_positions: dict[str, float] = {}
        self._last_joint_state_monotonic = 0.0
        self._home_joint_targets, self._home_tolerance_rad = self._load_home_targets()
        if self.node is not None:
            teleop_state_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._service_callback_group = ReentrantCallbackGroup()
            self._teleop_state_subscription = self.node.create_subscription(
                String,
                "/tracker_teleop_controller/teleop_state",
                self._on_teleop_state,
                teleop_state_qos,
            )
            self._joint_state_subscription = self.node.create_subscription(
                JointState,
                "/joint_states",
                self._on_joint_state,
                qos_profile_sensor_data,
            )
            self._motion_go_home_client = self.node.create_client(
                Trigger,
                "/marvin_motion/go_home",
                callback_group=self._service_callback_group,
            )
            self._list_controllers_client = self.node.create_client(
                ListControllers,
                "/controller_manager/list_controllers",
                callback_group=self._service_callback_group,
            )
            self._motion_set_mode_client = self.node.create_client(
                SetMotionMode,
                "/marvin_motion/set_mode",
                callback_group=self._service_callback_group,
            )
            self._motion_get_mode_client = self.node.create_client(
                GetMotionMode,
                "/marvin_motion/get_mode",
                callback_group=self._service_callback_group,
            )
            self._motion_get_status_client = self.node.create_client(
                GetMotionStatus,
                "/marvin_motion/get_status",
                callback_group=self._service_callback_group,
            )
            self._motion_set_enabled_client = self.node.create_client(
                SetBool,
                "/marvin_motion/set_enabled",
                callback_group=self._service_callback_group,
            )

    def precheck(self) -> AdapterResult:
        if shutil.which("ros2") is None:
            return AdapterResult.failed("ros2 CLI not found in PATH.")

        launch_arguments = self._launch_arguments()
        invalid_keys = [key for key in ("launch_file", "launch_package") if key in launch_arguments]
        if invalid_keys:
            return AdapterResult.failed(
                f"{self.device.device_id}: launch_arguments must not override reserved keys {invalid_keys}"
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: Marvin launch precheck passed.",
            metadata={"launch_arguments": launch_arguments},
        )

    def bringup(self) -> AdapterResult:
        if self._managed_launch is not None and self._managed_launch.is_running():
            return AdapterResult.ok(f"{self.device.device_id}: Marvin bringup already running.")
        if self._process is not None and self._process.poll() is None:
            return AdapterResult.ok(f"{self.device.device_id}: Marvin bringup already running.")

        self._ready_relaunch_count = 0
        return self._start_launch_process()

    def wait_ready(self, timeout_sec: float) -> AdapterResult:
        ready_timeout_sec = self._ready_timeout_sec(timeout_sec)
        relaunch_budget = self._ready_relaunch_attempts()
        no_ready_restart_timeout_sec = self._startup_no_ready_restart_timeout_sec()

        while True:
            process = self._process
            managed_launch = self._managed_launch
            if process is None and managed_launch is None:
                return AdapterResult.failed(
                    f"{self.device.device_id}: Marvin launch was not started."
                )

            last_controllers: dict[str, str] | None = None
            last_missing_controllers = list(self._required_controller_states().keys())
            last_missing_services: list[str] | None = None
            last_motion_status = "not queried"
            next_progress_report_monotonic = time.monotonic() + 5.0
            attempt_started_monotonic = time.monotonic()
            deadline = time.monotonic() + ready_timeout_sec
            failure_summary = ""
            restarted_early = False
            while time.monotonic() < deadline:
                launch_running = (
                    managed_launch.is_running()
                    if managed_launch is not None
                    else (process is not None and process.poll() is None)
                )
                if not launch_running:
                    failure_summary = (
                        f"{self.device.device_id}: Marvin launch exited before READY.\n"
                        f"{self._diagnostic_summary()}"
                    )
                    break

                controllers = self._list_controllers()
                if controllers is not None:
                    last_controllers = controllers
                    last_missing_controllers = self._missing_required_controllers(controllers)
                else:
                    last_motion_status = "waiting for controller_manager/list_controllers"

                if controllers is not None and not last_missing_controllers:
                    missing_services = [
                        service_name
                        for service_name, service_type in self.REQUIRED_SERVICES.items()
                        if not self._service_available(service_name, service_type)
                    ]
                    last_missing_services = missing_services
                    if not missing_services:
                        status_response, status_error = self._motion_status_response(timeout_sec=1.0)
                        last_motion_status = self._motion_status_debug_summary(
                            status_response,
                            status_error,
                        )
                        if status_response is not None:
                            if (
                                bool(getattr(status_response, "success", False))
                                and not bool(getattr(status_response, "motion_busy", False))
                                and bool(getattr(status_response, "controller_interlock_ok", True))
                            ):
                                return AdapterResult.ok(
                                    f"{self.device.device_id}: controller_manager and marvin_motion are READY "
                                    f"(teleop and go_home readiness satisfied).",
                                    metadata={
                                        "controllers": controllers,
                                        "mode": str(getattr(status_response, "mode", "")).strip(),
                                        "teleop_state": str(
                                            getattr(status_response, "teleop_state", "")
                                        ).strip(),
                                        "go_home_ready": True,
                                        "log_path": None if self._log_path is None else str(self._log_path),
                                    },
                                )
                    else:
                        last_motion_status = "waiting for required marvin_motion services"

                if time.monotonic() >= next_progress_report_monotonic:
                    if self.node is not None:
                        self.node.get_logger().warn(
                            f"{self.device.device_id}: waiting for READY. "
                            f"{self._ready_wait_debug_summary(last_controllers, last_missing_controllers, last_missing_services, last_motion_status).replace(chr(10), ' | ')}"
                        )
                    next_progress_report_monotonic = time.monotonic() + 5.0

                early_restart_reason = self._early_restart_reason_from_log()
                if relaunch_budget > 0 and early_restart_reason:
                    restart_result = self._restart_launch_for_ready_failure(
                        reason=early_restart_reason,
                    )
                    if restart_result.is_failure():
                        return AdapterResult.failed(
                            f"{self.device.device_id}: Marvin early auto-restart failed. "
                            f"{restart_result.summary}"
                        )
                    relaunch_budget -= 1
                    restarted_early = True
                    break

                if (
                    relaunch_budget > 0
                    and no_ready_restart_timeout_sec > 0.0
                    and time.monotonic() - attempt_started_monotonic >= no_ready_restart_timeout_sec
                ):
                    restart_result = self._restart_launch_for_ready_failure(
                        reason=(
                            f"marvin did not satisfy READY conditions within "
                            f"{no_ready_restart_timeout_sec:.1f}s on first launch"
                        ),
                    )
                    if restart_result.is_failure():
                        return AdapterResult.failed(
                            f"{self.device.device_id}: Marvin early auto-restart failed. "
                            f"{restart_result.summary}"
                        )
                    relaunch_budget -= 1
                    restarted_early = True
                    break

                time.sleep(0.5)
            else:
                failure_summary = (
                    f"{self.device.device_id}: Marvin READY timeout after "
                    f"{ready_timeout_sec:.1f}s.\n"
                    f"{self._ready_wait_debug_summary(last_controllers, last_missing_controllers, last_missing_services, last_motion_status)}\n"
                    f"{self._diagnostic_summary()}"
                )

            if restarted_early:
                continue

            if relaunch_budget <= 0:
                return AdapterResult.failed(failure_summary)

            relaunch_budget -= 1
            restart_result = self._restart_launch_for_ready_failure(
                reason=(
                    "marvin launch exited before READY"
                    if not launch_running
                    else "marvin did not satisfy READY conditions on first launch"
                ),
            )
            if restart_result.is_failure():
                return AdapterResult.failed(
                    f"{failure_summary}\nAuto-restart failed: {restart_result.summary}"
                )

    def shutdown(self) -> AdapterResult:
        if self._launch_is_running() and self._ros_context_is_usable():
            self._call_set_bool_service("/marvin_motion/set_enabled", False, timeout_sec=3.0)
            self._call_set_mode_service("SAFE_HOLD", timeout_sec=3.0)

        managed_launch = self._managed_launch
        if managed_launch is not None:
            if not managed_launch.is_running():
                exit_code = managed_launch.launch_exit_code()
                self._cleanup_process_state()
                cleanup_result = self._ensure_graph_cleanup(
                    timeout_sec=self._graph_cleanup_timeout_sec(),
                    context_label="managed launch exit",
                    allow_external_cleanup=True,
                )
                if cleanup_result.is_failure():
                    return cleanup_result
                return AdapterResult.ok(
                    f"{self.device.device_id}: Marvin launch already exited with code {exit_code}."
                )

            if not managed_launch.shutdown(timeout_sec=18.0):
                return AdapterResult.failed(
                    f"{self.device.device_id}: managed Marvin launch did not stop within timeout."
                )

            self._cleanup_process_state()
            cleanup_result = self._ensure_graph_cleanup(
                timeout_sec=self._graph_cleanup_timeout_sec(),
                context_label="managed shutdown",
                allow_external_cleanup=True,
            )
            if cleanup_result.is_failure():
                return cleanup_result
            return AdapterResult.ok(f"{self.device.device_id}: Marvin launch stopped.")

        process = self._process
        if process is None:
            self._cleanup_process_state()
            cleanup_result = self._ensure_graph_cleanup(
                timeout_sec=self._graph_cleanup_timeout_sec(),
                context_label="shutdown with no tracked process",
                allow_external_cleanup=True,
            )
            if cleanup_result.is_failure():
                return cleanup_result
            return AdapterResult.ok(f"{self.device.device_id}: Marvin launch already stopped.")

        if process.poll() is not None:
            exit_code = process.returncode
            self._cleanup_process_state()
            cleanup_result = self._ensure_graph_cleanup(
                timeout_sec=self._graph_cleanup_timeout_sec(),
                context_label="subprocess exit",
                allow_external_cleanup=True,
            )
            if cleanup_result.is_failure():
                return cleanup_result
            return AdapterResult.ok(
                f"{self.device.device_id}: Marvin launch already exited with code {exit_code}."
            )

        # Always drain the full process group before returning. The ros2 launch parent can
        # exit while child nodes are still shutting down, and those stale child processes can
        # confuse the next bringup attempt by keeping old ROS graph services alive.
        if not self._signal_process_group(process, signal.SIGINT, timeout_sec=12.0):
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=6.0)
        if not self._wait_for_process_group_exit(process.pid, timeout_sec=2.0):
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=4.0)
        if not self._wait_for_process_group_exit(process.pid, timeout_sec=2.0):
            self._signal_process_group(process, signal.SIGKILL, timeout_sec=2.0)
        self._wait_for_process_group_exit(process.pid, timeout_sec=1.0)

        self._cleanup_process_state()
        cleanup_result = self._ensure_graph_cleanup(
            timeout_sec=self._graph_cleanup_timeout_sec(),
            context_label="subprocess shutdown",
            allow_external_cleanup=True,
        )
        if cleanup_result.is_failure():
            return cleanup_result
        return AdapterResult.ok(f"{self.device.device_id}: Marvin launch stopped.")

    def diagnose(self) -> AdapterResult:
        if self._managed_launch is None and self._process is None:
            return AdapterResult.failed(f"{self.device.device_id}: Marvin launch not started.")
        if self._managed_launch is not None and not self._managed_launch.is_running():
            return AdapterResult.failed(
                f"{self.device.device_id}: Marvin launch exited with code {self._managed_launch.launch_exit_code()}."
            )
        if self._managed_launch is None and self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: Marvin launch exited with code {self._process.returncode}."
            )

        status_response, status_error = self._call_service_response(
            client=self._motion_get_status_client,
            request=GetMotionStatus.Request(),
            timeout_sec=2.0,
        )
        if status_response is None:
            return AdapterResult.degraded(
                f"{self.device.device_id}: motion status unavailable: {status_error}",
                metadata={
                    "teleop_state": self._latest_teleop_state,
                    "log_path": None if self._log_path is None else str(self._log_path),
                    "command": self._launch_command,
                },
            )

        if not bool(getattr(status_response, "success", False)):
            return AdapterResult.degraded(
                f"{self.device.device_id}: motion layer reported unhealthy status: "
                f"{getattr(status_response, 'message', '')}",
                metadata={
                    "mode": getattr(status_response, "mode", ""),
                    "teleop_state": getattr(status_response, "teleop_state", self._latest_teleop_state),
                    "primary_controller_state": getattr(status_response, "primary_controller_state", ""),
                    "trajectory_controller_state": getattr(status_response, "trajectory_controller_state", ""),
                    "log_path": None if self._log_path is None else str(self._log_path),
                    "command": self._launch_command,
                },
            )

        teleop_state = str(getattr(status_response, "teleop_state", "")).strip() or "unknown"
        mode = str(getattr(status_response, "mode", "")).strip() or "unknown"
        return AdapterResult.ok(
            f"{self.device.device_id}: motion layer healthy (mode={mode}, teleop_state={teleop_state}).",
            metadata={
                "mode": mode,
                "teleop_state": teleop_state,
                "log_path": None if self._log_path is None else str(self._log_path),
                "command": self._launch_command,
            },
        )

    def dump_metadata(self) -> dict:
        metadata = super().dump_metadata()
        metadata.update(
            {
                "launch_package": self._launch_package(),
                "launch_file": self._launch_file(),
                "launch_arguments": self._launch_arguments(),
                "log_path": None if self._log_path is None else str(self._log_path),
            }
        )
        if self._managed_launch is not None:
            metadata.update(self._managed_launch.metadata())
        return metadata

    def record_topics(self) -> list[str]:
        topics = self.device.config.get("record_topics", [])
        if isinstance(topics, list):
            return [str(topic) for topic in topics]
        return ["/joint_states", "/tf", "/tf_static"]

    def arm(self) -> AdapterResult:
        return self._call_set_mode_service("TELEOP", timeout_sec=10.0)

    def disarm(self) -> AdapterResult:
        return self._call_set_mode_service("SAFE_HOLD", timeout_sec=10.0)

    def home(self) -> AdapterResult:
        command_result = self._call_trigger_service(
            "/marvin_motion/go_home",
            timeout_sec=self._home_command_timeout_sec(),
        )
        if command_result.is_failure():
            return self._home_failure_result(command_result.summary)

        status_result = self._wait_for_motion_idle(timeout_sec=self._home_settle_timeout_sec())
        if status_result.is_failure():
            return self._home_failure_result(status_result.summary)

        verification_result = self._wait_for_home_verification(
            timeout_sec=self._home_verification_timeout_sec(),
        )
        if verification_result.is_failure():
            return self._home_failure_result(verification_result.summary)

        if command_result.summary and verification_result.summary:
            return AdapterResult.ok(
                f"{command_result.summary} {verification_result.summary}".strip()
            )
        if command_result.summary:
            return AdapterResult.ok(command_result.summary)
        return verification_result

    def before_session(self) -> AdapterResult:
        return self._call_set_bool_service(
            "/marvin_motion/set_enabled",
            True,
            timeout_sec=10.0,
        )

    def after_session(self) -> AdapterResult:
        return self._call_set_bool_service(
            "/marvin_motion/set_enabled",
            False,
            timeout_sec=10.0,
        )

    def _on_teleop_state(self, msg: String) -> None:
        self._latest_teleop_state = str(msg.data).strip()

    def _on_joint_state(self, msg: JointState) -> None:
        self._joint_positions = {
            str(name): float(position)
            for name, position in zip(msg.name, msg.position)
        }
        self._last_joint_state_monotonic = time.monotonic()

    def _launch_package(self) -> str:
        return str(self.device.config.get("launch_package", self.DEFAULT_LAUNCH_PACKAGE))

    def _launch_file(self) -> str:
        return str(self.device.config.get("launch_file", self.DEFAULT_LAUNCH_FILE))

    def _launch_arguments(self) -> dict[str, object]:
        arguments = {
            "gui": False,
            "use_keyboard_gate": False,
            "start_tracker_publisher": False,
            "start_cameras": False,
            "show_camera_views": False,
            "use_mock_hardware": False,
            "collision_guard_enabled": True,
            "collision_guard_near_distance_m": 0.04,
            "collision_guard_hard_collision_distance_m": 0.01,
            "collision_guard_escape_min_distance_improvement_m": 0.0002,
            "collision_guard_check_rate_hz": 30.0,
            "collision_guard_interpolation_steps": 6,
            "collision_guard_binary_search_steps": 5,
            "enable_moveit_go_home": True,
            "motion_allow_legacy_home_fallback": False,
            "target_pose_file_dump_requires_recording_state": True,
        }
        raw_arguments = self.device.config.get("launch_arguments", {})
        if isinstance(raw_arguments, dict):
            arguments.update(raw_arguments)
        return arguments

    def _build_launch_command(self) -> list[str]:
        command = [
            "ros2",
            "launch",
            self._launch_package(),
            self._launch_file(),
        ]
        for key, value in self._launch_arguments().items():
            command.append(f"{key}:={self._launch_value_to_string(value)}")
        return command

    def _launch_value_to_string(self, value: object) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        return str(value)

    def _launch_file_path(self) -> str:
        launch_file = self._launch_file()
        launch_path = Path(launch_file).expanduser()
        if launch_path.is_absolute() or "/" in launch_file:
            return str(launch_path)

        package_share = Path(get_package_share_directory(self._launch_package()))
        candidates = [
            package_share / "bringup" / "launch" / launch_file,
            package_share / "launch" / launch_file,
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        return str(candidates[0])

    def _matches_marvin_process(self, process_name: str, cmd: list[str]) -> bool:
        executable_markers = {
            "ros2_control_node",
            "robot_state_publisher",
            "move_group_wrapper.py",
            "motion_server",
            "manus_gripper_node",
            "spawner",
        }
        if process_name in executable_markers:
            return True
        joined = " ".join(cmd)
        if any(marker in joined for marker in executable_markers):
            return True
        return "marvin_system" in joined or "tracker_teleop_controller" in joined

    def _ready_timeout_sec(self, timeout_sec: float) -> float:
        configured = self.device.config.get("ready_timeout_sec", 30.0)
        if timeout_sec > 0.0:
            return timeout_sec
        try:
            return float(configured)
        except (TypeError, ValueError):
            return 30.0

    def _startup_no_ready_restart_timeout_sec(self) -> float:
        configured = self.device.config.get("startup_no_ready_restart_timeout_sec", 20.0)
        try:
            return max(0.0, float(configured))
        except (TypeError, ValueError):
            return 20.0

    def _ready_relaunch_attempts(self) -> int:
        configured = self.device.config.get("ready_relaunch_attempts", 1)
        try:
            return max(0, int(configured))
        except (TypeError, ValueError):
            return 1

    def _ready_relaunch_backoff_sec(self) -> float:
        configured = self.device.config.get("ready_relaunch_backoff_sec", 1.0)
        try:
            return max(0.0, float(configured))
        except (TypeError, ValueError):
            return 1.0

    def _process_environment(self) -> dict[str, str]:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        return env

    def _run_ros2_command(self, command: list[str], timeout_sec: float) -> subprocess.CompletedProcess | None:
        try:
            return subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout_sec,
                env=self._process_environment(),
            )
        except (OSError, subprocess.TimeoutExpired):
            return None

    def _graph_cleanup_timeout_sec(self) -> float:
        configured = self.device.config.get("graph_cleanup_timeout_sec", 12.0)
        try:
            return max(1.0, float(configured))
        except (TypeError, ValueError):
            return 12.0

    def _prelaunch_graph_cleanup_timeout_sec(self) -> float:
        configured = self.device.config.get(
            "prelaunch_graph_cleanup_timeout_sec",
            self._graph_cleanup_timeout_sec(),
        )
        try:
            return max(1.0, float(configured))
        except (TypeError, ValueError):
            return self._graph_cleanup_timeout_sec()

    def _graph_cleanup_stability_window_sec(self) -> float:
        configured = self.device.config.get("graph_cleanup_stability_window_sec", 0.5)
        try:
            return max(0.0, float(configured))
        except (TypeError, ValueError):
            return 0.5

    def _external_cleanup_backoff_sec(self) -> float:
        configured = self.device.config.get("external_cleanup_backoff_sec", 0.5)
        try:
            return max(0.0, float(configured))
        except (TypeError, ValueError):
            return 0.5

    def _current_service_names(self) -> set[str]:
        if self._ros_context_is_usable():
            try:
                return {
                    str(name).strip()
                    for name, _types in self.node.get_service_names_and_types()
                    if str(name).strip()
                }
            except Exception:
                pass

        completed = self._run_ros2_command(["ros2", "service", "list"], timeout_sec=4.0)
        if completed is None or completed.returncode != 0:
            return set()
        return {line.strip() for line in completed.stdout.splitlines() if line.strip()}

    def _current_node_basenames(self) -> set[str]:
        node_names: set[str] = set()
        if self._ros_context_is_usable():
            try:
                for name, namespace in self.node.get_node_names_and_namespaces():
                    raw_name = str(name).strip()
                    if not raw_name:
                        continue
                    raw_namespace = str(namespace).strip()
                    full_name = raw_name
                    if raw_namespace and raw_namespace != "/":
                        full_name = f"{raw_namespace.rstrip('/')}/{raw_name}"
                    node_names.add(full_name.rsplit("/", 1)[-1].lstrip("/"))
            except Exception:
                node_names.clear()

        if node_names:
            return node_names

        completed = self._run_ros2_command(["ros2", "node", "list"], timeout_sec=4.0)
        if completed is None or completed.returncode != 0:
            return set()
        return {
            line.strip().rsplit("/", 1)[-1].lstrip("/")
            for line in completed.stdout.splitlines()
            if line.strip()
        }

    def _stale_graph_resources(self) -> tuple[list[str], list[str]]:
        current_services = self._current_service_names()
        current_nodes = self._current_node_basenames()
        stale_services = [
            service_name
            for service_name in self.GRAPH_CLEANUP_SERVICES
            if service_name in current_services
        ]
        stale_nodes = [
            node_basename
            for node_basename in self.GRAPH_CLEANUP_NODE_BASENAMES
            if node_basename in current_nodes
        ]
        return stale_services, stale_nodes

    def _wait_for_graph_cleanup(
        self,
        *,
        timeout_sec: float,
        context_label: str,
    ) -> AdapterResult:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        clear_since = 0.0
        last_stale_services: list[str] = []
        last_stale_nodes: list[str] = []
        stability_window_sec = self._graph_cleanup_stability_window_sec()

        while time.monotonic() < deadline:
            stale_services, stale_nodes = self._stale_graph_resources()
            last_stale_services = stale_services
            last_stale_nodes = stale_nodes

            if not stale_services and not stale_nodes:
                if clear_since <= 0.0:
                    clear_since = time.monotonic()
                if time.monotonic() - clear_since >= stability_window_sec:
                    return AdapterResult.ok(
                        f"{self.device.device_id}: Marvin ROS graph cleanup confirmed after {context_label}."
                    )
            else:
                clear_since = 0.0

            time.sleep(0.2)

        details = []
        if last_stale_services:
            details.append("services=" + ", ".join(last_stale_services))
        if last_stale_nodes:
            details.append("nodes=" + ", ".join(last_stale_nodes))
        detail_suffix = " | ".join(details) if details else "resources=unknown"
        return AdapterResult.failed(
            f"{self.device.device_id}: Marvin ROS graph still contains stale resources after "
            f"{context_label}: {detail_suffix}"
        )

    def _external_cleanup_match_reason(self, cmd: str) -> str:
        lowered = cmd.lower()
        for marker in self.EXTERNAL_PROCESS_MATCH_MARKERS:
            if marker in cmd:
                return f"marker={marker}"

        if "ros2_control_node" in lowered and (
            "marvin_tracker_teleop_controllers.yaml" in lowered
            or "dual_arm_controller_manager.yaml" in lowered
            or "tracker_teleop_kine_params" in lowered
        ):
            return "ros2_control_node+marvin controller config"

        if "controller_manager/spawner" in lowered and (
            "tracker_teleop_controller" in lowered
            or "dual_arm_trajectory_controller" in lowered
            or "gripper_l_controller" in lowered
            or "gripper_r_controller" in lowered
        ):
            return "spawner+marvin controller"

        if "robot_state_publisher" in lowered and (
            "marvin_system" in lowered or "marvin_dual" in lowered
        ):
            return "robot_state_publisher+marvin"

        return ""

    def _external_cleanup_candidate_process_groups(self) -> tuple[list[tuple[int, str, str]], list[str]]:
        completed = self._run_ros2_command(["ps", "-eo", "pid=,pgid=,args="], timeout_sec=4.0)
        if completed is None or completed.returncode != 0:
            return [], []

        current_pid = os.getpid()
        current_pgid = os.getpgrp()
        candidate_by_pgid: dict[int, tuple[str, str]] = {}
        suspicious_samples: list[str] = []
        for raw_line in completed.stdout.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split(None, 2)
            if len(parts) < 3:
                continue
            try:
                pid = int(parts[0])
                pgid = int(parts[1])
            except ValueError:
                continue
            cmd = parts[2].strip()
            if pid <= 0 or pgid <= 0:
                continue
            if pid == current_pid or pgid == current_pgid:
                continue
            reason = self._external_cleanup_match_reason(cmd)
            if reason:
                candidate_by_pgid.setdefault(pgid, (cmd, reason))
                continue
            lowered = cmd.lower()
            if (
                "ros2_control_node" in lowered
                or "motion_server" in lowered
                or "move_group" in lowered
                or "robot_state_publisher" in lowered
                or "spawner" in lowered
                or "marvin" in lowered
            ):
                suspicious_samples.append(cmd)

        candidates = [
            (pgid, cmd, reason)
            for pgid, (cmd, reason) in sorted(candidate_by_pgid.items())
        ]
        return candidates, suspicious_samples[:8]

    def _signal_external_process_group(self, pgid: int, sig: signal.Signals, timeout_sec: float) -> bool:
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                return True
            time.sleep(0.2)

        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return True
        return False

    def _scavenge_external_process_groups(self, *, context_label: str) -> AdapterResult:
        candidates, suspicious_samples = self._external_cleanup_candidate_process_groups()
        if not candidates:
            if self.node is not None and suspicious_samples:
                for sample in suspicious_samples:
                    self.node.get_logger().warn(
                        f"{self.device.device_id}: external cleanup found suspicious process "
                        f"during {context_label} but no Marvin match rule claimed it. cmd={sample}"
                    )
            return AdapterResult.failed(
                f"{self.device.device_id}: no external Marvin process groups matched for {context_label}."
            )

        if self.node is not None:
            for pgid, cmd, reason in candidates:
                self.node.get_logger().warn(
                    f"{self.device.device_id}: scavenging external Marvin process group "
                    f"pgid={pgid} during {context_label}. reason={reason} cmd={cmd}"
                )

        failed_pgids: list[str] = []
        for pgid, _cmd, _reason in candidates:
            stopped = False
            for sig, wait_s in (
                (signal.SIGINT, 8.0),
                (signal.SIGTERM, 4.0),
                (signal.SIGKILL, 2.0),
            ):
                if self._signal_external_process_group(pgid, sig, timeout_sec=wait_s):
                    stopped = True
                    break
            if not stopped:
                failed_pgids.append(str(pgid))

        if failed_pgids:
            return AdapterResult.failed(
                f"{self.device.device_id}: external Marvin cleanup failed for pgid="
                + ", ".join(failed_pgids)
            )

        backoff_sec = self._external_cleanup_backoff_sec()
        if backoff_sec > 0.0:
            time.sleep(backoff_sec)
        return AdapterResult.ok(
            f"{self.device.device_id}: external Marvin process cleanup completed for {context_label}."
        )

    def _ensure_graph_cleanup(
        self,
        *,
        timeout_sec: float,
        context_label: str,
        allow_external_cleanup: bool,
    ) -> AdapterResult:
        cleanup_result = self._wait_for_graph_cleanup(
            timeout_sec=timeout_sec,
            context_label=context_label,
        )
        if not cleanup_result.is_failure():
            return cleanup_result
        if not allow_external_cleanup:
            return cleanup_result

        scavenger_result = self._scavenge_external_process_groups(context_label=context_label)
        if scavenger_result.is_failure():
            return cleanup_result

        second_cleanup_result = self._wait_for_graph_cleanup(
            timeout_sec=timeout_sec,
            context_label=f"{context_label} after external cleanup",
        )
        if not second_cleanup_result.is_failure():
            return AdapterResult.ok(
                f"{self.device.device_id}: Marvin ROS graph cleanup confirmed after "
                f"{context_label} with external scavenging."
            )
        return second_cleanup_result

    def _list_controllers(self) -> dict[str, str] | None:
        if self._list_controllers_client is not None:
            response, error_message = self._call_service_response(
                client=self._list_controllers_client,
                request=ListControllers.Request(),
                timeout_sec=1.0,
            )
            if response is None:
                return None

            controllers: dict[str, str] = {}
            for controller in getattr(response, "controller", []):
                name = str(getattr(controller, "name", "")).strip()
                state = str(getattr(controller, "state", "")).strip()
                if name:
                    controllers[name] = state
            return controllers

        completed = self._run_ros2_command(
            ["ros2", "control", "list_controllers"],
            timeout_sec=3.0,
        )
        if completed is None or completed.returncode != 0:
            return None

        controllers: dict[str, str] = {}
        for raw_line in completed.stdout.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            controllers[parts[0]] = parts[-1]
        return controllers

    def _controllers_ready(self, controllers: dict[str, str]) -> bool:
        for controller_name, required_states in self._required_controller_states().items():
            if controllers.get(controller_name) not in required_states:
                return False
        return True

    def _missing_required_controllers(self, controllers: dict[str, str]) -> list[str]:
        missing = []
        for controller_name, required_states in self._required_controller_states().items():
            actual_state = controllers.get(controller_name)
            if actual_state not in required_states:
                actual_label = "missing" if actual_state is None else actual_state
                expected_label = "/".join(sorted(required_states))
                missing.append(
                    f"{controller_name}={actual_label} (expected {expected_label})"
                )
        return missing

    def _motion_status_debug_summary(self, response, error_message: str) -> str:
        if response is None:
            return f"unavailable ({error_message or 'unknown error'})"

        message = str(getattr(response, "message", "")).strip() or "n/a"
        return (
            f"success={bool(getattr(response, 'success', False))} "
            f"mode={str(getattr(response, 'mode', '')).strip() or 'unknown'} "
            f"teleop={str(getattr(response, 'teleop_state', '')).strip() or 'unknown'} "
            f"busy={bool(getattr(response, 'motion_busy', False))} "
            f"interlock={bool(getattr(response, 'controller_interlock_ok', True))} "
            f"primary={str(getattr(response, 'primary_controller_state', '')).strip() or 'unknown'} "
            f"trajectory={str(getattr(response, 'trajectory_controller_state', '')).strip() or 'unknown'} "
            f"message={message}"
        )

    def _ready_wait_debug_summary(
        self,
        controllers: dict[str, str] | None,
        missing_controllers: list[str],
        missing_services: list[str] | None,
        motion_status: str,
    ) -> str:
        lines = []
        if controllers is None:
            lines.append("controller_snapshot=unavailable")
        else:
            required_snapshot = ", ".join(
                f"{name}={controllers.get(name, 'missing')}"
                for name in sorted(self._required_controller_states().keys())
            )
            lines.append(f"required_controllers={required_snapshot}")

        if missing_controllers:
            lines.append(
                "missing_required_controllers=" + ", ".join(missing_controllers)
            )
        else:
            lines.append("missing_required_controllers=none")

        if missing_services is None:
            lines.append("missing_required_services=not checked")
        elif missing_services:
            lines.append("missing_required_services=" + ", ".join(missing_services))
        else:
            lines.append("missing_required_services=none")

        lines.append(f"last_motion_status={motion_status}")
        return "\n".join(lines)

    def _required_controller_states(self) -> dict[str, set[str]]:
        required = {
            controller_name: {required_state}
            for controller_name, required_state in self.REQUIRED_CONTROLLER_STATES.items()
        }
        launch_arguments = self._launch_arguments()
        if self._launch_arg_enabled(launch_arguments.get("use_gripper_L", False)):
            required["gripper_L_controller"] = {"active"}
        if self._launch_arg_enabled(launch_arguments.get("use_gripper_R", False)):
            required["gripper_R_controller"] = {"active"}
        if self._launch_arg_enabled(launch_arguments.get("enable_moveit_go_home", False)):
            # GoHome only needs the trajectory controller to be loaded; it is expected
            # to stay inactive until motion_server switches into MOTION mode.
            required[self.TRAJECTORY_CONTROLLER_NAME] = {"inactive", "active"}
        return required

    def _launch_arg_enabled(self, value: object) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() == "true"
        return bool(value)

    def _launch_is_running(self) -> bool:
        if self._managed_launch is not None:
            return self._managed_launch.is_running()
        return self._process is not None and self._process.poll() is None

    def _ros_context_is_usable(self) -> bool:
        if self.node is None:
            return False
        try:
            return self.node.context.ok() and rclpy.ok(context=self.node.context)
        except Exception:
            return False

    def _service_available(self, service_name: str, expected_type: str) -> bool:
        del expected_type
        if not self._ros_context_is_usable():
            return False
        client = self._client_for_service(service_name)
        if client is not None:
            try:
                return client.service_is_ready()
            except Exception:
                return False
        return False

    def _call_set_bool_service(
        self, service_name: str, value: bool, timeout_sec: float
    ) -> AdapterResult:
        if not self._launch_is_running():
            return AdapterResult.failed(
                f"{self.device.device_id}: Marvin launch is not running for {service_name}."
            )

        client = self._client_for_service(service_name)
        if client is None:
            return AdapterResult.failed(
                f"{self.device.device_id}: no native client for {service_name}."
            )
        request = SetBool.Request()
        request.data = value
        return self._call_service(
            client=client,
            request=request,
            timeout_sec=timeout_sec,
            success_summary=f"{self.device.device_id}: {service_name} accepted data={value}.",
            failure_prefix=f"{self.device.device_id}: {service_name} failed",
        )

    def _call_trigger_service(self, service_name: str, timeout_sec: float) -> AdapterResult:
        if not self._launch_is_running():
            return AdapterResult.failed(
                f"{self.device.device_id}: Marvin launch is not running for {service_name}."
            )

        client = self._client_for_service(service_name)
        if client is None:
            return AdapterResult.failed(
                f"{self.device.device_id}: no native client for {service_name}."
            )
        request = Trigger.Request()
        return self._call_service(
            client=client,
            request=request,
            timeout_sec=timeout_sec,
            success_summary=f"{self.device.device_id}: {service_name} accepted.",
            failure_prefix=f"{self.device.device_id}: {service_name} failed",
        )

    def _call_service(
        self,
        *,
        client,
        request,
        timeout_sec: float,
        success_summary: str,
        failure_prefix: str,
    ) -> AdapterResult:
        if not self._ros_context_is_usable():
            return AdapterResult.failed(f"{failure_prefix}: ROS context unavailable.")
        try:
            service_ready = client.wait_for_service(timeout_sec=timeout_sec)
        except Exception as exc:
            return AdapterResult.failed(f"{failure_prefix}: {exc}")
        if not service_ready:
            return AdapterResult.failed(f"{failure_prefix}: service unavailable.")

        try:
            future = client.call_async(request)
        except Exception as exc:
            return AdapterResult.failed(f"{failure_prefix}: {exc}")
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.01)

        if not future.done():
            return AdapterResult.failed(f"{failure_prefix}: command timeout or spawn failure.")

        if future.exception() is not None:
            return AdapterResult.failed(f"{failure_prefix}: {future.exception()}")

        response = future.result()
        success = bool(getattr(response, "success", False))
        message = str(getattr(response, "message", "")).strip()
        if not success:
            return AdapterResult.failed(f"{failure_prefix}: {message or 'request rejected'}")
        if message:
            return AdapterResult.ok(f"{success_summary} message={message}")
        return AdapterResult.ok(success_summary)

    def _call_service_response(self, *, client, request, timeout_sec: float):
        if client is None:
            return None, "native client unavailable"
        if not self._ros_context_is_usable():
            return None, "ROS context unavailable"
        try:
            service_ready = client.wait_for_service(timeout_sec=timeout_sec)
        except Exception as exc:
            return None, str(exc)
        if not service_ready:
            return None, "service unavailable"

        try:
            future = client.call_async(request)
        except Exception as exc:
            return None, str(exc)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.01)

        if not future.done():
            return None, "request timeout or spawn failure"
        if future.exception() is not None:
            return None, str(future.exception())
        return future.result(), ""

    def _call_set_mode_service(self, mode: str, timeout_sec: float) -> AdapterResult:
        if not self._launch_is_running():
            return AdapterResult.failed(
                f"{self.device.device_id}: Marvin launch is not running for /marvin_motion/set_mode."
            )

        response, error_message = self._call_service_response(
            client=self._motion_set_mode_client,
            request=SetMotionMode.Request(mode=mode),
            timeout_sec=timeout_sec,
        )
        if response is None:
            return AdapterResult.failed(
                f"{self.device.device_id}: /marvin_motion/set_mode failed: {error_message}"
            )
        if not bool(getattr(response, "success", False)):
            return AdapterResult.failed(
                f"{self.device.device_id}: /marvin_motion/set_mode rejected: "
                f"{getattr(response, 'message', '')}"
            )

        active_mode = str(getattr(response, "active_mode", "")).strip() or mode
        message = str(getattr(response, "message", "")).strip()
        summary = f"{self.device.device_id}: /marvin_motion/set_mode accepted mode={active_mode}."
        if message:
            summary = f"{summary} message={message}"
        return AdapterResult.ok(summary)

    def _client_for_service(self, service_name: str):
        if service_name == "/marvin_motion/go_home":
            return self._motion_go_home_client
        if service_name == "/marvin_motion/set_mode":
            return self._motion_set_mode_client
        if service_name == "/marvin_motion/get_mode":
            return self._motion_get_mode_client
        if service_name == "/marvin_motion/get_status":
            return self._motion_get_status_client
        if service_name == "/marvin_motion/set_enabled":
            return self._motion_set_enabled_client
        return None

    def _controller_config_path(self) -> Path:
        raw_value = self.device.config.get("controller_config")
        if raw_value:
            return Path(str(raw_value)).expanduser()

        package_share = Path(get_package_share_directory("marvin_system"))
        return package_share / self.DEFAULT_CONTROLLER_CONFIG_RELATIVE_PATH

    def _home_pose_config_path(self) -> Path:
        raw_value = self.device.config.get("home_pose_config")
        if raw_value:
            return Path(str(raw_value)).expanduser()

        package_share = Path(get_package_share_directory("marvin_system"))
        return package_share / self.DEFAULT_HOME_POSES_RELATIVE_PATH

    def _load_home_targets(self) -> tuple[dict[str, float], float]:
        motion_config_path = self._home_pose_config_path()
        try:
            with motion_config_path.open("r", encoding="utf-8") as handle:
                motion_data = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError):
            motion_data = {}

        motion_params = motion_data.get("marvin_motion_server", {}).get("ros__parameters", {})
        named_home = motion_params.get("named_poses", {}).get("home", {})
        home_left = named_home.get("left", [])
        home_right = named_home.get("right", [])
        if isinstance(home_left, list) and isinstance(home_right, list):
            joint_names = [
                "Joint1_L",
                "Joint2_L",
                "Joint3_L",
                "Joint4_L",
                "Joint5_L",
                "Joint6_L",
                "Joint7_L",
                "Joint1_R",
                "Joint2_R",
                "Joint3_R",
                "Joint4_R",
                "Joint5_R",
                "Joint6_R",
                "Joint7_R",
            ]
            home_values = list(home_left) + list(home_right)
            if len(home_values) == len(joint_names):
                return (
                    {
                        joint_name: float(value)
                        for joint_name, value in zip(joint_names, home_values)
                    },
                    math.radians(0.5),
                )

        config_path = self._controller_config_path()
        try:
            with config_path.open("r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except OSError:
            return {}, math.radians(0.5)
        except yaml.YAMLError:
            return {}, math.radians(0.5)

        params = data.get("tracker_teleop_controller", {}).get("ros__parameters", {})
        joint_names = params.get("joints", [])
        if not isinstance(joint_names, list):
            return {}, math.radians(0.5)

        home_left = params.get("home_joint_positions", {}).get("left", [])
        home_right = params.get("home_joint_positions", {}).get("right", [])
        if not isinstance(home_left, list) or not isinstance(home_right, list):
            return {}, math.radians(0.5)

        left_names = [str(name) for name in joint_names[: len(home_left)]]
        right_names = [str(name) for name in joint_names[len(home_left): len(home_left) + len(home_right)]]

        home_targets = {
            name: float(value)
            for name, value in zip(left_names + right_names, home_left + home_right)
        }
        tolerance_deg = params.get("home_tolerance_deg", 0.5)
        try:
            tolerance_rad = math.radians(float(tolerance_deg))
        except (TypeError, ValueError):
            tolerance_rad = math.radians(0.5)
        return home_targets, max(tolerance_rad, 1.0e-4)

    def _home_timeout_sec(self) -> float:
        configured = self.device.config.get("home_timeout_sec", 20.0)
        try:
            return max(1.0, float(configured))
        except (TypeError, ValueError):
            return 20.0

    def _home_command_timeout_sec(self) -> float:
        configured = self.device.config.get("home_command_timeout_sec", self._home_timeout_sec())
        try:
            return max(5.0, float(configured))
        except (TypeError, ValueError):
            return self._home_timeout_sec()

    def _managed_launch_start_timeout_sec(self) -> float:
        configured = self.device.config.get("managed_launch_start_timeout_sec", 8.0)
        try:
            return max(2.0, float(configured))
        except (TypeError, ValueError):
            return 8.0

    def _home_settle_timeout_sec(self) -> float:
        configured = self.device.config.get("home_settle_timeout_sec", 5.0)
        try:
            return max(1.0, float(configured))
        except (TypeError, ValueError):
            return 5.0

    def _home_verification_timeout_sec(self) -> float:
        configured = self.device.config.get(
            "home_verification_timeout_sec",
            self._home_timeout_sec(),
        )
        try:
            return max(1.0, float(configured))
        except (TypeError, ValueError):
            return self._home_timeout_sec()

    def _motion_status_response(self, timeout_sec: float):
        return self._call_service_response(
            client=self._motion_get_status_client,
            request=GetMotionStatus.Request(),
            timeout_sec=timeout_sec,
        )

    def _wait_for_motion_idle(self, timeout_sec: float) -> AdapterResult:
        deadline = time.monotonic() + timeout_sec
        last_mode = "UNKNOWN"
        last_message = ""
        while time.monotonic() < deadline:
            if not self._launch_is_running():
                return AdapterResult.failed(
                    f"{self.device.device_id}: Marvin launch exited while waiting for motion idle."
                )

            status_response, status_error = self._motion_status_response(timeout_sec=1.0)
            if status_response is None:
                time.sleep(0.05)
                last_message = status_error
                continue

            last_mode = str(getattr(status_response, "mode", "")).strip() or "UNKNOWN"
            last_message = str(getattr(status_response, "message", "")).strip()
            if not bool(getattr(status_response, "controller_interlock_ok", True)):
                return AdapterResult.failed(
                    f"{self.device.device_id}: marvin_motion controller interlock violation."
                )
            if last_mode == "FAULT":
                return AdapterResult.failed(
                    f"{self.device.device_id}: marvin_motion entered FAULT during go_home. "
                    f"{last_message}".strip()
                )
            if bool(getattr(status_response, "motion_busy", False)):
                time.sleep(0.05)
                continue

            return AdapterResult.ok(
                f"{self.device.device_id}: marvin_motion idle after home "
                f"(mode={last_mode}, teleop={getattr(status_response, 'teleop_state', 'UNKNOWN')})."
            )

        return AdapterResult.failed(
            f"{self.device.device_id}: marvin_motion did not become idle within "
            f"{timeout_sec:.1f}s. last_mode={last_mode} message={last_message or 'n/a'}"
        )

    def _wait_for_home_verification(self, timeout_sec: float) -> AdapterResult:
        if not self._home_joint_targets:
            return AdapterResult.ok(
                f"{self.device.device_id}: go_home completed; home verification unavailable."
            )

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._managed_launch is not None and not self._managed_launch.is_running():
                return AdapterResult.failed(
                    f"{self.device.device_id}: Marvin launch exited while verifying home pose."
                )
            if self._managed_launch is None and self._process is not None and self._process.poll() is not None:
                return AdapterResult.failed(
                    f"{self.device.device_id}: Marvin launch exited while verifying home pose."
                )
            if self._is_home_reached():
                return AdapterResult.ok(f"{self.device.device_id}: home pose reached.")
            time.sleep(0.05)

        if self._last_joint_state_monotonic <= 0.0:
            return AdapterResult.ok(
                f"{self.device.device_id}: go_home completed; joint verification unavailable."
            )
        if time.monotonic() - self._last_joint_state_monotonic > 1.0:
            return AdapterResult.ok(
                f"{self.device.device_id}: go_home completed; joint verification is stale."
            )

        max_error_joint, max_error_rad = self._largest_home_error()
        return AdapterResult.failed(
            f"{self.device.device_id}: go_home completed but joint verification "
            f"does not match the expected home pose within {timeout_sec:.1f}s"
            + (
                f" (largest error: {max_error_joint}={max_error_rad:.4f} rad)."
                if max_error_joint
                else "."
            )
        )

    def _home_failure_result(self, summary: str) -> AdapterResult:
        soft_failure_markers = (
            "MoveIt planning failed",
            "MoveIt execution failed",
            "Recovery ",
            "go_home completed but joint verification",
            "marvin_motion did not become idle within",
        )
        hard_failure_markers = (
            "service unavailable",
            "launch exited",
            "controller interlock violation",
            "entered FAULT",
            "launch is not running",
            "no native client",
        )

        if any(marker in summary for marker in hard_failure_markers):
            return AdapterResult.failed(summary)
        if any(marker in summary for marker in soft_failure_markers):
            return AdapterResult.degraded(summary)
        return AdapterResult.failed(summary)

    def _is_home_reached(self) -> bool:
        if self._last_joint_state_monotonic <= 0.0:
            return False
        if time.monotonic() - self._last_joint_state_monotonic > 1.0:
            return False

        for joint_name, target_position in self._home_joint_targets.items():
            current_position = self._joint_positions.get(joint_name)
            if current_position is None:
                return False
            if abs(current_position - target_position) > self._home_tolerance_rad:
                return False
        return True

    def _largest_home_error(self) -> tuple[str, float]:
        largest_joint = ""
        largest_error = 0.0
        for joint_name, target_position in self._home_joint_targets.items():
            current_position = self._joint_positions.get(joint_name)
            if current_position is None:
                continue
            error = abs(current_position - target_position)
            if error > largest_error:
                largest_error = error
                largest_joint = joint_name
        return largest_joint, largest_error

    def _signal_process_group(
        self,
        process: subprocess.Popen,
        sig: signal.Signals,
        timeout_sec: float,
    ) -> bool:
        if process.poll() is not None:
            return True

        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return True

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return True
            time.sleep(0.2)
        return process.poll() is not None

    def _wait_for_process_group_exit(self, pgid: int, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                return True
            time.sleep(0.2)

        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return True
        return False

    def _signal_process(
        self,
        process: subprocess.Popen,
        sig: signal.Signals,
        timeout_sec: float,
    ) -> bool:
        if process.poll() is not None:
            return True

        try:
            os.kill(process.pid, sig)
        except ProcessLookupError:
            return True

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return True
            time.sleep(0.2)
        return process.poll() is not None

    def _cleanup_process_state(self) -> None:
        if self._managed_launch is not None:
            self._managed_launch.close()
            self._managed_launch = None
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None
        self._process = None

    def _start_launch_process(self) -> AdapterResult:
        self._cleanup_process_state()
        self._latest_teleop_state = ""
        self._joint_positions.clear()
        self._last_joint_state_monotonic = 0.0
        self._launch_command = self._build_launch_command()

        use_managed_launch = bool(self.device.config.get("use_managed_launch", False))
        if use_managed_launch and self.launch_manager is not None:
            managed_launch = ManagedLaunchSession(
                launch_manager=self.launch_manager,
                label=f"{self.device.device_id}_marvin",
                launch_file_path=self._launch_file_path(),
                launch_arguments=self._launch_arguments(),
                process_matcher=self._matches_marvin_process,
                logger=None if self.node is None else self.node.get_logger(),
            )
            self._managed_launch = managed_launch
            self._log_path = Path(managed_launch.log_path)
            try:
                managed_launch.start()
            except Exception as exc:
                self._cleanup_process_state()
                return AdapterResult.failed(
                    f"{self.device.device_id}: failed to start managed Marvin launch: {exc}"
                )

            managed_launch.wait_started(timeout_sec=self._managed_launch_start_timeout_sec())
            if not managed_launch.is_running():
                return AdapterResult.failed(
                    f"{self.device.device_id}: Marvin launch exited early.\n{self._diagnostic_summary()}"
                )

            return AdapterResult.ok(
                f"{self.device.device_id}: Marvin launch started.",
                metadata={
                    **managed_launch.metadata(),
                    "log_path": str(self._log_path),
                    "command": self._launch_command,
                },
            )

        prelaunch_cleanup = self._ensure_graph_cleanup(
            timeout_sec=self._prelaunch_graph_cleanup_timeout_sec(),
            context_label="prelaunch",
            allow_external_cleanup=True,
        )
        if prelaunch_cleanup.is_failure():
            self._cleanup_process_state()
            return prelaunch_cleanup

        log_dir = Path(tempfile.mkdtemp(prefix="marvin_adapter_"))
        self._log_path = log_dir / "launch.log"
        self._log_handle = self._log_path.open("w", encoding="utf-8")

        try:
            self._process = subprocess.Popen(
                self._launch_command,
                stdout=self._log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=self._process_environment(),
            )
        except OSError as exc:
            self._cleanup_process_state()
            return AdapterResult.failed(
                f"{self.device.device_id}: failed to start Marvin launch: {exc}"
            )

        time.sleep(0.5)
        if self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: Marvin launch exited early.\n{self._diagnostic_summary()}"
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: Marvin launch started.",
            metadata={
                "pid": self._process.pid,
                "log_path": str(self._log_path),
                "command": self._launch_command,
            },
        )

    def _restart_launch_for_ready_failure(self, reason: str) -> AdapterResult:
        self._ready_relaunch_count += 1
        if self.node is not None:
            self.node.get_logger().warn(
                f"{self.device.device_id}: Marvin not READY on first launch; "
                f"performing auto-restart {self._ready_relaunch_count}. reason={reason}"
            )

        shutdown_result = self.shutdown()
        if shutdown_result.is_failure():
            return shutdown_result

        backoff_sec = self._ready_relaunch_backoff_sec()
        if backoff_sec > 0.0:
            time.sleep(backoff_sec)

        return self._start_launch_process()

    def _diagnostic_summary(self) -> str:
        if self._managed_launch is not None:
            status = (
                "running"
                if self._managed_launch.is_running()
                else f"exited({self._managed_launch.launch_exit_code()})"
            )
        elif self._process is None:
            status = "not started"
        else:
            status = "running" if self._process.poll() is None else f"exited({self._process.returncode})"

        tail = self._read_log_tail(max_lines=25)
        command = shlex.join(self._launch_command) if self._launch_command else "<none>"
        log_path = "<none>" if self._log_path is None else str(self._log_path)
        summary = [
            f"process_status={status}",
            f"command={command}",
            f"log_path={log_path}",
        ]
        if self._managed_launch is not None:
            summary.append(f"child_pids={self._managed_launch.metadata().get('child_pids', [])}")
        if tail:
            summary.append("log_tail:")
            summary.append(tail)
        return "\n".join(summary)

    def _read_log_tail(self, max_lines: int) -> str:
        if self._log_path is None or not self._log_path.exists():
            return ""
        with self._log_path.open("r", encoding="utf-8", errors="replace") as handle:
            lines = handle.readlines()
        return "".join(lines[-max_lines:]).rstrip()

    def _early_restart_reason_from_log(self) -> str:
        log_tail = self._read_log_tail(max_lines=80)
        if not log_tail:
            return ""

        if "SDK send failed 10 times consecutively" in log_tail:
            return "sdk send failures exceeded retry budget during startup"
        if "No frame update for" in log_tail:
            return "hardware feedback frames stopped updating during startup"
        if "Failed to activate controller" in log_tail:
            return "controller activation failed during startup"
        if "Unable to activate controller" in log_tail:
            return "controller command interfaces became unavailable during startup"
        return ""
