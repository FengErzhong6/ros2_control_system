from __future__ import annotations

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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
import yaml

from .base import AdapterBase, AdapterResult


class MarvinAdapter(AdapterBase):
    DEFAULT_LAUNCH_PACKAGE = "marvin_system"
    DEFAULT_LAUNCH_FILE = "marvin_tracker_teleop.launch.py"
    DEFAULT_HOME_POSES_RELATIVE_PATH = "motion/config/home_poses.yaml"
    DEFAULT_CONTROLLER_CONFIG_RELATIVE_PATH = "bringup/config/marvin_tracker_teleop_controllers.yaml"
    REQUIRED_CONTROLLERS = {
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

    def __init__(self, device, node=None) -> None:
        super().__init__(device, node=node)
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._log_path: Path | None = None
        self._launch_command: list[str] = []
        self._latest_teleop_state = ""
        self._teleop_state_subscription = None
        self._joint_state_subscription = None
        self._service_callback_group = None
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
        if self._process is not None and self._process.poll() is None:
            return AdapterResult.ok(f"{self.device.device_id}: Marvin bringup already running.")

        self._cleanup_process_state()
        log_dir = Path(tempfile.mkdtemp(prefix="marvin_adapter_"))
        self._log_path = log_dir / "launch.log"
        self._log_handle = self._log_path.open("w", encoding="utf-8")

        self._launch_command = self._build_launch_command()
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
            return AdapterResult.failed(f"{self.device.device_id}: failed to start Marvin launch: {exc}")

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

    def wait_ready(self, timeout_sec: float) -> AdapterResult:
        process = self._process
        if process is None:
            return AdapterResult.failed(f"{self.device.device_id}: Marvin launch was not started.")

        deadline = time.monotonic() + self._ready_timeout_sec(timeout_sec)
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return AdapterResult.failed(
                    f"{self.device.device_id}: Marvin launch exited before READY.\n"
                    f"{self._diagnostic_summary()}"
                )

            controllers = self._list_controllers()
            if controllers is not None and self._controllers_ready(controllers):
                missing_services = [
                    service_name
                    for service_name, service_type in self.REQUIRED_SERVICES.items()
                    if not self._service_available(service_name, service_type)
                ]
                if not missing_services:
                    status_response, status_error = self._motion_status_response(timeout_sec=1.0)
                    if status_response is not None:
                        if (
                            bool(getattr(status_response, "success", False))
                            and not bool(getattr(status_response, "motion_busy", False))
                            and bool(getattr(status_response, "controller_interlock_ok", True))
                        ):
                            return AdapterResult.ok(
                                f"{self.device.device_id}: controller_manager and marvin_motion are READY.",
                                metadata={
                                    "controllers": controllers,
                                    "mode": str(getattr(status_response, "mode", "")).strip(),
                                    "teleop_state": str(
                                        getattr(status_response, "teleop_state", "")
                                    ).strip(),
                                    "log_path": None if self._log_path is None else str(self._log_path),
                                },
                            )
                    elif status_error:
                        time.sleep(0.1)

            time.sleep(0.5)

        return AdapterResult.failed(
            f"{self.device.device_id}: Marvin READY timeout after "
            f"{self._ready_timeout_sec(timeout_sec):.1f}s.\n{self._diagnostic_summary()}"
        )

    def shutdown(self) -> AdapterResult:
        self._call_set_bool_service("/marvin_motion/set_enabled", False, timeout_sec=3.0)
        self._call_set_mode_service("SAFE_HOLD", timeout_sec=3.0)

        process = self._process
        if process is None:
            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: Marvin launch already stopped.")

        if process.poll() is not None:
            exit_code = process.returncode
            self._cleanup_process_state()
            return AdapterResult.ok(
                f"{self.device.device_id}: Marvin launch already exited with code {exit_code}."
            )

        # Ask the ros2 launch parent process to exit first so it can shut down child nodes
        # in launch-managed order. Escalate to the full process group only if it gets stuck.
        if not self._signal_process(process, signal.SIGINT, timeout_sec=12.0):
            self._signal_process(process, signal.SIGTERM, timeout_sec=6.0)
        if process.poll() is None:
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=4.0)
        if process.poll() is None:
            self._signal_process_group(process, signal.SIGKILL, timeout_sec=2.0)

        self._cleanup_process_state()
        return AdapterResult.ok(f"{self.device.device_id}: Marvin launch stopped.")

    def diagnose(self) -> AdapterResult:
        if self._process is None:
            return AdapterResult.failed(f"{self.device.device_id}: Marvin launch not started.")
        if self._process.poll() is not None:
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
            "collision_guard_near_distance_m": 0.10,
            "collision_guard_hard_collision_distance_m": 0.05,
            "collision_guard_escape_min_distance_improvement_m": 0.0002,
            "collision_guard_check_rate_hz": 30.0,
            "collision_guard_interpolation_steps": 6,
            "collision_guard_binary_search_steps": 5,
            "enable_moveit_go_home": True,
            "motion_allow_legacy_home_fallback": False,
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

    def _ready_timeout_sec(self, timeout_sec: float) -> float:
        configured = self.device.config.get("ready_timeout_sec", 30.0)
        if timeout_sec > 0.0:
            return timeout_sec
        try:
            return float(configured)
        except (TypeError, ValueError):
            return 30.0

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

    def _list_controllers(self) -> dict[str, str] | None:
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
        for controller_name, required_state in self._required_controllers().items():
            if controllers.get(controller_name) != required_state:
                return False
        return True

    def _required_controllers(self) -> dict[str, str]:
        required = dict(self.REQUIRED_CONTROLLERS)
        launch_arguments = self._launch_arguments()
        if self._launch_arg_enabled(launch_arguments.get("use_gripper_L", False)):
            required["gripper_L_controller"] = "active"
        if self._launch_arg_enabled(launch_arguments.get("use_gripper_R", False)):
            required["gripper_R_controller"] = "active"
        return required

    def _launch_arg_enabled(self, value: object) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() == "true"
        return bool(value)

    def _service_available(self, service_name: str, expected_type: str) -> bool:
        del expected_type
        client = self._client_for_service(service_name)
        if client is not None:
            return client.service_is_ready()
        return False

    def _call_set_bool_service(
        self, service_name: str, value: bool, timeout_sec: float
    ) -> AdapterResult:
        if self._process is None or self._process.poll() is not None:
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
        if self._process is None or self._process.poll() is not None:
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
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return AdapterResult.failed(f"{failure_prefix}: service unavailable.")

        future = client.call_async(request)
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
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None, "service unavailable"

        future = client.call_async(request)
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
        if self._process is None or self._process.poll() is not None:
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
            if self._process is None or self._process.poll() is not None:
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
            if self._process is not None and self._process.poll() is not None:
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
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None
        self._process = None

    def _diagnostic_summary(self) -> str:
        if self._process is None:
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
