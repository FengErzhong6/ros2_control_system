from __future__ import annotations

from controller_manager_msgs.srv import ListControllers
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool
import yaml

from ..managed_launch import ManagedLaunchSession
from .base import AdapterBase, AdapterResult


class WujihandAdapter(AdapterBase):
    DEFAULT_LAUNCH_PACKAGE = "wujihand_system"
    DEFAULT_LAUNCH_FILE = "wujihand_manus_teleop.launch.py"
    DEFAULT_IDENTITY_FILE_RELATIVE_PATH = "config/wujihand_identities.yaml"
    DEFAULT_TELEOP_CONFIG_RELATIVE_PATH = "config/manus_input.yaml"
    DEFAULT_LEFT_CONTROLLERS_RELATIVE_PATH = "config/wujihand_left_controllers.yaml"
    DEFAULT_RIGHT_CONTROLLERS_RELATIVE_PATH = "config/wujihand_right_controllers.yaml"
    DEFAULT_LEFT_ROS2_CONTROL_RELATIVE_PATH = "ros2_control/wujihand-left.ros2_control.xacro"
    DEFAULT_RIGHT_ROS2_CONTROL_RELATIVE_PATH = "ros2_control/wujihand-right.ros2_control.xacro"
    REQUIRED_CONTROLLER_STATES = {
        "joint_state_broadcaster": {"active"},
        "forward_position_controller": {"active"},
    }

    def __init__(self, device, node=None) -> None:
        super().__init__(device, node=node)
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._managed_launch: ManagedLaunchSession | None = None
        self._log_path: Path | None = None
        self._launch_command: list[str] = []
        self._generated_teleop_config_path: Path | None = None
        self._service_callback_group = None
        self._list_controllers_client = None
        self._set_enabled_client = None
        self._neutral_command_publisher = None

        if self.node is not None:
            self._service_callback_group = ReentrantCallbackGroup()
            self._list_controllers_client = self.node.create_client(
                ListControllers,
                self._controller_manager_service_name(),
                callback_group=self._service_callback_group,
            )
            self._set_enabled_client = self.node.create_client(
                SetBool,
                self._pipeline_set_enabled_service_name(),
                callback_group=self._service_callback_group,
            )
            self._neutral_command_publisher = self.node.create_publisher(
                Float64MultiArray,
                self._command_topic(),
                10,
            )

    def precheck(self) -> AdapterResult:
        if shutil.which("ros2") is None:
            return AdapterResult.failed("ros2 CLI not found in PATH.")

        side = self._hand_side()
        if side not in {"left", "right"}:
            return AdapterResult.failed(
                f"{self.device.device_id}: invalid hand_side '{side}'. Expected left or right."
            )

        launch_arguments = self._raw_launch_arguments()
        invalid_keys = [
            key
            for key in (
                "launch_package",
                "launch_file",
                "session_mode",
                "start_manus",
                "start_wujihand",
                "use_pipeline",
                "start_left",
                "start_right",
                "pipeline_namespace",
                "pipeline_start_enabled",
            )
            if key in launch_arguments
        ]
        if invalid_keys:
            return AdapterResult.failed(
                f"{self.device.device_id}: launch_arguments must not override reserved keys {invalid_keys}"
            )

        identity_file = self._identity_file_path()
        if not identity_file.exists():
            return AdapterResult.failed(
                f"{self.device.device_id}: identity file does not exist: {identity_file}"
            )

        teleop_config = self._teleop_config_path()
        if not teleop_config.exists():
            return AdapterResult.failed(
                f"{self.device.device_id}: teleop config does not exist: {teleop_config}"
            )

        controllers_file = self._controllers_file_path()
        if not controllers_file.exists():
            return AdapterResult.failed(
                f"{self.device.device_id}: controllers file does not exist: {controllers_file}"
            )

        try:
            neutral_pose = self._neutral_pose()
        except Exception as exc:
            return AdapterResult.failed(
                f"{self.device.device_id}: failed to resolve neutral pose: {exc}"
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: WujiHand launch precheck passed.",
            metadata={
                "hand_side": side,
                "namespace": self._hand_namespace(),
                "command_topic": self._command_topic(),
                "controller_manager_service": self._controller_manager_service_name(),
                "pipeline_service": self._pipeline_set_enabled_service_name(),
                "neutral_pose": neutral_pose,
            },
        )

    def bringup(self) -> AdapterResult:
        if self._managed_launch is not None and self._managed_launch.is_running():
            return AdapterResult.ok(f"{self.device.device_id}: WujiHand bringup already running.")
        if self._process is not None and self._process.poll() is None:
            return AdapterResult.ok(f"{self.device.device_id}: WujiHand bringup already running.")
        return self._start_launch_process()

    def wait_ready(self, timeout_sec: float) -> AdapterResult:
        deadline = time.monotonic() + self._ready_timeout_sec(timeout_sec)
        last_controllers = None
        while time.monotonic() < deadline:
            if not self._launch_is_running():
                return AdapterResult.failed(
                    f"{self.device.device_id}: WujiHand launch exited before READY.\n"
                    f"{self._diagnostic_summary()}"
                )

            controllers = self._list_controllers()
            if controllers is not None:
                last_controllers = controllers
                if self._controllers_ready(controllers) and self._service_available(
                    self._set_enabled_client,
                    timeout_sec=0.2,
                ):
                    return AdapterResult.ok(
                        f"{self.device.device_id}: WujiHand READY. controllers active and teleop service available.",
                        metadata={
                            "controllers": controllers,
                            "command_topic": self._command_topic(),
                            "pipeline_service": self._pipeline_set_enabled_service_name(),
                            "log_path": None if self._log_path is None else str(self._log_path),
                        },
                    )

            time.sleep(0.1)

        return AdapterResult.failed(
            f"{self.device.device_id}: WujiHand READY timeout after {self._ready_timeout_sec(timeout_sec):.1f}s.\n"
            f"controllers={last_controllers}\n"
            f"{self._diagnostic_summary()}"
        )

    def shutdown(self) -> AdapterResult:
        if self._launch_is_running():
            self._call_set_enabled(False, timeout_sec=3.0)

        managed_launch = self._managed_launch
        if managed_launch is not None:
            if not managed_launch.is_running():
                exit_code = managed_launch.launch_exit_code()
                self._cleanup_process_state()
                return AdapterResult.ok(
                    f"{self.device.device_id}: WujiHand launch already exited with code {exit_code}."
                )

            if not managed_launch.shutdown(timeout_sec=12.0):
                return AdapterResult.failed(
                    f"{self.device.device_id}: managed WujiHand launch did not stop within timeout."
                )

            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: WujiHand launch stopped.")

        process = self._process
        if process is None:
            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: WujiHand launch already stopped.")

        if process.poll() is not None:
            exit_code = process.returncode
            self._cleanup_process_state()
            return AdapterResult.ok(
                f"{self.device.device_id}: WujiHand launch already exited with code {exit_code}."
            )

        if not self._signal_process_group(process, signal.SIGINT, timeout_sec=8.0):
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=4.0)
        if not self._wait_for_process_group_exit(process.pid, timeout_sec=2.0):
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=4.0)
        if not self._wait_for_process_group_exit(process.pid, timeout_sec=2.0):
            self._signal_process_group(process, signal.SIGKILL, timeout_sec=2.0)
        self._wait_for_process_group_exit(process.pid, timeout_sec=1.0)

        self._cleanup_process_state()
        return AdapterResult.ok(f"{self.device.device_id}: WujiHand launch stopped.")

    def diagnose(self) -> AdapterResult:
        if not self._launch_is_running():
            if self._managed_launch is None and self._process is None:
                return AdapterResult.failed(f"{self.device.device_id}: WujiHand launch not started.")
            if self._managed_launch is not None:
                return AdapterResult.failed(
                    f"{self.device.device_id}: WujiHand launch exited with code {self._managed_launch.launch_exit_code()}."
                )
            return AdapterResult.failed(
                f"{self.device.device_id}: WujiHand launch exited with code {self._process.returncode}."
            )

        controllers = self._list_controllers()
        if controllers is None:
            return AdapterResult.degraded(
                f"{self.device.device_id}: controller_manager/list_controllers unavailable.",
                metadata={"log_path": None if self._log_path is None else str(self._log_path)},
            )

        missing = self._missing_required_controllers(controllers)
        if missing:
            return AdapterResult.degraded(
                f"{self.device.device_id}: missing required controllers: {', '.join(missing)}",
                metadata={
                    "controllers": controllers,
                    "log_path": None if self._log_path is None else str(self._log_path),
                },
            )

        if not self._service_available(self._set_enabled_client, timeout_sec=0.2):
            return AdapterResult.degraded(
                f"{self.device.device_id}: teleop enable service unavailable.",
                metadata={
                    "pipeline_service": self._pipeline_set_enabled_service_name(),
                    "log_path": None if self._log_path is None else str(self._log_path),
                },
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: WujiHand healthy.",
            metadata={
                "controllers": controllers,
                "command_topic": self._command_topic(),
                "pipeline_service": self._pipeline_set_enabled_service_name(),
                "log_path": None if self._log_path is None else str(self._log_path),
                "command": self._launch_command,
            },
        )

    def before_session(self) -> AdapterResult:
        return self._call_set_enabled(True, timeout_sec=5.0)

    def after_session(self) -> AdapterResult:
        disable_result = self._call_set_enabled(False, timeout_sec=5.0)
        neutral_result = self._publish_neutral_pose()
        if disable_result.is_failure() and neutral_result.is_failure():
            return AdapterResult.failed(
                f"{disable_result.summary} {neutral_result.summary}".strip()
            )
        if disable_result.is_failure():
            return disable_result
        if neutral_result.is_failure():
            return neutral_result
        return AdapterResult.ok(
            f"{self.device.device_id}: teleop disabled and neutral pose commanded."
        )

    def home(self) -> AdapterResult:
        return self._publish_neutral_pose()

    def dump_metadata(self) -> dict:
        metadata = super().dump_metadata()
        metadata.update(
            {
                "hand_side": self._hand_side(),
                "namespace": self._hand_namespace(),
                "launch_package": self._launch_package(),
                "launch_file": self._launch_file(),
                "launch_arguments": self._runtime_arguments(),
                "teleop_config": str(self._teleop_config_path()),
                "identity_file": str(self._identity_file_path()),
                "command_topic": self._command_topic(),
                "pipeline_service": self._pipeline_set_enabled_service_name(),
                "log_path": None if self._log_path is None else str(self._log_path),
            }
        )
        if self._managed_launch is not None:
            metadata.update(self._managed_launch.metadata())
        return metadata

    def record_topics(self) -> list[str]:
        topics = self.device.config.get("record_topics", [])
        if isinstance(topics, list) and topics:
            return [str(topic) for topic in topics]
        return [self._joint_state_topic()]

    def _hand_side(self) -> str:
        raw_value = self.device.config.get("hand_side")
        if raw_value is not None:
            return str(raw_value).strip().lower()
        device_id = str(self.device.device_id).strip().lower()
        if device_id.startswith("left"):
            return "left"
        if device_id.startswith("right"):
            return "right"
        return ""

    def _hand_namespace(self) -> str:
        raw_value = self.device.config.get("namespace")
        if raw_value:
            return str(raw_value).strip().strip("/")
        return self._hand_side()

    def _pipeline_namespace(self) -> str:
        raw_value = self.device.config.get("pipeline_namespace")
        if raw_value is not None:
            return str(raw_value).strip().strip("/")
        return self._hand_namespace()

    def _absolute_namespace(self, namespace: str) -> str:
        namespace = str(namespace).strip().strip("/")
        return f"/{namespace}" if namespace else ""

    def _controller_manager_service_name(self) -> str:
        return f"{self._absolute_namespace(self._hand_namespace())}/controller_manager/list_controllers"

    def _pipeline_set_enabled_service_name(self) -> str:
        namespace = self._absolute_namespace(self._pipeline_namespace())
        return f"{namespace}/wujihand_manus_pipeline/set_enabled"

    def _command_topic(self) -> str:
        raw_value = self.device.config.get("command_topic")
        if raw_value:
            return str(raw_value)
        namespace = self._absolute_namespace(self._hand_namespace())
        return f"{namespace}/forward_position_controller/commands"

    def _joint_state_topic(self) -> str:
        raw_value = self.device.config.get("joint_state_topic")
        if raw_value:
            return str(raw_value)
        namespace = self._absolute_namespace(self._hand_namespace())
        return f"{namespace}/joint_states"

    def _launch_package(self) -> str:
        return str(self.device.config.get("launch_package", self.DEFAULT_LAUNCH_PACKAGE))

    def _launch_file(self) -> str:
        return str(self.device.config.get("launch_file", self.DEFAULT_LAUNCH_FILE))

    def _identity_file_path(self) -> Path:
        raw_value = self.device.config.get("identity_file")
        if raw_value:
            return Path(str(raw_value)).expanduser()
        package_share = Path(get_package_share_directory("wujihand_system"))
        return package_share / self.DEFAULT_IDENTITY_FILE_RELATIVE_PATH

    def _teleop_config_path(self) -> Path:
        raw_value = self.device.config.get("teleop_config")
        if raw_value:
            return Path(str(raw_value)).expanduser()
        package_share = Path(get_package_share_directory("wujihand_system"))
        return package_share / self.DEFAULT_TELEOP_CONFIG_RELATIVE_PATH

    def _controllers_file_path(self) -> Path:
        raw_value = self.device.config.get("controllers_file")
        if raw_value:
            return Path(str(raw_value)).expanduser()
        package_share = Path(get_package_share_directory("wujihand_system"))
        if self._hand_side() == "left":
            return package_share / self.DEFAULT_LEFT_CONTROLLERS_RELATIVE_PATH
        return package_share / self.DEFAULT_RIGHT_CONTROLLERS_RELATIVE_PATH

    def _ros2_control_xacro_path(self) -> Path:
        raw_value = self.device.config.get("ros2_control_xacro")
        if raw_value:
            return Path(str(raw_value)).expanduser()
        package_share = Path(get_package_share_directory("wujihand_system"))
        if self._hand_side() == "left":
            return package_share / self.DEFAULT_LEFT_ROS2_CONTROL_RELATIVE_PATH
        return package_share / self.DEFAULT_RIGHT_ROS2_CONTROL_RELATIVE_PATH

    def _raw_launch_arguments(self) -> dict[str, object]:
        raw_arguments = self.device.config.get("launch_arguments", {})
        if isinstance(raw_arguments, dict):
            return dict(raw_arguments)
        return {}

    def _prepare_side_teleop_config(self) -> Path:
        if self._generated_teleop_config_path is not None:
            return self._generated_teleop_config_path

        source_path = self._teleop_config_path()
        with source_path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"Expected mapping at YAML root: {source_path}")

        is_left = self._hand_side() == "left"
        command_topic = self._command_topic()
        data["publish_hand_input"] = False
        data["include_left_hand"] = is_left
        data["include_right_hand"] = not is_left
        data["enable_left_hand"] = is_left
        data["enable_right_hand"] = not is_left
        data["require_both_hands"] = False
        data["single_hand_fallback_side"] = self._hand_side()
        if is_left:
            data["left_command_topic"] = command_topic
        else:
            data["right_command_topic"] = command_topic

        handle = tempfile.NamedTemporaryFile(
            mode="w",
            prefix=f"{self.device.device_id}_wujihand_teleop_",
            suffix=".yaml",
            delete=False,
            encoding="utf-8",
        )
        try:
            yaml.safe_dump(data, handle, sort_keys=False)
        finally:
            handle.close()
        self._generated_teleop_config_path = Path(handle.name)
        return self._generated_teleop_config_path

    def _runtime_arguments(self) -> dict[str, object]:
        is_left = self._hand_side() == "left"
        arguments = {
            "session_mode": "manual",
            "start_manus": False,
            "start_wujihand": True,
            "use_pipeline": True,
            "activate_forward_controller": True,
            "enable_rviz": False,
            "start_left": is_left,
            "start_right": not is_left,
            "left_namespace": self._hand_namespace() if is_left else "left",
            "right_namespace": "right" if is_left else self._hand_namespace(),
            "pipeline_namespace": self._pipeline_namespace(),
            "pipeline_start_enabled": False,
            "identity_file": str(self._identity_file_path()),
            "teleop_config": str(self._prepare_side_teleop_config()),
        }
        launch_arguments = self._raw_launch_arguments()
        if "use_mock_hardware" in launch_arguments:
            arguments["use_mock_hardware"] = launch_arguments.pop("use_mock_hardware")
        else:
            arguments["use_mock_hardware"] = False
        arguments.update(launch_arguments)
        return arguments

    def _build_command(self) -> list[str]:
        command = [
            "ros2",
            "launch",
            self._launch_package(),
            self._launch_file(),
        ]
        for key, value in self._runtime_arguments().items():
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

    def _matches_wujihand_process(self, process_name: str, cmd: list[str]) -> bool:
        joined = " ".join(cmd)
        namespace = self._absolute_namespace(self._hand_namespace())
        markers = [
            f"__ns:={namespace}",
            f"{namespace}/controller_manager",
            self._command_topic(),
            str(self._generated_teleop_config_path) if self._generated_teleop_config_path else "",
        ]
        if not any(marker and marker in joined for marker in markers):
            return False
        executable_markers = {
            "ros2_control_node",
            "robot_state_publisher",
            "spawner",
            "wujihand_manus_pipeline.py",
        }
        if process_name in executable_markers:
            return True
        return any(marker in joined for marker in executable_markers)

    def _start_launch_process(self) -> AdapterResult:
        self._cleanup_process_state()
        self._launch_command = self._build_command()

        use_managed_launch = bool(self.device.config.get("use_managed_launch", False))
        if use_managed_launch and self.launch_manager is not None:
            managed_launch = ManagedLaunchSession(
                launch_manager=self.launch_manager,
                label=f"{self.device.device_id}_wujihand",
                launch_file_path=self._launch_file_path(),
                launch_arguments=self._runtime_arguments(),
                process_matcher=self._matches_wujihand_process,
                logger=None if self.node is None else self.node.get_logger(),
            )
            self._managed_launch = managed_launch
            self._log_path = Path(managed_launch.log_path)
            try:
                managed_launch.start()
            except Exception as exc:
                self._cleanup_process_state()
                return AdapterResult.failed(
                    f"{self.device.device_id}: failed to start managed WujiHand launch: {exc}"
                )

            managed_launch.wait_started(timeout_sec=2.0)
            if not managed_launch.is_running():
                return AdapterResult.failed(
                    f"{self.device.device_id}: WujiHand launch exited early.\n{self._diagnostic_summary()}"
                )

            return AdapterResult.ok(
                f"{self.device.device_id}: WujiHand launch started.",
                metadata={
                    **managed_launch.metadata(),
                    "log_path": str(self._log_path),
                    "command": self._launch_command,
                },
            )

        log_dir = Path(tempfile.mkdtemp(prefix="wujihand_adapter_"))
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
                f"{self.device.device_id}: failed to start WujiHand launch: {exc}"
            )

        time.sleep(0.5)
        if self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: WujiHand launch exited early.\n{self._diagnostic_summary()}"
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: WujiHand launch started.",
            metadata={
                "pid": self._process.pid,
                "log_path": str(self._log_path),
                "command": self._launch_command,
            },
        )

    def _ready_timeout_sec(self, timeout_sec: float) -> float:
        configured = self.device.config.get("ready_timeout_sec", 20.0)
        if timeout_sec > 0.0:
            return timeout_sec
        try:
            return float(configured)
        except (TypeError, ValueError):
            return 20.0

    def _launch_is_running(self) -> bool:
        if self._managed_launch is not None:
            return self._managed_launch.is_running()
        return self._process is not None and self._process.poll() is None

    def _process_environment(self) -> dict[str, str]:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        return env

    def _list_controllers(self) -> dict[str, str] | None:
        if self._list_controllers_client is None:
            return None
        response, error_message = self._call_service_response(
            client=self._list_controllers_client,
            request=ListControllers.Request(),
            timeout_sec=1.0,
        )
        if response is None:
            del error_message
            return None

        controllers: dict[str, str] = {}
        for controller in getattr(response, "controller", []):
            name = str(getattr(controller, "name", "")).strip()
            state = str(getattr(controller, "state", "")).strip()
            if name:
                controllers[name] = state
        return controllers

    def _controllers_ready(self, controllers: dict[str, str]) -> bool:
        return not self._missing_required_controllers(controllers)

    def _missing_required_controllers(self, controllers: dict[str, str]) -> list[str]:
        missing = []
        for controller_name, required_states in self.REQUIRED_CONTROLLER_STATES.items():
            actual_state = controllers.get(controller_name)
            if actual_state not in required_states:
                actual_label = "missing" if actual_state is None else actual_state
                expected_label = "/".join(sorted(required_states))
                missing.append(f"{controller_name}={actual_label} (expected {expected_label})")
        return missing

    def _service_available(self, client, timeout_sec: float) -> bool:
        if client is None or not self._ros_context_is_usable():
            return False
        try:
            return client.service_is_ready() or client.wait_for_service(timeout_sec=timeout_sec)
        except Exception:
            return False

    def _ros_context_is_usable(self) -> bool:
        if self.node is None:
            return False
        try:
            return self.node.context.ok()
        except Exception:
            return False

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

    def _call_set_enabled(self, enabled: bool, timeout_sec: float) -> AdapterResult:
        if not self._launch_is_running():
            return AdapterResult.failed(
                f"{self.device.device_id}: WujiHand launch is not running for {self._pipeline_set_enabled_service_name()}."
            )
        request = SetBool.Request()
        request.data = enabled
        response, error_message = self._call_service_response(
            client=self._set_enabled_client,
            request=request,
            timeout_sec=timeout_sec,
        )
        if response is None:
            return AdapterResult.failed(
                f"{self.device.device_id}: {self._pipeline_set_enabled_service_name()} failed: {error_message}"
            )
        if not bool(getattr(response, "success", False)):
            return AdapterResult.failed(
                f"{self.device.device_id}: {self._pipeline_set_enabled_service_name()} rejected: {getattr(response, 'message', '')}"
            )
        return AdapterResult.ok(
            f"{self.device.device_id}: pipeline {'enabled' if enabled else 'disabled'}."
        )

    def _publish_neutral_pose(self) -> AdapterResult:
        if self._neutral_command_publisher is None:
            return AdapterResult.failed(
                f"{self.device.device_id}: neutral pose publisher unavailable."
            )
        neutral_pose = self._neutral_pose()
        msg = Float64MultiArray()
        msg.data = neutral_pose
        for _ in range(3):
            self._neutral_command_publisher.publish(msg)
            time.sleep(0.05)
        return AdapterResult.ok(f"{self.device.device_id}: neutral pose published.")

    def _neutral_pose(self) -> list[float]:
        override = self.device.config.get("neutral_pose")
        if isinstance(override, list) and override:
            return [float(value) for value in override]

        controllers = self._load_yaml_map(self._controllers_file_path())
        controller_config = controllers.get("forward_position_controller", {})
        ros_params = controller_config.get("ros__parameters", {})
        joints = ros_params.get("joints", [])
        if not isinstance(joints, list) or not joints:
            raise RuntimeError("forward_position_controller joints are missing")

        root = ET.parse(self._ros2_control_xacro_path()).getroot()
        initial_values = {}
        for joint in root.iter("joint"):
            joint_name = joint.attrib.get("name", "").strip()
            if not joint_name:
                continue
            initial_value = None
            for command_interface in joint.findall("command_interface"):
                if command_interface.attrib.get("name") != "position":
                    continue
                for param in command_interface.findall("param"):
                    if param.attrib.get("name") == "initial_value":
                        initial_value = (param.text or "").strip()
                        break
                if initial_value is not None:
                    break
            if initial_value is not None:
                initial_values[joint_name] = float(initial_value)

        resolved = []
        for joint_name in joints:
            joint_key = str(joint_name).strip()
            if joint_key not in initial_values:
                raise RuntimeError(f"initial_value missing for joint {joint_key}")
            resolved.append(initial_values[joint_key])
        return resolved

    def _load_yaml_map(self, path: Path) -> dict:
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"Expected YAML mapping at root: {path}")
        return data

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

    def _cleanup_process_state(self) -> None:
        if self._managed_launch is not None:
            self._managed_launch.close()
            self._managed_launch = None
        if self._log_handle is not None:
            self._log_handle.close()
            self._log_handle = None
        self._process = None
        if self._generated_teleop_config_path is not None:
            try:
                self._generated_teleop_config_path.unlink()
            except FileNotFoundError:
                pass
            self._generated_teleop_config_path = None

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
