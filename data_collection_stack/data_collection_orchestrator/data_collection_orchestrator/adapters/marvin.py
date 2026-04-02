from __future__ import annotations

import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import tempfile
import time

from .base import AdapterBase, AdapterResult


class MarvinAdapter(AdapterBase):
    DEFAULT_LAUNCH_PACKAGE = "marvin_system"
    DEFAULT_LAUNCH_FILE = "marvin_tracker_teleop.launch.py"
    REQUIRED_CONTROLLERS = {
        "joint_state_broadcaster": "active",
        "tracker_teleop_controller": "active",
    }
    REQUIRED_SERVICES = {
        "/tracker_teleop_controller/set_armed": "std_srvs/srv/SetBool",
        "/tracker_teleop_controller/set_enabled": "std_srvs/srv/SetBool",
        "/tracker_teleop_controller/go_home": "std_srvs/srv/Trigger",
    }

    def __init__(self, device, node=None) -> None:
        super().__init__(device, node=node)
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._log_path: Path | None = None
        self._launch_command: list[str] = []

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
        if self._process is None:
            return AdapterResult.failed(f"{self.device.device_id}: Marvin launch was not started.")

        deadline = time.monotonic() + self._ready_timeout_sec(timeout_sec)
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
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
                    return AdapterResult.ok(
                        f"{self.device.device_id}: controller_manager and tracker teleop services are READY.",
                        metadata={
                            "controllers": controllers,
                            "log_path": None if self._log_path is None else str(self._log_path),
                        },
                    )

            time.sleep(0.5)

        return AdapterResult.failed(
            f"{self.device.device_id}: Marvin READY timeout after "
            f"{self._ready_timeout_sec(timeout_sec):.1f}s.\n{self._diagnostic_summary()}"
        )

    def shutdown(self) -> AdapterResult:
        self._call_set_bool_service("/tracker_teleop_controller/set_enabled", False, timeout_sec=3.0)
        self._call_set_bool_service("/tracker_teleop_controller/set_armed", False, timeout_sec=3.0)

        if self._process is None:
            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: Marvin launch already stopped.")

        if self._process.poll() is not None:
            exit_code = self._process.returncode
            self._cleanup_process_state()
            return AdapterResult.ok(
                f"{self.device.device_id}: Marvin launch already exited with code {exit_code}."
            )

        if not self._signal_process_group(signal.SIGINT, timeout_sec=10.0):
            self._signal_process_group(signal.SIGTERM, timeout_sec=5.0)
        if self._process is not None and self._process.poll() is None:
            self._signal_process_group(signal.SIGKILL, timeout_sec=2.0)

        self._cleanup_process_state()
        return AdapterResult.ok(f"{self.device.device_id}: Marvin launch stopped.")

    def diagnose(self) -> AdapterResult:
        return AdapterResult.ok(
            self._diagnostic_summary(),
            metadata={
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
        return self._call_set_bool_service(
            "/tracker_teleop_controller/set_armed",
            True,
            timeout_sec=5.0,
        )

    def disarm(self) -> AdapterResult:
        disable_result = self._call_set_bool_service(
            "/tracker_teleop_controller/set_enabled",
            False,
            timeout_sec=5.0,
        )
        if disable_result.is_failure():
            return disable_result
        return self._call_set_bool_service(
            "/tracker_teleop_controller/set_armed",
            False,
            timeout_sec=5.0,
        )

    def home(self) -> AdapterResult:
        return self._call_trigger_service(
            "/tracker_teleop_controller/go_home",
            timeout_sec=5.0,
        )

    def before_session(self) -> AdapterResult:
        return self._call_set_bool_service(
            "/tracker_teleop_controller/set_enabled",
            True,
            timeout_sec=5.0,
        )

    def after_session(self) -> AdapterResult:
        return self._call_set_bool_service(
            "/tracker_teleop_controller/set_enabled",
            False,
            timeout_sec=5.0,
        )

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
        for controller_name, required_state in self.REQUIRED_CONTROLLERS.items():
            if controllers.get(controller_name) != required_state:
                return False
        return True

    def _service_available(self, service_name: str, expected_type: str) -> bool:
        completed = self._run_ros2_command(
            ["ros2", "service", "type", service_name],
            timeout_sec=2.0,
        )
        if completed is None or completed.returncode != 0:
            return False
        return completed.stdout.strip() == expected_type

    def _call_set_bool_service(
        self, service_name: str, value: bool, timeout_sec: float
    ) -> AdapterResult:
        if self._process is None or self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: Marvin launch is not running for {service_name}."
            )

        completed = self._run_ros2_command(
            [
                "ros2",
                "service",
                "call",
                service_name,
                "std_srvs/srv/SetBool",
                f"{{data: {'true' if value else 'false'}}}",
            ],
            timeout_sec=timeout_sec,
        )
        return self._parse_service_response(
            completed,
            success_summary=(
                f"{self.device.device_id}: {service_name} accepted data={value}."
            ),
            failure_prefix=f"{self.device.device_id}: {service_name} failed",
        )

    def _call_trigger_service(self, service_name: str, timeout_sec: float) -> AdapterResult:
        if self._process is None or self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: Marvin launch is not running for {service_name}."
            )

        completed = self._run_ros2_command(
            [
                "ros2",
                "service",
                "call",
                service_name,
                "std_srvs/srv/Trigger",
                "{}",
            ],
            timeout_sec=timeout_sec,
        )
        return self._parse_service_response(
            completed,
            success_summary=f"{self.device.device_id}: {service_name} accepted.",
            failure_prefix=f"{self.device.device_id}: {service_name} failed",
        )

    def _parse_service_response(
        self,
        completed: subprocess.CompletedProcess | None,
        *,
        success_summary: str,
        failure_prefix: str,
    ) -> AdapterResult:
        if completed is None:
            return AdapterResult.failed(f"{failure_prefix}: command timeout or spawn failure.")
        if completed.returncode != 0:
            message = completed.stderr.strip() or completed.stdout.strip()
            return AdapterResult.failed(f"{failure_prefix}: {message}")

        output = completed.stdout.strip()
        success_match = re.search(r"success=(True|False)", output)
        message_match = re.search(r"message='([^']*)'", output)
        message = "" if message_match is None else message_match.group(1)

        if success_match is None:
            return AdapterResult.failed(f"{failure_prefix}: unable to parse response: {output}")
        if success_match.group(1) != "True":
            if message:
                return AdapterResult.failed(f"{failure_prefix}: {message}")
            return AdapterResult.failed(f"{failure_prefix}: {output}")

        if message:
            return AdapterResult.ok(f"{success_summary} message={message}")
        return AdapterResult.ok(success_summary)

    def _signal_process_group(self, sig: signal.Signals, timeout_sec: float) -> bool:
        if self._process is None:
            return True

        try:
            os.killpg(self._process.pid, sig)
        except ProcessLookupError:
            return True

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                return True
            time.sleep(0.2)
        return self._process.poll() is not None

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
