from __future__ import annotations

import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import tempfile
import time

from ament_index_python.packages import get_package_share_directory
from manus_system.msg import ManusGloveRawArray

from ..managed_launch import ManagedLaunchSession
from .base import AdapterBase, AdapterResult


class ManusAdapter(AdapterBase):
    DEFAULT_LAUNCH_PACKAGE = "manus_system"
    DEFAULT_LAUNCH_FILE = "manus_raw_publisher.launch.py"
    DEFAULT_CONFIG_RELATIVE_PATH = "config/manus_raw_publisher.yaml"
    DEFAULT_READY_TOPIC = "/manus_raw_publisher_node/gloves_raw"

    def __init__(self, device, node=None) -> None:
        super().__init__(device, node=node)
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._managed_launch: ManagedLaunchSession | None = None
        self._log_path: Path | None = None
        self._launch_command: list[str] = []
        self._last_message_monotonic = 0.0
        self._ready_subscription = None
        if self.node is not None:
            self._ready_subscription = self.node.create_subscription(
                ManusGloveRawArray,
                self._ready_topic(),
                self._on_ready_message,
                10,
            )

    def precheck(self) -> AdapterResult:
        if shutil.which("ros2") is None:
            return AdapterResult.failed("ros2 CLI not found in PATH.")

        mock_enabled = self._mock_enabled()
        config_file = self._config_file_path()
        if not mock_enabled and not config_file.exists():
            return AdapterResult.failed(
                f"{self.device.device_id}: MANUS config does not exist: {config_file}"
            )

        user_name = self._user_name()
        if not mock_enabled and not user_name:
            return AdapterResult.failed(
                f"{self.device.device_id}: MANUS user_name is required for calibration loading."
            )

        launch_arguments = self._launch_arguments()
        invalid_keys = [key for key in ("launch_file", "launch_package") if key in launch_arguments]
        if invalid_keys:
            return AdapterResult.failed(
                f"{self.device.device_id}: launch_arguments must not override reserved keys {invalid_keys}"
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: MANUS launch precheck passed.",
            metadata={
                "config_file": str(config_file),
                "user_name": user_name,
                "mock_enabled": mock_enabled,
                "ready_topic": self._ready_topic(),
                "launch_arguments": launch_arguments,
            },
        )

    def bringup(self) -> AdapterResult:
        if self._managed_launch is not None and self._managed_launch.is_running():
            return AdapterResult.ok(f"{self.device.device_id}: MANUS bringup already running.")
        if self._process is not None and self._process.poll() is None:
            return AdapterResult.ok(f"{self.device.device_id}: MANUS bringup already running.")

        self._last_message_monotonic = 0.0
        return self._start_launch_process()

    def wait_ready(self, timeout_sec: float) -> AdapterResult:
        process = self._process
        managed_launch = self._managed_launch
        if process is None and managed_launch is None:
            return AdapterResult.failed(f"{self.device.device_id}: MANUS launch was not started.")

        ready_topic = self._ready_topic()
        deadline = time.monotonic() + self._ready_timeout_sec(timeout_sec)

        while time.monotonic() < deadline:
            launch_running = (
                managed_launch.is_running()
                if managed_launch is not None
                else (process is not None and process.poll() is None)
            )
            if not launch_running:
                return AdapterResult.failed(
                    f"{self.device.device_id}: MANUS launch exited before READY.\n"
                    f"{self._diagnostic_summary()}"
                )

            if self._has_fresh_message():
                return AdapterResult.ok(
                    f"{self.device.device_id}: MANUS READY at {ready_topic}.",
                    metadata={
                        "ready_topic": ready_topic,
                        "config_file": str(self._config_file_path()),
                        "user_name": self._user_name(),
                        "log_path": None if self._log_path is None else str(self._log_path),
                    },
                )

            time.sleep(0.1)

        return AdapterResult.failed(
            f"{self.device.device_id}: MANUS READY timeout after "
            f"{self._ready_timeout_sec(timeout_sec):.1f}s waiting for {ready_topic}.\n"
            f"{self._diagnostic_summary()}"
        )

    def shutdown(self) -> AdapterResult:
        managed_launch = self._managed_launch
        if managed_launch is not None:
            if not managed_launch.is_running():
                exit_code = managed_launch.launch_exit_code()
                self._cleanup_process_state()
                return AdapterResult.ok(
                    f"{self.device.device_id}: MANUS launch already exited with code {exit_code}."
                )

            if not managed_launch.shutdown(timeout_sec=12.0):
                return AdapterResult.failed(
                    f"{self.device.device_id}: managed MANUS launch did not stop within timeout."
                )

            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: MANUS launch stopped.")

        process = self._process
        if process is None:
            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: MANUS launch already stopped.")

        if process.poll() is not None:
            exit_code = process.returncode
            self._cleanup_process_state()
            return AdapterResult.ok(
                f"{self.device.device_id}: MANUS launch already exited with code {exit_code}."
            )

        if not self._signal_process_group(process, signal.SIGINT, timeout_sec=8.0):
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=4.0)
        if not self._wait_for_process_group_exit(process.pid, timeout_sec=2.0):
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=4.0)
        if not self._wait_for_process_group_exit(process.pid, timeout_sec=2.0):
            self._signal_process_group(process, signal.SIGKILL, timeout_sec=2.0)
        self._wait_for_process_group_exit(process.pid, timeout_sec=1.0)

        self._cleanup_process_state()
        return AdapterResult.ok(f"{self.device.device_id}: MANUS launch stopped.")

    def diagnose(self) -> AdapterResult:
        if self._managed_launch is None and self._process is None:
            return AdapterResult.failed(f"{self.device.device_id}: MANUS launch not started.")
        if self._managed_launch is not None and not self._managed_launch.is_running():
            return AdapterResult.failed(
                f"{self.device.device_id}: MANUS launch exited with code {self._managed_launch.launch_exit_code()}."
            )
        if self._managed_launch is None and self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: MANUS launch exited with code {self._process.returncode}."
            )

        ready_topic = self._ready_topic()
        if not self._has_fresh_message():
            return AdapterResult.degraded(
                f"{self.device.device_id}: no recent MANUS sample on {ready_topic}.",
                metadata={
                    "ready_topic": ready_topic,
                    "config_file": str(self._config_file_path()),
                    "user_name": self._user_name(),
                    "log_path": None if self._log_path is None else str(self._log_path),
                },
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: MANUS topic healthy at {ready_topic}.",
            metadata={
                "ready_topic": ready_topic,
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
                "config_file": str(self._config_file_path()),
                "mock_enabled": self._mock_enabled(),
                "user_name": self._user_name(),
                "ready_topic": self._ready_topic(),
                "record_topics": self.record_topics(),
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
        return [self._ready_topic()]

    def _launch_package(self) -> str:
        return str(self.device.config.get("launch_package", self.DEFAULT_LAUNCH_PACKAGE))

    def _launch_file(self) -> str:
        return str(self.device.config.get("launch_file", self.DEFAULT_LAUNCH_FILE))

    def _config_file_path(self) -> Path:
        raw_value = self.device.config.get("config_file")
        if raw_value:
            return Path(str(raw_value)).expanduser()

        package_share = Path(get_package_share_directory("manus_system"))
        return package_share / self.DEFAULT_CONFIG_RELATIVE_PATH

    def _user_name(self) -> str:
        raw_value = self.device.config.get("user_name")
        if raw_value is None:
            return ""
        return str(raw_value).strip()

    def _ready_topic(self) -> str:
        raw_value = self.device.config.get("ready_topic")
        if raw_value:
            return str(raw_value)
        return self.DEFAULT_READY_TOPIC

    def _launch_arguments(self) -> dict[str, object]:
        raw_arguments = self.device.config.get("launch_arguments", {})
        if isinstance(raw_arguments, dict):
            return dict(raw_arguments)
        return {}

    def _mock_enabled(self) -> bool:
        raw_value = self._launch_arguments().get("mock", False)
        if isinstance(raw_value, bool):
            return raw_value
        if isinstance(raw_value, str):
            return raw_value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(raw_value)

    def _build_command(self) -> list[str]:
        command = [
            "ros2",
            "launch",
            self._launch_package(),
            self._launch_file(),
            f"config_file:={self._config_file_path()}",
            f"user_name:={self._user_name()}",
        ]
        for key, value in self._launch_arguments().items():
            command.append(f"{key}:={self._value_to_string(value)}")
        return command

    def _runtime_arguments(self) -> dict[str, object]:
        arguments = self._launch_arguments()
        arguments.setdefault("config_file", str(self._config_file_path()))
        arguments.setdefault("user_name", self._user_name())
        return arguments

    def _launch_file_path(self) -> str:
        launch_file = self._launch_file()
        launch_path = Path(launch_file).expanduser()
        if launch_path.is_absolute() or "/" in launch_file:
            return str(launch_path)

        package_share = Path(get_package_share_directory(self._launch_package()))
        candidates = [
            package_share / "launch" / launch_file,
            package_share / "bringup" / "launch" / launch_file,
        ]
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        return str(candidates[0])

    def _matches_manus_process(self, process_name: str, cmd: list[str]) -> bool:
        executable_markers = {"manus_raw_publisher_node", "manus_raw_publisher_node.exe"}
        if process_name in executable_markers:
            return True
        joined = " ".join(cmd)
        return any(marker in joined for marker in executable_markers)

    def _start_launch_process(self) -> AdapterResult:
        self._cleanup_process_state()
        self._launch_command = self._build_command()

        use_managed_launch = bool(self.device.config.get("use_managed_launch", False))
        if use_managed_launch and self.launch_manager is not None:
            managed_launch = ManagedLaunchSession(
                launch_manager=self.launch_manager,
                label=f"{self.device.device_id}_manus",
                launch_file_path=self._launch_file_path(),
                launch_arguments=self._runtime_arguments(),
                process_matcher=self._matches_manus_process,
                logger=None if self.node is None else self.node.get_logger(),
            )
            self._managed_launch = managed_launch
            self._log_path = Path(managed_launch.log_path)
            try:
                managed_launch.start()
            except Exception as exc:
                self._cleanup_process_state()
                return AdapterResult.failed(
                    f"{self.device.device_id}: failed to start managed MANUS launch: {exc}"
                )

            managed_launch.wait_started(timeout_sec=2.0)
            if not managed_launch.is_running():
                return AdapterResult.failed(
                    f"{self.device.device_id}: MANUS launch exited early.\n{self._diagnostic_summary()}"
                )

            return AdapterResult.ok(
                f"{self.device.device_id}: MANUS launch started.",
                metadata={
                    **managed_launch.metadata(),
                    "log_path": str(self._log_path),
                    "command": self._launch_command,
                },
            )

        log_dir = Path(tempfile.mkdtemp(prefix="manus_adapter_"))
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
                f"{self.device.device_id}: failed to start MANUS launch: {exc}"
            )

        time.sleep(0.5)
        if self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: MANUS launch exited early.\n{self._diagnostic_summary()}"
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: MANUS launch started.",
            metadata={
                "pid": self._process.pid,
                "log_path": str(self._log_path),
                "command": self._launch_command,
            },
        )

    def _value_to_string(self, value: object) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        return str(value)

    def _ready_timeout_sec(self, timeout_sec: float) -> float:
        configured = self.device.config.get("ready_timeout_sec", 20.0)
        if timeout_sec > 0.0:
            return timeout_sec
        try:
            return float(configured)
        except (TypeError, ValueError):
            return 20.0

    def _diagnose_timeout_sec(self) -> float:
        configured = self.device.config.get("diagnose_timeout_sec", 2.0)
        try:
            return max(0.1, float(configured))
        except (TypeError, ValueError):
            return 2.0

    def _on_ready_message(self, msg: ManusGloveRawArray) -> None:
        del msg
        self._last_message_monotonic = time.monotonic()

    def _has_fresh_message(self) -> bool:
        if self._last_message_monotonic <= 0.0:
            return False
        return time.monotonic() - self._last_message_monotonic <= self._diagnose_timeout_sec()

    def _topic_has_message(self, topic_name: str, timeout_sec: float) -> bool:
        timeout_arg = str(max(1, int(timeout_sec)))
        try:
            completed = subprocess.run(
                [
                    "ros2",
                    "topic",
                    "echo",
                    "--once",
                    "--timeout",
                    timeout_arg,
                    "--field",
                    "header.stamp",
                    topic_name,
                ],
                capture_output=True,
                text=True,
                timeout=timeout_sec + 1.0,
                env=self._process_environment(),
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return completed.returncode == 0 and bool(completed.stdout.strip())

    def _process_environment(self) -> dict[str, str]:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        return env

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
