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
from tf2_msgs.msg import TFMessage
import yaml

from ..managed_launch import ManagedLaunchSession
from .base import AdapterBase, AdapterResult


class HtcAdapter(AdapterBase):
    DEFAULT_LAUNCH_PACKAGE = "htc_system"
    DEFAULT_LAUNCH_FILE = "tracker_publisher.launch.py"
    DEFAULT_TRACKERS_CONFIG_RELATIVE_PATH = "bringup/config/trackers.yaml"

    def __init__(self, device, node=None) -> None:
        super().__init__(device, node=node)
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._managed_launch: ManagedLaunchSession | None = None
        self._log_path: Path | None = None
        self._launch_command: list[str] = []
        self._frame_last_seen_monotonic: dict[str, float] = {}
        self._tf_subscription = None
        if self.node is not None:
            self._tf_subscription = self.node.create_subscription(
                TFMessage,
                "/tf",
                self._on_tf_message,
                10,
            )

    def precheck(self) -> AdapterResult:
        if shutil.which("ros2") is None:
            return AdapterResult.failed("ros2 CLI not found in PATH.")

        trackers_config = self._trackers_config_path()
        if not trackers_config.exists():
            return AdapterResult.failed(
                f"{self.device.device_id}: trackers config does not exist: {trackers_config}"
            )

        expected_frames = self._expected_frames(trackers_config)
        if not expected_frames:
            return AdapterResult.failed(
                f"{self.device.device_id}: no expected tracker frame_ids found in {trackers_config}"
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: HTC launch precheck passed.",
            metadata={
                "trackers_config": str(trackers_config),
                "expected_frames": expected_frames,
                "runtime_arguments": self._runtime_arguments(),
            },
        )

    def bringup(self) -> AdapterResult:
        if self._managed_launch is not None and self._managed_launch.is_running():
            return AdapterResult.ok(f"{self.device.device_id}: HTC bringup already running.")
        if self._process is not None and self._process.poll() is None:
            return AdapterResult.ok(f"{self.device.device_id}: HTC bringup already running.")

        return self._start_launch_process()

    def wait_ready(self, timeout_sec: float) -> AdapterResult:
        process = self._process
        managed_launch = self._managed_launch
        if process is None and managed_launch is None:
            return AdapterResult.failed(f"{self.device.device_id}: HTC launch was not started.")

        trackers_config = self._trackers_config_path()
        expected_frames = self._expected_frames(trackers_config)
        deadline = time.monotonic() + self._ready_timeout_sec(timeout_sec)

        while time.monotonic() < deadline:
            launch_running = (
                managed_launch.is_running()
                if managed_launch is not None
                else (process is not None and process.poll() is None)
            )
            if not launch_running:
                return AdapterResult.failed(
                    f"{self.device.device_id}: HTC launch exited before READY.\n"
                    f"{self._diagnostic_summary()}"
                )

            published_frames = self._fresh_frames(expected_frames)
            if len(published_frames) == len(expected_frames):
                return AdapterResult.ok(
                    f"{self.device.device_id}: tracker TF READY with frames {expected_frames}.",
                    metadata={
                        "expected_frames": expected_frames,
                        "published_frames": published_frames,
                        "trackers_config": str(trackers_config),
                    },
                )

            time.sleep(0.1)

        return AdapterResult.failed(
            f"{self.device.device_id}: HTC READY timeout after "
            f"{self._ready_timeout_sec(timeout_sec):.1f}s waiting for frames {expected_frames}.\n"
            f"{self._diagnostic_summary()}"
        )

    def shutdown(self) -> AdapterResult:
        managed_launch = self._managed_launch
        if managed_launch is not None:
            if not managed_launch.is_running():
                exit_code = managed_launch.launch_exit_code()
                self._cleanup_process_state()
                return AdapterResult.ok(
                    f"{self.device.device_id}: HTC launch already exited with code {exit_code}."
                )

            if not managed_launch.shutdown(timeout_sec=12.0):
                return AdapterResult.failed(
                    f"{self.device.device_id}: managed HTC launch did not stop within timeout."
                )

            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: HTC launch stopped.")

        process = self._process
        if process is None:
            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: HTC launch already stopped.")

        if process.poll() is not None:
            exit_code = process.returncode
            self._cleanup_process_state()
            return AdapterResult.ok(
                f"{self.device.device_id}: HTC launch already exited with code {exit_code}."
            )

        if not self._signal_process_group(process, signal.SIGINT, timeout_sec=8.0):
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=4.0)
        if not self._wait_for_process_group_exit(process.pid, timeout_sec=2.0):
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=4.0)
        if not self._wait_for_process_group_exit(process.pid, timeout_sec=2.0):
            self._signal_process_group(process, signal.SIGKILL, timeout_sec=2.0)
        self._wait_for_process_group_exit(process.pid, timeout_sec=1.0)

        self._cleanup_process_state()
        return AdapterResult.ok(f"{self.device.device_id}: HTC launch stopped.")

    def diagnose(self) -> AdapterResult:
        if self._managed_launch is None and self._process is None:
            return AdapterResult.failed(f"{self.device.device_id}: HTC launch not started.")
        if self._managed_launch is not None and not self._managed_launch.is_running():
            return AdapterResult.failed(
                f"{self.device.device_id}: HTC launch exited with code {self._managed_launch.launch_exit_code()}."
            )
        if self._managed_launch is None and self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: HTC launch exited with code {self._process.returncode}."
            )

        trackers_config = self._trackers_config_path()
        expected_frames = self._expected_frames(trackers_config)
        published_frames = self._fresh_frames(expected_frames)
        if len(published_frames) != len(expected_frames):
            return AdapterResult.degraded(
                f"{self.device.device_id}: missing tracker TF frames. "
                f"expected={expected_frames} seen={published_frames}",
                metadata={
                    "expected_frames": expected_frames,
                    "published_frames": published_frames,
                    "trackers_config": str(trackers_config),
                    "log_path": None if self._log_path is None else str(self._log_path),
                },
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: tracker TF healthy with frames {expected_frames}.",
            metadata={
                "expected_frames": expected_frames,
                "published_frames": published_frames,
                "log_path": None if self._log_path is None else str(self._log_path),
                "command": self._launch_command,
            },
        )

    def dump_metadata(self) -> dict:
        metadata = super().dump_metadata()
        trackers_config = self._trackers_config_path()
        metadata.update(
            {
                "launch_package": self._launch_package(),
                "launch_file": self._launch_file(),
                "runtime_arguments": self._runtime_arguments(),
                "trackers_config": str(trackers_config),
                "expected_frames": self._expected_frames(trackers_config),
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
        return ["/tf", "/tf_static"]

    def _launch_package(self) -> str:
        return str(self.device.config.get("launch_package", self.DEFAULT_LAUNCH_PACKAGE))

    def _launch_file(self) -> str:
        return str(self.device.config.get("launch_file", self.DEFAULT_LAUNCH_FILE))

    def _runtime_arguments(self) -> dict[str, object]:
        arguments = self._raw_launch_arguments()
        arguments.setdefault("trackers_config", str(self._trackers_config_path()))
        return arguments

    def _build_command(self) -> list[str]:
        command = [
            "ros2",
            "launch",
            self._launch_package(),
            self._launch_file(),
        ]
        for key, value in self._runtime_arguments().items():
            command.append(f"{key}:={self._value_to_string(value)}")
        return command

    def _value_to_string(self, value: object) -> str:
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

    def _trackers_config_path(self) -> Path:
        raw_value = self.device.config.get("trackers_config")
        if raw_value:
            return Path(str(raw_value)).expanduser()

        launch_argument_value = self._raw_launch_arguments().get("trackers_config")
        if launch_argument_value:
            return Path(str(launch_argument_value)).expanduser()

        package_share = Path(get_package_share_directory("htc_system"))
        return package_share / self.DEFAULT_TRACKERS_CONFIG_RELATIVE_PATH

    def _raw_launch_arguments(self) -> dict[str, object]:
        raw_arguments = self.device.config.get("launch_arguments", {})
        if isinstance(raw_arguments, dict):
            return dict(raw_arguments)
        return {}

    def _matches_tracker_process(self, process_name: str, cmd: list[str]) -> bool:
        executable_markers = {"tracker_publisher", "tracker_publisher.exe"}
        if process_name in executable_markers:
            return True
        joined = " ".join(cmd)
        return any(marker in joined for marker in executable_markers)

    def _start_launch_process(self) -> AdapterResult:
        self._cleanup_process_state()
        self._frame_last_seen_monotonic.clear()
        self._launch_command = self._build_command()

        use_managed_launch = bool(self.device.config.get("use_managed_launch", False))
        if use_managed_launch and self.launch_manager is not None:
            managed_launch = ManagedLaunchSession(
                launch_manager=self.launch_manager,
                label=f"{self.device.device_id}_htc",
                launch_file_path=self._launch_file_path(),
                launch_arguments=self._runtime_arguments(),
                process_matcher=self._matches_tracker_process,
                logger=None if self.node is None else self.node.get_logger(),
            )
            self._managed_launch = managed_launch
            self._log_path = Path(managed_launch.log_path)
            try:
                managed_launch.start()
            except Exception as exc:
                self._cleanup_process_state()
                return AdapterResult.failed(
                    f"{self.device.device_id}: failed to start managed HTC launch: {exc}"
                )

            managed_launch.wait_started(timeout_sec=2.0)
            if not managed_launch.is_running():
                return AdapterResult.failed(
                    f"{self.device.device_id}: HTC launch exited early.\n{self._diagnostic_summary()}"
                )

            return AdapterResult.ok(
                f"{self.device.device_id}: HTC launch started.",
                metadata={
                    **managed_launch.metadata(),
                    "log_path": str(self._log_path),
                    "command": self._launch_command,
                },
            )

        log_dir = Path(tempfile.mkdtemp(prefix="htc_adapter_"))
        self._log_path = log_dir / "launch.log"
        self._log_handle = self._log_path.open("w", encoding="utf-8")

        try:
            self._process = subprocess.Popen(
                self._launch_command,
                stdout=self._log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as exc:
            self._cleanup_process_state()
            return AdapterResult.failed(f"{self.device.device_id}: failed to start HTC launch: {exc}")

        time.sleep(0.5)
        if self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: HTC launch exited early.\n{self._diagnostic_summary()}"
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: HTC launch started.",
            metadata={
                "pid": self._process.pid,
                "log_path": str(self._log_path),
                "command": self._launch_command,
            },
        )

    def _expected_frames(self, trackers_config: Path) -> list[str]:
        override = self.device.config.get("expected_frames")
        if isinstance(override, list) and override:
            return [str(frame) for frame in override]

        with trackers_config.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        params = data.get("tracker_publisher", {}).get("ros__parameters", {})
        frame_ids = params.get("trackers", {}).get("frame_ids", [])
        return [str(frame) for frame in frame_ids if str(frame).strip()]

    def _ready_timeout_sec(self, timeout_sec: float) -> float:
        configured = self.device.config.get("ready_timeout_sec", 15.0)
        if timeout_sec > 0.0:
            return timeout_sec
        try:
            return float(configured)
        except (TypeError, ValueError):
            return 15.0

    def _diagnose_timeout_sec(self) -> float:
        configured = self.device.config.get("diagnose_timeout_sec", 2.0)
        try:
            return max(0.2, float(configured))
        except (TypeError, ValueError):
            return 2.0

    def _on_tf_message(self, msg: TFMessage) -> None:
        now = time.monotonic()
        for transform in msg.transforms:
            child_frame_id = str(transform.child_frame_id).strip()
            if child_frame_id:
                self._frame_last_seen_monotonic[child_frame_id] = now

    def _fresh_frames(self, expected_frames: list[str]) -> list[str]:
        freshness_timeout_sec = self._diagnose_timeout_sec()
        now = time.monotonic()
        published_frames = []
        for frame in expected_frames:
            last_seen = self._frame_last_seen_monotonic.get(frame)
            if last_seen is not None and now - last_seen <= freshness_timeout_sec:
                published_frames.append(frame)
        return published_frames

    def _read_tf_message(self, timeout_sec: float) -> str:
        try:
            completed = subprocess.run(
                ["ros2", "topic", "echo", "--once", "/tf"],
                capture_output=True,
                text=True,
                timeout=timeout_sec,
            )
        except (OSError, subprocess.TimeoutExpired):
            return ""
        if completed.returncode != 0:
            return ""
        return completed.stdout

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
