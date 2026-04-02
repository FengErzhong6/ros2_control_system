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
import yaml

from .base import AdapterBase, AdapterResult


class HtcAdapter(AdapterBase):
    DEFAULT_LAUNCH_PACKAGE = "htc_system"
    DEFAULT_LAUNCH_FILE = "tracker_publisher.launch.py"
    DEFAULT_TRACKERS_CONFIG_RELATIVE_PATH = "bringup/config/trackers.yaml"

    def __init__(self, device, node=None) -> None:
        super().__init__(device, node=node)
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._log_path: Path | None = None
        self._launch_command: list[str] = []

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
        if self._process is not None and self._process.poll() is None:
            return AdapterResult.ok(f"{self.device.device_id}: HTC bringup already running.")

        self._cleanup_process_state()
        log_dir = Path(tempfile.mkdtemp(prefix="htc_adapter_"))
        self._log_path = log_dir / "launch.log"
        self._log_handle = self._log_path.open("w", encoding="utf-8")
        self._launch_command = self._build_command()

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

    def wait_ready(self, timeout_sec: float) -> AdapterResult:
        if self._process is None:
            return AdapterResult.failed(f"{self.device.device_id}: HTC launch was not started.")

        trackers_config = self._trackers_config_path()
        expected_frames = self._expected_frames(trackers_config)
        deadline = time.monotonic() + self._ready_timeout_sec(timeout_sec)

        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                return AdapterResult.failed(
                    f"{self.device.device_id}: HTC launch exited before READY.\n"
                    f"{self._diagnostic_summary()}"
                )

            output = self._read_tf_message(timeout_sec=2.0)
            if output:
                published_frames = [
                    frame
                    for frame in expected_frames
                    if f"child_frame_id: {frame}" in output
                ]
                if len(published_frames) == len(expected_frames):
                    return AdapterResult.ok(
                        f"{self.device.device_id}: tracker TF READY with frames {expected_frames}.",
                        metadata={
                            "expected_frames": expected_frames,
                            "published_frames": published_frames,
                            "trackers_config": str(trackers_config),
                        },
                    )

            time.sleep(0.3)

        return AdapterResult.failed(
            f"{self.device.device_id}: HTC READY timeout after "
            f"{self._ready_timeout_sec(timeout_sec):.1f}s waiting for frames {expected_frames}.\n"
            f"{self._diagnostic_summary()}"
        )

    def shutdown(self) -> AdapterResult:
        if self._process is None:
            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: HTC launch already stopped.")

        if self._process.poll() is not None:
            exit_code = self._process.returncode
            self._cleanup_process_state()
            return AdapterResult.ok(
                f"{self.device.device_id}: HTC launch already exited with code {exit_code}."
            )

        if not self._signal_process_group(signal.SIGINT, timeout_sec=8.0):
            self._signal_process_group(signal.SIGTERM, timeout_sec=4.0)
        if self._process is not None and self._process.poll() is None:
            self._signal_process_group(signal.SIGKILL, timeout_sec=2.0)

        self._cleanup_process_state()
        return AdapterResult.ok(f"{self.device.device_id}: HTC launch stopped.")

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
        runtime_arguments = self._runtime_arguments()
        trackers_config = str(self._trackers_config_path())
        return [
            "ros2",
            "run",
            "htc_system",
            "tracker_publisher",
            "--ros-args",
            "--params-file",
            trackers_config,
        ]

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
