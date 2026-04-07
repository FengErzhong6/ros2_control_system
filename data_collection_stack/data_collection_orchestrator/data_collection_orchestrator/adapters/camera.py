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
from rclpy.qos import qos_profile_sensor_data
from rclpy.wait_for_message import wait_for_message
from sensor_msgs.msg import Image
import yaml

from .base import AdapterBase, AdapterResult


class CameraAdapter(AdapterBase):
    DEFAULT_LAUNCH_PACKAGE = "camera_system"
    ORBBEC_LAUNCH_FILE = "single_orbbec.launch.py"
    REALSENSE_LAUNCH_FILE = "single_realsense.launch.py"
    DEFAULT_SITE_CONFIG_RELATIVE_PATH = "cameras.yaml"

    def __init__(self, device, node=None) -> None:
        super().__init__(device, node=node)
        self._process: subprocess.Popen | None = None
        self._log_handle = None
        self._log_path: Path | None = None
        self._launch_command: list[str] = []

    def precheck(self) -> AdapterResult:
        if shutil.which("ros2") is None:
            return AdapterResult.failed("ros2 CLI not found in PATH.")

        cameras_config = self._cameras_config_path()
        if not cameras_config.exists():
            return AdapterResult.failed(
                f"{self.device.device_id}: cameras config does not exist: {cameras_config}"
            )

        camera_cfg = self._camera_cfg(cameras_config)
        launch_file = self._launch_file(camera_cfg)
        image_topic = self._image_topic(camera_cfg)
        return AdapterResult.ok(
            f"{self.device.device_id}: camera launch precheck passed.",
            metadata={
                "cameras_config": str(cameras_config),
                "driver": camera_cfg["driver"],
                "launch_file": launch_file,
                "image_topic": image_topic,
            },
        )

    def bringup(self) -> AdapterResult:
        if self._process is not None and self._process.poll() is None:
            return AdapterResult.ok(f"{self.device.device_id}: camera bringup already running.")

        self._cleanup_process_state()
        log_dir = Path(tempfile.mkdtemp(prefix="camera_adapter_"))
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
            return AdapterResult.failed(
                f"{self.device.device_id}: failed to start camera launch: {exc}"
            )

        time.sleep(0.5)
        if self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: camera launch exited early.\n{self._diagnostic_summary()}"
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: camera launch started.",
            metadata={
                "pid": self._process.pid,
                "log_path": str(self._log_path),
                "command": self._launch_command,
            },
        )

    def wait_ready(self, timeout_sec: float) -> AdapterResult:
        if self._process is None:
            return AdapterResult.failed(f"{self.device.device_id}: camera launch was not started.")

        cameras_config = self._cameras_config_path()
        camera_cfg = self._camera_cfg(cameras_config)
        image_topic = self._image_topic(camera_cfg)
        deadline = time.monotonic() + self._ready_timeout_sec(timeout_sec)

        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                return AdapterResult.failed(
                    f"{self.device.device_id}: camera launch exited before READY.\n"
                    f"{self._diagnostic_summary()}"
                )

            if self._topic_has_message(image_topic, timeout_sec=2):
                return AdapterResult.ok(
                    f"{self.device.device_id}: image topic READY at {image_topic}.",
                    metadata={
                        "driver": camera_cfg["driver"],
                        "image_topic": image_topic,
                        "log_path": None if self._log_path is None else str(self._log_path),
                    },
                )
            time.sleep(0.3)

        return AdapterResult.failed(
            f"{self.device.device_id}: camera READY timeout after "
            f"{self._ready_timeout_sec(timeout_sec):.1f}s waiting for {image_topic}.\n"
            f"{self._diagnostic_summary()}"
        )

    def shutdown(self) -> AdapterResult:
        if self._process is None:
            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: camera launch already stopped.")

        if self._process.poll() is not None:
            exit_code = self._process.returncode
            self._cleanup_process_state()
            return AdapterResult.ok(
                f"{self.device.device_id}: camera launch already exited with code {exit_code}."
            )

        if not self._signal_process_group(signal.SIGINT, timeout_sec=8.0):
            self._signal_process_group(signal.SIGTERM, timeout_sec=4.0)
        if self._process is not None and self._process.poll() is None:
            self._signal_process_group(signal.SIGKILL, timeout_sec=2.0)

        self._cleanup_process_state()
        return AdapterResult.ok(f"{self.device.device_id}: camera launch stopped.")

    def diagnose(self) -> AdapterResult:
        if self._process is None:
            return AdapterResult.failed(f"{self.device.device_id}: camera launch not started.")
        if self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: camera launch exited with code {self._process.returncode}."
            )

        cameras_config = self._cameras_config_path()
        camera_cfg = self._camera_cfg(cameras_config)
        image_topic = self._image_topic(camera_cfg)
        if not self._topic_has_message(image_topic, timeout_sec=self._diagnose_timeout_sec()):
            return AdapterResult.degraded(
                f"{self.device.device_id}: no recent image received on {image_topic}.",
                metadata={
                    "image_topic": image_topic,
                    "driver": camera_cfg["driver"],
                    "log_path": None if self._log_path is None else str(self._log_path),
                },
            )

        return AdapterResult.ok(
            f"{self.device.device_id}: preview/record path healthy at {image_topic}.",
            metadata={
                "image_topic": image_topic,
                "driver": camera_cfg["driver"],
                "log_path": None if self._log_path is None else str(self._log_path),
                "command": self._launch_command,
            },
        )

    def dump_metadata(self) -> dict:
        metadata = super().dump_metadata()
        cameras_config = self._cameras_config_path()
        camera_cfg = self._camera_cfg(cameras_config)
        metadata.update(
            {
                "cameras_config": str(cameras_config),
                "driver": camera_cfg["driver"],
                "namespace": camera_cfg["namespace"],
                "launch_file": self._launch_file(camera_cfg),
                "image_topic": self._image_topic(camera_cfg),
                "preview_topics": self.preview_topics(),
                "record_topics": self.record_topics(),
                "log_path": None if self._log_path is None else str(self._log_path),
            }
        )
        return metadata

    def record_topics(self) -> list[str]:
        camera_cfg = self._camera_cfg(self._cameras_config_path())
        topics = camera_cfg.get("record_topics")
        if isinstance(topics, list) and topics:
            return [str(topic) for topic in topics]
        return [self._image_topic(camera_cfg)]

    def preview_topics(self) -> list[str]:
        camera_cfg = self._camera_cfg(self._cameras_config_path())
        preview_topic = camera_cfg.get("preview_topic")
        if isinstance(preview_topic, str) and preview_topic.strip():
            return [preview_topic]
        return [self._image_topic(camera_cfg)]

    def _build_command(self) -> list[str]:
        cameras_config = self._cameras_config_path()
        camera_cfg = self._camera_cfg(cameras_config)
        runtime_arguments = self._runtime_arguments(camera_cfg)
        command = [
            "ros2",
            "launch",
            self.DEFAULT_LAUNCH_PACKAGE,
            self._launch_file(camera_cfg),
        ]
        for key, value in runtime_arguments.items():
            command.append(f"{key}:={self._value_to_string(value)}")
        return command

    def _runtime_arguments(self, camera_cfg: dict) -> dict[str, object]:
        arguments = self._raw_launch_arguments()
        arguments.setdefault("camera_name", self.device.device_id)
        arguments.setdefault("cameras_config", str(self._cameras_config_path()))
        arguments.setdefault("use_showimage", False)
        arguments.setdefault("respawn", False)
        if camera_cfg["driver"] == "orbbec":
            pkg_share = Path(get_package_share_directory("camera_system"))
            arguments.setdefault(
                "orbbec_defaults_config",
                str(pkg_share / "bringup" / "config" / "orbbec_defaults.yaml"),
            )
        else:
            pkg_share = Path(get_package_share_directory("camera_system"))
            arguments.setdefault(
                "realsense_defaults_config",
                str(pkg_share / "bringup" / "config" / "realsense_defaults.yaml"),
            )
        return arguments

    def _value_to_string(self, value: object) -> str:
        if isinstance(value, bool):
            return "true" if value else "false"
        return str(value)

    def _cameras_config_path(self) -> Path:
        raw_value = self.device.config.get("cameras_config")
        if raw_value:
            return Path(str(raw_value)).expanduser()

        package_share = Path(get_package_share_directory("camera_system"))
        return package_share / "bringup" / "config" / "cameras.yaml"

    def _camera_cfg(self, cameras_config: Path) -> dict:
        with cameras_config.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        camera_cfg = data.get("cameras", {}).get(self.device.device_id)
        if not isinstance(camera_cfg, dict):
            raise RuntimeError(
                f"Camera '{self.device.device_id}' not found in cameras config {cameras_config}"
            )
        return camera_cfg

    def _launch_file(self, camera_cfg: dict) -> str:
        driver = str(camera_cfg.get("driver", "")).strip()
        if driver == "orbbec":
            return self.ORBBEC_LAUNCH_FILE
        if driver == "realsense":
            return self.REALSENSE_LAUNCH_FILE
        raise RuntimeError(f"Unsupported camera driver for {self.device.device_id}: {driver}")

    def _image_topic(self, camera_cfg: dict) -> str:
        if isinstance(camera_cfg.get("record_topics"), list) and camera_cfg["record_topics"]:
            first_topic = str(camera_cfg["record_topics"][0])
            if first_topic.strip():
                return first_topic

        namespace = str(camera_cfg["namespace"])
        driver = str(camera_cfg["driver"])
        if driver == "orbbec":
            return f"/{namespace}/image_raw"
        return f"/{namespace}/{namespace}/color/image_raw"

    def _ready_timeout_sec(self, timeout_sec: float) -> float:
        configured = self.device.config.get("ready_timeout_sec", 20.0)
        if timeout_sec > 0.0:
            return timeout_sec
        try:
            return float(configured)
        except (TypeError, ValueError):
            return 20.0

    def _diagnose_timeout_sec(self) -> float:
        configured = self.device.config.get("diagnose_timeout_sec", 0.75)
        try:
            return max(0.1, float(configured))
        except (TypeError, ValueError):
            return 0.75

    def _topic_has_message(self, topic_name: str, timeout_sec: float) -> bool:
        if self.node is not None:
            try:
                received, _ = wait_for_message(
                    Image,
                    self.node,
                    topic_name,
                    qos_profile=qos_profile_sensor_data,
                    time_to_wait=timeout_sec,
                )
                return bool(received)
            except Exception:
                return False

        try:
            completed = subprocess.run(
                [
                    "ros2",
                    "topic",
                    "echo",
                    "--once",
                    "--timeout",
                    str(int(timeout_sec)),
                    "--field",
                    "header.stamp",
                    topic_name,
                ],
                capture_output=True,
                text=True,
                timeout=timeout_sec + 1.0,
            )
        except (OSError, subprocess.TimeoutExpired):
            return False
        return completed.returncode == 0 and bool(completed.stdout.strip())

    def _raw_launch_arguments(self) -> dict[str, object]:
        raw_arguments = self.device.config.get("launch_arguments", {})
        if isinstance(raw_arguments, dict):
            return dict(raw_arguments)
        return {}

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
