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
from sensor_msgs.msg import Image
import yaml

from ..managed_launch import ManagedLaunchSession
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
        self._managed_launch: ManagedLaunchSession | None = None
        self._log_path: Path | None = None
        self._launch_command: list[str] = []
        self._last_image_monotonic = 0.0
        self._image_subscription = None
        self._subscribed_image_topic = ""
        self._ready_relaunch_count = 0

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
        if self._managed_launch is not None and self._managed_launch.is_running():
            return AdapterResult.ok(f"{self.device.device_id}: camera bringup already running.")
        if self._process is not None and self._process.poll() is None:
            return AdapterResult.ok(f"{self.device.device_id}: camera bringup already running.")

        self._ready_relaunch_count = 0
        return self._start_launch_process()

    def wait_ready(self, timeout_sec: float) -> AdapterResult:
        cameras_config = self._cameras_config_path()
        camera_cfg = self._camera_cfg(cameras_config)
        image_topic = self._image_topic(camera_cfg)
        self._ensure_image_subscription(image_topic)
        ready_timeout_sec = self._ready_timeout_sec(timeout_sec)
        early_restart_timeout_sec = self._startup_no_image_restart_timeout_sec(camera_cfg)
        relaunch_budget = self._ready_relaunch_attempts(camera_cfg)

        while True:
            process = self._process
            managed_launch = self._managed_launch
            if process is None and managed_launch is None:
                return AdapterResult.failed(
                    f"{self.device.device_id}: camera launch was not started."
                )

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
                        f"{self.device.device_id}: camera launch exited before READY.\n"
                        f"{self._diagnostic_summary()}"
                    )
                    break

                if self._has_recent_image(max(1.0, self._diagnose_timeout_sec())):
                    return AdapterResult.ok(
                        f"{self.device.device_id}: image topic READY at {image_topic}.",
                        metadata={
                            "driver": camera_cfg["driver"],
                            "image_topic": image_topic,
                            "log_path": None if self._log_path is None else str(self._log_path),
                        },
                    )

                early_restart_reason = self._early_restart_reason_from_log(camera_cfg)
                if relaunch_budget > 0 and early_restart_reason:
                    restart_result = self._restart_launch_for_ready_failure(
                        reason=early_restart_reason,
                    )
                    if restart_result.is_failure():
                        return AdapterResult.failed(
                            f"{self.device.device_id}: camera early auto-restart failed. "
                            f"{restart_result.summary}"
                        )
                    relaunch_budget -= 1
                    restarted_early = True
                    break

                if (
                    relaunch_budget > 0
                    and early_restart_timeout_sec > 0.0
                    and time.monotonic() - attempt_started_monotonic >= early_restart_timeout_sec
                ):
                    restart_result = self._restart_launch_for_ready_failure(
                        reason=(
                            f"no image received on {image_topic} within "
                            f"{early_restart_timeout_sec:.1f}s during startup"
                        ),
                    )
                    if restart_result.is_failure():
                        return AdapterResult.failed(
                            f"{self.device.device_id}: camera early auto-restart failed. "
                            f"{restart_result.summary}"
                        )
                    relaunch_budget -= 1
                    restarted_early = True
                    break
                time.sleep(0.1)
            else:
                failure_summary = (
                    f"{self.device.device_id}: camera READY timeout after "
                    f"{ready_timeout_sec:.1f}s waiting for {image_topic}.\n"
                    f"{self._diagnostic_summary()}"
                )

            if restarted_early:
                continue

            if relaunch_budget <= 0:
                return AdapterResult.failed(failure_summary)

            relaunch_budget -= 1
            restart_result = self._restart_launch_for_ready_failure(
                reason=(
                    f"no image received on {image_topic} during startup"
                    if launch_running
                    else "camera launch exited before READY"
                ),
            )
            if restart_result.is_failure():
                return AdapterResult.failed(
                    f"{failure_summary}\nAuto-restart failed: {restart_result.summary}"
                )

    def shutdown(self) -> AdapterResult:
        managed_launch = self._managed_launch
        if managed_launch is not None:
            if not managed_launch.is_running():
                exit_code = managed_launch.launch_exit_code()
                self._cleanup_process_state()
                return AdapterResult.ok(
                    f"{self.device.device_id}: camera launch already exited with code {exit_code}."
                )

            if not managed_launch.shutdown(timeout_sec=12.0):
                return AdapterResult.failed(
                    f"{self.device.device_id}: managed camera launch did not stop within timeout."
                )

            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: camera launch stopped.")

        process = self._process
        if process is None:
            self._cleanup_process_state()
            return AdapterResult.ok(f"{self.device.device_id}: camera launch already stopped.")

        if process.poll() is not None:
            exit_code = process.returncode
            self._cleanup_process_state()
            return AdapterResult.ok(
                f"{self.device.device_id}: camera launch already exited with code {exit_code}."
            )

        if not self._signal_process_group(process, signal.SIGINT, timeout_sec=8.0):
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=4.0)
        if not self._wait_for_process_group_exit(process.pid, timeout_sec=2.0):
            self._signal_process_group(process, signal.SIGTERM, timeout_sec=4.0)
        if not self._wait_for_process_group_exit(process.pid, timeout_sec=2.0):
            self._signal_process_group(process, signal.SIGKILL, timeout_sec=2.0)
        self._wait_for_process_group_exit(process.pid, timeout_sec=1.0)

        self._cleanup_process_state()
        return AdapterResult.ok(f"{self.device.device_id}: camera launch stopped.")

    def diagnose(self) -> AdapterResult:
        if self._managed_launch is None and self._process is None:
            return AdapterResult.failed(f"{self.device.device_id}: camera launch not started.")
        if self._managed_launch is not None and not self._managed_launch.is_running():
            return AdapterResult.failed(
                f"{self.device.device_id}: camera launch exited with code {self._managed_launch.launch_exit_code()}."
            )
        if self._managed_launch is None and self._process.poll() is not None:
            return AdapterResult.failed(
                f"{self.device.device_id}: camera launch exited with code {self._process.returncode}."
            )

        cameras_config = self._cameras_config_path()
        camera_cfg = self._camera_cfg(cameras_config)
        image_topic = self._image_topic(camera_cfg)
        self._ensure_image_subscription(image_topic)
        if not self._has_recent_image(self._diagnose_timeout_sec()):
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
        if self._managed_launch is not None:
            metadata.update(self._managed_launch.metadata())
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

    def _launch_file_path(self, camera_cfg: dict) -> str:
        package_share = Path(get_package_share_directory(self.DEFAULT_LAUNCH_PACKAGE))
        return str(package_share / "bringup" / "launch" / self._launch_file(camera_cfg))

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

    def _ready_relaunch_attempts(self, camera_cfg: dict) -> int:
        configured = self.device.config.get("ready_relaunch_attempts")
        if configured is not None:
            try:
                return max(0, int(configured))
            except (TypeError, ValueError):
                return 0

        driver = str(camera_cfg.get("driver", "")).strip().lower()
        if driver == "realsense":
            return 1
        return 0

    def _startup_no_image_restart_timeout_sec(self, camera_cfg: dict) -> float:
        configured = self.device.config.get("startup_no_image_restart_timeout_sec")
        if configured is not None:
            try:
                return max(0.0, float(configured))
            except (TypeError, ValueError):
                return 0.0

        driver = str(camera_cfg.get("driver", "")).strip().lower()
        if driver == "realsense":
            return 12.0
        return 0.0

    def _early_restart_reason_from_log(self, camera_cfg: dict) -> str:
        driver = str(camera_cfg.get("driver", "")).strip().lower()
        if driver != "realsense":
            return ""

        log_tail = self._read_log_tail(max_lines=80)
        if not log_tail:
            return ""
        if "Frames didn't arrived within 5 seconds" in log_tail:
            return "realsense started but RGB frames did not arrive on first launch"
        if "Frames Timeout" in log_tail:
            return "realsense reported frame timeout on first launch"
        return ""

    def _ready_relaunch_backoff_sec(self) -> float:
        configured = self.device.config.get("ready_relaunch_backoff_sec", 1.0)
        try:
            return max(0.0, float(configured))
        except (TypeError, ValueError):
            return 1.0

    def _diagnose_timeout_sec(self) -> float:
        configured = self.device.config.get("diagnose_timeout_sec", 1.5)
        try:
            return max(0.1, float(configured))
        except (TypeError, ValueError):
            return 1.5

    def _ensure_image_subscription(self, image_topic: str) -> None:
        if self.node is None:
            return
        if self._subscribed_image_topic == image_topic and self._image_subscription is not None:
            return
        self._image_subscription = self.node.create_subscription(
            Image,
            image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )
        self._subscribed_image_topic = image_topic

    def _on_image(self, msg: Image) -> None:
        del msg
        self._last_image_monotonic = time.monotonic()

    def _has_recent_image(self, freshness_timeout_sec: float) -> bool:
        if self._last_image_monotonic <= 0.0:
            return False
        return time.monotonic() - self._last_image_monotonic <= freshness_timeout_sec

    def _raw_launch_arguments(self) -> dict[str, object]:
        raw_arguments = self.device.config.get("launch_arguments", {})
        if isinstance(raw_arguments, dict):
            return dict(raw_arguments)
        return {}

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
        self._last_image_monotonic = 0.0
        cameras_config = self._cameras_config_path()
        camera_cfg = self._camera_cfg(cameras_config)
        self._launch_command = self._build_command()

        use_managed_launch = bool(self.device.config.get("use_managed_launch", False))
        if use_managed_launch and self.launch_manager is not None:
            launch_arguments = self._runtime_arguments(camera_cfg)
            managed_launch = ManagedLaunchSession(
                launch_manager=self.launch_manager,
                label=f"{self.device.device_id}_camera",
                launch_file_path=self._launch_file_path(camera_cfg),
                launch_arguments=launch_arguments,
                process_matcher=lambda process_name, cmd: self.device.device_id in " ".join(cmd),
                logger=None if self.node is None else self.node.get_logger(),
            )
            self._managed_launch = managed_launch
            self._log_path = Path(managed_launch.log_path)
            try:
                managed_launch.start()
            except Exception as exc:
                self._cleanup_process_state()
                return AdapterResult.failed(
                    f"{self.device.device_id}: failed to start managed camera launch: {exc}"
                )

            managed_launch.wait_started(timeout_sec=2.0)
            if not managed_launch.is_running():
                return AdapterResult.failed(
                    f"{self.device.device_id}: camera launch exited early.\n{self._diagnostic_summary()}"
                )

            return AdapterResult.ok(
                f"{self.device.device_id}: camera launch started.",
                metadata={
                    **managed_launch.metadata(),
                    "log_path": str(self._log_path),
                    "command": self._launch_command,
                },
            )

        log_dir = Path(tempfile.mkdtemp(prefix="camera_adapter_"))
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

    def _restart_launch_for_ready_failure(self, reason: str) -> AdapterResult:
        self._ready_relaunch_count += 1
        if self.node is not None:
            self.node.get_logger().warn(
                f"{self.device.device_id}: camera not READY on first launch; "
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
