#!/usr/bin/env python3

import os
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import yaml


class CameraState(Enum):
    PENDING = "pending"
    STARTING = "starting"
    WAITING_READY = "waiting_ready"
    READY = "ready"
    RETRYING = "retrying"
    FAILED = "failed"


@dataclass(frozen=True)
class CameraSpec:
    camera_name: str
    driver: str
    ready_topic: str
    launch_file: str


@dataclass
class ManagedCamera:
    spec: CameraSpec
    process: subprocess.Popen


class TrackerCameraStartupManager(Node):
    def __init__(self) -> None:
        super().__init__("tracker_camera_startup_manager")

        self.declare_parameter("cameras_config", "")
        self.declare_parameter("high_camera_name", "cam_high")
        self.declare_parameter("ready_timeout_sec", 15.0)
        self.declare_parameter("max_attempts", 3)
        self.declare_parameter("retry_delay_sec", 1.0)
        self.declare_parameter("monitor_runtime_failures", True)

        self._ros2_bin = shutil.which("ros2")
        if not self._ros2_bin:
            raise RuntimeError("Could not find 'ros2' in PATH")

        self._cameras_config = str(self.get_parameter("cameras_config").value)
        self._high_camera_name = str(self.get_parameter("high_camera_name").value)
        self._ready_timeout_sec = float(self.get_parameter("ready_timeout_sec").value)
        self._max_attempts = int(self.get_parameter("max_attempts").value)
        self._retry_delay_sec = float(self.get_parameter("retry_delay_sec").value)
        self._monitor_runtime_failures = bool(
            self.get_parameter("monitor_runtime_failures").value
        )

        if not self._cameras_config:
            raise RuntimeError("Parameter 'cameras_config' must not be empty")
        if self._ready_timeout_sec <= 0.0:
            raise RuntimeError("Parameter 'ready_timeout_sec' must be positive")
        if self._max_attempts <= 0:
            raise RuntimeError("Parameter 'max_attempts' must be positive")
        if self._retry_delay_sec < 0.0:
            raise RuntimeError("Parameter 'retry_delay_sec' must be non-negative")

        self._managed_cameras: List[ManagedCamera] = []
        self._stop_requested = False

        signal.signal(signal.SIGINT, self._request_stop)
        signal.signal(signal.SIGTERM, self._request_stop)

        cameras_cfg = self._load_yaml(self._cameras_config)
        self._sequence = self._build_camera_sequence(cameras_cfg)

    def _request_stop(self, signum, _frame) -> None:
        self.get_logger().info(
            f"Received signal {signum}; stopping camera startup manager."
        )
        self._stop_requested = True

    def _load_yaml(self, path_str: str) -> dict:
        path = Path(path_str)
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        if not isinstance(data, dict):
            raise RuntimeError(f"Expected mapping at YAML root: {path}")
        return data

    def _build_camera_sequence(self, cameras_cfg: dict) -> List[CameraSpec]:
        cameras = cameras_cfg.get("cameras", {})
        sequence = [
            self._build_camera_spec(cameras, self._high_camera_name),
            self._build_camera_spec(cameras, "cam_left_wrist"),
            self._build_camera_spec(cameras, "cam_right_wrist"),
        ]
        return sequence

    def _build_camera_spec(self, cameras: dict, camera_name: str) -> CameraSpec:
        camera_cfg = cameras.get(camera_name)
        if not isinstance(camera_cfg, dict):
            raise RuntimeError(f"Camera '{camera_name}' not found in cameras config")

        namespace = camera_cfg.get("namespace", camera_name)
        driver = str(camera_cfg.get("driver", "")).strip().lower()
        if driver == "realsense":
            ready_topic = f"/{namespace}/{namespace}/color/image_raw"
            launch_file = "single_realsense.launch.py"
        elif driver == "orbbec":
            ready_topic = f"/{namespace}/image_raw"
            launch_file = "single_orbbec.launch.py"
        else:
            raise RuntimeError(
                f"Camera '{camera_name}' has unsupported driver '{camera_cfg.get('driver')}'"
            )

        return CameraSpec(
            camera_name=camera_name,
            driver=driver,
            ready_topic=ready_topic,
            launch_file=launch_file,
        )

    def run(self) -> int:
        try:
            for spec in self._sequence:
                if self._stop_requested:
                    return 0
                if not self._start_camera_with_retries(spec):
                    return 1

            self.get_logger().info("All tracker cameras reached READY.")

            while rclpy.ok() and not self._stop_requested:
                runtime_failure = self._poll_runtime_failures()
                if runtime_failure and self._monitor_runtime_failures:
                    self.get_logger().error(runtime_failure)
                    return 1
                rclpy.spin_once(self, timeout_sec=0.2)

            return 0
        finally:
            self._shutdown_children()

    def _start_camera_with_retries(self, spec: CameraSpec) -> bool:
        for attempt in range(1, self._max_attempts + 1):
            if self._stop_requested:
                return False

            self._log_state(spec, CameraState.STARTING, attempt)
            process = self._launch_camera(spec)
            ready = False

            try:
                self._log_state(spec, CameraState.WAITING_READY, attempt)
                ready = self._wait_for_first_image(spec, process)
            finally:
                if not ready:
                    self._terminate_process_group(process, reason="startup failed")

            if ready:
                self._managed_cameras.append(ManagedCamera(spec=spec, process=process))
                self._log_state(spec, CameraState.READY, attempt)
                return True

            if attempt < self._max_attempts:
                self._log_state(spec, CameraState.RETRYING, attempt)
                if not self._sleep_with_spin(self._retry_delay_sec):
                    return False

        self._log_state(spec, CameraState.FAILED, self._max_attempts)
        return False

    def _launch_camera(self, spec: CameraSpec) -> subprocess.Popen:
        cmd = [
            self._ros2_bin,
            "launch",
            "camera_system",
            spec.launch_file,
            f"camera_name:={spec.camera_name}",
            f"cameras_config:={self._cameras_config}",
            "use_showimage:=false",
            "respawn:=false",
        ]

        self.get_logger().info(f"[{spec.camera_name}] launch command: {' '.join(cmd)}")
        return subprocess.Popen(
            cmd,
            start_new_session=True,
        )

    def _wait_for_first_image(
        self, spec: CameraSpec, process: subprocess.Popen
    ) -> bool:
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        image_received = {"value": False}

        def on_image(_msg: Image) -> None:
            image_received["value"] = True

        subscription = self.create_subscription(Image, spec.ready_topic, on_image, qos)
        deadline = time.monotonic() + self._ready_timeout_sec

        try:
            while rclpy.ok() and not self._stop_requested:
                if image_received["value"]:
                    return True

                if process.poll() is not None:
                    self.get_logger().error(
                        f"[{spec.camera_name}] launch process exited before READY "
                        f"(code={process.returncode})."
                    )
                    return False

                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        f"[{spec.camera_name}] no image received on {spec.ready_topic} "
                        f"within {self._ready_timeout_sec:.1f}s."
                    )
                    return False

                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.destroy_subscription(subscription)

        return False

    def _sleep_with_spin(self, duration_sec: float) -> bool:
        deadline = time.monotonic() + duration_sec
        while rclpy.ok() and not self._stop_requested and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.1, max(deadline - time.monotonic(), 0.0)))
        return rclpy.ok() and not self._stop_requested

    def _poll_runtime_failures(self) -> Optional[str]:
        for managed in self._managed_cameras:
            if managed.process.poll() is not None:
                return (
                    f"[{managed.spec.camera_name}] camera launch exited unexpectedly "
                    f"after READY (code={managed.process.returncode})."
                )
        return None

    def _shutdown_children(self) -> None:
        for managed in reversed(self._managed_cameras):
            self._terminate_process_group(managed.process, reason="shutdown")
        self._managed_cameras.clear()

    def _terminate_process_group(self, process: subprocess.Popen, *, reason: str) -> None:
        if process.poll() is not None:
            return

        try:
            os.killpg(process.pid, signal.SIGINT)
        except ProcessLookupError:
            return

        deadline = time.monotonic() + 5.0
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.1)

        if process.poll() is None:
            self.get_logger().warn(
                f"Process group {process.pid} did not stop after SIGINT during {reason}; sending SIGTERM."
            )
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                return
            deadline = time.monotonic() + 2.0
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.1)

        if process.poll() is None:
            self.get_logger().error(
                f"Process group {process.pid} did not stop after SIGTERM during {reason}; sending SIGKILL."
            )
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                return
            process.wait(timeout=2.0)

    def _log_state(self, spec: CameraSpec, state: CameraState, attempt: int) -> None:
        self.get_logger().info(
            f"[{spec.camera_name}] state={state.value} attempt={attempt}/{self._max_attempts}"
        )


def main() -> int:
    rclpy.init(args=sys.argv)
    manager: Optional[TrackerCameraStartupManager] = None
    try:
        manager = TrackerCameraStartupManager()
        return manager.run()
    except KeyboardInterrupt:
        return 0
    except Exception as exc:  # pragma: no cover - fatal startup path
        print(f"[tracker_camera_startup_manager] fatal error: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if manager is not None:
            manager.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
