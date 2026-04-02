from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import threading
import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from data_collection_interfaces.msg import DeviceState, FaultEvent, SystemState
import yaml

from .view_model import CameraStreamConfig, UiRuntimeConfig


def _load_yaml_map(path: Path) -> dict:
    if not path.exists():
        return {}

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected YAML mapping at root: {path}")
    return data


@dataclass(frozen=True)
class SystemStateSnapshot:
    system_state: str = "IDLE"
    recipe_id: str = ""
    active_session_id: str = ""
    summary: str = ""
    allowed_commands: tuple[str, ...] = ()
    devices: tuple["DeviceSnapshot", ...] = ()
    active_faults: tuple["FaultSnapshot", ...] = ()


@dataclass(frozen=True)
class DeviceSnapshot:
    device_id: str
    device_class: str
    lifecycle_state: int
    health_state: int
    is_required: bool
    summary: str
    last_ready_stamp_sec: float | None


@dataclass(frozen=True)
class FaultSnapshot:
    fault_id: str
    device_id: str
    severity: int
    summary: str
    detail: str
    stamp_sec: float | None


@dataclass(frozen=True)
class CameraStreamSnapshot:
    camera_id: str
    title: str
    preview_topic: str
    generation: int
    image_msg: Image | None
    rx_fps: float
    header_stamp_sec: float | None


@dataclass
class _CameraStreamRuntime:
    config: CameraStreamConfig
    latest_message: Image | None = None
    generation: int = 0
    rx_window_start: float = 0.0
    rx_window_frames: int = 0
    rx_fps: float = 0.0
    last_receive_monotonic: float = 0.0


class RosClient:
    def __init__(self, args: list[str] | None = None) -> None:
        rclpy.init(args=args)
        self._node = Node("data_collection_ui")
        self._declare_parameters()
        self._config = self._load_runtime_config()
        self._lock = threading.Lock()
        self._system_state = SystemStateSnapshot(recipe_id=self._config.recipe_id)
        self._camera_runtimes = {
            stream.camera_id: _CameraStreamRuntime(config=stream, rx_window_start=time.monotonic())
            for stream in self._config.camera_streams
        }
        self._subscriptions = [
            self._node.create_subscription(SystemState, "system_state", self._on_system_state, 10)
        ]
        for stream in self._config.camera_streams:
            self._subscriptions.append(
                self._node.create_subscription(
                    Image,
                    stream.preview_topic,
                    lambda msg, camera_id=stream.camera_id: self._on_image(camera_id, msg),
                    qos_profile_sensor_data,
                )
            )
        self._node.get_logger().info(
            "Preview UI configured. "
            f"recipe={self._config.recipe_id} refresh_hz={self._config.refresh_hz} "
            f"camera_topics={[stream.preview_topic for stream in self._config.camera_streams]}"
        )

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._spin_executor,
            name="data_collection_ui_ros_spin",
            daemon=True,
        )
        self._executor_thread.start()

    @property
    def config(self) -> UiRuntimeConfig:
        return self._config

    def shutdown(self) -> None:
        executor = getattr(self, "_executor", None)
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        thread = getattr(self, "_executor_thread", None)
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        node = getattr(self, "_node", None)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()

    def system_state_snapshot(self) -> SystemStateSnapshot:
        with self._lock:
            return self._system_state

    def camera_stream_snapshots(self) -> list[CameraStreamSnapshot]:
        now = time.monotonic()
        with self._lock:
            snapshots = []
            for stream in self._config.camera_streams:
                runtime = self._camera_runtimes[stream.camera_id]
                rx_fps = runtime.rx_fps
                if runtime.last_receive_monotonic == 0.0 or now - runtime.last_receive_monotonic > 1.5:
                    rx_fps = 0.0
                elif runtime.rx_window_frames > 0:
                    elapsed = max(now - runtime.rx_window_start, 1e-3)
                    rx_fps = max(rx_fps, runtime.rx_window_frames / elapsed)

                header_stamp_sec = None
                if runtime.latest_message is not None:
                    stamp = runtime.latest_message.header.stamp
                    header_stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9

                snapshots.append(
                    CameraStreamSnapshot(
                        camera_id=stream.camera_id,
                        title=stream.title,
                        preview_topic=stream.preview_topic,
                        generation=runtime.generation,
                        image_msg=runtime.latest_message,
                        rx_fps=rx_fps,
                        header_stamp_sec=header_stamp_sec,
                    )
                )
        return snapshots

    def _declare_parameters(self) -> None:
        defaults = self._default_paths()
        self._node.declare_parameter("recipe_id", "marvin_tracker_collection")
        self._node.declare_parameter("recipe_directory", str(defaults["recipe_directory"]))
        self._node.declare_parameter("cameras_config", str(defaults["cameras_config"]))
        self._node.declare_parameter("ui_config", str(defaults["ui_config"]))

    def _default_paths(self) -> dict[str, Path]:
        try:
            bringup_share = Path(get_package_share_directory("data_collection_bringup"))
        except PackageNotFoundError:
            bringup_share = (
                Path(__file__).resolve().parents[2] / "data_collection_bringup"
            )
        try:
            camera_share = Path(get_package_share_directory("camera_system"))
        except PackageNotFoundError:
            camera_share = Path(__file__).resolve().parents[3] / "camera_system"

        return {
            "recipe_directory": bringup_share / "config" / "recipes",
            "cameras_config": camera_share / "bringup" / "config" / "cameras.yaml",
            "ui_config": bringup_share / "config" / "session" / "ui.yaml",
        }

    def _load_runtime_config(self) -> UiRuntimeConfig:
        recipe_id = str(self._node.get_parameter("recipe_id").value)
        recipe_directory = Path(str(self._node.get_parameter("recipe_directory").value)).expanduser()
        cameras_config = Path(str(self._node.get_parameter("cameras_config").value)).expanduser()
        ui_config_path = Path(str(self._node.get_parameter("ui_config").value)).expanduser()

        ui_data = _load_yaml_map(ui_config_path)
        refresh_hz = max(1, int(ui_data.get("refresh_hz", 20)))
        camera_grid_columns = max(1, int(ui_data.get("camera_grid_columns", 2)))
        enable_hotkeys = bool(ui_data.get("enable_hotkeys", True))
        window_title = str(ui_data.get("default_window_title", "Data Collection Console"))
        camera_streams = tuple(
            self._load_camera_stream_configs(
                recipe_id=recipe_id,
                recipe_directory=recipe_directory,
                cameras_config=cameras_config,
            )
        )

        return UiRuntimeConfig(
            title=window_title,
            refresh_hz=refresh_hz,
            camera_grid_columns=camera_grid_columns,
            enable_hotkeys=enable_hotkeys,
            camera_streams=camera_streams,
            recipe_id=recipe_id,
        )

    def _load_camera_stream_configs(
        self,
        *,
        recipe_id: str,
        recipe_directory: Path,
        cameras_config: Path,
    ) -> list[CameraStreamConfig]:
        site_cameras = _load_yaml_map(cameras_config).get("cameras", {})
        if not isinstance(site_cameras, dict):
            raise RuntimeError(f"Expected cameras mapping in {cameras_config}")

        recipe_devices = _load_yaml_map(self._resolve_recipe_path(recipe_id, recipe_directory)).get(
            "devices", []
        )
        ordered_camera_ids: list[str] = []
        if isinstance(recipe_devices, list):
            for raw_device in recipe_devices:
                if not isinstance(raw_device, dict):
                    continue
                if str(raw_device.get("adapter", "")).strip() != "camera":
                    continue
                camera_id = str(raw_device.get("id", "")).strip()
                if camera_id:
                    ordered_camera_ids.append(camera_id)

        if not ordered_camera_ids:
            ordered_camera_ids = [str(camera_id) for camera_id in site_cameras.keys()]

        stream_configs: list[CameraStreamConfig] = []
        for camera_id in ordered_camera_ids:
            camera_cfg = site_cameras.get(camera_id)
            if not isinstance(camera_cfg, dict):
                self._node.get_logger().warning(
                    f"Camera '{camera_id}' from recipe not found in site cameras config."
                )
                continue

            preview_topic = self._preview_topic(camera_cfg)
            if not preview_topic:
                self._node.get_logger().warning(
                    f"Camera '{camera_id}' has no preview topic; skipping UI subscription."
                )
                continue

            stream_configs.append(
                CameraStreamConfig(
                    camera_id=camera_id,
                    title=str(camera_cfg.get("title", camera_id)),
                    preview_topic=preview_topic,
                )
            )
        return stream_configs

    def _resolve_recipe_path(self, recipe_id: str, recipe_directory: Path) -> Path:
        candidates = [recipe_directory / recipe_id]
        if not recipe_id.endswith(".yaml"):
            candidates.append(recipe_directory / f"{recipe_id}.yaml")

        for candidate in candidates:
            if candidate.exists():
                return candidate
        raise RuntimeError(f"Recipe '{recipe_id}' not found under {recipe_directory}")

    def _preview_topic(self, camera_cfg: dict) -> str:
        preview_topic = camera_cfg.get("preview_topic")
        if isinstance(preview_topic, str) and preview_topic.strip():
            return preview_topic.strip()

        record_topics = camera_cfg.get("record_topics")
        if isinstance(record_topics, list):
            for topic in record_topics:
                if isinstance(topic, str) and topic.strip():
                    return topic.strip()
        return ""

    def _on_system_state(self, msg: SystemState) -> None:
        snapshot = SystemStateSnapshot(
            system_state=msg.system_state,
            recipe_id=msg.recipe_id,
            active_session_id=msg.active_session_id,
            summary=msg.summary,
            allowed_commands=tuple(msg.allowed_commands),
            devices=tuple(self._device_snapshot(device) for device in msg.devices),
            active_faults=tuple(self._fault_snapshot(fault) for fault in msg.active_faults),
        )
        with self._lock:
            self._system_state = snapshot

    def _on_image(self, camera_id: str, msg: Image) -> None:
        now = time.monotonic()
        with self._lock:
            runtime = self._camera_runtimes.get(camera_id)
            if runtime is None:
                return

            is_first_frame = runtime.generation == 0
            runtime.latest_message = msg
            runtime.generation += 1
            runtime.last_receive_monotonic = now
            runtime.rx_window_frames += 1
            elapsed = now - runtime.rx_window_start
            if elapsed >= 1.0:
                runtime.rx_fps = runtime.rx_window_frames / elapsed
                runtime.rx_window_start = now
                runtime.rx_window_frames = 0
            if is_first_frame:
                self._node.get_logger().info(
                    f"First preview frame received for {camera_id} on "
                    f"{runtime.config.preview_topic}: {msg.width}x{msg.height} {msg.encoding}"
                )

    def _spin_executor(self) -> None:
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass

    def _device_snapshot(self, msg: DeviceState) -> DeviceSnapshot:
        last_ready_stamp_sec = None
        if msg.last_ready_stamp.sec != 0 or msg.last_ready_stamp.nanosec != 0:
            last_ready_stamp_sec = float(msg.last_ready_stamp.sec) + float(
                msg.last_ready_stamp.nanosec
            ) * 1e-9

        return DeviceSnapshot(
            device_id=msg.device_id,
            device_class=msg.device_class,
            lifecycle_state=msg.lifecycle_state,
            health_state=msg.health_state,
            is_required=msg.is_required,
            summary=msg.summary,
            last_ready_stamp_sec=last_ready_stamp_sec,
        )

    def _fault_snapshot(self, msg: FaultEvent) -> FaultSnapshot:
        stamp_sec = None
        if msg.stamp.sec != 0 or msg.stamp.nanosec != 0:
            stamp_sec = float(msg.stamp.sec) + float(msg.stamp.nanosec) * 1e-9

        return FaultSnapshot(
            fault_id=msg.fault_id,
            device_id=msg.device_id,
            severity=msg.severity,
            summary=msg.summary,
            detail=msg.detail,
            stamp_sec=stamp_sec,
        )
