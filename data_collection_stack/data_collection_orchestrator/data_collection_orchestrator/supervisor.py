from __future__ import annotations

from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
from copy import deepcopy
from datetime import datetime, timezone
from pathlib import Path
import signal
import threading
import time
from typing import Any, Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml

from data_collection_interfaces.action import (
    GoHome,
    ShutdownSystem,
    StartSession,
    StartSystem,
    StopSession,
)
from data_collection_interfaces.msg import DeviceState, FaultEvent, SystemState
from data_collection_interfaces.srv import (
    AcknowledgeFault,
    GetSystemState,
    ListRecipes,
)

from data_collection_bringup.relay_mapping import relay_output_topic
from .topic_relay_utils import normalize_topic

from .adapters import AdapterResult, create_adapter
from .command_server import CommandServer
from .event_log import EventLog
from .health_monitor import HealthMonitor
from .managed_launch import LaunchManager
from .models import (
    ActiveSession,
    DeviceSpec,
    RecipeSpec,
    RecordingPolicy,
    StartupPolicy,
    SupervisorConfig,
)
from .recipe_loader import discover_recipes, load_recipe
from .record_manager import RecordManager
from .runtime_manifest import RuntimeManifest
from .session_manager import SessionManager
from .state_machine import Commands, SystemStates, allowed_commands_for, is_command_allowed


def _optional_path(raw_value: str) -> Optional[Path]:
    if not raw_value:
        return None
    return Path(raw_value).expanduser()


def _optional_text(raw_value: str) -> Optional[str]:
    value = raw_value.strip()
    if not value:
        return None
    return value


def _coerce_bool(raw_value: object) -> bool:
    if isinstance(raw_value, bool):
        return raw_value
    if isinstance(raw_value, str):
        return raw_value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(raw_value)


def _load_yaml_map(path: Path | None) -> dict:
    if path is None or not path.exists():
        return {}

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected YAML mapping at root: {path}")
    return data


def _nonnegative_float(raw_value: object, default: float) -> float:
    try:
        parsed = float(raw_value)
    except (TypeError, ValueError):
        return default
    if parsed < 0.0:
        return default
    return parsed


def _nonnegative_int(raw_value: object, default: int) -> int:
    try:
        parsed = int(raw_value)
    except (TypeError, ValueError):
        return default
    if parsed < 0:
        return default
    return parsed


class DataCollectionSupervisor(Node):
    def __init__(
        self,
        *,
        launch_manager: LaunchManager | None = None,
        record_manager: RecordManager | None = None,
    ) -> None:
        super().__init__("data_collection_supervisor")

        self.launch_manager = launch_manager
        self._state_lock = threading.RLock()
        self._feedback_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._active_command_name = ""
        self._active_command_started_monotonic = 0.0
        self._config = self._declare_config()
        self._runtime_manifest = RuntimeManifest()
        self._startup_policy = self._load_startup_policy()
        self._recording_policy = self._load_recording_policy()
        self._event_log = EventLog()
        self._health_monitor = HealthMonitor()
        self._session_manager = SessionManager(
            session_root=self._recording_policy.session_root
        )
        self._record_manager = record_manager or RecordManager(self)
        self._recipes = discover_recipes(self._config.recipe_directory)
        self._selected_recipe_id = self._config.recipe_id
        self._current_recipe: RecipeSpec | None = None
        self._current_operator_id = ""
        self._current_site_name = ""
        self._adapters = {}
        self._device_specs: dict[str, DeviceSpec] = {}
        self._devices: dict[str, DeviceState] = {}
        self._state = SystemState()

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(SystemState, "system_state", qos)
        self.create_service(GetSystemState, "get_system_state", self._handle_get_system_state)
        self.create_service(ListRecipes, "list_recipes", self._handle_list_recipes)
        self.create_service(
            AcknowledgeFault,
            "acknowledge_fault",
            self._handle_acknowledge_fault,
        )
        self._command_server = CommandServer(self, self)
        self.create_timer(1.0, self._publish_state)
        self.create_timer(2.0, self._run_health_monitor)

        stale_cleanup_notes = self._runtime_manifest.cleanup_stale_runtime()
        stale_cleanup_notes.extend(self._runtime_manifest.cleanup_known_orphans())
        for note in stale_cleanup_notes:
            self.get_logger().warn(note)
            self._event_log.record(note)

        self._set_system_state(
            SystemStates.IDLE,
            "Phase 1 supervisor ready: services and action servers are available.",
        )
        self._publish_state()

        self._event_log.record("Phase 1 supervisor started.")
        self.get_logger().info(
            "Supervisor ready. "
            f"recipe_id={self._config.recipe_id} recipes={len(self._recipes)} "
            f"actions={self._command_server.describe()} "
            f"startup_retry_count={self._startup_policy.retry_count} "
            f"startup_retry_backoff_sec={self._startup_policy.retry_backoff_sec:.1f}"
        )

    def _declare_config(self) -> SupervisorConfig:
        self.declare_parameter("recipe_id", "marvin_tracker_manus_camera_collection")
        self.declare_parameter("recipe_directory", "")
        self.declare_parameter("startup_policy_config", "")
        self.declare_parameter("fault_policy_config", "")
        self.declare_parameter("recording_config", "")
        self.declare_parameter("cameras_config", "")
        self.declare_parameter("trackers_config", "")
        self.declare_parameter("manus_config", "")
        self.declare_parameter("manus_user_name", "")
        self.declare_parameter("wujihand_identity_file", "")
        self.declare_parameter("wujihand_teleop_config", "")
        self.declare_parameter("wujihand_use_mock_hardware", False)
        self.declare_parameter("mock_manus", False)
        self.declare_parameter("marvin_mock_grippers", False)

        return SupervisorConfig(
            recipe_id=self.get_parameter("recipe_id").value,
            recipe_directory=_optional_path(self.get_parameter("recipe_directory").value),
            startup_policy_config=_optional_path(
                self.get_parameter("startup_policy_config").value
            ),
            fault_policy_config=_optional_path(
                self.get_parameter("fault_policy_config").value
            ),
            recording_config=_optional_path(self.get_parameter("recording_config").value),
            cameras_config=_optional_path(self.get_parameter("cameras_config").value),
            trackers_config=_optional_path(self.get_parameter("trackers_config").value),
            manus_config=_optional_path(self.get_parameter("manus_config").value),
            manus_user_name=_optional_text(self.get_parameter("manus_user_name").value),
            wujihand_identity_file=_optional_path(
                self.get_parameter("wujihand_identity_file").value
            ),
            wujihand_teleop_config=_optional_path(
                self.get_parameter("wujihand_teleop_config").value
            ),
            wujihand_use_mock_hardware=_coerce_bool(
                self.get_parameter("wujihand_use_mock_hardware").value
            ),
            mock_manus=_coerce_bool(self.get_parameter("mock_manus").value),
            marvin_mock_grippers=_coerce_bool(
                self.get_parameter("marvin_mock_grippers").value
            ),
        )

    def _load_startup_policy(self) -> StartupPolicy:
        raw_policy = _load_yaml_map(self._config.startup_policy_config)
        defaults = StartupPolicy()
        return StartupPolicy(
            startup_timeout_sec=_nonnegative_float(
                raw_policy.get("startup_timeout_sec"), defaults.startup_timeout_sec
            ),
            ready_stability_window_sec=_nonnegative_float(
                raw_policy.get("ready_stability_window_sec"),
                defaults.ready_stability_window_sec,
            ),
            retry_count=_nonnegative_int(
                raw_policy.get("retry_count"), defaults.retry_count
            ),
            retry_backoff_sec=_nonnegative_float(
                raw_policy.get("retry_backoff_sec"), defaults.retry_backoff_sec
            ),
        )

    def _load_recording_policy(self) -> RecordingPolicy:
        raw_policy = _load_yaml_map(self._config.recording_config)
        defaults = RecordingPolicy()
        canonical_timestamp_semantics = raw_policy.get(
            "canonical_timestamp_semantics",
            defaults.canonical_timestamp_semantics,
        )
        if not isinstance(canonical_timestamp_semantics, dict):
            canonical_timestamp_semantics = dict(defaults.canonical_timestamp_semantics)
        return RecordingPolicy(
            session_root=_optional_path(str(raw_policy.get("session_root", "")))
            or defaults.session_root,
            bag_directory_name=str(
                raw_policy.get("bag_directory_name", defaults.bag_directory_name)
            ),
            metadata_file_name=str(
                raw_policy.get("metadata_file_name", defaults.metadata_file_name)
            ),
            event_log_file_name=str(
                raw_policy.get("event_log_file_name", defaults.event_log_file_name)
            ),
            timing_report_file_name=str(
                raw_policy.get("timing_report_file_name", defaults.timing_report_file_name)
            ),
            snapshot_recipe=_coerce_bool(
                raw_policy.get("snapshot_recipe", defaults.snapshot_recipe)
            ),
            snapshot_site_config=_coerce_bool(
                raw_policy.get("snapshot_site_config", defaults.snapshot_site_config)
            ),
            snapshot_startup_policy=_coerce_bool(
                raw_policy.get("snapshot_startup_policy", defaults.snapshot_startup_policy)
            ),
            snapshot_fault_policy=_coerce_bool(
                raw_policy.get("snapshot_fault_policy", defaults.snapshot_fault_policy)
            ),
            snapshot_time_sync_config=_coerce_bool(
                raw_policy.get("snapshot_time_sync_config", defaults.snapshot_time_sync_config)
            ),
            snapshot_robot_description=_coerce_bool(
                raw_policy.get("snapshot_robot_description", defaults.snapshot_robot_description)
            ),
            snapshot_camera_calibration=_coerce_bool(
                raw_policy.get("snapshot_camera_calibration", defaults.snapshot_camera_calibration)
            ),
            prepare_recorder_on_start_system=_coerce_bool(
                raw_policy.get(
                    "prepare_recorder_on_start_system",
                    defaults.prepare_recorder_on_start_system,
                )
            ),
            use_start_paused=_coerce_bool(
                raw_policy.get("use_start_paused", defaults.use_start_paused)
            ),
            recorder_node_name_prefix=str(
                raw_policy.get(
                    "recorder_node_name_prefix", defaults.recorder_node_name_prefix
                )
            ),
            service_timeout_sec=_nonnegative_float(
                raw_policy.get("service_timeout_sec"), defaults.service_timeout_sec
            ),
            stop_timeout_sec=_nonnegative_float(
                raw_policy.get("stop_timeout_sec"), defaults.stop_timeout_sec
            ),
            cleanup_timeout_sec=_nonnegative_float(
                raw_policy.get("cleanup_timeout_sec"), defaults.cleanup_timeout_sec
            ),
            storage_id=str(raw_policy.get("storage_id", defaults.storage_id)),
            compression_mode=str(
                raw_policy.get("compression_mode", defaults.compression_mode)
            ),
            compression_format=str(
                raw_policy.get("compression_format", defaults.compression_format)
            ),
            capture_git_state=_coerce_bool(
                raw_policy.get("capture_git_state", defaults.capture_git_state)
            ),
            preview_drop_policy=str(
                raw_policy.get("preview_drop_policy", defaults.preview_drop_policy)
            ),
            require_time_sync_healthy=_coerce_bool(
                raw_policy.get(
                    "require_time_sync_healthy",
                    defaults.require_time_sync_healthy,
                )
            ),
            time_sync_mode=str(raw_policy.get("time_sync_mode", defaults.time_sync_mode)),
            canonical_timestamp_clock=str(
                raw_policy.get(
                    "canonical_timestamp_clock",
                    defaults.canonical_timestamp_clock,
                )
            ),
            canonical_timestamp_semantics={
                str(key): str(value)
                for key, value in canonical_timestamp_semantics.items()
            },
            relay_record_topics=_coerce_bool(
                raw_policy.get("relay_record_topics", defaults.relay_record_topics)
            ),
            relay_record_topic_prefix=str(
                raw_policy.get(
                    "relay_record_topic_prefix", defaults.relay_record_topic_prefix
                )
            ),
            canonical_joint_state_topic=str(
                raw_policy.get(
                    "canonical_joint_state_topic", defaults.canonical_joint_state_topic
                )
            ),
            time_sync_max_offset_ns=_nonnegative_int(
                raw_policy.get("time_sync_max_offset_ns"), defaults.time_sync_max_offset_ns
            ),
            joint_state_min_rate_hz=_nonnegative_float(
                raw_policy.get("joint_state_min_rate_hz"), defaults.joint_state_min_rate_hz
            ),
            joint_state_max_jitter_ms=_nonnegative_float(
                raw_policy.get("joint_state_max_jitter_ms"), defaults.joint_state_max_jitter_ms
            ),
            post_session_timing_validation=_coerce_bool(
                raw_policy.get(
                    "post_session_timing_validation",
                    defaults.post_session_timing_validation,
                )
            ),
            training_ready_requires_canonical_timestamps=_coerce_bool(
                raw_policy.get(
                    "training_ready_requires_canonical_timestamps",
                    defaults.training_ready_requires_canonical_timestamps,
                )
            ),
        )


    def _publish_state(self) -> None:
        with self._state_lock:
            if not rclpy.ok():
                return
            self._state.stamp = self.get_clock().now().to_msg()
            self._state.devices = list(self._devices.values())
            active_session = self._session_manager.active_session
            self._state.active_session_id = (
                "" if active_session is None else active_session.session_id
            )
            state_snapshot = deepcopy(self._state)

        try:
            self._publisher.publish(state_snapshot)
        except Exception:
            return

    def recording_policy(self) -> RecordingPolicy:
        return self._recording_policy

    def _record_topic(self, topic: str) -> str:
        normalized = normalize_topic(topic)
        if not self._recording_policy.relay_record_topics:
            return normalized
        return relay_output_topic(normalized, self._recording_policy.relay_record_topic_prefix)

    def _session_record_topics(self) -> list[str]:
        if self._current_recipe is None:
            return []
        if self._current_recipe.record_topics:
            topics = [self._record_topic(topic) for topic in self._current_recipe.record_topics]
            return self._record_manager.validate_record_topics(topics)

        topics: list[str] = []
        seen: set[str] = set()
        for device in self._current_recipe.devices:
            adapter = self._adapters.get(device.device_id)
            if adapter is None:
                continue
            for topic in adapter.record_topics():
                normalized = self._record_topic(str(topic).strip())
                if not normalized or normalized in seen:
                    continue
                topics.append(normalized)
                seen.add(normalized)
        return self._record_manager.validate_record_topics(topics)

    def _now_wall_time(self) -> str:
        return datetime.now(timezone.utc).astimezone().isoformat()

    def _now_ros_time_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)

    def _time_sync_health_summary(self, topics: list[str]) -> dict[str, Any]:
        image_topics = [topic for topic in topics if "image_raw" in topic]
        has_any_joint_state = any(topic.endswith("/joint_states") for topic in topics)
        canonical_joint_state_topic = normalize_topic(
            self._recording_policy.canonical_joint_state_topic
        )
        has_canonical_joint_states = canonical_joint_state_topic in topics
        healthy = bool(image_topics) and has_any_joint_state
        summary = {
            "healthy": healthy,
            "mode": self._recording_policy.time_sync_mode,
            "required": self._recording_policy.require_time_sync_healthy,
            "camera_topics": image_topics,
            "joint_states_present": has_any_joint_state,
            "canonical_joint_state_topic": canonical_joint_state_topic,
            "canonical_topics_ok": bool(image_topics) and has_canonical_joint_states,
            "joint_state_rate_hz": self._recording_policy.joint_state_min_rate_hz,
            "joint_state_jitter_ms": self._recording_policy.joint_state_max_jitter_ms,
            "max_offset_ns": self._recording_policy.time_sync_max_offset_ns,
            "checks": {
                "pre_session": "passed" if healthy else "failed",
                "post_session": "pending",
            },
        }
        return summary

    def _set_system_state(self, system_state: str, summary: str) -> None:
        with self._state_lock:
            self._state.system_state = system_state
            self._state.summary = summary
            self._state.allowed_commands = allowed_commands_for(system_state)
            self._state.recipe_id = self._selected_recipe_id
            self._event_log.record(f"{system_state}: {summary}")
            self._runtime_manifest.set_context(
                recipe_id=self._selected_recipe_id,
                system_state=system_state,
            )
        self._publish_state()

    def _set_fault(self, summary: str, device_id: str = "") -> None:
        with self._state_lock:
            fault = FaultEvent()
            fault.stamp = self.get_clock().now().to_msg()
            fault.fault_id = f"fault_{len(self._state.active_faults) + 1:04d}"
            fault.device_id = device_id
            fault.severity = FaultEvent.SEVERITY_FATAL
            fault.summary = summary
            fault.detail = summary
            self._state.active_faults = [fault]
        self._set_system_state(SystemStates.FAULT, summary)

    def _clear_faults(self) -> None:
        with self._state_lock:
            self._state.active_faults = []

    def _reset_recipe_runtime(self, shutdown_adapters: bool = False) -> None:
        if shutdown_adapters:
            if self._current_recipe is not None:
                shutdown_failure = self._shutdown_devices()
                if shutdown_failure is not None:
                    self.get_logger().error(
                        f"Shutdown during runtime reset reported failure: {shutdown_failure}"
                    )
            else:
                for adapter in self._adapters.values():
                    try:
                        adapter.shutdown()
                    except Exception as exc:  # pragma: no cover - defensive cleanup path
                        self.get_logger().error(f"Adapter shutdown during reset failed: {exc}")
        self._record_manager.reset_runtime()
        self._adapters = {}
        self._device_specs = {}
        self._devices = {}
        self._current_recipe = None
        self._health_monitor.reset()
        self._session_manager.end_session()
        self._runtime_manifest.clear()
        self._current_operator_id = ""
        self._current_site_name = ""
        self._selected_recipe_id = self._config.recipe_id

    def _load_recipe_or_fail(self, recipe_id: str) -> RecipeSpec | None:
        recipe = load_recipe(self._config.recipe_directory, recipe_id)
        if recipe is None:
            return None
        return recipe

    def _install_recipe(self, recipe: RecipeSpec) -> None:
        with self._state_lock:
            runtime_devices = [self._apply_runtime_config(device) for device in recipe.devices]
            self._current_recipe = RecipeSpec(
                recipe_id=recipe.recipe_id,
                path=recipe.path,
                devices=runtime_devices,
                record_topics=list(recipe.record_topics),
            )
            self._selected_recipe_id = recipe.recipe_id
            self._device_specs = {device.device_id: device for device in runtime_devices}
            self._devices = {
                device.device_id: self._create_device_state(device)
                for device in runtime_devices
            }
            self._adapters = {
                device.device_id: create_adapter(device, node=self)
                for device in runtime_devices
            }
            self._health_monitor.reset()
            self._clear_faults()
            self._runtime_manifest.clear()
            self._runtime_manifest.set_context(
                recipe_id=recipe.recipe_id,
                system_state=self._state.system_state,
            )
        self._publish_state()

    def _apply_runtime_config(self, device: DeviceSpec) -> DeviceSpec:
        config = dict(device.config)

        if device.adapter == "camera" and self._config.cameras_config is not None:
            config.setdefault("cameras_config", str(self._config.cameras_config))

        if device.adapter == "htc" and self._config.trackers_config is not None:
            config.setdefault("trackers_config", str(self._config.trackers_config))

        if device.adapter == "marvin":
            raw_launch_arguments = config.get("launch_arguments")
            launch_arguments = {}
            if isinstance(raw_launch_arguments, dict):
                launch_arguments.update(raw_launch_arguments)
            if self._config.cameras_config is not None:
                launch_arguments.setdefault("cameras_config", str(self._config.cameras_config))
            if self._config.trackers_config is not None:
                launch_arguments.setdefault("trackers_config", str(self._config.trackers_config))
            if self._config.marvin_mock_grippers:
                launch_arguments.setdefault("mock_grippers", True)
            if launch_arguments:
                config["launch_arguments"] = launch_arguments

        if device.adapter == "manus":
            raw_launch_arguments = config.get("launch_arguments")
            launch_arguments = {}
            if isinstance(raw_launch_arguments, dict):
                launch_arguments.update(raw_launch_arguments)
            if self._config.mock_manus:
                launch_arguments.setdefault("mock", True)
            if launch_arguments:
                config["launch_arguments"] = launch_arguments
            if self._config.manus_config is not None:
                config.setdefault("config_file", str(self._config.manus_config))
            if self._config.manus_user_name is not None:
                config.setdefault("user_name", self._config.manus_user_name)

        if device.adapter == "wujihand":
            raw_launch_arguments = config.get("launch_arguments")
            launch_arguments = {}
            if isinstance(raw_launch_arguments, dict):
                launch_arguments.update(raw_launch_arguments)
            if self._config.wujihand_use_mock_hardware:
                launch_arguments.setdefault("use_mock_hardware", True)
            if launch_arguments:
                config["launch_arguments"] = launch_arguments
            if self._config.wujihand_identity_file is not None:
                config.setdefault("identity_file", str(self._config.wujihand_identity_file))
            if self._config.wujihand_teleop_config is not None:
                config.setdefault("teleop_config", str(self._config.wujihand_teleop_config))
            if self._config.manus_user_name is not None:
                config.setdefault("manus_user_name", self._config.manus_user_name)

        return DeviceSpec(
            device_id=device.device_id,
            adapter=device.adapter,
            required=device.required,
            depends_on=list(device.depends_on),
            config=config,
        )

    def _create_device_state(self, device: DeviceSpec) -> DeviceState:
        msg = DeviceState()
        msg.device_id = device.device_id
        msg.device_class = device.adapter
        msg.lifecycle_state = DeviceState.LIFECYCLE_IDLE
        msg.health_state = DeviceState.HEALTH_UNKNOWN
        msg.is_required = device.required
        msg.summary = "Waiting for system start."
        return msg

    def _mark_device(
        self,
        device_id: str,
        lifecycle_state: int,
        health_state: int,
        summary: str,
        ready: bool = False,
    ) -> None:
        with self._state_lock:
            msg = self._devices[device_id]
            msg.lifecycle_state = lifecycle_state
            msg.health_state = health_state
            msg.summary = summary
            if ready:
                msg.last_ready_stamp = self.get_clock().now().to_msg()
        self._publish_state()

    def _update_device_health(self, device_id: str, health_state: int, summary: str) -> None:
        with self._state_lock:
            msg = self._devices[device_id]
            if msg.health_state == health_state and msg.summary == summary:
                return
            msg.health_state = health_state
            msg.summary = summary
            if health_state == DeviceState.HEALTH_OK:
                msg.last_ready_stamp = self.get_clock().now().to_msg()
            self._event_log.record(f"{device_id}: health={health_state} summary={summary}")
        self._publish_state()

    def _health_monitor_enabled(self) -> bool:
        with self._state_lock:
            return self._state.system_state in {
                SystemStates.READY,
                SystemStates.ARMED,
                SystemStates.RECORDING,
                SystemStates.PAUSED,
            }

    def _run_health_monitor(self) -> None:
        if self._current_recipe is None or not self._health_monitor_enabled():
            return

        with self._state_lock:
            fault_on_unhealthy = self._state.system_state == SystemStates.RECORDING

        reports = self._health_monitor.evaluate(
            self._current_recipe.devices,
            self._adapters,
            fault_on_unhealthy=fault_on_unhealthy,
        )
        fatal_device_id = ""
        fatal_summary = ""
        for device in self._current_recipe.devices:
            report = reports.get(device.device_id)
            if report is None:
                continue
            self._update_device_health(
                device.device_id,
                report.health_state,
                report.summary,
            )
            if report.should_fault and not fatal_summary:
                fatal_device_id = device.device_id
                fatal_summary = report.summary

        with self._state_lock:
            already_fault = self._state.system_state == SystemStates.FAULT
        if fatal_summary and not already_fault:
            self._set_fault(fatal_summary, device_id=fatal_device_id)

    def _all_dependency_devices_ready(self, device: DeviceSpec) -> bool:
        for dependency_id in device.depends_on:
            dependency_state = self._devices.get(dependency_id)
            if dependency_state is None:
                return False
            if dependency_state.lifecycle_state != DeviceState.LIFECYCLE_READY:
                return False
        return True

    def _publish_feedback(self, goal_handle, action_type, detail: str) -> None:
        with self._state_lock:
            current_state = self._state.system_state
        with self._feedback_lock:
            feedback = action_type.Feedback()
            feedback.current_state = current_state
            feedback.detail = detail
            goal_handle.publish_feedback(feedback)

    def _device_ready_timeout_sec(self, device: DeviceSpec) -> float:
        raw_timeout = device.config.get("ready_timeout_sec")
        if raw_timeout is None:
            return self._startup_policy.startup_timeout_sec
        return _nonnegative_float(
            raw_timeout,
            self._startup_policy.startup_timeout_sec,
        )

    def _register_runtime_process(
        self,
        device: DeviceSpec,
        metadata: dict | None,
    ) -> None:
        with self._state_lock:
            self._runtime_manifest.register_device(
                device_id=device.device_id,
                adapter=device.adapter,
                metadata=metadata,
            )

    def _startup_layers(self) -> list[list[DeviceSpec]]:
        assert self._current_recipe is not None

        remaining_ids = {device.device_id for device in self._current_recipe.devices}
        ordered_devices = list(self._current_recipe.devices)
        satisfied_ids: set[str] = set()
        layers: list[list[DeviceSpec]] = []

        while remaining_ids:
            layer = [
                device
                for device in ordered_devices
                if device.device_id in remaining_ids
                and all(dependency_id in satisfied_ids for dependency_id in device.depends_on)
            ]
            if not layer:
                unresolved = [
                    device.device_id
                    for device in ordered_devices
                    if device.device_id in remaining_ids
                ]
                raise RuntimeError(
                    "Unable to resolve startup layers; dependency cycle or missing dependency "
                    f"detected among {unresolved}."
                )

            layers.append(layer)
            for device in layer:
                remaining_ids.remove(device.device_id)
                satisfied_ids.add(device.device_id)

        return layers

    def _command_blocked(self, command: str) -> str | None:
        with self._state_lock:
            system_state = self._state.system_state
            allowed_commands = list(self._state.allowed_commands)
        with self._command_lock:
            active_command_name = self._active_command_name
            active_command_started_monotonic = self._active_command_started_monotonic
        if active_command_name and active_command_name != command:
            elapsed_sec = max(0.0, time.monotonic() - active_command_started_monotonic)
            return (
                f"{command} is temporarily blocked because {active_command_name} "
                f"is already in progress ({elapsed_sec:.1f}s)."
            )
        if is_command_allowed(system_state, command):
            return None
        return (
            f"{command} is not allowed while system_state={system_state}. "
            f"allowed={allowed_commands}"
        )

    def try_begin_command(self, command: str) -> tuple[bool, str]:
        with self._command_lock:
            if self._active_command_name:
                elapsed_sec = max(
                    0.0, time.monotonic() - self._active_command_started_monotonic
                )
                return (
                    False,
                    f"{command} rejected because {self._active_command_name} is already "
                    f"in progress ({elapsed_sec:.1f}s).",
                )
            self._active_command_name = command
            self._active_command_started_monotonic = time.monotonic()
        return True, ""

    def finish_command(self, command: str, *, success: bool, detail: str = "") -> None:
        with self._command_lock:
            if self._active_command_name != command:
                return
            elapsed_sec = max(0.0, time.monotonic() - self._active_command_started_monotonic)
            self._active_command_name = ""
            self._active_command_started_monotonic = 0.0

        outcome = "succeeded" if success else "failed"
        message = f"{command} {outcome} in {elapsed_sec:.1f}s."
        if detail:
            message = f"{message} {detail}"
        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)
        self._event_log.record(message)

    def _rollback_started_devices(self, started_device_ids: list[str]) -> str | None:
        started_device_id_set = set(started_device_ids)
        rollback_failure = None
        for layer in reversed(self._startup_layers()):
            layer_device_ids = [
                device.device_id
                for device in layer
                if device.device_id in started_device_id_set
            ]
            if not layer_device_ids:
                continue

            failure = self._shutdown_device_ids_parallel(
                layer_device_ids,
                stopping_summary="Rolling back after startup failure.",
            )
            if failure is not None and rollback_failure is None:
                rollback_failure = failure

        return rollback_failure

    def _collect_startup_completion(
        self,
        future,
        active_futures: dict,
        started_device_ids: list[str],
        ready_device_ids: set[str],
    ) -> str | None:
        active_futures.pop(future)
        device_id, failure = future.result()
        if failure is not None:
            return failure

        started_device_ids.append(device_id)
        ready_device_ids.add(device_id)
        return None

    def _ready_device_ids(self) -> set[str]:
        return {
            device_id
            for device_id, device_state in self._devices.items()
            if device_state.lifecycle_state == DeviceState.LIFECYCLE_READY
        }

    def _next_startable_devices(
        self,
        pending_devices: list[DeviceSpec],
        ready_device_ids: set[str],
    ) -> list[DeviceSpec]:
        return [
            device
            for device in pending_devices
            if all(dependency_id in ready_device_ids for dependency_id in device.depends_on)
        ]

    def _validate_startup_dependencies(self) -> None:
        assert self._current_recipe is not None

        known_device_ids = {
            device.device_id for device in self._current_recipe.devices
        }
        unresolved = [
            device.device_id
            for device in self._current_recipe.devices
            if any(dependency_id not in known_device_ids for dependency_id in device.depends_on)
        ]
        if unresolved:
            raise RuntimeError(
                "Unable to resolve startup order; missing dependency detected among "
                f"{unresolved}."
            )

        pending_device_ids = set(known_device_ids)
        ready_device_ids: set[str] = set()
        while pending_device_ids:
            progress_ids = {
                device.device_id
                for device in self._current_recipe.devices
                if device.device_id in pending_device_ids
                and all(dependency_id in ready_device_ids for dependency_id in device.depends_on)
            }
            if not progress_ids:
                unresolved_pending = [
                    device.device_id
                    for device in self._current_recipe.devices
                    if device.device_id in pending_device_ids
                ]
                raise RuntimeError(
                    "Unable to resolve startup order; dependency cycle detected among "
                    f"{unresolved_pending}."
                )
            pending_device_ids -= progress_ids
            ready_device_ids |= progress_ids

    def _attempt_device_full_start(
        self,
        startup_slot_index: int,
        device: DeviceSpec,
        goal_handle,
    ) -> str | None:
        failure = self._attempt_device_bringup(startup_slot_index, device, goal_handle)
        if failure is not None:
            return failure
        return self._attempt_device_start(startup_slot_index, device, goal_handle)

    def _device_startup_runner(
        self,
        startup_slot_index: int,
        device: DeviceSpec,
        goal_handle,
    ) -> tuple[str, str | None]:
        try:
            failure = self._attempt_device_full_start(
                startup_slot_index,
                device,
                goal_handle,
            )
        except Exception as exc:  # pragma: no cover - defensive runtime path
            self.get_logger().error(
                f"Startup raised for {device.device_id}: {exc}"
            )
            return device.device_id, f"{device.device_id}: startup raised {exc!r}"
        return device.device_id, failure

    def _bringup_devices_dynamic(self, goal_handle) -> str | None:
        assert self._current_recipe is not None

        try:
            self._validate_startup_dependencies()
        except RuntimeError as exc:
            return str(exc)

        pending_devices = list(self._current_recipe.devices)
        active_futures = {}
        started_device_ids: list[str] = []
        ready_device_ids = self._ready_device_ids()
        startup_slot_index = 0

        self._set_system_state(
            SystemStates.STARTING,
            f"Starting recipe {self._current_recipe.recipe_id} with "
            f"{len(self._current_recipe.devices)} devices.",
        )
        self._publish_feedback(
            goal_handle,
            StartSystem,
            f"Recipe {self._current_recipe.recipe_id} loaded, starting devices.",
        )

        max_workers = max(1, len(self._current_recipe.devices))
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            while pending_devices or active_futures:
                startable_devices = self._next_startable_devices(
                    pending_devices,
                    ready_device_ids,
                )
                for device in startable_devices:
                    startup_slot_index += 1
                    self._publish_feedback(
                        goal_handle,
                        StartSystem,
                        f"Launching startup slot {startup_slot_index}: {device.device_id}",
                    )
                    future = executor.submit(
                        self._device_startup_runner,
                        startup_slot_index,
                        device,
                        goal_handle,
                    )
                    active_futures[future] = device
                    pending_devices.remove(device)

                if not active_futures:
                    blocked_devices = ", ".join(device.device_id for device in pending_devices)
                    return (
                        "Unable to continue startup; no startable devices remain while "
                        f"pending={blocked_devices}."
                    )

                done_futures, _ = wait(active_futures.keys(), return_when=FIRST_COMPLETED)
                startup_failure = None
                for future in done_futures:
                    failure = self._collect_startup_completion(
                        future,
                        active_futures,
                        started_device_ids,
                        ready_device_ids,
                    )
                    if failure is not None and startup_failure is None:
                        startup_failure = failure

                if startup_failure is None:
                    continue

                while active_futures:
                    done_futures, _ = wait(active_futures.keys(), return_when=FIRST_COMPLETED)
                    for future in done_futures:
                        failure = self._collect_startup_completion(
                            future,
                            active_futures,
                            started_device_ids,
                            ready_device_ids,
                        )
                        if failure is not None and startup_failure is None:
                            startup_failure = failure

                rollback_failure = self._rollback_started_devices(started_device_ids)
                if rollback_failure is not None:
                    return f"{startup_failure}\nRollback failure: {rollback_failure}"
                return startup_failure

        return None

    def _shutdown_device_ids_parallel(
        self,
        device_ids: list[str],
        *,
        stopping_summary: str,
    ) -> str | None:
        if not device_ids:
            return None

        for device_id in device_ids:
            self._mark_device(
                device_id,
                DeviceState.LIFECYCLE_STOPPING,
                DeviceState.HEALTH_UNKNOWN,
                stopping_summary,
            )

        def _shutdown_one(device_id: str):
            adapter = self._adapters[device_id]
            try:
                return adapter.shutdown()
            except Exception as exc:  # pragma: no cover - defensive runtime path
                self.get_logger().error(
                    f"Shutdown raised for {device_id}: {exc}"
                )
                from .adapters.base import AdapterResult

                return AdapterResult.failed(
                    f"{device_id}: shutdown raised {exc!r}"
                )

        max_workers = max(1, len(device_ids))
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            future_by_id = {
                device_id: executor.submit(_shutdown_one, device_id)
                for device_id in device_ids
            }
            results = {
                device_id: future.result()
                for device_id, future in future_by_id.items()
            }

        shutdown_failure = None
        for device_id in device_ids:
            device = self._device_specs[device_id]
            result = results[device_id]
            if result.is_failure() and device.required and shutdown_failure is None:
                shutdown_failure = result.summary
            if not result.is_failure():
                with self._state_lock:
                    self._runtime_manifest.unregister_device(device_id)
            self._mark_device(
                device_id,
                DeviceState.LIFECYCLE_IDLE,
                DeviceState.HEALTH_UNKNOWN,
                result.summary,
            )

        return shutdown_failure

    def _shutdown_layer_devices(self, layer_device_ids: list[str]) -> str | None:
        return self._shutdown_device_ids_parallel(
            layer_device_ids,
            stopping_summary="Retry cleanup after startup failure.",
        )

    def _attempt_device_bringup(
        self,
        layer_index: int,
        device: DeviceSpec,
        goal_handle,
    ) -> str | None:
        max_attempts = 1 + self._startup_policy.retry_count

        for attempt_index in range(max_attempts):
            attempt_number = attempt_index + 1
            attempt_label = f"attempt {attempt_number}/{max_attempts}"

            self._publish_feedback(
                goal_handle,
                StartSystem,
                f"Launching device {device.device_id} in layer {layer_index} ({attempt_label}).",
            )
            self._mark_device(
                device.device_id,
                DeviceState.LIFECYCLE_STARTING,
                DeviceState.HEALTH_UNKNOWN,
                f"Running precheck ({attempt_label}).",
            )
            self._publish_feedback(
                goal_handle,
                StartSystem,
                f"Precheck {device.device_id} ({attempt_label}).",
            )
            precheck = self._adapters[device.device_id].precheck()
            if precheck.is_failure():
                failure = precheck.summary
                if attempt_number >= max_attempts:
                    self._mark_device(
                        device.device_id,
                        DeviceState.LIFECYCLE_FAULT,
                        DeviceState.HEALTH_FAILED,
                        failure,
                    )
                else:
                    self._mark_device(
                        device.device_id,
                        DeviceState.LIFECYCLE_STARTING,
                        DeviceState.HEALTH_DEGRADED,
                        f"{failure} [{attempt_label}]",
                    )
            else:
                bringup = self._adapters[device.device_id].bringup()
                if bringup.is_failure():
                    failure = bringup.summary
                    if attempt_number >= max_attempts:
                        self._mark_device(
                            device.device_id,
                            DeviceState.LIFECYCLE_FAULT,
                            DeviceState.HEALTH_FAILED,
                            failure,
                        )
                    else:
                        self._mark_device(
                            device.device_id,
                            DeviceState.LIFECYCLE_STARTING,
                            DeviceState.HEALTH_DEGRADED,
                            f"{failure} [{attempt_label}]",
                        )
                    cleanup_failure = self._shutdown_device_ids_parallel(
                        [device.device_id],
                        stopping_summary="Retry cleanup after device launch failure.",
                    )
                    if cleanup_failure is not None:
                        return f"{failure}\nDevice cleanup failure: {cleanup_failure}"
                else:
                    self._register_runtime_process(device, bringup.metadata)
                    return None

            self.get_logger().warn(
                f"Launch device {device.device_id} in layer {layer_index} "
                f"{attempt_label} failed: {failure}"
            )
            if attempt_number >= max_attempts:
                return failure

            if self._startup_policy.retry_backoff_sec > 0.0:
                self._publish_feedback(
                    goal_handle,
                    StartSystem,
                    f"Device {device.device_id} launch failed {attempt_label}; retrying in "
                    f"{self._startup_policy.retry_backoff_sec:.1f}s.",
                )
                time.sleep(self._startup_policy.retry_backoff_sec)

        return (
            f"Device {device.device_id} in layer {layer_index} "
            "launch attempts exhausted unexpectedly."
        )

    def _attempt_device_start(
        self,
        layer_index: int,
        device: DeviceSpec,
        goal_handle,
    ) -> str | None:
        max_attempts = 1 + self._startup_policy.retry_count

        for attempt_index in range(max_attempts):
            attempt_number = attempt_index + 1
            attempt_label = f"attempt {attempt_number}/{max_attempts}"

            self._publish_feedback(
                goal_handle,
                StartSystem,
                f"Starting device {device.device_id} in layer {layer_index} ({attempt_label}).",
            )
            self._mark_device(
                device.device_id,
                DeviceState.LIFECYCLE_STARTING,
                DeviceState.HEALTH_UNKNOWN,
                f"Running precheck ({attempt_label}).",
            )
            self._publish_feedback(
                goal_handle,
                StartSystem,
                f"Precheck {device.device_id} ({attempt_label}).",
            )
            precheck = self._adapters[device.device_id].precheck()
            if precheck.is_failure():
                failure = precheck.summary
                if attempt_number >= max_attempts:
                    self._mark_device(
                        device.device_id,
                        DeviceState.LIFECYCLE_FAULT,
                        DeviceState.HEALTH_FAILED,
                        failure,
                    )
                else:
                    self._mark_device(
                        device.device_id,
                        DeviceState.LIFECYCLE_STARTING,
                        DeviceState.HEALTH_DEGRADED,
                        f"{failure} [{attempt_label}]",
                    )
            else:
                bringup = self._adapters[device.device_id].bringup()
                if bringup.is_failure():
                    failure = bringup.summary
                    if attempt_number >= max_attempts:
                        self._mark_device(
                            device.device_id,
                            DeviceState.LIFECYCLE_FAULT,
                            DeviceState.HEALTH_FAILED,
                            failure,
                        )
                    else:
                        self._mark_device(
                            device.device_id,
                            DeviceState.LIFECYCLE_STARTING,
                            DeviceState.HEALTH_DEGRADED,
                            f"{failure} [{attempt_label}]",
                        )
                else:
                    self._register_runtime_process(device, bringup.metadata)
                    ready = self._adapters[device.device_id].wait_ready(
                        timeout_sec=self._device_ready_timeout_sec(device)
                    )
                    if ready.is_failure():
                        failure = ready.summary
                        if attempt_number >= max_attempts:
                            self._mark_device(
                                device.device_id,
                                DeviceState.LIFECYCLE_FAULT,
                                DeviceState.HEALTH_FAILED,
                                failure,
                            )
                        else:
                            self._mark_device(
                                device.device_id,
                                DeviceState.LIFECYCLE_STARTING,
                                DeviceState.HEALTH_DEGRADED,
                                f"{failure} [{attempt_label}]",
                            )
                    else:
                        self._mark_device(
                            device.device_id,
                            DeviceState.LIFECYCLE_READY,
                            DeviceState.HEALTH_OK,
                            ready.summary,
                            ready=True,
                        )
                        self._publish_feedback(
                            goal_handle,
                            StartSystem,
                            f"Device {device.device_id} ready ({attempt_label}).",
                        )
                        return None

                    cleanup_failure = self._shutdown_device_ids_parallel(
                        [device.device_id],
                        stopping_summary="Retry cleanup after device startup failure.",
                    )
                    if cleanup_failure is not None:
                        return f"{failure}\nDevice cleanup failure: {cleanup_failure}"

            self.get_logger().warn(
                f"Startup device {device.device_id} in layer {layer_index} "
                f"{attempt_label} failed: {failure}"
            )
            if attempt_number >= max_attempts:
                return failure

            if self._startup_policy.retry_backoff_sec > 0.0:
                self._publish_feedback(
                    goal_handle,
                    StartSystem,
                    f"Device {device.device_id} failed {attempt_label}; retrying in "
                    f"{self._startup_policy.retry_backoff_sec:.1f}s.",
                )
                time.sleep(self._startup_policy.retry_backoff_sec)

        return (
            f"Device {device.device_id} in layer {layer_index} "
            "startup attempts exhausted unexpectedly."
        )

    def _attempt_layer_start(
        self,
        layer_index: int,
        layer: list[DeviceSpec],
        goal_handle,
    ) -> str | None:
        layer_device_ids = [device.device_id for device in layer]
        layer_name = ", ".join(layer_device_ids)

        self._publish_feedback(
            goal_handle,
            StartSystem,
            f"Starting layer {layer_index} [{layer_name}].",
        )

        max_workers = max(1, len(layer))
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            bringup_futures = {
                device.device_id: executor.submit(
                    self._attempt_device_bringup,
                    layer_index,
                    device,
                    goal_handle,
                )
                for device in layer
            }
            bringup_failures: list[str] = []
            for device in layer:
                failure = bringup_futures[device.device_id].result()
                if failure is not None:
                    bringup_failures.append(failure)

            if bringup_failures:
                # Do not roll back unrelated siblings in the same dependency layer.
                # Startup failure cleanup will disconnect the whole runtime afterwards.
                return bringup_failures[0]

            start_futures = {
                device.device_id: executor.submit(
                    self._attempt_device_start,
                    layer_index,
                    device,
                    goal_handle,
                )
                for device in layer
            }
            start_failures: list[str] = []
            for device in layer:
                failure = start_futures[device.device_id].result()
                if failure is not None:
                    start_failures.append(failure)

            if start_failures:
                # Keep successful siblings alive until the outer startup failure reset.
                # Otherwise one independent device failure can spuriously flap another.
                return start_failures[0]

        return None

    def _bringup_devices(self, goal_handle) -> str | None:
        return self._bringup_devices_dynamic(goal_handle)

    def _run_home_sequence(self) -> tuple[AdapterResult | None, str]:
        assert self._current_recipe is not None
        for device in self._current_recipe.devices:
            result = self._adapters[device.device_id].home()
            if result.status == "OK":
                self._mark_device(
                    device.device_id,
                    DeviceState.LIFECYCLE_READY,
                    DeviceState.HEALTH_OK,
                    result.summary,
                    ready=True,
                )
                continue
            if result.status == "DEGRADED":
                self._mark_device(
                    device.device_id,
                    DeviceState.LIFECYCLE_READY,
                    DeviceState.HEALTH_DEGRADED,
                    result.summary,
                    ready=True,
                )
                if device.required:
                    return result, device.device_id
                continue
            if result.is_failure() and device.required:
                return result, device.device_id
        return None, ""

    def _run_arm_sequence(self) -> str | None:
        assert self._current_recipe is not None
        for device in self._current_recipe.devices:
            result = self._adapters[device.device_id].arm()
            if result.is_failure() and device.required:
                return result.summary
            if result.status == "OK":
                self._mark_device(
                    device.device_id,
                    DeviceState.LIFECYCLE_ACTIVE,
                    DeviceState.HEALTH_OK,
                    result.summary,
                    ready=True,
                )
        return None

    def _run_disarm_sequence(self) -> str | None:
        assert self._current_recipe is not None
        for device in self._current_recipe.devices:
            result = self._adapters[device.device_id].disarm()
            if result.is_failure() and device.required:
                return result.summary
            if result.status == "OK":
                self._mark_device(
                    device.device_id,
                    DeviceState.LIFECYCLE_READY,
                    DeviceState.HEALTH_OK,
                    result.summary,
                    ready=True,
                )
        return None

    def _run_before_session(self) -> str | None:
        assert self._current_recipe is not None
        for device in self._current_recipe.devices:
            result = self._adapters[device.device_id].before_session()
            if result.is_failure() and device.required:
                return result.summary
            if result.status == "OK":
                self._mark_device(
                    device.device_id,
                    DeviceState.LIFECYCLE_ACTIVE,
                    DeviceState.HEALTH_OK,
                    result.summary,
                    ready=True,
                )
        return None

    def _run_after_session(self) -> str | None:
        assert self._current_recipe is not None
        for device in self._current_recipe.devices:
            result = self._adapters[device.device_id].after_session()
            if result.is_failure() and device.required:
                return result.summary
            if result.status == "OK":
                self._mark_device(
                    device.device_id,
                    DeviceState.LIFECYCLE_READY,
                    DeviceState.HEALTH_OK,
                    result.summary,
                    ready=True,
                )
        return None

    def _shutdown_devices(self) -> str | None:
        if self._current_recipe is None:
            return None

        shutdown_failure = None
        for layer in reversed(self._startup_layers()):
            layer_device_ids = [device.device_id for device in layer]
            failure = self._shutdown_device_ids_parallel(
                layer_device_ids,
                stopping_summary="Shutting down.",
            )
            if failure is not None and shutdown_failure is None:
                shutdown_failure = failure
        return shutdown_failure

    def _stop_active_session_runtime(self) -> tuple[ActiveSession | None, str | None]:
        session = self._session_manager.active_session
        if session is None:
            return None, "No active session."

        try:
            stop_summary = self._record_manager.stop_recording(
                active_session=session,
                recording_policy=self._recording_policy,
                event_log_entries=self._event_log.snapshot(),
            )
        except Exception as exc:
            return None, str(exc)
        after_failure = self._run_after_session()
        if after_failure is not None:
            return None, after_failure
        disarm_failure = self._run_disarm_sequence()
        if disarm_failure is not None:
            return None, disarm_failure

        ended_session = self._session_manager.end_session()
        if stop_summary.get("success", False):
            self._set_system_state(
                SystemStates.READY,
                f"Session {session.session_id} stopped and system returned to READY.",
            )
            return ended_session, None

        warning = (
            f"Session {session.session_id} stopped with preserved incomplete artifacts."
        )
        self._set_system_state(SystemStates.READY, warning)
        return ended_session, warning

    def _reset_after_startup_failure(self, failure: str) -> None:
        failure_head = failure.splitlines()[0].strip() if failure else "unknown startup failure"
        self.get_logger().error(f"StartSystem failed: {failure_head}")
        self._event_log.record(f"STARTUP_FAILED: {failure}")
        self._set_system_state(
            SystemStates.STOPPING,
            "StartSystem failed during bringup. Disconnecting partial runtime.",
        )
        self._reset_recipe_runtime(shutdown_adapters=True)
        self._clear_faults()
        self._set_system_state(
            SystemStates.IDLE,
            f"StartSystem failed and runtime was disconnected: {failure_head}",
        )

    def execute_start_system(self, goal_handle):
        request = goal_handle.request
        result = StartSystem.Result()

        blocked_reason = self._command_blocked(Commands.START_SYSTEM)
        if blocked_reason is not None:
            goal_handle.abort()
            result.success = False
            result.message = blocked_reason
            return result

        recipe_id = request.recipe_id or self._config.recipe_id
        recipe = self._load_recipe_or_fail(recipe_id)
        if recipe is None:
            goal_handle.abort()
            result.success = False
            result.message = f"Recipe not found: {recipe_id}"
            return result

        self._current_operator_id = request.operator_id
        self._current_site_name = request.site_name
        self._install_recipe(recipe)
        self._set_system_state(
            SystemStates.PREFLIGHT,
            f"Preflight checks for recipe {recipe.recipe_id}.",
        )
        self._publish_feedback(
            goal_handle,
            StartSystem,
            f"Loaded recipe {recipe.recipe_id}, entering PREFLIGHT.",
        )

        failure = self._bringup_devices(goal_handle)
        if failure is not None:
            self._reset_after_startup_failure(failure)
            goal_handle.abort()
            result.success = False
            result.message = (
                f"{failure}\nStartup failed; runtime was disconnected automatically."
            )
            return result

        if self._recording_policy.require_time_sync_healthy:
            try:
                session_topics = self._session_record_topics()
            except Exception as exc:
                self._reset_after_startup_failure(str(exc))
                goal_handle.abort()
                result.success = False
                result.message = str(exc)
                return result
            sync_summary = self._time_sync_health_summary(session_topics)
            if not bool(sync_summary.get("healthy", False)):
                failure = "Timing prerequisites are not satisfied for the configured recording contract."
                self._reset_after_startup_failure(failure)
                goal_handle.abort()
                result.success = False
                result.message = failure
                return result

        self._set_system_state(
            SystemStates.READY,
            f"Recipe {recipe.recipe_id} is READY. "
            f"site={request.site_name or 'default'} operator={request.operator_id or 'unknown'}",
        )
        self._publish_feedback(
            goal_handle,
            StartSystem,
            f"Recipe {recipe.recipe_id} reached READY.",
        )
        goal_handle.succeed()
        result.success = True
        result.message = f"System started with recipe {recipe.recipe_id}"
        return result

    def execute_shutdown_system(self, goal_handle):
        request = goal_handle.request
        result = ShutdownSystem.Result()

        blocked_reason = self._command_blocked(Commands.SHUTDOWN_SYSTEM)
        if blocked_reason is not None and not (
            request.force and self._state.system_state == SystemStates.RECORDING
        ):
            goal_handle.abort()
            result.success = False
            result.message = blocked_reason
            return result

        self._publish_feedback(
            goal_handle,
            ShutdownSystem,
            f"Shutdown requested. force={request.force}",
        )

        if self._state.system_state == SystemStates.RECORDING:
            active_session = self._session_manager.active_session
            if active_session is not None:
                active_session.session_stop_ros_time_ns = self._now_ros_time_ns()
                active_session.session_stop_wall_time = self._now_wall_time()
                active_session.session_stop_monotonic_sec = time.monotonic()
            session, stop_failure = self._stop_active_session_runtime()
            if stop_failure is not None and session is None:
                self._set_fault(stop_failure)
                goal_handle.abort()
                result.success = False
                result.message = stop_failure
                return result
            if session is not None:
                self._publish_feedback(
                    goal_handle,
                    ShutdownSystem,
                    f"Stopped active session {session.session_id} before shutdown.",
                )

        self._set_system_state(SystemStates.STOPPING, "Shutting down active devices.")
        shutdown_failure = self._shutdown_devices()
        if shutdown_failure is not None:
            self._set_fault(shutdown_failure)
            goal_handle.abort()
            result.success = False
            result.message = shutdown_failure
            return result

        self._reset_recipe_runtime()
        self._clear_faults()
        self._set_system_state(
            SystemStates.IDLE,
            "System shutdown complete and runtime state reset.",
        )
        self._publish_feedback(goal_handle, ShutdownSystem, "System returned to IDLE.")
        goal_handle.succeed()
        result.success = True
        result.message = "System shutdown complete"
        return result

    def execute_start_session(self, goal_handle):
        request = goal_handle.request
        result = StartSession.Result()

        blocked_reason = self._command_blocked(Commands.START_SESSION)
        if blocked_reason is not None:
            goal_handle.abort()
            result.success = False
            result.session_id = ""
            result.message = blocked_reason
            return result

        if self._current_recipe is None:
            goal_handle.abort()
            result.success = False
            result.session_id = ""
            result.message = "No active recipe. StartSystem must succeed first."
            return result

        armed_this_session = False
        before_session_ran = False
        session: ActiveSession | None = None
        try:
            if self._state.system_state == SystemStates.READY:
                self._publish_feedback(goal_handle, StartSession, "Arming supported devices.")
                arm_failure = self._run_arm_sequence()
                if arm_failure is not None:
                    self._set_fault(arm_failure)
                    goal_handle.abort()
                    result.success = False
                    result.session_id = ""
                    result.message = arm_failure
                    return result
                armed_this_session = True
                self._set_system_state(
                    SystemStates.ARMED,
                    "System armed and ready to enter RECORDING.",
                )

            self._publish_feedback(goal_handle, StartSession, "Running before_session hooks.")
            before_session_failure = self._run_before_session()
            if before_session_failure is not None:
                self._set_fault(before_session_failure)
                goal_handle.abort()
                result.success = False
                result.session_id = ""
                result.message = before_session_failure
                return result
            before_session_ran = True

            operator_id = request.operator_id or self._current_operator_id or "unknown"
            session = self._session_manager.begin_session(
                recipe_id=self._current_recipe.recipe_id,
                operator_id=operator_id,
                session_tag=request.session_tag,
                site_name=self._current_site_name,
            )
            record_topics = self._session_record_topics()
            sync_summary = self._time_sync_health_summary(record_topics)
            if self._recording_policy.require_time_sync_healthy and not bool(
                sync_summary.get("healthy", False)
            ):
                raise RuntimeError("Pre-session synchronization health checks failed.")

            self._record_manager.prepare_session(
                active_session=session,
                recipe=self._current_recipe,
                supervisor_config=self._config,
                recording_policy=self._recording_policy,
                normalized_topics=record_topics,
                event_log_snapshot_source=self._event_log.snapshot,
            )
            session.time_sync_summary = dict(sync_summary)
            session.record_topics = list(record_topics)
            session.session_start_ros_time_ns = self._now_ros_time_ns()
            session.session_start_wall_time = self._now_wall_time()
            session.session_start_monotonic_sec = time.monotonic()
            self._record_manager.start_recording(
                active_session=session,
                recording_policy=self._recording_policy,
                sync_health_summary=sync_summary,
            )
        except Exception as exc:
            failure_message = str(exc)
            if session is not None:
                try:
                    self._record_manager.abort_recording(
                        active_session=session,
                        recording_policy=self._recording_policy,
                        message=failure_message,
                        event_log_entries=self._event_log.snapshot(),
                    )
                except Exception:
                    pass
                self._session_manager.end_session()
            rollback_failure = None
            if before_session_ran:
                rollback_failure = self._run_after_session()
            if rollback_failure is None and armed_this_session:
                rollback_failure = self._run_disarm_sequence()
            if rollback_failure is not None:
                self._set_fault(rollback_failure)
                goal_handle.abort()
                result.success = False
                result.session_id = ""
                result.message = f"{failure_message}\nRollback failure: {rollback_failure}"
                return result
            self._set_system_state(SystemStates.READY, f"StartSession failed: {failure_message}")
            goal_handle.abort()
            result.success = False
            result.session_id = ""
            result.message = failure_message
            return result

        assert session is not None
        self._set_system_state(
            SystemStates.RECORDING,
            f"Session {session.session_id} started for recipe {session.recipe_id}.",
        )
        self._publish_feedback(
            goal_handle,
            StartSession,
            f"Session {session.session_id} entered RECORDING.",
        )

        goal_handle.succeed()
        result.success = True
        result.session_id = session.session_id
        result.message = "Session started"
        return result

    def execute_stop_session(self, goal_handle):
        request = goal_handle.request
        result = StopSession.Result()

        blocked_reason = self._command_blocked(Commands.STOP_SESSION)
        if blocked_reason is not None:
            goal_handle.abort()
            result.success = False
            result.session_id = ""
            result.message = blocked_reason
            return result

        active_session = self._session_manager.active_session
        if active_session is not None:
            active_session.session_stop_ros_time_ns = self._now_ros_time_ns()
            active_session.session_stop_wall_time = self._now_wall_time()
            active_session.session_stop_monotonic_sec = time.monotonic()

        self._publish_feedback(
            goal_handle,
            StopSession,
            f"Stopping session. reason={request.reason or 'unspecified'}",
        )

        session, failure = self._stop_active_session_runtime()
        if failure is not None and session is None:
            self._set_fault(failure)
            goal_handle.abort()
            result.success = False
            result.session_id = ""
            result.message = failure
            return result

        if failure is not None:
            goal_handle.abort()
            result.success = False
            result.session_id = "" if session is None else session.session_id
            result.message = failure
            return result

        goal_handle.succeed()
        result.success = True
        result.session_id = "" if session is None else session.session_id
        result.message = "Session stopped"
        return result

    def execute_go_home(self, goal_handle):
        request = goal_handle.request
        result = GoHome.Result()

        blocked_reason = self._command_blocked(Commands.GO_HOME)
        if blocked_reason is not None:
            goal_handle.abort()
            result.success = False
            result.message = blocked_reason
            return result

        self._publish_feedback(goal_handle, GoHome, "Running home hooks.")
        home_result, failing_device_id = self._run_home_sequence()
        if home_result is not None:
            if home_result.is_failure():
                self._set_fault(home_result.summary, device_id=failing_device_id)
            else:
                self._set_system_state(
                    SystemStates.READY,
                    f"GoHome degraded: {home_result.summary}",
                )
            goal_handle.abort()
            result.success = False
            result.message = home_result.summary
            return result

        next_state = SystemStates.READY
        if request.arm_after_home:
            self._publish_feedback(goal_handle, GoHome, "Arming after home.")
            arm_failure = self._run_arm_sequence()
            if arm_failure is not None:
                self._set_fault(arm_failure)
                goal_handle.abort()
                result.success = False
                result.message = arm_failure
                return result
            next_state = SystemStates.ARMED

        self._set_system_state(
            next_state,
            f"GoHome completed. next_state={next_state}",
        )
        self._publish_feedback(
            goal_handle,
            GoHome,
            f"Home sequence complete. next_state={next_state}",
        )
        goal_handle.succeed()
        result.success = True
        result.message = "GoHome completed"
        return result

    def _handle_get_system_state(
        self, request: GetSystemState.Request, response: GetSystemState.Response
    ) -> GetSystemState.Response:
        del request
        with self._state_lock:
            response.state = deepcopy(self._state)
        return response

    def _handle_list_recipes(
        self, request: ListRecipes.Request, response: ListRecipes.Response
    ) -> ListRecipes.Response:
        del request
        response.recipe_ids = sorted(self._recipes.keys())
        with self._state_lock:
            response.active_recipe_id = self._state.recipe_id
        return response

    def _handle_acknowledge_fault(
        self,
        request: AcknowledgeFault.Request,
        response: AcknowledgeFault.Response,
    ) -> AcknowledgeFault.Response:
        fault_id = request.fault_id or "<latest>"
        with self._state_lock:
            system_state = self._state.system_state
        if system_state != SystemStates.FAULT:
            response.success = False
            response.message = f"No active fault to acknowledge: {fault_id}"
            return response

        self._clear_faults()
        self._reset_recipe_runtime(shutdown_adapters=True)
        self._set_system_state(
            SystemStates.IDLE,
            f"Fault acknowledged: {fault_id}. Runtime awaits a new StartSystem request.",
        )
        response.success = True
        response.message = f"Acknowledged {fault_id} and returned to IDLE"
        return response

    def shutdown_runtime(self) -> None:
        self._reset_recipe_runtime(shutdown_adapters=True)
        self._clear_faults()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    launch_manager = LaunchManager()
    node = DataCollectionSupervisor(launch_manager=launch_manager)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="data_collection_supervisor_rclpy_spin",
        daemon=True,
    )
    executor_thread.start()
    shutdown_lock = threading.Lock()
    shutdown_started = False
    shutdown_request_lock = threading.Lock()
    shutdown_request_thread: threading.Thread | None = None

    def shutdown_once(reason: str) -> None:
        nonlocal shutdown_started
        with shutdown_lock:
            if shutdown_started:
                return
            shutdown_started = True

        try:
            node.get_logger().warn(f"Shutdown requested via {reason}. Cleaning runtime.")
        except Exception:
            pass

        try:
            node.shutdown_runtime()
        except Exception as exc:
            try:
                node.get_logger().error(f"Shutdown runtime cleanup failed: {exc}")
            except Exception:
                pass

        try:
            executor.shutdown()
        except Exception:
            pass
        if executor_thread.is_alive():
            executor_thread.join(timeout=2.0)

        try:
            launch_manager.shutdown()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def request_shutdown(reason: str) -> None:
        nonlocal shutdown_request_thread
        with shutdown_request_lock:
            if shutdown_request_thread is not None and shutdown_request_thread.is_alive():
                return
            shutdown_request_thread = threading.Thread(
                target=shutdown_once,
                args=(reason,),
                name="data_collection_supervisor_shutdown",
                daemon=True,
            )
            shutdown_request_thread.start()

    previous_sigint_handler = signal.getsignal(signal.SIGINT)
    previous_sigterm_handler = signal.getsignal(signal.SIGTERM)

    def _handle_signal(signum, frame) -> None:
        del frame
        try:
            signal_name = signal.Signals(signum).name
        except Exception:
            signal_name = f"signal {signum}"
        request_shutdown(signal_name)

    try:
        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)
        launch_manager.run_forever()
    except KeyboardInterrupt:
        shutdown_once("KeyboardInterrupt")
    finally:
        try:
            signal.signal(signal.SIGINT, previous_sigint_handler)
            signal.signal(signal.SIGTERM, previous_sigterm_handler)
        except Exception:
            pass
        shutdown_once("process exit")
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
