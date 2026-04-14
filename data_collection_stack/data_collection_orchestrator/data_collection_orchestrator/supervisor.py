from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import time
from typing import Optional

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

from .adapters import AdapterResult, create_adapter
from .command_server import CommandServer
from .event_log import EventLog
from .health_monitor import HealthMonitor
from .models import ActiveSession, DeviceSpec, RecipeSpec, StartupPolicy, SupervisorConfig
from .recipe_loader import discover_recipes, load_recipe
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
    def __init__(self) -> None:
        super().__init__("data_collection_supervisor")

        self._config = self._declare_config()
        self._runtime_manifest = RuntimeManifest()
        self._startup_policy = self._load_startup_policy()
        self._event_log = EventLog()
        self._health_monitor = HealthMonitor()
        self._session_manager = SessionManager()
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
        self.declare_parameter("cameras_config", "")
        self.declare_parameter("trackers_config", "")
        self.declare_parameter("manus_config", "")
        self.declare_parameter("manus_user_name", "")
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
            cameras_config=_optional_path(self.get_parameter("cameras_config").value),
            trackers_config=_optional_path(self.get_parameter("trackers_config").value),
            manus_config=_optional_path(self.get_parameter("manus_config").value),
            manus_user_name=_optional_text(self.get_parameter("manus_user_name").value),
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

    def _publish_state(self) -> None:
        if not rclpy.ok():
            return
        self._state.stamp = self.get_clock().now().to_msg()
        self._state.devices = list(self._devices.values())
        active_session = self._session_manager.active_session
        self._state.active_session_id = "" if active_session is None else active_session.session_id
        try:
            self._publisher.publish(self._state)
        except Exception:
            return

    def _set_system_state(self, system_state: str, summary: str) -> None:
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
        self._adapters = {}
        self._device_specs = {}
        self._devices = {}
        self._current_recipe = None
        self._health_monitor.reset()
        self._session_manager.end_session()
        self._runtime_manifest.clear()

    def _load_recipe_or_fail(self, recipe_id: str) -> RecipeSpec | None:
        recipe = load_recipe(self._config.recipe_directory, recipe_id)
        if recipe is None:
            return None
        return recipe

    def _install_recipe(self, recipe: RecipeSpec) -> None:
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
            if self._config.manus_config is not None:
                config.setdefault("config_file", str(self._config.manus_config))
            if self._config.manus_user_name is not None:
                config.setdefault("user_name", self._config.manus_user_name)

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
        msg = self._devices[device_id]
        msg.lifecycle_state = lifecycle_state
        msg.health_state = health_state
        msg.summary = summary
        if ready:
            msg.last_ready_stamp = self.get_clock().now().to_msg()
        self._publish_state()

    def _update_device_health(self, device_id: str, health_state: int, summary: str) -> None:
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
        return self._state.system_state in {
            SystemStates.READY,
            SystemStates.ARMED,
            SystemStates.RECORDING,
            SystemStates.PAUSED,
        }

    def _run_health_monitor(self) -> None:
        if self._current_recipe is None or not self._health_monitor_enabled():
            return

        reports = self._health_monitor.evaluate(
            self._current_recipe.devices,
            self._adapters,
            fault_on_unhealthy=self._state.system_state == SystemStates.RECORDING,
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

        if fatal_summary and self._state.system_state != SystemStates.FAULT:
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
        feedback = action_type.Feedback()
        feedback.current_state = self._state.system_state
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
        if is_command_allowed(self._state.system_state, command):
            return None
        return (
            f"{command} is not allowed while system_state={self._state.system_state}. "
            f"allowed={self._state.allowed_commands}"
        )

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
        started_device_ids: list[str] = []

        self._publish_feedback(
            goal_handle,
            StartSystem,
            f"Starting layer {layer_index} [{layer_name}].",
        )

        for device in layer:
            failure = self._attempt_device_bringup(layer_index, device, goal_handle)
            if failure is None:
                started_device_ids.append(device.device_id)
                continue

            rollback_failure = self._shutdown_layer_devices(started_device_ids)
            if rollback_failure is not None:
                return f"{failure}\nLayer cleanup failure: {rollback_failure}"
            return failure

        for device in layer:
            failure = self._attempt_device_start(layer_index, device, goal_handle)
            if failure is None:
                continue

            rollback_failure = self._shutdown_layer_devices(started_device_ids)
            if rollback_failure is not None:
                return f"{failure}\nLayer cleanup failure: {rollback_failure}"
            return failure

        return None

    def _bringup_devices(self, goal_handle) -> str | None:
        assert self._current_recipe is not None
        started_device_ids: list[str] = []
        try:
            startup_layers = self._startup_layers()
        except RuntimeError as exc:
            return str(exc)

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

        for layer_index, layer in enumerate(startup_layers, start=1):
            layer_name = ", ".join(device.device_id for device in layer)
            self._publish_feedback(
                goal_handle,
                StartSystem,
                f"Launching startup layer {layer_index}: {layer_name}",
            )
            failure = self._attempt_layer_start(layer_index, layer, goal_handle)
            if failure is not None:
                rollback_failure = self._rollback_started_devices(started_device_ids)
                if rollback_failure is not None:
                    return f"{failure}\nRollback failure: {rollback_failure}"
                return failure

            started_device_ids.extend(device.device_id for device in layer)

        return None

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

        failure = self._run_after_session()
        if failure is not None:
            return None, failure

        disarm_failure = self._run_disarm_sequence()
        if disarm_failure is not None:
            return None, disarm_failure

        ended_session = self._session_manager.end_session()
        self._set_system_state(
            SystemStates.READY,
            f"Session {session.session_id} stopped and system returned to READY.",
        )
        return ended_session, None

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
            self._set_fault(failure)
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
            session, stop_failure = self._stop_active_session_runtime()
            if stop_failure is not None:
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

        operator_id = request.operator_id or self._current_operator_id or "unknown"
        session = self._session_manager.begin_session(
            recipe_id=self._current_recipe.recipe_id,
            operator_id=operator_id,
            session_tag=request.session_tag,
        )
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

        self._publish_feedback(
            goal_handle,
            StopSession,
            f"Stopping session. reason={request.reason or 'unspecified'}",
        )

        session, failure = self._stop_active_session_runtime()
        if failure is not None:
            self._set_fault(failure)
            goal_handle.abort()
            result.success = False
            result.session_id = ""
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
        response.state = self._state
        return response

    def _handle_list_recipes(
        self, request: ListRecipes.Request, response: ListRecipes.Response
    ) -> ListRecipes.Response:
        del request
        response.recipe_ids = sorted(self._recipes.keys())
        response.active_recipe_id = self._state.recipe_id
        return response

    def _handle_acknowledge_fault(
        self,
        request: AcknowledgeFault.Request,
        response: AcknowledgeFault.Response,
    ) -> AcknowledgeFault.Response:
        fault_id = request.fault_id or "<latest>"
        if self._state.system_state != SystemStates.FAULT:
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
    node = DataCollectionSupervisor()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_runtime()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
