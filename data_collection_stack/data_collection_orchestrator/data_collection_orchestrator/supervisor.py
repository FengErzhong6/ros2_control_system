from __future__ import annotations

from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

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

from .adapters import create_adapter
from .command_server import CommandServer
from .event_log import EventLog
from .models import ActiveSession, DeviceSpec, RecipeSpec, SupervisorConfig
from .recipe_loader import discover_recipes, load_recipe
from .session_manager import SessionManager
from .state_machine import Commands, SystemStates, allowed_commands_for, is_command_allowed


def _optional_path(raw_value: str) -> Optional[Path]:
    if not raw_value:
        return None
    return Path(raw_value).expanduser()


class DataCollectionSupervisor(Node):
    def __init__(self) -> None:
        super().__init__("data_collection_supervisor")

        self._config = self._declare_config()
        self._event_log = EventLog()
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

        self._set_system_state(
            SystemStates.IDLE,
            "Phase 1 supervisor ready: services and action servers are available.",
        )
        self._publish_state()

        self._event_log.record("Phase 1 supervisor started.")
        self.get_logger().info(
            "Supervisor ready. "
            f"recipe_id={self._config.recipe_id} recipes={len(self._recipes)} "
            f"actions={self._command_server.describe()}"
        )

    def _declare_config(self) -> SupervisorConfig:
        self.declare_parameter("recipe_id", "marvin_tracker_collection")
        self.declare_parameter("recipe_directory", "")
        self.declare_parameter("site_config_root", "")
        self.declare_parameter("operator_config", "")
        self.declare_parameter("startup_policy_config", "")
        self.declare_parameter("fault_policy_config", "")

        return SupervisorConfig(
            recipe_id=self.get_parameter("recipe_id").value,
            recipe_directory=_optional_path(self.get_parameter("recipe_directory").value),
            site_config_root=_optional_path(self.get_parameter("site_config_root").value),
            operator_config=_optional_path(self.get_parameter("operator_config").value),
            startup_policy_config=_optional_path(
                self.get_parameter("startup_policy_config").value
            ),
            fault_policy_config=_optional_path(
                self.get_parameter("fault_policy_config").value
            ),
        )

    def _publish_state(self) -> None:
        self._state.stamp = self.get_clock().now().to_msg()
        self._state.devices = list(self._devices.values())
        active_session = self._session_manager.active_session
        self._state.active_session_id = "" if active_session is None else active_session.session_id
        self._publisher.publish(self._state)

    def _set_system_state(self, system_state: str, summary: str) -> None:
        self._state.system_state = system_state
        self._state.summary = summary
        self._state.allowed_commands = allowed_commands_for(system_state)
        self._state.recipe_id = self._selected_recipe_id
        self._event_log.record(f"{system_state}: {summary}")
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
            for adapter in self._adapters.values():
                try:
                    adapter.shutdown()
                except Exception as exc:  # pragma: no cover - defensive cleanup path
                    self.get_logger().error(f"Adapter shutdown during reset failed: {exc}")
        self._adapters = {}
        self._device_specs = {}
        self._devices = {}
        self._current_recipe = None
        self._session_manager.end_session()

    def _load_recipe_or_fail(self, recipe_id: str) -> RecipeSpec | None:
        recipe = load_recipe(self._config.recipe_directory, recipe_id)
        if recipe is None:
            return None
        return recipe

    def _install_recipe(self, recipe: RecipeSpec) -> None:
        self._current_recipe = recipe
        self._selected_recipe_id = recipe.recipe_id
        self._device_specs = {device.device_id: device for device in recipe.devices}
        self._devices = {
            device.device_id: self._create_device_state(device)
            for device in recipe.devices
        }
        self._adapters = {
            device.device_id: create_adapter(device, node=self)
            for device in recipe.devices
        }
        self._clear_faults()
        self._publish_state()

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

    def _command_blocked(self, command: str) -> str | None:
        if is_command_allowed(self._state.system_state, command):
            return None
        return (
            f"{command} is not allowed while system_state={self._state.system_state}. "
            f"allowed={self._state.allowed_commands}"
        )

    def _bringup_devices(self, goal_handle) -> str | None:
        assert self._current_recipe is not None

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

        for device in self._current_recipe.devices:
            if not self._all_dependency_devices_ready(device):
                return (
                    f"Device {device.device_id} has unresolved dependencies: "
                    f"{device.depends_on}"
                )

            adapter = self._adapters[device.device_id]
            self._mark_device(
                device.device_id,
                DeviceState.LIFECYCLE_STARTING,
                DeviceState.HEALTH_UNKNOWN,
                "Running precheck.",
            )
            self._publish_feedback(
                goal_handle, StartSystem, f"Precheck {device.device_id}."
            )

            precheck = adapter.precheck()
            if precheck.is_failure():
                return precheck.summary

            bringup = adapter.bringup()
            if bringup.is_failure():
                return bringup.summary

            ready = adapter.wait_ready(timeout_sec=0.0)
            if ready.is_failure():
                return ready.summary

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
                f"Device {device.device_id} ready.",
            )

        return None

    def _run_home_sequence(self) -> str | None:
        assert self._current_recipe is not None
        for device in self._current_recipe.devices:
            result = self._adapters[device.device_id].home()
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

        for device in reversed(self._current_recipe.devices):
            self._mark_device(
                device.device_id,
                DeviceState.LIFECYCLE_STOPPING,
                DeviceState.HEALTH_UNKNOWN,
                "Shutting down.",
            )
            result = self._adapters[device.device_id].shutdown()
            if result.is_failure() and device.required:
                return result.summary
            self._mark_device(
                device.device_id,
                DeviceState.LIFECYCLE_IDLE,
                DeviceState.HEALTH_UNKNOWN,
                result.summary,
            )
        return None

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
        home_failure = self._run_home_sequence()
        if home_failure is not None:
            self._set_fault(home_failure)
            goal_handle.abort()
            result.success = False
            result.message = home_failure
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_runtime()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
