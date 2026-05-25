from __future__ import annotations

from pathlib import Path
import shutil
import time
from typing import Any

from ament_index_python.packages import get_package_share_directory
from controller_manager_msgs.srv import ListControllers, SwitchController

from .marvin_adapter import SubprocessLaunchSession
from .models import LaunchSpec


class WujihandAdapter:
    DEFAULT_IDENTITY_FILE_RELATIVE_PATH = "config/wujihand_identities.yaml"
    DEFAULT_CONTROLLERS_FILE_RELATIVE_PATH = "config/wujihand_right_controllers.yaml"
    REQUIRED_CONTROLLER_STATES = {
        "joint_state_broadcaster": {"active"},
        "forward_position_controller": {"active"},
    }

    def __init__(
        self,
        *,
        supervisor: Any,
        launch_spec: LaunchSpec,
        launch_path: str,
        launch_arguments: dict[str, object],
    ) -> None:
        self._supervisor = supervisor
        self._launch_spec = launch_spec
        self._launch_path = launch_path
        self._launch_arguments = dict(launch_arguments)
        self._hand_side = self._infer_hand_side()
        self._namespace = self._infer_namespace()
        service_callback_group = getattr(supervisor, "_service_callback_group", None)
        self._list_controllers_client = supervisor.create_client(
            ListControllers,
            self._controller_manager_service_name(),
            callback_group=service_callback_group,
        )
        self._switch_controller_client = supervisor.create_client(
            SwitchController,
            self._controller_manager_switch_service_name(),
            callback_group=service_callback_group,
        )
        self._last_forward_controller_activation_attempt_monotonic = 0.0

    def start_with_ready_retry(self, recipe_timeout_sec: float) -> Any:
        attempts = 1 + max(0, int(self._launch_spec.ready_relaunch_attempts))
        startup_timeout = max(self._launch_spec.startup_timeout_sec, recipe_timeout_sec)
        last_error = ""

        for attempt in range(1, attempts + 1):
            session: SubprocessLaunchSession | None = None
            try:
                self.precheck()
                session = self.bringup(startup_timeout=startup_timeout)
                self.wait_ready(session=session, timeout_sec=startup_timeout)
                if attempt > 1:
                    self._supervisor.get_logger().info(
                        f"Launch {self._launch_spec.device_id} became READY after retry "
                        f"{attempt}/{attempts}."
                    )
                return session
            except Exception as exc:
                last_error = str(exc)
                final_attempt = attempt >= attempts
                self._supervisor.get_logger().warn(
                    f"Launch {self._launch_spec.device_id} did not become READY "
                    f"(attempt {attempt}/{attempts}): {exc}. "
                    + ("Cleaning up." if final_attempt else "Restarting.")
                )
                if session is not None:
                    self.shutdown_session(
                        session,
                        context_label=(
                            f"{self._launch_spec.device_id} startup failure attempt {attempt}"
                        ),
                    )
                if final_attempt:
                    raise
                backoff_sec = max(0.0, float(self._launch_spec.ready_relaunch_backoff_sec))
                if backoff_sec > 0.0:
                    time.sleep(backoff_sec)

        raise RuntimeError(last_error or f"Launch {self._launch_spec.device_id} failed.")

    def precheck(self) -> None:
        if shutil.which("ros2") is None:
            raise RuntimeError("ros2 CLI not found in PATH.")
        if self._hand_side not in {"left", "right"}:
            raise RuntimeError(
                f"{self._launch_spec.device_id}: invalid hand side '{self._hand_side}'."
            )
        identity_file = self._identity_file_path()
        if not identity_file.exists():
            raise RuntimeError(
                f"{self._launch_spec.device_id}: identity file does not exist: {identity_file}"
            )
        controllers_file = self._controllers_file_path()
        if not controllers_file.exists():
            raise RuntimeError(
                f"{self._launch_spec.device_id}: controllers file does not exist: {controllers_file}"
            )

    def bringup(self, *, startup_timeout: float) -> SubprocessLaunchSession:
        session = SubprocessLaunchSession(
            label=self._launch_spec.device_id,
            adapter="wujihand",
            launch_file_path=self._launch_path,
            launch_arguments=self._runtime_launch_arguments(),
            logger=self._supervisor.get_logger(),
        )
        self._supervisor._managed_sessions.append(session)
        session.start()
        if not session.wait_started(max(startup_timeout, 1.0)):
            self._remove_session(session)
            session.close()
            raise RuntimeError(
                f"Launch {self._launch_spec.device_id} did not start within "
                f"{startup_timeout:.1f} s."
            )

        self._supervisor._runtime_manifest.register_device(
            device_id=self._launch_spec.device_id,
            adapter="wujihand",
            metadata=session.metadata(),
        )
        return session

    def wait_ready(self, *, session: SubprocessLaunchSession, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        last_report = 0.0
        ready_since: float | None = None
        stability_window_sec = max(0.0, float(self._launch_spec.ready_stability_window_sec))
        last_controllers: dict[str, str] | None = None

        while time.monotonic() < deadline:
            if not session.is_running():
                raise RuntimeError(
                    f"Launch {self._launch_spec.device_id} exited before READY. "
                    f"log={session.log_path}"
                )

            controllers = self._list_controllers()
            if controllers is not None:
                last_controllers = controllers
                if self._should_activate_forward_controller(controllers):
                    activation_result = self._activate_forward_controller(timeout_sec=2.0)
                    if activation_result:
                        refreshed_controllers = self._list_controllers()
                        if refreshed_controllers is not None:
                            controllers = refreshed_controllers
                            last_controllers = controllers

                missing_services = self._supervisor._missing_services(
                    self._launch_spec.ready_services
                )
                missing_topics = self._supervisor._missing_topics(self._launch_spec.ready_topics)
                missing_messages = self._supervisor._missing_observation_messages(
                    self._launch_spec.ready_topics
                )
                missing_controllers = self._missing_required_controllers(controllers)

                if (
                    not missing_services
                    and not missing_topics
                    and not missing_messages
                    and not missing_controllers
                ):
                    if stability_window_sec <= 0.0:
                        return
                    if ready_since is None:
                        ready_since = time.monotonic()
                    elif time.monotonic() - ready_since >= stability_window_sec:
                        return
                else:
                    ready_since = None

                now = time.monotonic()
                if now - last_report >= 5.0:
                    waiting = []
                    if missing_services:
                        waiting.append(f"services={', '.join(missing_services)}")
                    if missing_topics:
                        waiting.append(f"topics={', '.join(missing_topics)}")
                    if missing_messages:
                        waiting.append(f"messages={', '.join(missing_messages)}")
                    if missing_controllers:
                        waiting.append(f"controllers={', '.join(missing_controllers)}")
                    if ready_since is not None and not waiting and stability_window_sec > 0.0:
                        waiting.append(
                            f"stabilizing={time.monotonic() - ready_since:.1f}/"
                            f"{stability_window_sec:.1f}s"
                        )
                    self._supervisor.get_logger().info(
                        f"Waiting for {self._launch_spec.device_id} READY: {'; '.join(waiting)}"
                    )
                    last_report = now

            time.sleep(0.1)

        missing_services = self._supervisor._missing_services(self._launch_spec.ready_services)
        missing_topics = self._supervisor._missing_topics(self._launch_spec.ready_topics)
        missing_messages = self._supervisor._missing_observation_messages(
            self._launch_spec.ready_topics
        )
        missing_controllers = self._missing_required_controllers(last_controllers or {})
        detail = []
        if missing_services:
            detail.append(f"services={', '.join(missing_services)}")
        if missing_topics:
            detail.append(f"topics={', '.join(missing_topics)}")
        if missing_messages:
            detail.append(f"messages={', '.join(missing_messages)}")
        if missing_controllers:
            detail.append(f"controllers={', '.join(missing_controllers)}")
        raise RuntimeError(
            f"Timed out waiting for {self._launch_spec.device_id} READY after {timeout_sec:.1f}s"
            + (f": {'; '.join(detail)}" if detail else "")
            + f". log={session.log_path}"
        )

    def shutdown_session(
        self,
        session: SubprocessLaunchSession,
        *,
        context_label: str,
    ) -> None:
        try:
            session.shutdown(timeout_sec=12.0)
            session.close()
        finally:
            self._remove_session(session)
            try:
                self._supervisor._runtime_manifest.unregister_device(self._launch_spec.device_id)
            except Exception:
                pass
        self._supervisor.get_logger().info(
            f"{self._launch_spec.device_id}: shutdown completed for {context_label}."
        )

    def _remove_session(self, session: SubprocessLaunchSession) -> None:
        self._supervisor._managed_sessions = [
            item for item in self._supervisor._managed_sessions if item is not session
        ]

    def _infer_hand_side(self) -> str:
        raw_side = str(self._launch_arguments.get("hand_side", "")).strip().lower()
        if raw_side in {"left", "right"}:
            return raw_side
        namespace = str(self._launch_arguments.get("namespace", "")).strip().strip("/")
        if namespace in {"left", "right"}:
            return namespace
        device_id = str(self._launch_spec.device_id).strip().lower()
        if device_id.startswith("left"):
            return "left"
        if device_id.startswith("right"):
            return "right"
        return "right"

    def _infer_namespace(self) -> str:
        raw_namespace = str(self._launch_arguments.get("namespace", "")).strip().strip("/")
        return raw_namespace

    def _absolute_namespace(self, namespace: str) -> str:
        namespace = str(namespace).strip().strip("/")
        return f"/{namespace}" if namespace else ""

    def _controller_manager_service_name(self) -> str:
        namespace = self._absolute_namespace(self._namespace)
        return f"{namespace}/controller_manager/list_controllers"

    def _controller_manager_switch_service_name(self) -> str:
        namespace = self._absolute_namespace(self._namespace)
        return f"{namespace}/controller_manager/switch_controller"

    def _identity_file_path(self) -> Path:
        raw_value = self._launch_arguments.get("identity_file")
        package_share = Path(get_package_share_directory("wujihand_system"))
        if raw_value:
            path = Path(str(raw_value)).expanduser()
            return path if path.is_absolute() else package_share / path
        return package_share / self.DEFAULT_IDENTITY_FILE_RELATIVE_PATH

    def _controllers_file_path(self) -> Path:
        raw_value = self._launch_arguments.get("controllers_file")
        package_share = Path(get_package_share_directory("wujihand_system"))
        if raw_value:
            path = Path(str(raw_value)).expanduser()
            return path if path.is_absolute() else package_share / path
        if self._hand_side == "left":
            return package_share / "config" / "wujihand_left_controllers.yaml"
        return package_share / self.DEFAULT_CONTROLLERS_FILE_RELATIVE_PATH

    def _runtime_launch_arguments(self) -> dict[str, object]:
        arguments = dict(self._launch_arguments)
        arguments.setdefault("use_mock_hardware", False)
        arguments.setdefault("identity_file", str(self._identity_file_path()))
        arguments.setdefault("controllers_file", str(self._controllers_file_path()))
        arguments.setdefault("gui", False)
        arguments.setdefault("use_jsp_gui", False)
        return arguments

    def _required_controller_states(self) -> dict[str, set[str]]:
        required = {
            name: set(states) for name, states in self.REQUIRED_CONTROLLER_STATES.items()
        }
        return required

    def _missing_required_controllers(self, controllers: dict[str, str]) -> list[str]:
        missing: list[str] = []
        for controller_name, required_states in self._required_controller_states().items():
            actual_state = controllers.get(controller_name)
            if actual_state not in required_states:
                actual_label = "missing" if actual_state is None else actual_state
                expected_label = "/".join(sorted(required_states))
                missing.append(f"{controller_name}={actual_label} (expected {expected_label})")
        return missing

    def _list_controllers(self) -> dict[str, str] | None:
        response, _error = self._call_service_response(
            self._list_controllers_client,
            ListControllers.Request(),
            timeout_sec=1.0,
        )
        if response is None:
            return None
        controllers: dict[str, str] = {}
        for controller in getattr(response, "controller", []):
            name = str(getattr(controller, "name", "")).strip()
            state = str(getattr(controller, "state", "")).strip()
            if name:
                controllers[name] = state
        return controllers

    def _should_activate_forward_controller(self, controllers: dict[str, str]) -> bool:
        if controllers.get("joint_state_broadcaster") != "active":
            return False
        return controllers.get("forward_position_controller") == "inactive"

    def _activate_forward_controller(self, timeout_sec: float) -> bool:
        now = time.monotonic()
        if now - self._last_forward_controller_activation_attempt_monotonic < 1.0:
            return False
        self._last_forward_controller_activation_attempt_monotonic = now

        request = SwitchController.Request()
        request.activate_controllers = ["forward_position_controller"]
        request.deactivate_controllers = []
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        whole_seconds = int(timeout_sec)
        request.timeout.sec = whole_seconds
        request.timeout.nanosec = int((timeout_sec - whole_seconds) * 1_000_000_000)

        response, error_message = self._call_service_response(
            client=self._switch_controller_client,
            request=request,
            timeout_sec=timeout_sec,
        )
        if response is None:
            self._supervisor.get_logger().warn(
                f"{self._launch_spec.device_id}: failed to activate forward_position_controller: "
                f"{error_message}"
            )
            return False
        if not bool(getattr(response, "ok", False)):
            self._supervisor.get_logger().warn(
                f"{self._launch_spec.device_id}: forward_position_controller activation returned "
                f"ok=false"
            )
            return False
        return True

    def _call_service_response(self, client, request, timeout_sec: float):
        if client is None:
            return None, "native client unavailable"
        try:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                return None, "service unavailable"
            future = client.call_async(request)
        except Exception as exc:
            return None, str(exc)

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.01)
        if not future.done():
            return None, "request timeout"
        if future.exception() is not None:
            return None, str(future.exception())
        return future.result(), ""

    @staticmethod
    def _launch_arg_enabled(value: object) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)
