from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
import os
import signal
import subprocess
import threading
import time
from typing import Any, Optional

from controller_manager_msgs.srv import ListControllers
import numpy as np
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from marvin_system.srv import GetMotionStatus, SetMotionMode
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger
import yaml

from policy_deployment_interfaces.action import (
    ConnectSystem,
    DisconnectSystem,
    ExecutePolicy,
    GoHome,
    StopPolicy,
)
from policy_deployment_interfaces.msg import DeploymentState, DeviceState, FaultEvent, PolicyChunkStatus
from policy_deployment_interfaces.srv import GetDeploymentState, ListPolicyProfiles

from .command_server import CommandServer
from .image_preprocessing import decode_image_message, preprocess_image_for_topic
from .managed_launch import LaunchManager, ManagedLaunchSession
from .marvin_adapter import PolicyMarvinAdapter
from .models import LaunchSpec, PolicyProfileSpec, SupervisorConfig
from .policy_client import WebsocketPolicyClient
from .policy_profile_loader import discover_policy_profiles, to_msg
from .recipe_loader import discover_recipes, load_recipe
from .runtime_manifest import RuntimeManifest
from .state_machine import Commands, RolloutStates, SystemStates, allowed_commands_for
from .wujihand_adapter import WujihandAdapter


def _optional_path(raw_value: str) -> Optional[Path]:
    if not raw_value:
        return None
    return Path(raw_value).expanduser()


def _optional_text(raw_value: str) -> Optional[str]:
    value = str(raw_value).strip()
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
    return default if parsed < 0.0 else parsed


def _resolve_launch_file_path(package_name: str, launch_file: str) -> str:
    launch_path = Path(launch_file)
    if launch_path.is_absolute() and launch_path.exists():
        return str(launch_path)
    pkg_share = Path(get_package_share_directory(package_name))
    return str((pkg_share / launch_file).resolve())


def _process_matches_launch(device_id: str, process_name: str, cmd: list[str]) -> bool:
    joined = " ".join(cmd)
    process_name = str(process_name)
    matchers: dict[str, tuple[str, ...]] = {
        "policy_cameras": (
            "realsense2_camera_node",
            "orbbec_camera_node",
            "camera_preview_bridge",
            "single_realsense.launch.py",
            "single_orbbec.launch.py",
            "policy_cameras.launch.py",
        ),
        "cam_high": (
            "realsense2_camera_node",
            "camera_preview_bridge",
            "single_realsense.launch.py",
            "cam_high",
        ),
        "cam_left_wrist": (
            "orbbec_camera_node",
            "camera_preview_bridge",
            "single_orbbec.launch.py",
            "cam_left_wrist",
        ),
        "right_wujihand": (
            "/right",
            "__ns:=/right",
            "wujihand_right_control.launch.py",
            "wujihand_right",
            "wujihand_right_controllers",
            "/right/controller_manager",
        ),
        "marvin_dual": (
            "marvin_dual_joint_gui_control.launch.py",
            "policy_marvin_forward.launch.py",
            "marvin_dual_controllers.yaml",
            "marvin_dual_trajectory_controllers.yaml",
            "marvin_system/lib/marvin_system/motion_server",
            "marvin_system/lib/marvin_system/move_group_wrapper.py",
            "marvin_motion_server",
            "forward_position_controller",
            "dual_arm_trajectory_controller",
            "/controller_manager",
            "/marvin",
        ),
    }
    markers = matchers.get(device_id)
    if markers is None:
        return True
    if process_name in {"realsense2_camera_node", "orbbec_camera_node", "camera_preview_bridge"}:
        if device_id == "policy_cameras":
            return True
        if process_name == "realsense2_camera_node":
            return device_id == "cam_high" and any(marker in joined for marker in markers)
        if process_name == "orbbec_camera_node":
            return device_id == "cam_left_wrist" and any(marker in joined for marker in markers)
        return any(marker in joined for marker in markers)
    if process_name == "ros2_control_node":
        return any(marker in joined for marker in markers)
    if process_name.startswith("spawner"):
        if device_id == "right_wujihand":
            return "__ns:=/right" in joined or "/right/controller_manager" in joined
        if device_id == "marvin_dual":
            if "__ns:=/right" in joined or "/right/controller_manager" in joined:
                return False
            return any(
                marker in joined
                for marker in (
                    "joint_state_broadcaster",
                    "forward_position_controller",
                    "dual_arm_trajectory_controller",
                )
            )
        return any(marker in joined for marker in markers)
    return any(marker in joined for marker in markers)


MARVIN_GRAPH_CLEANUP_SERVICES = (
    "/controller_manager/list_controllers",
    "/controller_manager/switch_controller",
    "/marvin_motion/go_home",
    "/marvin_motion/set_mode",
    "/marvin_motion/get_mode",
    "/marvin_motion/get_status",
    "/marvin_motion/set_enabled",
    "/marvin_dual/set_collision_guard_enabled",
)

MARVIN_GRAPH_CLEANUP_NODE_NAMES = (
    "/controller_manager",
    "/robot_state_publisher",
    "/marvin_motion_server",
    "/move_group",
)

MARVIN_EXTERNAL_PROCESS_MATCH_MARKERS = (
    "policy_marvin_forward.launch.py",
    "marvin_dual_joint_gui_control.launch.py",
    "ros2 launch marvin_system",
    "ros2 launch policy_deployment_bringup policy_marvin_forward.launch.py",
    "install/marvin_system/lib/marvin_system/motion_server",
    "install/marvin_system/lib/marvin_system/move_group_wrapper.py",
    "marvin_dual_controllers.yaml",
    "marvin_dual_trajectory_controllers.yaml",
    "marvin_motion_server",
    "dual_arm_trajectory_controller",
)


def _float64_array(values: list[float] | tuple[float, ...]) -> Float64MultiArray:
    msg = Float64MultiArray()
    msg.data = [float(value) for value in values]
    return msg


@dataclass
class _PolicyRuntime:
    active: bool = False
    stop_event: threading.Event = field(default_factory=threading.Event)
    thread: threading.Thread | None = None
    profile_id: str = ""
    prompt: str = ""
    max_steps: int = 0
    open_loop_horizon: int = 0
    steps_executed: int = 0
    chunk_index: int = 0
    step_index: int = 0
    last_infer_ms: float = 0.0
    last_error: str = ""


class PolicyDeploymentSupervisor(Node):
    def __init__(
        self,
        *,
        launch_manager: LaunchManager | None = None,
    ) -> None:
        super().__init__("policy_deployment_supervisor")

        self.launch_manager = launch_manager or LaunchManager()
        self._state_lock = threading.RLock()
        self._command_lock = threading.Lock()
        self._active_command_name = ""
        self._config = self._declare_config()
        self._runtime_manifest = RuntimeManifest()
        self._recipes = discover_recipes(self._config.recipe_directory)
        self._profiles = discover_policy_profiles(self._config.policy_profiles_config)
        self._default_profile_id = self._config.default_policy_profile_id
        if not self._default_profile_id and self._profiles:
            self._default_profile_id = next(iter(self._profiles))

        self._system_state = SystemStates.IDLE
        self._rollout_state = RolloutStates.IDLE
        self._summary = "Ready."
        self._recipe_id = ""
        self._active_profile_id = ""
        self._active_prompt = ""
        self._policy_runtime = _PolicyRuntime()
        self._current_recipe = None
        self._managed_sessions: list[Any] = []
        self._device_status: dict[str, tuple[bool, str]] = {}
        self._active_faults: list[FaultEvent] = []
        self._latest_images: dict[str, Image] = {}
        self._latest_joint_states: dict[str, JointState] = {}
        self._latest_image_receipt_monotonic: dict[str, float] = {}
        self._latest_joint_state_receipt_monotonic: dict[str, float] = {}
        self._command_publishers: dict[str, Any] = {}
        self._policy_client: WebsocketPolicyClient | None = None
        self._service_callback_group = ReentrantCallbackGroup()
        self._go_home_client = self.create_client(
            Trigger,
            "/marvin_motion/go_home",
            callback_group=self._service_callback_group,
        )
        self._list_controllers_client = self.create_client(
            ListControllers,
            "/controller_manager/list_controllers",
            callback_group=self._service_callback_group,
        )
        self._motion_status_client = self.create_client(
            GetMotionStatus,
            "/marvin_motion/get_status",
            callback_group=self._service_callback_group,
        )
        self._motion_set_enabled_client = self.create_client(
            SetBool,
            "/marvin_motion/set_enabled",
            callback_group=self._service_callback_group,
        )
        self._motion_set_mode_client = self.create_client(
            SetMotionMode,
            "/marvin_motion/set_mode",
            callback_group=self._service_callback_group,
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_publisher = self.create_publisher(DeploymentState, "deployment_state", qos)
        self._chunk_publisher = self.create_publisher(PolicyChunkStatus, "policy_chunk_status", 10)
        self.create_service(GetDeploymentState, "get_deployment_state", self._handle_get_state)
        self.create_service(ListPolicyProfiles, "list_policy_profiles", self._handle_list_profiles)
        self._command_server = CommandServer(self, self)
        self._create_observation_subscriptions()
        self.create_timer(1.0 / max(1.0, self._config.publish_rate_hz), self._publish_state)

        stale_cleanup_notes = self._runtime_manifest.cleanup_stale_runtime()
        stale_cleanup_notes.extend(self._runtime_manifest.cleanup_known_orphans())
        for note in stale_cleanup_notes:
            self.get_logger().warn(note)

        self._set_system_state(
            SystemStates.IDLE,
            "Policy deployment supervisor ready.",
        )
        self._publish_state()
        self.get_logger().info(
            "Supervisor ready. "
            f"recipe_id={self._config.recipe_id} "
            f"recipes={len(self._recipes)} profiles={len(self._profiles)} "
            f"actions={self._command_server.describe()}"
        )

    def _declare_config(self) -> SupervisorConfig:
        self.declare_parameter("recipe_id", "default_policy_deployment")
        self.declare_parameter("recipe_directory", "")
        self.declare_parameter("policy_profiles_config", "")
        self.declare_parameter("default_policy_profile_id", "")
        self.declare_parameter("operator_id", "")
        self.declare_parameter("site_name", "")
        self.declare_parameter("connect_timeout_sec", 20.0)
        self.declare_parameter("policy_step_timeout_sec", 5.0)
        self.declare_parameter("observation_max_image_age_sec", 1.0)
        self.declare_parameter("observation_max_joint_state_age_sec", 1.0)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("use_mock_hardware", False)

        return SupervisorConfig(
            recipe_id=str(self.get_parameter("recipe_id").value),
            recipe_directory=_optional_path(self.get_parameter("recipe_directory").value),
            policy_profiles_config=_optional_path(
                self.get_parameter("policy_profiles_config").value
            ),
            default_policy_profile_id=str(
                self.get_parameter("default_policy_profile_id").value
            ).strip(),
            operator_id=str(self.get_parameter("operator_id").value).strip(),
            site_name=str(self.get_parameter("site_name").value).strip(),
            connect_timeout_sec=_nonnegative_float(
                self.get_parameter("connect_timeout_sec").value, 20.0
            ),
            policy_step_timeout_sec=_nonnegative_float(
                self.get_parameter("policy_step_timeout_sec").value, 5.0
            ),
            observation_max_image_age_sec=_nonnegative_float(
                self.get_parameter("observation_max_image_age_sec").value, 1.0
            ),
            observation_max_joint_state_age_sec=_nonnegative_float(
                self.get_parameter("observation_max_joint_state_age_sec").value, 1.0
            ),
            publish_rate_hz=_nonnegative_float(
                self.get_parameter("publish_rate_hz").value, 10.0
            ),
            use_mock_hardware=_coerce_bool(self.get_parameter("use_mock_hardware").value),
        )

    def _create_observation_subscriptions(self) -> None:
        seen_image_topics: set[str] = set()
        seen_state_topics: set[str] = set()
        for profile in self._profiles.values():
            for image_input in profile.image_inputs:
                if image_input.topic in seen_image_topics:
                    continue
                seen_image_topics.add(image_input.topic)
                self.create_subscription(
                    Image,
                    image_input.topic,
                    lambda msg, topic=image_input.topic: self._on_image(topic, msg),
                    qos_profile_sensor_data,
                )
            for topic in (profile.right_arm_state.topic, profile.right_hand_state.topic):
                if topic in seen_state_topics:
                    continue
                seen_state_topics.add(topic)
                self.create_subscription(
                    JointState,
                    topic,
                    lambda msg, topic=topic: self._on_joint_state(topic, msg),
                    10,
                )

    def _wait_for_services(self, service_names: list[str] | tuple[str, ...], timeout_sec: float) -> None:
        targets = {str(name).strip() for name in service_names if str(name).strip()}
        if not targets:
            return

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            available = {name for name, _types in self.get_service_names_and_types()}
            if targets.issubset(available):
                return
            time.sleep(0.1)
        available = {name for name, _types in self.get_service_names_and_types()}
        missing = sorted(targets - available)
        raise RuntimeError(
            f"Timed out waiting for services: {', '.join(missing) if missing else ', '.join(sorted(targets))}"
        )

    def _wait_for_topics(self, topic_names: list[str] | tuple[str, ...], timeout_sec: float) -> None:
        targets = {str(name).strip() for name in topic_names if str(name).strip()}
        if not targets:
            return

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            available = {name for name, _types in self.get_topic_names_and_types()}
            if targets.issubset(available):
                return
            time.sleep(0.1)
        available = {name for name, _types in self.get_topic_names_and_types()}
        missing = sorted(targets - available)
        raise RuntimeError(
            f"Timed out waiting for topics: {', '.join(missing) if missing else ', '.join(sorted(targets))}"
        )

    def _missing_services(self, service_names: tuple[str, ...]) -> list[str]:
        targets = {str(name).strip() for name in service_names if str(name).strip()}
        if not targets:
            return []
        available = {name for name, _types in self.get_service_names_and_types()}
        return sorted(targets - available)

    def _missing_topics(self, topic_names: tuple[str, ...]) -> list[str]:
        targets = {str(name).strip() for name in topic_names if str(name).strip()}
        if not targets:
            return []
        available = {name for name, _types in self.get_topic_names_and_types()}
        return sorted(targets - available)

    def _profile_observation_topics(self) -> tuple[set[str], set[str]]:
        image_topics: set[str] = set()
        joint_state_topics: set[str] = set()
        for profile in self._profiles.values():
            image_topics.update(image_input.topic for image_input in profile.image_inputs)
            joint_state_topics.add(profile.right_arm_state.topic)
            joint_state_topics.add(profile.right_hand_state.topic)
        return image_topics, joint_state_topics

    def _missing_observation_messages(self, topic_names: tuple[str, ...]) -> list[str]:
        targets = {str(name).strip() for name in topic_names if str(name).strip()}
        if not targets:
            return []

        image_topics, joint_state_topics = self._profile_observation_topics()
        with self._state_lock:
            latest_image_topics = set(self._latest_images)
            latest_joint_state_topics = set(self._latest_joint_states)

        missing: list[str] = []
        for topic in sorted(targets):
            if topic in image_topics and topic not in latest_image_topics:
                missing.append(f"{topic} (no image message)")
            elif topic in joint_state_topics and topic not in latest_joint_state_topics:
                missing.append(f"{topic} (no joint_state message)")
        return missing

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

    def _controller_states(self) -> dict[str, str] | None:
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

    def _missing_controllers(self, launch_spec: LaunchSpec) -> list[str]:
        if not launch_spec.ready_controllers:
            return []
        controllers = self._controller_states()
        if controllers is None:
            return ["controller_manager/list_controllers unavailable"]
        missing: list[str] = []
        for name, expected_states in launch_spec.ready_controllers.items():
            actual_state = controllers.get(name)
            if actual_state not in expected_states:
                actual = "missing" if actual_state is None else actual_state
                missing.append(f"{name}={actual} (expected {'/'.join(expected_states)})")
        return missing

    def _motion_status_ready(self, timeout_sec: float) -> tuple[bool, str]:
        response, error = self._call_service_response(
            self._motion_status_client,
            GetMotionStatus.Request(),
            timeout_sec=timeout_sec,
        )
        if response is None:
            return False, error or "status unavailable"
        if not bool(getattr(response, "success", False)):
            return False, str(getattr(response, "message", "")).strip() or "status unhealthy"
        if bool(getattr(response, "motion_busy", False)):
            return False, "motion busy"
        if not bool(getattr(response, "controller_interlock_ok", True)):
            return False, "controller interlock violation"
        if not bool(getattr(response, "hardware_joint_impedance_ok", False)):
            return False, (
                str(getattr(response, "message", "")).strip()
                or "hardware joint impedance not confirmed"
            )
        return True, (
            f"mode={str(getattr(response, 'mode', '')).strip() or 'UNKNOWN'} "
            f"primary={str(getattr(response, 'primary_controller_state', '')).strip() or '-'} "
            f"trajectory={str(getattr(response, 'trajectory_controller_state', '')).strip() or '-'}"
        )

    def _marvin_set_enabled(self, enabled: bool, timeout_sec: float = 3.0) -> tuple[bool, str]:
        request = SetBool.Request()
        request.data = bool(enabled)
        response, error = self._call_service_response(
            self._motion_set_enabled_client,
            request,
            timeout_sec=timeout_sec,
        )
        if response is None:
            return False, error or "set_enabled unavailable"
        if not bool(getattr(response, "success", False)):
            return False, str(getattr(response, "message", "")).strip() or "set_enabled failed"
        return True, str(getattr(response, "message", "")).strip()

    def _marvin_set_mode(self, mode: str, timeout_sec: float = 3.0) -> tuple[bool, str]:
        request = SetMotionMode.Request()
        request.mode = str(mode)
        response, error = self._call_service_response(
            self._motion_set_mode_client,
            request,
            timeout_sec=timeout_sec,
        )
        if response is None:
            return False, error or "set_mode unavailable"
        if not bool(getattr(response, "success", False)):
            return False, str(getattr(response, "message", "")).strip() or "set_mode failed"
        return True, str(getattr(response, "message", "")).strip()

    def _prepare_marvin_safe_shutdown(self, context_label: str) -> None:
        enabled_ok, enabled_message = self._marvin_set_enabled(False, timeout_sec=3.0)
        if not enabled_ok:
            self.get_logger().warn(
                f"Marvin safe shutdown ({context_label}) could not disable motion: {enabled_message}"
            )

        mode_ok, mode_message = self._marvin_set_mode("SAFE_HOLD", timeout_sec=3.0)
        if not mode_ok:
            self.get_logger().warn(
                f"Marvin safe shutdown ({context_label}) could not enter SAFE_HOLD: {mode_message}"
            )

    @staticmethod
    def _read_log_tail(path: str, max_lines: int = 80) -> str:
        try:
            with Path(path).open("r", encoding="utf-8", errors="replace") as handle:
                lines = handle.readlines()
        except OSError:
            return ""
        return "".join(lines[-max_lines:]).rstrip()

    def _early_restart_reason_from_log(self, session: ManagedLaunchSession) -> str:
        log_tail = self._read_log_tail(session.log_path, max_lines=100)
        if not log_tail:
            return ""

        markers = (
            ("SDK send failed 10 times consecutively", "sdk send failures exceeded retry budget during startup"),
            ("No frame update for", "hardware feedback frames stopped updating during startup"),
            ("Collision guard blocked command path", "collision guard blocked startup command path"),
            ("Arm 0 in ERROR state", "left arm entered ERROR during startup"),
            ("Arm 1 in ERROR state", "right arm entered ERROR during startup"),
            ("No state interfaces found to publish", "hardware state interfaces became unavailable during startup"),
            ("Failed to activate controller", "controller activation failed during startup"),
            ("Unable to activate controller", "controller command interfaces became unavailable during startup"),
            ("Joint-impedance watchdog tripped", "joint-impedance watchdog tripped during startup"),
        )
        for marker, reason in markers:
            if marker in log_tail:
                return reason
        return ""

    def _run_system_command(
        self,
        command: list[str],
        timeout_sec: float,
    ) -> subprocess.CompletedProcess[str] | None:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        try:
            return subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=timeout_sec,
                env=env,
            )
        except (OSError, subprocess.TimeoutExpired):
            return None

    def _marvin_current_service_names(self) -> set[str]:
        try:
            return {
                str(name).strip()
                for name, _types in self.get_service_names_and_types()
                if str(name).strip()
            }
        except Exception:
            completed = self._run_system_command(
                ["ros2", "service", "list"],
                timeout_sec=4.0,
            )
            if completed is None or completed.returncode != 0:
                return set()
            return {line.strip() for line in completed.stdout.splitlines() if line.strip()}

    def _marvin_current_node_names(self) -> set[str]:
        node_names: set[str] = set()
        try:
            for name, namespace in self.get_node_names_and_namespaces():
                raw_name = str(name).strip().lstrip("/")
                if not raw_name:
                    continue
                raw_namespace = str(namespace).strip()
                if raw_namespace and raw_namespace != "/":
                    node_names.add(f"{raw_namespace.rstrip('/')}/{raw_name}")
                else:
                    node_names.add(f"/{raw_name}")
        except Exception:
            node_names.clear()

        if node_names:
            return node_names

        completed = self._run_system_command(
            ["ros2", "node", "list"],
            timeout_sec=4.0,
        )
        if completed is None or completed.returncode != 0:
            return set()
        return {line.strip() for line in completed.stdout.splitlines() if line.strip()}

    def _marvin_local_client_service_names(self) -> set[str]:
        try:
            client_name_list = self.get_client_names_and_types_by_node(
                self.get_name(),
                self.get_namespace(),
            )
        except Exception:
            return set()

        known_local_clients = set(MARVIN_GRAPH_CLEANUP_SERVICES)
        return {
            service_name
            for name, _types in client_name_list
            if (service_name := str(name).strip()) in known_local_clients
        }

    def _marvin_stale_graph_resources(self) -> tuple[list[str], list[str]]:
        current_services = self._marvin_current_service_names()
        current_nodes = self._marvin_current_node_names()
        local_client_services = self._marvin_local_client_service_names()
        stale_services = [
            service_name
            for service_name in MARVIN_GRAPH_CLEANUP_SERVICES
            if service_name in current_services
            and service_name not in local_client_services
        ]
        stale_nodes = [
            node_name
            for node_name in MARVIN_GRAPH_CLEANUP_NODE_NAMES
            if node_name in current_nodes
        ]
        return stale_services, stale_nodes

    def _wait_for_marvin_graph_cleanup(
        self,
        *,
        timeout_sec: float,
        context_label: str,
    ) -> tuple[bool, str]:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        clear_since = 0.0
        last_stale_services: list[str] = []
        last_stale_nodes: list[str] = []
        stability_window_sec = 0.5

        while time.monotonic() < deadline:
            stale_services, stale_nodes = self._marvin_stale_graph_resources()
            last_stale_services = stale_services
            last_stale_nodes = stale_nodes

            if not stale_services and not stale_nodes:
                if clear_since <= 0.0:
                    clear_since = time.monotonic()
                if time.monotonic() - clear_since >= stability_window_sec:
                    return True, (
                        f"Marvin ROS graph cleanup confirmed after {context_label}."
                    )
            else:
                clear_since = 0.0

            time.sleep(0.2)

        if not last_stale_services and set(last_stale_nodes) == {"/move_group"}:
            self.get_logger().warn(
                "Tolerating lingering /move_group graph residue after "
                f"{context_label}; process shutdown already completed."
            )
            return True, (
                "Marvin subprocesses stopped after "
                f"{context_label}; lingering /move_group graph residue tolerated."
            )

        if not last_stale_services and not last_stale_nodes:
            self.get_logger().warn(
                "ROS graph cleanup did not satisfy the "
                f"{stability_window_sec:.1f}s stability window after {context_label}, "
                "but no Marvin stale service or node was observed at the final check."
            )
            return True, (
                f"Marvin subprocesses stopped after {context_label}; "
                "no stale graph resources observed."
            )

        details = []
        if last_stale_services:
            details.append("services=" + ", ".join(last_stale_services))
        if last_stale_nodes:
            details.append("nodes=" + ", ".join(last_stale_nodes))
        detail_suffix = " | ".join(details) if details else "resources=unknown"
        return False, (
            "Marvin ROS graph still contains stale resources after "
            f"{context_label}: {detail_suffix}"
        )

    def _marvin_external_cleanup_match_reason(self, cmd: str) -> str:
        lowered = cmd.lower()
        for marker in MARVIN_EXTERNAL_PROCESS_MATCH_MARKERS:
            if marker in cmd:
                return f"marker={marker}"

        if "ros2_control_node" in lowered and (
            "marvin_dual_controllers.yaml" in lowered
            or "marvin_dual_trajectory_controllers.yaml" in lowered
            or "marvin_dual_joint_gui_control" in lowered
        ):
            return "ros2_control_node+marvin controller config"

        if "controller_manager/spawner" in lowered:
            if "__ns:=/right" in lowered or "/right/controller_manager" in lowered:
                return ""
            if (
                "dual_arm_trajectory_controller" in lowered
                or "forward_position_controller" in lowered
                or "joint_state_broadcaster" in lowered
            ):
                return "spawner+marvin controller"

        if "robot_state_publisher" in lowered and (
            "marvin_system" in lowered or "marvin_dual" in lowered
        ):
            return "robot_state_publisher+marvin"

        return ""

    def _marvin_external_cleanup_candidate_process_groups(
        self,
    ) -> tuple[list[tuple[int, str, str]], list[str]]:
        completed = self._run_system_command(
            ["ps", "-eo", "pid=,pgid=,args="],
            timeout_sec=4.0,
        )
        if completed is None or completed.returncode != 0:
            return [], []

        current_pid = os.getpid()
        current_pgid = os.getpgrp()
        candidate_by_pgid: dict[int, tuple[str, str]] = {}
        suspicious_samples: list[str] = []
        for raw_line in completed.stdout.splitlines():
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split(None, 2)
            if len(parts) < 3:
                continue
            try:
                pid = int(parts[0])
                pgid = int(parts[1])
            except ValueError:
                continue
            if pid <= 0 or pgid <= 0:
                continue
            if pid == current_pid or pgid == current_pgid:
                continue

            cmd = parts[2].strip()
            reason = self._marvin_external_cleanup_match_reason(cmd)
            if reason:
                candidate_by_pgid.setdefault(pgid, (cmd, reason))
                continue

            lowered = cmd.lower()
            if (
                "ros2_control_node" in lowered
                or "motion_server" in lowered
                or "move_group" in lowered
                or "robot_state_publisher" in lowered
                or "spawner" in lowered
                or "marvin" in lowered
            ):
                suspicious_samples.append(cmd)

        candidates = [
            (pgid, cmd, reason)
            for pgid, (cmd, reason) in sorted(candidate_by_pgid.items())
        ]
        return candidates, suspicious_samples[:8]

    def _signal_external_process_group(
        self,
        pgid: int,
        sig: signal.Signals,
        timeout_sec: float,
    ) -> bool:
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True

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

    def _scavenge_marvin_external_process_groups(
        self,
        *,
        context_label: str,
    ) -> tuple[bool, str]:
        candidates, suspicious_samples = self._marvin_external_cleanup_candidate_process_groups()
        if not candidates:
            for sample in suspicious_samples:
                self.get_logger().warn(
                    "External cleanup found suspicious process during "
                    f"{context_label} but no Marvin match rule claimed it. cmd={sample}"
                )
            return False, f"no external Marvin process groups matched for {context_label}."

        for pgid, cmd, reason in candidates:
            self.get_logger().warn(
                "Scavenging external Marvin process group "
                f"pgid={pgid} during {context_label}. reason={reason} cmd={cmd}"
            )

        failed_pgids: list[str] = []
        for pgid, _cmd, _reason in candidates:
            stopped = False
            for sig, wait_s in (
                (signal.SIGINT, 8.0),
                (signal.SIGTERM, 4.0),
                (signal.SIGKILL, 2.0),
            ):
                if self._signal_external_process_group(pgid, sig, timeout_sec=wait_s):
                    stopped = True
                    break
            if not stopped:
                failed_pgids.append(str(pgid))

        if failed_pgids:
            return False, "external Marvin cleanup failed for pgid=" + ", ".join(failed_pgids)

        time.sleep(0.5)
        return True, f"external Marvin process cleanup completed for {context_label}."

    def _ensure_marvin_graph_cleanup(
        self,
        *,
        timeout_sec: float,
        context_label: str,
        allow_external_cleanup: bool,
    ) -> None:
        cleanup_ok, cleanup_message = self._wait_for_marvin_graph_cleanup(
            timeout_sec=timeout_sec,
            context_label=context_label,
        )
        if cleanup_ok:
            return
        if not allow_external_cleanup:
            raise RuntimeError(cleanup_message)

        scavenger_ok, scavenger_message = self._scavenge_marvin_external_process_groups(
            context_label=context_label,
        )
        if not scavenger_ok:
            raise RuntimeError(f"{cleanup_message}; {scavenger_message}")

        second_ok, second_message = self._wait_for_marvin_graph_cleanup(
            timeout_sec=timeout_sec,
            context_label=f"{context_label} after external cleanup",
        )
        if not second_ok:
            raise RuntimeError(second_message)
        self.get_logger().info(scavenger_message)

    def _wait_for_launch_ready(
        self,
        session: ManagedLaunchSession,
        launch_spec: LaunchSpec,
        timeout_sec: float,
    ) -> None:
        deadline = time.monotonic() + timeout_sec
        last_report = 0.0
        ready_since: float | None = None
        ready_stability_window_sec = max(0.0, float(launch_spec.ready_stability_window_sec))
        while time.monotonic() < deadline:
            if not session.is_running():
                raise RuntimeError(
                    f"Launch {launch_spec.device_id} exited before READY. "
                    f"log={session.log_path}"
                )

            missing_services = self._missing_services(launch_spec.ready_services)
            missing_topics = self._missing_topics(launch_spec.ready_topics)
            missing_messages = self._missing_observation_messages(launch_spec.ready_topics)
            missing_controllers = self._missing_controllers(launch_spec)
            motion_ready = True
            motion_status = ""
            prerequisites_ready = (
                not missing_services
                and not missing_topics
                and not missing_messages
                and not missing_controllers
            )
            if launch_spec.ready_motion_status and prerequisites_ready:
                motion_ready, motion_status = self._motion_status_ready(
                    timeout_sec=launch_spec.ready_motion_status_timeout_sec
                )
            if (
                not missing_services
                and not missing_topics
                and not missing_messages
                and not missing_controllers
                and motion_ready
            ):
                if ready_stability_window_sec <= 0.0:
                    return
                if ready_since is None:
                    ready_since = time.monotonic()
                elif time.monotonic() - ready_since >= ready_stability_window_sec:
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
                if not motion_ready:
                    waiting.append(f"motion={motion_status}")
                if (
                    ready_since is not None
                    and not waiting
                    and ready_stability_window_sec > 0.0
                ):
                    waiting.append(
                        f"stabilizing={time.monotonic() - ready_since:.1f}/"
                        f"{ready_stability_window_sec:.1f}s"
                    )
                self.get_logger().info(
                    f"Waiting for {launch_spec.device_id} READY: {'; '.join(waiting)}"
                )
                last_report = now
            early_restart_reason = self._early_restart_reason_from_log(session)
            if early_restart_reason:
                raise RuntimeError(
                    f"{launch_spec.device_id} startup failed early: {early_restart_reason}. "
                    f"log={session.log_path}"
                )
            time.sleep(0.1)

        missing_services = self._missing_services(launch_spec.ready_services)
        missing_topics = self._missing_topics(launch_spec.ready_topics)
        missing_messages = self._missing_observation_messages(launch_spec.ready_topics)
        missing_controllers = self._missing_controllers(launch_spec)
        motion_ready = True
        motion_status = ""
        prerequisites_ready = (
            not missing_services
            and not missing_topics
            and not missing_messages
            and not missing_controllers
        )
        if launch_spec.ready_motion_status and prerequisites_ready:
            motion_ready, motion_status = self._motion_status_ready(
                timeout_sec=launch_spec.ready_motion_status_timeout_sec
            )
        detail = []
        if missing_services:
            detail.append(f"services={', '.join(missing_services)}")
        if missing_topics:
            detail.append(f"topics={', '.join(missing_topics)}")
        if missing_messages:
            detail.append(f"messages={', '.join(missing_messages)}")
        if missing_controllers:
            detail.append(f"controllers={', '.join(missing_controllers)}")
        if not motion_ready:
            detail.append(f"motion={motion_status}")
        raise RuntimeError(
            f"Timed out waiting for {launch_spec.device_id} READY after "
            f"{timeout_sec:.1f} s"
            + (f": {'; '.join(detail)}" if detail else "")
            + f". log={session.log_path}"
        )

    def _start_launch_with_ready_retry(
        self,
        launch_spec: LaunchSpec,
        recipe_timeout_sec: float,
    ) -> Any:
        attempts = 1 + max(0, int(launch_spec.ready_relaunch_attempts))
        startup_timeout = max(launch_spec.startup_timeout_sec, recipe_timeout_sec)
        last_error = ""
        launch_path = _resolve_launch_file_path(launch_spec.package, launch_spec.launch_file)

        if launch_spec.adapter == "marvin" or launch_spec.device_id == "marvin_dual":
            marvin_adapter = PolicyMarvinAdapter(
                supervisor=self,
                launch_spec=launch_spec,
                launch_path=launch_path,
                launch_arguments={
                    **launch_spec.arguments,
                    "use_mock_hardware": self._config.use_mock_hardware,
                },
            )
            return marvin_adapter.start_with_ready_retry(recipe_timeout_sec)
        if launch_spec.adapter == "wujihand":
            wujihand_adapter = WujihandAdapter(
                supervisor=self,
                launch_spec=launch_spec,
                launch_path=launch_path,
                launch_arguments={
                    **launch_spec.arguments,
                    "use_mock_hardware": self._config.use_mock_hardware,
                },
            )
            return wujihand_adapter.start_with_ready_retry(recipe_timeout_sec)

        launch_arguments = dict(launch_spec.arguments)
        if launch_spec.adapter != "camera":
            launch_arguments["use_mock_hardware"] = self._config.use_mock_hardware

        for attempt in range(1, attempts + 1):
            session = ManagedLaunchSession(
                launch_manager=self.launch_manager,
                label=launch_spec.device_id,
                adapter=launch_spec.adapter,
                launch_file_path=launch_path,
                launch_arguments=launch_arguments,
                process_matcher=lambda process_name, cmd, device_id=launch_spec.device_id: (
                    _process_matches_launch(device_id, process_name, cmd)
                ),
                logger=self.get_logger(),
            )
            self._managed_sessions.append(session)
            try:
                session.start()
                if not session.wait_started(startup_timeout):
                    raise RuntimeError(
                        f"Launch {launch_spec.device_id} did not start within "
                        f"{startup_timeout:.1f} s. log={session.log_path}"
                    )
                self._wait_for_launch_ready(session, launch_spec, startup_timeout)
                if attempt > 1:
                    self.get_logger().info(
                        f"Launch {launch_spec.device_id} became READY after retry "
                        f"{attempt}/{attempts}."
                    )
                return session
            except Exception as exc:
                last_error = str(exc)
                self.get_logger().warn(
                    f"Launch {launch_spec.device_id} did not become READY "
                    f"(attempt {attempt}/{attempts}): {exc}. "
                    + ("Restarting." if attempt < attempts else "Cleaning up.")
                )
                try:
                    session.shutdown(timeout_sec=0.0)
                    session.close()
                finally:
                    self._managed_sessions = [
                        item for item in self._managed_sessions if item is not session
                    ]
                if attempt >= attempts:
                    raise
                backoff_sec = max(0.0, float(launch_spec.ready_relaunch_backoff_sec))
                if backoff_sec > 0.0:
                    time.sleep(backoff_sec)

        raise RuntimeError(last_error or f"Launch {launch_spec.device_id} failed.")


    def _on_image(self, topic: str, msg: Image) -> None:
        with self._state_lock:
            self._latest_images[topic] = msg
            self._latest_image_receipt_monotonic[topic] = time.monotonic()

    def _on_joint_state(self, topic: str, msg: JointState) -> None:
        with self._state_lock:
            self._latest_joint_states[topic] = msg
            self._latest_joint_state_receipt_monotonic[topic] = time.monotonic()

    def try_begin_command(self, command: str) -> tuple[bool, str]:
        with self._command_lock:
            if self._active_command_name:
                return False, f"Command {command} blocked by {self._active_command_name}"
            if not self._is_command_allowed(command):
                return False, f"Command {command} is not allowed in state {self._system_state}"
            self._active_command_name = command
            return True, ""

    def finish_command(self, command: str, *, success: bool, detail: str = "") -> None:
        del command
        with self._command_lock:
            self._active_command_name = ""
        self._set_summary(detail if detail else self._summary)
        self._publish_state()

    def _is_command_allowed(self, command: str) -> bool:
        with self._state_lock:
            return command in allowed_commands_for(self._system_state)

    def _set_system_state(self, state: str, summary: str) -> None:
        with self._state_lock:
            self._system_state = state
            self._summary = summary
            recipe_id = self._recipe_id or self._config.recipe_id
        try:
            self._runtime_manifest.set_context(recipe_id=recipe_id, system_state=state)
        except Exception:
            pass
        self._publish_state()

    def _set_rollout_state(self, state: str, summary: str = "") -> None:
        with self._state_lock:
            self._rollout_state = state
            if summary:
                self._summary = summary
        self._publish_state()

    def _set_summary(self, summary: str) -> None:
        with self._state_lock:
            self._summary = summary

    def _handle_get_state(self, request, response):
        del request
        response.state = self._make_state_msg()
        return response

    def _handle_list_profiles(self, request, response):
        del request
        response.profiles = [to_msg(profile) for profile in self._profiles.values()]
        return response

    def _publish_state(self) -> None:
        self._state_publisher.publish(self._make_state_msg())

    def _make_state_msg(self) -> DeploymentState:
        msg = DeploymentState()
        msg.stamp = self.get_clock().now().to_msg()
        with self._state_lock:
            msg.system_state = self._system_state
            msg.recipe_id = self._recipe_id or self._config.recipe_id
            msg.active_profile_id = self._active_profile_id
            msg.active_prompt = self._active_prompt
            msg.rollout_state = self._rollout_state
            msg.summary = self._summary
            msg.chunk_index = int(self._policy_runtime.chunk_index)
            msg.step_index = int(self._policy_runtime.step_index)
            msg.last_infer_ms = float(self._policy_runtime.last_infer_ms)
            msg.allowed_commands = allowed_commands_for(self._system_state)
            msg.devices = self._build_device_state_messages()
            msg.active_faults = list(self._active_faults)
        return msg

    def _build_device_state_messages(self) -> list[DeviceState]:
        devices: list[DeviceState] = []
        for session in self._managed_sessions:
            device = DeviceState()
            device.device_id = session.label
            device.device_class = getattr(session, "adapter", "launch")
            running = session.is_running()
            device.lifecycle_state = (
                DeviceState.LIFECYCLE_ACTIVE if running else DeviceState.LIFECYCLE_STOPPING
            )
            device.health_state = DeviceState.HEALTH_OK if running else DeviceState.HEALTH_DEGRADED
            device.is_required = True
            device.last_ready_stamp = self.get_clock().now().to_msg()
            device.summary = session.log_path
            devices.append(device)
        return devices

    def _publish_chunk_status(
        self,
        *,
        profile_id: str,
        chunk_index: int,
        chunk_length: int,
        step_index: int,
        last_infer_ms: float,
        state: str,
        detail: str,
    ) -> None:
        msg = PolicyChunkStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.profile_id = profile_id
        msg.chunk_index = int(chunk_index)
        msg.chunk_length = int(chunk_length)
        msg.step_index = int(step_index)
        msg.last_infer_ms = float(last_infer_ms)
        msg.state = state
        msg.detail = detail
        self._chunk_publisher.publish(msg)

    def _load_recipe_or_fail(self, recipe_id: str):
        recipe = load_recipe(self._config.recipe_directory, recipe_id)
        if recipe is None:
            raise RuntimeError(f"Recipe not found: {recipe_id}")
        return recipe

    def _load_profile_or_fail(self, profile_id: str) -> PolicyProfileSpec:
        profile = self._profiles.get(profile_id)
        if profile is None:
            raise RuntimeError(f"Policy profile not found: {profile_id}")
        return profile

    def _validate_launch_dependencies(self, launches: list[LaunchSpec]) -> None:
        known_device_ids = {launch.device_id for launch in launches}
        unresolved = [
            launch.device_id
            for launch in launches
            if any(dependency_id not in known_device_ids for dependency_id in launch.depends_on)
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
                launch.device_id
                for launch in launches
                if launch.device_id in pending_device_ids
                and all(dependency_id in ready_device_ids for dependency_id in launch.depends_on)
            }
            if not progress_ids:
                unresolved_pending = [
                    launch.device_id
                    for launch in launches
                    if launch.device_id in pending_device_ids
                ]
                raise RuntimeError(
                    "Unable to resolve startup order; dependency cycle detected among "
                    f"{unresolved_pending}."
                )
            pending_device_ids -= progress_ids
            ready_device_ids |= progress_ids

    def _startup_layers(self, launches: list[LaunchSpec]) -> list[list[LaunchSpec]]:
        self._validate_launch_dependencies(launches)

        remaining_ids = {launch.device_id for launch in launches}
        ordered_launches = list(launches)
        satisfied_ids: set[str] = set()
        layers: list[list[LaunchSpec]] = []

        while remaining_ids:
            layer = [
                launch
                for launch in ordered_launches
                if launch.device_id in remaining_ids
                and all(dependency_id in satisfied_ids for dependency_id in launch.depends_on)
            ]
            if not layer:
                unresolved = [
                    launch.device_id
                    for launch in ordered_launches
                    if launch.device_id in remaining_ids
                ]
                raise RuntimeError(
                    "Unable to resolve startup layers; dependency cycle or missing dependency "
                    f"detected among {unresolved}."
                )

            layers.append(layer)
            for launch in layer:
                remaining_ids.remove(launch.device_id)
                satisfied_ids.add(launch.device_id)

        return layers

    def _bringup_recipe_launches(self, recipe: Any) -> None:
        launches = list(recipe.launches)
        if not launches:
            raise RuntimeError(f"Recipe {recipe.recipe_id} does not define any launches.")

        layers = self._startup_layers(launches)
        for layer_index, layer in enumerate(layers, start=1):
            layer_ids = ", ".join(launch.device_id for launch in layer)
            self.get_logger().info(
                f"Starting startup layer {layer_index}/{len(layers)}: {layer_ids}"
            )
            for launch_spec in layer:
                self._start_launch_with_ready_retry(launch_spec, recipe.ready_timeout_sec)

    def execute_connect_system(self, goal_handle):
        request = goal_handle.request
        result = ConnectSystem.Result()

        if self._system_state != SystemStates.IDLE:
            goal_handle.abort()
            result.success = False
            result.message = f"System must be IDLE before connect, current state: {self._system_state}"
            return result

        recipe_id = str(request.recipe_id).strip() or self._config.recipe_id
        profile_id = str(request.policy_profile_id).strip() or self._default_profile_id
        try:
            recipe = self._load_recipe_or_fail(recipe_id)
            profile = self._load_profile_or_fail(profile_id)
        except Exception as exc:
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result

        self._recipe_id = recipe.recipe_id
        self._active_profile_id = profile.profile_id
        self._active_prompt = profile.default_prompt
        self._current_recipe = recipe
        with self._state_lock:
            self._latest_images.clear()
            self._latest_joint_states.clear()
            self._latest_image_receipt_monotonic.clear()
            self._latest_joint_state_receipt_monotonic.clear()
        self._set_system_state(SystemStates.STARTING, f"Starting recipe {recipe.recipe_id}.")

        try:
            self._managed_sessions = []
            self._bringup_recipe_launches(recipe)
            self._set_system_state(
                SystemStates.READY,
                f"Recipe {recipe.recipe_id} ready with profile {profile.profile_id}.",
            )
            goal_handle.succeed()
            result.success = True
            result.message = f"Connected recipe {recipe.recipe_id} with profile {profile.profile_id}"
            return result
        except Exception as exc:
            self._record_fault(
                fault_id="connect_system",
                device_id=recipe.recipe_id,
                severity=FaultEvent.SEVERITY_BLOCKING,
                summary="Connect failed",
                detail=str(exc),
            )
            self._shutdown_managed_sessions()
            self._recipe_id = ""
            self._active_profile_id = ""
            self._active_prompt = ""
            self._current_recipe = None
            self._set_system_state(SystemStates.FAULT, f"Connect failed: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result

    def execute_disconnect_system(self, goal_handle):
        request = goal_handle.request
        del request
        result = DisconnectSystem.Result()

        self._stop_policy_rollout(wait=True)
        self._shutdown_managed_sessions()
        self._clear_faults()
        self._recipe_id = ""
        self._active_profile_id = ""
        self._active_prompt = ""
        self._current_recipe = None
        with self._state_lock:
            self._latest_images.clear()
            self._latest_joint_states.clear()
            self._latest_image_receipt_monotonic.clear()
            self._latest_joint_state_receipt_monotonic.clear()
        self._set_system_state(SystemStates.IDLE, "Disconnected from recipe.")
        goal_handle.succeed()
        result.success = True
        result.message = "System disconnected"
        return result

    def execute_policy(self, goal_handle):
        request = goal_handle.request
        result = ExecutePolicy.Result()

        if self._system_state != SystemStates.READY:
            goal_handle.abort()
            result.success = False
            result.message = f"System must be READY before execute, current state: {self._system_state}"
            return result
        if self._policy_runtime.active:
            goal_handle.abort()
            result.success = False
            result.message = "A policy rollout is already active"
            return result

        try:
            profile = self._load_profile_or_fail(
                str(request.policy_profile_id).strip() or self._active_profile_id
            )
        except Exception as exc:
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result
        prompt = str(request.prompt).strip() or profile.default_prompt
        max_steps = int(request.max_steps) if request.max_steps else 0
        open_loop_horizon = int(request.open_loop_horizon) if request.open_loop_horizon else 0

        self._policy_runtime = _PolicyRuntime(
            active=True,
            profile_id=profile.profile_id,
            prompt=prompt,
            max_steps=max_steps,
            open_loop_horizon=open_loop_horizon or profile.open_loop_horizon,
        )
        self._active_prompt = prompt
        self._set_system_state(SystemStates.EXECUTING, f"Executing policy profile {profile.profile_id}.")
        self._set_rollout_state(RolloutStates.EXECUTING, f"Executing policy profile {profile.profile_id}.")
        self._policy_runtime.stop_event.clear()
        self._policy_runtime.thread = threading.Thread(
            target=self._policy_rollout_worker,
            args=(profile,),
            name="policy_deployment_rollout",
            daemon=True,
        )
        self._policy_runtime.thread.start()
        goal_handle.succeed()
        result.success = True
        result.message = f"Policy rollout started for {profile.profile_id}"
        result.steps_executed = 0
        return result

    def execute_stop_policy(self, goal_handle):
        request = goal_handle.request
        del request
        result = StopPolicy.Result()

        self._stop_policy_rollout(wait=True)
        if self._system_state == SystemStates.FAULT:
            result.success = False
            result.message = "Policy rollout stopped with fault"
            goal_handle.abort()
            return result

        goal_handle.succeed()
        result.success = True
        result.message = "Policy rollout stopped"
        return result

    def execute_go_home(self, goal_handle):
        request = goal_handle.request
        result = GoHome.Result()

        self._stop_policy_rollout(wait=True)
        profile = self._profiles.get(self._active_profile_id)
        if profile is None:
            goal_handle.abort()
            result.success = False
            result.message = "No active policy profile to home"
            return result

        try:
            self._set_rollout_state(RolloutStates.STOPPING, "Going home.")
            if self._go_home_client.service_is_ready() or self._go_home_client.wait_for_service(timeout_sec=5.0):
                future = self._go_home_client.call_async(Trigger.Request())
                self._wait_for_future(future, timeout_sec=max(1.0, self._config.connect_timeout_sec))
                response = future.result()
                if response is None or not response.success:
                    raise RuntimeError(getattr(response, "message", "go_home failed"))
                if profile.right_hand_command.home:
                    self._publish_command(
                        profile.right_hand_command.topic,
                        profile.right_hand_command.home,
                    )
            else:
                if not profile.marvin_command.home:
                    raise RuntimeError(
                        "/marvin_motion/go_home is unavailable and no Marvin home fallback is configured"
                    )
                self._publish_home_commands(profile)

            if bool(request.arm_after_home):
                self._set_system_state(SystemStates.READY, f"Profile {profile.profile_id} homed and ready.")
            else:
                self._set_system_state(SystemStates.READY, f"Profile {profile.profile_id} homed.")
            goal_handle.succeed()
            result.success = True
            result.message = "Home completed"
            return result
        except Exception as exc:
            self._record_fault(
                fault_id="go_home",
                device_id="marvin_motion",
                severity=FaultEvent.SEVERITY_RECOVERABLE,
                summary="Go home failed",
                detail=str(exc),
            )
            self._set_system_state(SystemStates.FAULT, f"Go home failed: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            return result

    def _policy_rollout_worker(self, profile: PolicyProfileSpec) -> None:
        runtime = self._policy_runtime
        try:
            client = WebsocketPolicyClient(
                host=profile.server_host,
                port=profile.server_port,
                connect_timeout_sec=self._config.connect_timeout_sec,
            )
            self._policy_client = client
            client.connect()
            self.get_logger().info(f"Connected to policy server: {client.metadata}")

            chunk_index = 0
            step_index = 0
            max_steps = runtime.max_steps if runtime.max_steps > 0 else 0
            open_loop_horizon = runtime.open_loop_horizon or profile.open_loop_horizon
            control_period = 1.0 / max(1.0, profile.control_rate_hz)

            while not runtime.stop_event.is_set():
                if max_steps and step_index >= max_steps:
                    break
                observation = self._build_observation(profile, runtime.prompt)
                infer_start = time.monotonic()
                response = client.infer(observation)
                infer_ms = (time.monotonic() - infer_start) * 1000.0
                actions = response.get("actions")
                if actions is None:
                    raise RuntimeError("Policy server response did not contain actions")

                action_chunk = np.asarray(actions, dtype=np.float32)
                if action_chunk.ndim == 1:
                    action_chunk = action_chunk[None, :]
                if action_chunk.ndim != 2:
                    raise RuntimeError(
                        f"Policy returned action chunk with shape {action_chunk.shape}, expected [horizon, dim]"
                    )
                if action_chunk.shape[0] <= 0:
                    raise RuntimeError("Policy returned an empty action chunk")
                if action_chunk.shape[-1] < profile.action_dim:
                    raise RuntimeError(
                        f"Policy returned action dim {action_chunk.shape[-1]}, expected at least {profile.action_dim}"
                    )
                if action_chunk.shape[-1] > profile.action_dim:
                    action_chunk = action_chunk[..., : profile.action_dim]
                if not np.all(np.isfinite(action_chunk)):
                    raise RuntimeError("Policy returned non-finite action values")

                runtime.last_infer_ms = float(infer_ms)
                runtime.chunk_index = chunk_index
                runtime.step_index = step_index
                self._publish_chunk_status(
                    profile_id=profile.profile_id,
                    chunk_index=chunk_index,
                    chunk_length=int(action_chunk.shape[0]),
                    step_index=step_index,
                    last_infer_ms=infer_ms,
                    state="EXECUTING",
                    detail="chunk received",
                )

                for inner_index, action_row in enumerate(action_chunk):
                    if runtime.stop_event.is_set():
                        break
                    if max_steps and step_index >= max_steps:
                        break
                    self._execute_action_row(profile, action_row)
                    step_index += 1
                    runtime.step_index = step_index
                    runtime.steps_executed = step_index
                    self._publish_chunk_status(
                        profile_id=profile.profile_id,
                        chunk_index=chunk_index,
                        chunk_length=int(action_chunk.shape[0]),
                        step_index=step_index,
                        last_infer_ms=infer_ms,
                        state="EXECUTING",
                        detail=f"step {inner_index + 1}/{action_chunk.shape[0]}",
                    )
                    time.sleep(control_period)
                    if inner_index + 1 >= open_loop_horizon:
                        break
                chunk_index += 1

            if runtime.stop_event.is_set():
                self._set_rollout_state(RolloutStates.IDLE, "Policy rollout stopped.")
            else:
                self._set_rollout_state(RolloutStates.IDLE, "Policy rollout completed.")
        except Exception as exc:
            runtime.last_error = str(exc)
            self._record_fault(
                fault_id="policy_rollout",
                device_id=profile.profile_id,
                severity=FaultEvent.SEVERITY_FATAL,
                summary="Policy rollout failed",
                detail=str(exc),
            )
            self._set_rollout_state(RolloutStates.FAULT, f"Policy rollout failed: {exc}")
            self._set_system_state(SystemStates.FAULT, f"Policy rollout failed: {exc}")
            self.get_logger().error(f"Policy rollout failed: {exc}")
        finally:
            try:
                if self._policy_client is not None:
                    self._policy_client.close()
            finally:
                self._policy_client = None
                runtime.active = False
                runtime.thread = None
                runtime.stop_event.clear()
                if self._system_state != SystemStates.FAULT and self._rollout_state != RolloutStates.FAULT:
                    self._set_rollout_state(RolloutStates.IDLE, "Idle.")
                    if self._system_state == SystemStates.EXECUTING:
                        self._set_system_state(SystemStates.READY, "Ready.")

    def _build_observation(self, profile: PolicyProfileSpec, prompt: str) -> dict[str, Any]:
        with self._state_lock:
            latest_images = dict(self._latest_images)
            latest_joint_states = dict(self._latest_joint_states)
            latest_image_receipts = dict(self._latest_image_receipt_monotonic)
            latest_joint_state_receipts = dict(self._latest_joint_state_receipt_monotonic)

        now = time.monotonic()
        max_image_age_sec = max(0.0, float(self._config.observation_max_image_age_sec))
        max_joint_state_age_sec = max(0.0, float(self._config.observation_max_joint_state_age_sec))

        images: dict[str, np.ndarray] = {}
        for image_input in profile.image_inputs:
            msg = latest_images.get(image_input.topic)
            if msg is None:
                raise RuntimeError(f"Missing image topic {image_input.topic}")
            receipt_monotonic = latest_image_receipts.get(image_input.topic)
            if receipt_monotonic is None:
                raise RuntimeError(f"Missing receipt timestamp for image topic {image_input.topic}")
            if max_image_age_sec > 0.0 and now - receipt_monotonic > max_image_age_sec:
                raise RuntimeError(
                    f"Image topic {image_input.topic} is stale: {now - receipt_monotonic:.2f}s old"
                )
            raw_image = decode_image_message(msg)
            images[image_input.name] = preprocess_image_for_topic(
                raw_image,
                image_input.topic,
                square_crop_anchor=image_input.square_crop_anchor,
                resize_size=image_input.resize_size,
            )

        right_arm = self._joint_state_vector(
            latest_joint_states,
            profile.right_arm_state.topic,
            profile.right_arm_state.joint_names,
        )
        right_arm_receipt = latest_joint_state_receipts.get(profile.right_arm_state.topic)
        if right_arm_receipt is None:
            raise RuntimeError(
                f"Missing receipt timestamp for joint state topic {profile.right_arm_state.topic}"
            )
        if max_joint_state_age_sec > 0.0 and now - right_arm_receipt > max_joint_state_age_sec:
            raise RuntimeError(
                f"Joint state topic {profile.right_arm_state.topic} is stale: "
                f"{now - right_arm_receipt:.2f}s old"
            )
        right_hand = self._joint_state_vector(
            latest_joint_states,
            profile.right_hand_state.topic,
            profile.right_hand_state.joint_names,
        )
        right_hand_receipt = latest_joint_state_receipts.get(profile.right_hand_state.topic)
        if right_hand_receipt is None:
            raise RuntimeError(
                f"Missing receipt timestamp for joint state topic {profile.right_hand_state.topic}"
            )
        if max_joint_state_age_sec > 0.0 and now - right_hand_receipt > max_joint_state_age_sec:
            raise RuntimeError(
                f"Joint state topic {profile.right_hand_state.topic} is stale: "
                f"{now - right_hand_receipt:.2f}s old"
            )
        state = np.concatenate([right_arm, right_hand]).astype(np.float32)
        if state.shape[0] != profile.action_dim:
            raise RuntimeError(
                f"State dim {state.shape[0]} does not match profile action dim {profile.action_dim}"
            )

        return OrderedDict(
            images=images,
            state=state,
            prompt=prompt or profile.default_prompt,
        )

    def _execute_action_row(self, profile: PolicyProfileSpec, action_row: np.ndarray) -> None:
        if not np.all(np.isfinite(action_row)):
            raise RuntimeError("Policy returned non-finite action values")
        right_arm_spec = profile.slice_for("right_arm")
        right_hand_spec = profile.slice_for("right_hand")

        right_arm_target = self._slice_action(profile, action_row, right_arm_spec)
        right_hand_target = self._slice_action(profile, action_row, right_hand_spec)

        marvin_positions = self._compose_marvin_command(profile, right_arm_target)
        self._publish_command(profile.marvin_command.topic, marvin_positions)
        self._publish_command(profile.right_hand_command.topic, right_hand_target)

    def _slice_action(self, profile: PolicyProfileSpec, action_row: np.ndarray, spec) -> np.ndarray:
        segment = np.asarray(action_row[spec.start : spec.start + spec.length], dtype=np.float32)
        if spec.mode == "delta":
            latest_joint_states = dict(self._latest_joint_states)
            if spec.name == "right_arm":
                current = self._joint_state_vector(
                    latest_joint_states,
                    profile.right_arm_state.topic,
                    profile.right_arm_state.joint_names,
                )
            elif spec.name == "right_hand":
                current = self._joint_state_vector(
                    latest_joint_states,
                    profile.right_hand_state.topic,
                    profile.right_hand_state.joint_names,
                )
            else:
                current = np.zeros_like(segment)
            return current + segment
        return segment

    def _compose_marvin_command(self, profile: PolicyProfileSpec, right_arm_target: np.ndarray) -> list[float]:
        latest_joint_states = dict(self._latest_joint_states)
        current = self._joint_state_map(
            latest_joint_states,
            profile.right_arm_state.topic,
            profile.marvin_command.joint_names,
        )
        right_arm_names = profile.right_arm_state.joint_names
        if len(right_arm_target) != len(right_arm_names):
            raise RuntimeError(
                f"Right arm target dim {len(right_arm_target)} does not match {len(right_arm_names)} joints"
            )
        for name, value in zip(right_arm_names, right_arm_target, strict=True):
            if name in current:
                current[name] = float(value)
        return [current[name] for name in profile.marvin_command.joint_names]

    def _joint_state_vector(
        self,
        latest_joint_states: dict[str, JointState],
        topic: str,
        joint_names: tuple[str, ...],
    ) -> np.ndarray:
        joint_map = self._joint_state_map(latest_joint_states, topic, joint_names)
        return np.asarray([joint_map[name] for name in joint_names], dtype=np.float32)

    def _joint_state_map(
        self,
        latest_joint_states: dict[str, JointState],
        topic: str,
        joint_names: tuple[str, ...],
    ) -> dict[str, float]:
        msg = latest_joint_states.get(topic)
        if msg is None:
            raise RuntimeError(f"Missing joint state topic {topic}")
        name_to_index = {name: index for index, name in enumerate(msg.name)}
        joint_map: dict[str, float] = {}
        for joint_name in joint_names:
            index = name_to_index.get(joint_name)
            if index is None or index >= len(msg.position):
                raise RuntimeError(f"Joint {joint_name} not present on topic {topic}")
            joint_map[joint_name] = float(msg.position[index])
        return joint_map

    def _publish_command(self, topic: str, values: list[float] | np.ndarray) -> None:
        publisher = self._command_publishers.get(topic)
        if publisher is None:
            publisher = self.create_publisher(Float64MultiArray, topic, 10)
            self._command_publishers[topic] = publisher
        if isinstance(values, np.ndarray):
            values = values.tolist()
        publisher.publish(_float64_array(values))

    def _publish_home_commands(self, profile: PolicyProfileSpec) -> None:
        if profile.marvin_command.home:
            self._publish_command(profile.marvin_command.topic, profile.marvin_command.home)
        if profile.right_hand_command.home:
            self._publish_command(profile.right_hand_command.topic, profile.right_hand_command.home)

    def _record_fault(
        self,
        *,
        fault_id: str,
        device_id: str,
        severity: int,
        summary: str,
        detail: str,
    ) -> None:
        fault = FaultEvent()
        fault.stamp = self.get_clock().now().to_msg()
        fault.fault_id = fault_id
        fault.device_id = device_id
        fault.severity = severity
        fault.summary = summary
        fault.detail = detail
        with self._state_lock:
            self._active_faults = [fault]

    def _clear_faults(self) -> None:
        with self._state_lock:
            self._active_faults = []

    def _shutdown_managed_sessions(self) -> None:
        sessions = list(self._managed_sessions)
        self._managed_sessions = []
        saw_marvin_session = any(session.label == "marvin_dual" for session in sessions)
        registered_device_ids = [session.label for session in sessions]
        for session in sessions:
            try:
                if session.label == "marvin_dual":
                    self._prepare_marvin_safe_shutdown("managed session shutdown")
                timeout_sec = 18.0 if session.label == "marvin_dual" else 8.0
                session.shutdown(timeout_sec=timeout_sec)
                session.close()
            except Exception:
                pass
        for device_id in registered_device_ids:
            try:
                self._runtime_manifest.unregister_device(device_id)
            except Exception:
                pass
        if saw_marvin_session:
            try:
                self._ensure_marvin_graph_cleanup(
                    timeout_sec=12.0,
                    context_label="managed session shutdown",
                    allow_external_cleanup=True,
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Marvin cleanup after managed session shutdown reported: {exc}"
                )

    def _stop_policy_rollout(self, *, wait: bool = False) -> None:
        runtime = self._policy_runtime
        runtime.stop_event.set()
        if wait and runtime.thread is not None and runtime.thread.is_alive():
            runtime.thread.join(timeout=max(1.0, self._config.policy_step_timeout_sec))
        if runtime.thread is None or not runtime.thread.is_alive():
            runtime.active = False
        self._policy_runtime = runtime

    def _wait_for_future(self, future, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.05)
        if not future.done():
            raise RuntimeError("Timed out waiting for go_home response")

    def shutdown_runtime(self) -> None:
        self._stop_policy_rollout(wait=True)
        self._shutdown_managed_sessions()
        self._clear_faults()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    launch_manager = LaunchManager()
    node = PolicyDeploymentSupervisor(launch_manager=launch_manager)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="policy_deployment_supervisor_rclpy_spin",
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
                name="policy_deployment_supervisor_shutdown",
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
