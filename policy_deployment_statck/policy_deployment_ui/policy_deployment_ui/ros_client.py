from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
import threading
import time

from marvin_system.srv import GetMotionStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

from policy_deployment_interfaces.action import (
    ConnectSystem,
    DisconnectSystem,
    ExecutePolicy,
    GoHome,
    StopPolicy,
)
from policy_deployment_interfaces.msg import DeploymentState
from policy_deployment_interfaces.srv import ListPolicyProfiles

from .view_model import DeploymentStateSnapshot, PolicyProfileEntry, PreviewStreamConfig, UiViewModel


@dataclass(frozen=True)
class UiEventEntry:
    stamp_sec: float
    level: str
    message: str


@dataclass(frozen=True)
class PreviewImageSnapshot:
    config: PreviewStreamConfig
    generation: int
    image_msg: Image | None
    rx_fps: float
    header_stamp_sec: float | None


@dataclass(frozen=True)
class MotionStatusSnapshot:
    available: bool = False
    mode: str = "UNKNOWN"
    teleop_state: str = "UNKNOWN"
    teleop_armed: bool = False
    teleop_enabled: bool = False
    primary_controller_state: str = "-"
    trajectory_controller_state: str = "-"
    motion_busy: bool = False
    controller_interlock_ok: bool = True
    hardware_joint_impedance_ok: bool = False
    active_control_profile: int = 0
    requested_control_profile: int = 0
    left_sdk_cur_state: int = 0
    right_sdk_cur_state: int = 0
    left_sdk_err_code: int = 0
    right_sdk_err_code: int = 0
    left_sdk_imp_type: int = 0
    right_sdk_imp_type: int = 0
    message: str = ""


@dataclass
class _PreviewRuntime:
    config: PreviewStreamConfig
    latest_message: Image | None = None
    generation: int = 0
    rx_window_start: float = field(default_factory=time.monotonic)
    rx_window_frames: int = 0
    rx_fps: float = 0.0
    last_receive_monotonic: float = 0.0


class RosClient:
    def __init__(self, args: list[str] | None = None) -> None:
        rclpy.init(args=args)
        self._node = Node("policy_deployment_ui")
        self._declare_parameters()
        self._config = self._load_config()
        self._lock = threading.Lock()
        self._event_entries: deque[UiEventEntry] = deque(maxlen=200)
        self._state = DeploymentStateSnapshot(recipe_id=self._config.recipe_id)
        self._motion_status = MotionStatusSnapshot()
        self._motion_status_request_pending = False
        self._profiles: list[PolicyProfileEntry] = []
        self._preview_streams_by_profile = self._load_preview_streams()
        self._preview_runtimes: dict[str, _PreviewRuntime] = {}
        self._preview_subscriptions = []

        self._connect_client = ActionClient(self._node, ConnectSystem, "connect_system")
        self._disconnect_client = ActionClient(self._node, DisconnectSystem, "disconnect_system")
        self._execute_client = ActionClient(self._node, ExecutePolicy, "execute_policy")
        self._stop_client = ActionClient(self._node, StopPolicy, "stop_policy")
        self._go_home_client = ActionClient(self._node, GoHome, "go_home")
        self._profiles_client = self._node.create_client(ListPolicyProfiles, "list_policy_profiles")
        self._motion_status_client = self._node.create_client(
            GetMotionStatus,
            "/marvin_motion/get_status",
        )

        self._state_sub = self._node.create_subscription(
            DeploymentState,
            "deployment_state",
            self._on_state,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._create_preview_subscriptions()
        self._motion_status_timer = self._node.create_timer(0.5, self._poll_motion_status)

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._executor_thread = threading.Thread(
            target=self._spin_executor,
            name="policy_deployment_ui_ros_spin",
            daemon=True,
        )
        self._executor_thread.start()

        self._request_profiles_async()
        self._record_event("INFO", "UI ready.")
        preview_count = sum(len(streams) for streams in self._preview_streams_by_profile.values())
        self._record_event("INFO", f"Configured {preview_count} processed image previews.")

    @property
    def config(self):
        return self._config

    @property
    def state(self) -> DeploymentStateSnapshot:
        with self._lock:
            return self._state

    @property
    def profiles(self) -> list[PolicyProfileEntry]:
        with self._lock:
            return list(self._profiles)

    @property
    def events(self) -> list[UiEventEntry]:
        with self._lock:
            return list(self._event_entries)

    def motion_status_snapshot(self) -> MotionStatusSnapshot:
        with self._lock:
            return self._motion_status

    def preview_stream_configs(self, profile_id: str = "") -> list[PreviewStreamConfig]:
        profile_id = profile_id.strip() or self._config.profile_id
        with self._lock:
            if profile_id in self._preview_streams_by_profile:
                return list(self._preview_streams_by_profile[profile_id])
            if self._preview_streams_by_profile:
                first_profile_id = next(iter(self._preview_streams_by_profile))
                return list(self._preview_streams_by_profile[first_profile_id])
        return []

    def preview_stream_snapshots(self, profile_id: str = "") -> list[PreviewImageSnapshot]:
        streams = self.preview_stream_configs(profile_id)
        now = time.monotonic()
        snapshots: list[PreviewImageSnapshot] = []
        with self._lock:
            for stream in streams:
                runtime = self._preview_runtimes.get(stream.preview_topic)
                if runtime is None:
                    snapshots.append(
                        PreviewImageSnapshot(
                            config=stream,
                            generation=0,
                            image_msg=None,
                            rx_fps=0.0,
                            header_stamp_sec=None,
                        )
                    )
                    continue
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
                    PreviewImageSnapshot(
                        config=stream,
                        generation=runtime.generation,
                        image_msg=runtime.latest_message,
                        rx_fps=rx_fps,
                        header_stamp_sec=header_stamp_sec,
                    )
                )
        return snapshots

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
            try:
                rclpy.shutdown()
            except Exception:
                pass

    def connect_system(self, profile_id: str, prompt: str, operator_id: str = "", site_name: str = "") -> None:
        goal = ConnectSystem.Goal()
        goal.recipe_id = self._config.recipe_id
        goal.policy_profile_id = profile_id
        goal.site_name = site_name
        goal.operator_id = operator_id
        self._send_goal(self._connect_client, goal, "connect_system")

    def disconnect_system(self, force: bool = False) -> None:
        goal = DisconnectSystem.Goal()
        goal.force = bool(force)
        self._send_goal(self._disconnect_client, goal, "disconnect_system")

    def execute_policy(self, profile_id: str, prompt: str, max_steps: int = 0, open_loop_horizon: int = 0) -> None:
        goal = ExecutePolicy.Goal()
        goal.policy_profile_id = profile_id
        goal.prompt = prompt
        goal.max_steps = int(max_steps)
        goal.open_loop_horizon = int(open_loop_horizon)
        self._send_goal(self._execute_client, goal, "execute_policy")

    def stop_policy(self, reason: str = "ui_request") -> None:
        goal = StopPolicy.Goal()
        goal.reason = reason
        self._send_goal(self._stop_client, goal, "stop_policy")

    def go_home(self, arm_after_home: bool = True) -> None:
        goal = GoHome.Goal()
        goal.arm_after_home = bool(arm_after_home)
        self._send_goal(self._go_home_client, goal, "go_home")

    def _declare_parameters(self) -> None:
        self._node.declare_parameter("title", "Policy Deployment Console")
        self._node.declare_parameter("recipe_id", "default_policy_deployment")
        self._node.declare_parameter("default_profile_id", "")
        self._node.declare_parameter("policy_profiles_config", "")
        self._node.declare_parameter("processed_preview_topic_prefix", "/policy_preview")

    def _load_config(self):
        title = str(self._node.get_parameter("title").value)
        recipe_id = str(self._node.get_parameter("recipe_id").value)
        default_profile_id = str(self._node.get_parameter("default_profile_id").value)
        policy_profiles_config = str(self._node.get_parameter("policy_profiles_config").value)
        processed_preview_topic_prefix = str(
            self._node.get_parameter("processed_preview_topic_prefix").value
        )
        return UiViewModel(
            title=title,
            recipe_id=recipe_id,
            profile_id=default_profile_id,
            prompt="",
            policy_profiles_config=policy_profiles_config,
            processed_preview_topic_prefix=processed_preview_topic_prefix,
            state=DeploymentStateSnapshot(recipe_id=recipe_id),
        )

    def _load_preview_streams(self) -> dict[str, tuple[PreviewStreamConfig, ...]]:
        path = Path(self._config.policy_profiles_config).expanduser()
        if not path.exists():
            self._node.get_logger().warning(f"Policy profiles config not found: {path}")
            return {}
        with path.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}
        profiles = data.get("profiles", {})
        profile_items = []
        if isinstance(profiles, list):
            for item in profiles:
                if isinstance(item, dict) and item.get("profile_id"):
                    profile_items.append((str(item["profile_id"]), item))
        elif isinstance(profiles, dict):
            profile_items = [(str(profile_id), profile) for profile_id, profile in profiles.items()]

        result: dict[str, tuple[PreviewStreamConfig, ...]] = {}
        for profile_id, profile in profile_items:
            if not isinstance(profile, dict):
                continue
            streams: list[PreviewStreamConfig] = []
            for item in profile.get("image_inputs", []):
                if not isinstance(item, dict):
                    continue
                camera_id = str(item.get("name", "")).strip()
                if not camera_id:
                    continue
                streams.append(
                    PreviewStreamConfig(
                        profile_id=profile_id,
                        camera_id=camera_id,
                        title=f"{camera_id} processed",
                        preview_topic=self._preview_topic(profile_id, camera_id),
                    )
                )
            result[profile_id] = tuple(streams)
        return result

    def _preview_topic(self, profile_id: str, camera_id: str) -> str:
        prefix = self._config.processed_preview_topic_prefix.strip()
        if not prefix.startswith("/"):
            prefix = f"/{prefix}"
        return f"{prefix}/{profile_id}/{camera_id}/image_raw"

    def _create_preview_subscriptions(self) -> None:
        seen_topics: set[str] = set()
        for streams in self._preview_streams_by_profile.values():
            for stream in streams:
                if stream.preview_topic in seen_topics:
                    continue
                seen_topics.add(stream.preview_topic)
                self._preview_runtimes[stream.preview_topic] = _PreviewRuntime(config=stream)
                self._preview_subscriptions.append(
                    self._node.create_subscription(
                        Image,
                        stream.preview_topic,
                        lambda msg, topic=stream.preview_topic: self._on_preview_image(topic, msg),
                        qos_profile_sensor_data,
                    )
                )

    def _spin_executor(self) -> None:
        try:
            self._executor.spin()
        except Exception:
            pass

    def _on_state(self, msg: DeploymentState) -> None:
        snapshot = DeploymentStateSnapshot(
            system_state=msg.system_state,
            rollout_state=msg.rollout_state,
            recipe_id=msg.recipe_id,
            active_profile_id=msg.active_profile_id,
            active_prompt=msg.active_prompt,
            summary=msg.summary,
            chunk_index=int(msg.chunk_index),
            step_index=int(msg.step_index),
            last_infer_ms=float(msg.last_infer_ms),
            allowed_commands=tuple(msg.allowed_commands),
            device_count=len(msg.devices),
            fault_count=len(msg.active_faults),
        )
        with self._lock:
            self._state = snapshot

    def _on_preview_image(self, preview_topic: str, msg: Image) -> None:
        now = time.monotonic()
        with self._lock:
            runtime = self._preview_runtimes.get(preview_topic)
            if runtime is None:
                return
            runtime.latest_message = msg
            runtime.generation += 1
            runtime.rx_window_frames += 1
            runtime.last_receive_monotonic = now
            elapsed = now - runtime.rx_window_start
            if elapsed >= 1.0:
                runtime.rx_fps = runtime.rx_window_frames / elapsed
                runtime.rx_window_frames = 0
                runtime.rx_window_start = now

    def _poll_motion_status(self) -> None:
        with self._lock:
            if self._motion_status_request_pending:
                return
        if not self._motion_status_client.service_is_ready():
            previous = self.motion_status_snapshot()
            if previous.available:
                self._set_motion_status_snapshot(
                    MotionStatusSnapshot(
                        available=False,
                        message="marvin_motion status service unavailable.",
                    )
                )
            return

        with self._lock:
            self._motion_status_request_pending = True
        future = self._motion_status_client.call_async(GetMotionStatus.Request())
        future.add_done_callback(self._handle_motion_status_result)

    def _handle_motion_status_result(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._set_motion_status_snapshot(
                MotionStatusSnapshot(
                    available=False,
                    message=f"Failed to query marvin_motion status: {exc}",
                )
            )
            return

        snapshot = MotionStatusSnapshot(
            available=bool(getattr(response, "success", False)),
            mode=str(getattr(response, "mode", "")).strip() or "UNKNOWN",
            teleop_state=str(getattr(response, "teleop_state", "")).strip() or "UNKNOWN",
            teleop_armed=bool(getattr(response, "teleop_armed", False)),
            teleop_enabled=bool(getattr(response, "teleop_enabled", False)),
            primary_controller_state=(
                str(getattr(response, "primary_controller_state", "")).strip() or "-"
            ),
            trajectory_controller_state=(
                str(getattr(response, "trajectory_controller_state", "")).strip() or "-"
            ),
            motion_busy=bool(getattr(response, "motion_busy", False)),
            controller_interlock_ok=bool(getattr(response, "controller_interlock_ok", True)),
            hardware_joint_impedance_ok=bool(
                getattr(response, "hardware_joint_impedance_ok", False)
            ),
            active_control_profile=int(getattr(response, "active_control_profile", 0)),
            requested_control_profile=int(getattr(response, "requested_control_profile", 0)),
            left_sdk_cur_state=int(getattr(response, "left_sdk_cur_state", 0)),
            right_sdk_cur_state=int(getattr(response, "right_sdk_cur_state", 0)),
            left_sdk_err_code=int(getattr(response, "left_sdk_err_code", 0)),
            right_sdk_err_code=int(getattr(response, "right_sdk_err_code", 0)),
            left_sdk_imp_type=int(getattr(response, "left_sdk_imp_type", 0)),
            right_sdk_imp_type=int(getattr(response, "right_sdk_imp_type", 0)),
            message=str(getattr(response, "message", "")).strip(),
        )
        self._set_motion_status_snapshot(snapshot)

    def _set_motion_status_snapshot(self, snapshot: MotionStatusSnapshot) -> None:
        events: list[tuple[str, str]] = []
        with self._lock:
            previous = self._motion_status
            self._motion_status = snapshot
            self._motion_status_request_pending = False

            if snapshot.available != previous.available:
                if snapshot.available:
                    events.append(
                        ("INFO", f"marvin_motion online: mode={snapshot.mode}.")
                    )
                else:
                    events.append(
                        ("WARN", snapshot.message or "marvin_motion status unavailable.")
                    )
            elif snapshot.available and (
                snapshot.mode != previous.mode
                or snapshot.primary_controller_state != previous.primary_controller_state
                or snapshot.trajectory_controller_state != previous.trajectory_controller_state
            ):
                events.append(
                    (
                        "INFO",
                        "marvin_motion "
                        f"mode={snapshot.mode} primary={snapshot.primary_controller_state} "
                        f"trajectory={snapshot.trajectory_controller_state}.",
                    )
                )

            if previous.controller_interlock_ok and not snapshot.controller_interlock_ok:
                events.append(("ERROR", "marvin_motion controller interlock violation detected."))
            if previous.hardware_joint_impedance_ok and not snapshot.hardware_joint_impedance_ok:
                events.append(("ERROR", "Marvin hardware joint impedance is not confirmed."))

        for level, message in events:
            self._record_event(level, message)

    def _request_profiles_async(self) -> None:
        def worker() -> None:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                if self._profiles_client.wait_for_service(timeout_sec=0.2):
                    break
            if not self._profiles_client.service_is_ready():
                self._record_event("WARN", "list_policy_profiles service not ready.")
                return
            future = self._profiles_client.call_async(ListPolicyProfiles.Request())
            future.add_done_callback(self._on_profiles_ready)

        threading.Thread(target=worker, name="policy_profiles_request", daemon=True).start()

    def _on_profiles_ready(self, future) -> None:
        try:
            response = future.result()
            profiles = [
                PolicyProfileEntry(
                    profile_id=profile.profile_id,
                    title=profile.title,
                    server_host=profile.server_host,
                    server_port=int(profile.server_port),
                    default_prompt=profile.default_prompt,
                    summary=profile.summary,
                )
                for profile in getattr(response, "profiles", [])
            ]
            with self._lock:
                self._profiles = profiles
                if profiles and not self._state.active_profile_id:
                    self._state = DeploymentStateSnapshot(
                        **{
                            **self._state.__dict__,
                            "fault_count": self._state.fault_count,
                        }
                    )
            self._record_event("INFO", f"Loaded {len(profiles)} policy profiles.")
        except Exception as exc:
            self._record_event("WARN", f"Failed to load policy profiles: {exc}")

    def _send_goal(self, client: ActionClient, goal, label: str) -> None:
        def worker() -> None:
            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                if client.wait_for_server(timeout_sec=0.2):
                    break
            if not client.server_is_ready():
                self._record_event("WARN", f"{label} action server not ready.")
                return
            future = client.send_goal_async(goal)
            future.add_done_callback(lambda f, action_label=label: self._on_goal_response(action_label, f))

        threading.Thread(target=worker, name=f"{label}_goal", daemon=True).start()

    def _on_goal_response(self, label: str, future) -> None:
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self._record_event("WARN", f"{label} rejected.")
                return
            self._record_event("INFO", f"{label} accepted.")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f, action_label=label: self._on_goal_result(action_label, f)
            )
        except Exception as exc:
            self._record_event("WARN", f"{label} goal failed: {exc}")

    def _on_goal_result(self, label: str, future) -> None:
        try:
            result = future.result().result
            success = bool(getattr(result, "success", False))
            message = str(getattr(result, "message", "")).strip()
            level = "INFO" if success else "WARN"
            self._record_event(level, f"{label} completed: {message}")
        except Exception as exc:
            self._record_event("WARN", f"{label} result error: {exc}")

    def _record_event(self, level: str, message: str) -> None:
        entry = UiEventEntry(stamp_sec=time.time(), level=level, message=message)
        with self._lock:
            self._event_entries.append(entry)
