from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from data_collection_interfaces.action import ShutdownSystem, StartSystem
from data_collection_interfaces.msg import SystemState
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
import yaml


def _load_yaml_map(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected YAML mapping at root: {path}")
    return data


@dataclass(frozen=True)
class TopicSpec:
    topic: str
    kind: str
    expected_fps: float
    topic_type: str
    camera_id: str


@dataclass(frozen=True)
class CameraLaunchSpec:
    camera_id: str
    driver: str


@dataclass
class TopicSample:
    sample_duration_sec: float
    message_count: int = 0
    fps: float = 0.0
    avg_age_ms: float | None = None
    max_age_ms: float | None = None
    seen: bool = False


@dataclass
class TopicComparison:
    topic: str
    kind: str
    camera_id: str
    expected_fps: float
    baseline_fps: float | None = None
    loaded_fps: float | None = None
    drop_pct: float | None = None
    loaded_avg_age_ms: float | None = None
    loaded_max_age_ms: float | None = None


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str


@dataclass
class TopicRuntime:
    spec: TopicSpec
    count: int = 0
    age_sum_ms: float = 0.0
    age_count: int = 0
    max_age_ms: float = 0.0
    enabled: bool = False


class TopicSamplerNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_preview_performance_smoke")

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._latest_system_state = None
        self.create_subscription(
            SystemState,
            "system_state",
            self._on_system_state,
            state_qos,
        )
        self._start_client = ActionClient(self, StartSystem, "start_system")
        self._shutdown_client = ActionClient(self, ShutdownSystem, "shutdown_system")
        self._lock = threading.Lock()
        self._topic_runtimes: dict[str, TopicRuntime] = {}
        self._subscriptions = []

    def wait_for_action_servers(self, timeout_sec: float) -> None:
        if not self._start_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("start_system action server did not become available.")
        if not self._shutdown_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError("shutdown_system action server did not become available.")

    def send_start_system(self, timeout_sec: float) -> str:
        goal = StartSystem.Goal()
        goal_future = self._start_client.send_goal_async(goal)
        goal_handle = self._await_future(goal_future, timeout_sec=timeout_sec)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("start_system goal was not accepted.")

        result_future = goal_handle.get_result_async()
        result = self._await_future(result_future, timeout_sec=timeout_sec)
        if result is None:
            raise RuntimeError("Timed out waiting for start_system result.")
        if not result.result.success:
            raise RuntimeError(f"start_system failed: {result.result.message}")
        return result.result.message

    def send_shutdown_system(self, timeout_sec: float) -> str:
        goal = ShutdownSystem.Goal()
        goal.force = True
        goal_future = self._shutdown_client.send_goal_async(goal)
        goal_handle = self._await_future(goal_future, timeout_sec=timeout_sec)
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("shutdown_system goal was not accepted.")

        result_future = goal_handle.get_result_async()
        result = self._await_future(result_future, timeout_sec=timeout_sec)
        if result is None:
            raise RuntimeError("Timed out waiting for shutdown_system result.")
        if not result.result.success:
            raise RuntimeError(f"shutdown_system failed: {result.result.message}")
        return result.result.message

    def register_topics(self, specs: list[TopicSpec], *, enabled: bool) -> None:
        for spec in specs:
            if spec.topic in self._topic_runtimes:
                self._topic_runtimes[spec.topic].enabled = enabled
                continue

            runtime = TopicRuntime(spec=spec, enabled=enabled)
            self._topic_runtimes[spec.topic] = runtime
            message_type = Image if spec.topic_type == "image" else CameraInfo
            subscription = self.create_subscription(
                message_type,
                spec.topic,
                lambda msg, topic=spec.topic: self._on_message(topic, msg),
                qos_profile_sensor_data,
            )
            self._subscriptions.append(subscription)

    def set_topics_enabled(self, specs: list[TopicSpec], enabled: bool) -> None:
        for spec in specs:
            runtime = self._topic_runtimes.get(spec.topic)
            if runtime is not None:
                runtime.enabled = enabled

    def sample(self, specs: list[TopicSpec], duration_sec: float) -> dict[str, TopicSample]:
        with self._lock:
            for spec in specs:
                runtime = self._topic_runtimes[spec.topic]
                runtime.count = 0
                runtime.age_sum_ms = 0.0
                runtime.age_count = 0
                runtime.max_age_ms = 0.0

        time.sleep(duration_sec)

        samples: dict[str, TopicSample] = {}
        with self._lock:
            for spec in specs:
                runtime = self._topic_runtimes[spec.topic]
                avg_age_ms = None
                if runtime.age_count > 0:
                    avg_age_ms = runtime.age_sum_ms / runtime.age_count
                samples[spec.topic] = TopicSample(
                    sample_duration_sec=duration_sec,
                    message_count=runtime.count,
                    fps=runtime.count / max(duration_sec, 1e-3),
                    avg_age_ms=avg_age_ms,
                    max_age_ms=runtime.max_age_ms if runtime.age_count > 0 else None,
                    seen=runtime.count > 0,
                )
        return samples

    def _await_future(self, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            time.sleep(0.05)
        return None

    def _on_system_state(self, msg) -> None:
        self._latest_system_state = msg

    def _on_message(self, topic: str, msg) -> None:
        now_sec = time.time()
        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        age_ms = max(0.0, (now_sec - stamp_sec) * 1000.0)

        with self._lock:
            runtime = self._topic_runtimes.get(topic)
            if runtime is None or not runtime.enabled:
                return

            runtime.count += 1
            runtime.age_sum_ms += age_ms
            runtime.age_count += 1
            runtime.max_age_ms = max(runtime.max_age_ms, age_ms)


class ExecutorThread:
    def __init__(self, node: Node) -> None:
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._spin, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def shutdown(self) -> None:
        try:
            self._executor.shutdown()
        except Exception:
            pass
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except ExternalShutdownException:
            pass


def _default_bringup_share() -> Path:
    try:
        return Path(get_package_share_directory("data_collection_bringup"))
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1]


def _default_recipe_directory(bringup_share: Path) -> Path:
    return bringup_share / "config" / "recipes"


def _camera_system_share() -> Path:
    try:
        return Path(get_package_share_directory("camera_system"))
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[3] / "camera_system"


def _default_cameras_config() -> Path:
    return _camera_system_share() / "bringup" / "config" / "cameras.yaml"


def _expected_record_fps(camera_cfg: dict) -> float:
    driver = str(camera_cfg.get("driver", "")).strip()
    profile_name = str(camera_cfg.get("profile", "")).strip()
    if not profile_name:
        return 0.0

    camera_share = _camera_system_share()
    if driver == "realsense":
        defaults_path = camera_share / "bringup" / "config" / "realsense_defaults.yaml"
    elif driver == "orbbec":
        defaults_path = camera_share / "bringup" / "config" / "orbbec_defaults.yaml"
    else:
        return 0.0

    profiles = _load_yaml_map(defaults_path).get("profiles", {})
    if not isinstance(profiles, dict):
        return 0.0
    profile_cfg = profiles.get(profile_name)
    if not isinstance(profile_cfg, dict):
        return 0.0
    try:
        return float(profile_cfg.get("color_fps", 0.0))
    except (TypeError, ValueError):
        return 0.0


def _camera_launch_specs(recipe_path: Path, cameras_config: Path) -> list[CameraLaunchSpec]:
    recipe_data = _load_yaml_map(recipe_path)
    recipe_devices = recipe_data.get("devices", [])
    if not isinstance(recipe_devices, list):
        raise RuntimeError(f"Expected devices list in recipe {recipe_path}")

    site_cameras = _load_yaml_map(cameras_config).get("cameras", {})
    if not isinstance(site_cameras, dict):
        raise RuntimeError(f"Expected cameras mapping in {cameras_config}")

    specs: list[CameraLaunchSpec] = []
    for raw_device in recipe_devices:
        if not isinstance(raw_device, dict):
            continue
        if str(raw_device.get("adapter", "")).strip() != "camera":
            continue

        camera_id = str(raw_device.get("id", "")).strip()
        if not camera_id:
            continue
        camera_cfg = site_cameras.get(camera_id)
        if not isinstance(camera_cfg, dict):
            raise RuntimeError(f"Camera '{camera_id}' missing in site cameras config.")
        specs.append(
            CameraLaunchSpec(
                camera_id=camera_id,
                driver=str(camera_cfg.get("driver", "")).strip(),
            )
        )
    return specs


def _build_topic_specs(recipe_path: Path, cameras_config: Path) -> tuple[list[TopicSpec], list[TopicSpec]]:
    recipe_data = _load_yaml_map(recipe_path)
    recipe_devices = recipe_data.get("devices", [])
    if not isinstance(recipe_devices, list):
        raise RuntimeError(f"Expected devices list in recipe {recipe_path}")

    site_cameras = _load_yaml_map(cameras_config).get("cameras", {})
    if not isinstance(site_cameras, dict):
        raise RuntimeError(f"Expected cameras mapping in {cameras_config}")

    record_specs: list[TopicSpec] = []
    preview_specs: list[TopicSpec] = []
    for raw_device in recipe_devices:
        if not isinstance(raw_device, dict):
            continue
        if str(raw_device.get("adapter", "")).strip() != "camera":
            continue

        camera_id = str(raw_device.get("id", "")).strip()
        if not camera_id:
            continue

        camera_cfg = site_cameras.get(camera_id)
        if not isinstance(camera_cfg, dict):
            raise RuntimeError(f"Camera '{camera_id}' missing in site cameras config.")

        expected_record_fps = _expected_record_fps(camera_cfg)
        preview_fps = float(camera_cfg.get("preview_fps", expected_record_fps))
        expected_preview_fps = (
            min(expected_record_fps, preview_fps) if expected_record_fps > 0 else preview_fps
        )

        record_topics = camera_cfg.get("record_topics", [])
        if not isinstance(record_topics, list):
            raise RuntimeError(f"record_topics for {camera_id} must be a list.")

        for topic in record_topics:
            if not isinstance(topic, str) or not topic.strip():
                continue
            record_specs.append(
                TopicSpec(
                    topic=topic.strip(),
                    kind="record",
                    expected_fps=expected_record_fps,
                    topic_type="camera_info" if topic.endswith("/camera_info") else "image",
                    camera_id=camera_id,
                )
            )

        preview_topic = camera_cfg.get("preview_topic")
        if isinstance(preview_topic, str) and preview_topic.strip():
            preview_specs.append(
                TopicSpec(
                    topic=preview_topic.strip(),
                    kind="preview",
                    expected_fps=expected_preview_fps,
                    topic_type="image",
                    camera_id=camera_id,
                )
            )

    return record_specs, preview_specs


def _resolve_recipe_path(recipe_directory: Path, recipe_id: str) -> Path:
    candidates = [recipe_directory / recipe_id]
    if not recipe_id.endswith(".yaml"):
        candidates.append(recipe_directory / f"{recipe_id}.yaml")
    for candidate in candidates:
        if candidate.exists():
            return candidate
    raise RuntimeError(f"Recipe '{recipe_id}' not found under {recipe_directory}")


def _launch_stack(
    recipe_id: str,
    use_ui: bool,
    bringup_share: Path,
    cameras_config: Path,
) -> subprocess.Popen:
    recipe_directory = _default_recipe_directory(bringup_share)
    startup_policy = bringup_share / "config" / "policies" / "startup.yaml"
    fault_policy = bringup_share / "config" / "policies" / "fault.yaml"
    ui_config = bringup_share / "config" / "session" / "ui.yaml"

    command = [
        "ros2",
        "launch",
        "data_collection_bringup",
        "collection_app.launch.py",
        f"recipe_id:={recipe_id}",
        f"use_ui:={'true' if use_ui else 'false'}",
        f"recipe_directory:={recipe_directory}",
        f"cameras_config:={cameras_config}",
        f"startup_policy_config:={startup_policy}",
        f"fault_policy_config:={fault_policy}",
        f"ui_config:={ui_config}",
    ]
    env = os.environ.copy()
    if use_ui:
        env.setdefault("QT_QPA_PLATFORM", "offscreen")
    return subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )


def _launch_direct_cameras(
    recipe_path: Path,
    cameras_config: Path,
) -> list[subprocess.Popen]:
    processes: list[subprocess.Popen] = []
    env = os.environ.copy()

    for spec in _camera_launch_specs(recipe_path, cameras_config):
        if spec.driver == "realsense":
            launch_file = "single_realsense.launch.py"
        elif spec.driver == "orbbec":
            launch_file = "single_orbbec.launch.py"
        else:
            raise RuntimeError(f"Unsupported camera driver for {spec.camera_id}: {spec.driver}")

        command = [
            "ros2",
            "launch",
            "camera_system",
            launch_file,
            f"camera_name:={spec.camera_id}",
            f"cameras_config:={cameras_config}",
            "use_showimage:=false",
            "respawn:=false",
        ]
        processes.append(
            subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
                env=env,
            )
        )
    return processes


def _ensure_process_running(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        raise RuntimeError(f"Launch process exited early with code {process.returncode}.")


def _terminate_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=8.0)
        return
    except Exception:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=4.0)
        return
    except Exception:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except Exception:
        pass


def _drain_process_output(process: subprocess.Popen) -> str:
    if process.stdout is None:
        return ""
    try:
        return process.stdout.read()
    except Exception:
        return ""


def _terminate_processes(processes: list[subprocess.Popen]) -> None:
    for process in processes:
        _terminate_process(process)


def _drain_process_outputs(processes: list[subprocess.Popen]) -> str:
    chunks: list[str] = []
    for index, process in enumerate(processes, start=1):
        output = _drain_process_output(process)
        if output.strip():
            chunks.append(f"[process {index} pid={process.pid}]\n{output}")
    return "\n".join(chunks)


def _comparison_report(
    record_specs: list[TopicSpec],
    preview_specs: list[TopicSpec],
    baseline: dict[str, TopicSample],
    loaded_record: dict[str, TopicSample],
    loaded_preview: dict[str, TopicSample],
    *,
    min_record_ratio: float,
    max_record_drop_pct: float,
    min_preview_ratio: float,
    max_preview_age_ms: float,
) -> tuple[list[TopicComparison], list[CheckResult]]:
    comparisons: list[TopicComparison] = []
    checks: list[CheckResult] = []

    for spec in record_specs:
        baseline_sample = baseline[spec.topic]
        loaded_sample = loaded_record[spec.topic]
        drop_pct = None
        if baseline_sample.fps > 1e-3:
            drop_pct = max(0.0, (baseline_sample.fps - loaded_sample.fps) / baseline_sample.fps * 100.0)

        comparisons.append(
            TopicComparison(
                topic=spec.topic,
                kind=spec.kind,
                camera_id=spec.camera_id,
                expected_fps=spec.expected_fps,
                baseline_fps=baseline_sample.fps,
                loaded_fps=loaded_sample.fps,
                drop_pct=drop_pct,
                loaded_avg_age_ms=loaded_sample.avg_age_ms,
                loaded_max_age_ms=loaded_sample.max_age_ms,
            )
        )

        checks.append(
            CheckResult(
                name=f"record_seen:{spec.topic}",
                ok=loaded_sample.seen and baseline_sample.seen,
                detail=(
                    f"baseline_seen={baseline_sample.seen} loaded_seen={loaded_sample.seen} "
                    f"expected_fps={spec.expected_fps:.1f}"
                ),
            )
        )
        checks.append(
            CheckResult(
                name=f"record_rate:{spec.topic}",
                ok=loaded_sample.fps >= max(1.0, spec.expected_fps * min_record_ratio),
                detail=(
                    f"loaded_fps={loaded_sample.fps:.2f} "
                    f"expected>={max(1.0, spec.expected_fps * min_record_ratio):.2f}"
                ),
            )
        )
        if drop_pct is not None:
            checks.append(
                CheckResult(
                    name=f"record_drop:{spec.topic}",
                    ok=drop_pct <= max_record_drop_pct,
                    detail=f"baseline_fps={baseline_sample.fps:.2f} loaded_fps={loaded_sample.fps:.2f} drop_pct={drop_pct:.1f}",
                )
            )

    for spec in preview_specs:
        loaded_sample = loaded_preview[spec.topic]
        comparisons.append(
            TopicComparison(
                topic=spec.topic,
                kind=spec.kind,
                camera_id=spec.camera_id,
                expected_fps=spec.expected_fps,
                loaded_fps=loaded_sample.fps,
                loaded_avg_age_ms=loaded_sample.avg_age_ms,
                loaded_max_age_ms=loaded_sample.max_age_ms,
            )
        )
        checks.append(
            CheckResult(
                name=f"preview_seen:{spec.topic}",
                ok=loaded_sample.seen,
                detail=f"loaded_seen={loaded_sample.seen} expected_fps={spec.expected_fps:.1f}",
            )
        )
        checks.append(
            CheckResult(
                name=f"preview_rate:{spec.topic}",
                ok=loaded_sample.fps >= max(1.0, spec.expected_fps * min_preview_ratio),
                detail=(
                    f"loaded_fps={loaded_sample.fps:.2f} "
                    f"expected>={max(1.0, spec.expected_fps * min_preview_ratio):.2f}"
                ),
            )
        )
        if loaded_sample.avg_age_ms is not None:
            checks.append(
                CheckResult(
                    name=f"preview_age:{spec.topic}",
                    ok=loaded_sample.avg_age_ms <= max_preview_age_ms,
                    detail=f"avg_age_ms={loaded_sample.avg_age_ms:.1f} max_age_ms={loaded_sample.max_age_ms:.1f}",
                )
            )

    return comparisons, checks


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Camera preview/record performance smoke test.")
    parser.add_argument("--recipe-id", default="camera_preview_perf_collection")
    parser.add_argument("--baseline-sec", type=float, default=4.0)
    parser.add_argument("--loaded-sec", type=float, default=4.0)
    parser.add_argument("--bringup-share", default="")
    parser.add_argument("--cameras-config", default="")
    parser.add_argument("--launch-mode", choices=["direct", "stack"], default="direct")
    parser.add_argument("--use-ui", action="store_true")
    parser.add_argument("--report-json", default="")
    parser.add_argument("--min-record-ratio", type=float, default=0.80)
    parser.add_argument("--max-record-drop-pct", type=float, default=15.0)
    parser.add_argument("--min-preview-ratio", type=float, default=0.70)
    parser.add_argument("--max-preview-age-ms", type=float, default=250.0)
    parser.add_argument("--start-timeout-sec", type=float, default=180.0)
    parser.add_argument("--shutdown-timeout-sec", type=float, default=60.0)
    args = parser.parse_args(argv)

    bringup_share = (
        Path(args.bringup_share).expanduser() if args.bringup_share else _default_bringup_share()
    )
    recipe_directory = _default_recipe_directory(bringup_share)
    cameras_config = (
        Path(args.cameras_config).expanduser() if args.cameras_config else _default_cameras_config()
    )
    recipe_path = _resolve_recipe_path(recipe_directory, args.recipe_id)
    record_specs, preview_specs = _build_topic_specs(recipe_path, cameras_config)
    if not record_specs or not preview_specs:
        raise RuntimeError("Performance smoke requires both record and preview camera topics.")

    rclpy.init(args=None)
    node = TopicSamplerNode()
    executor_thread = ExecutorThread(node)
    launch_processes: list[subprocess.Popen] = []
    dump_launch_output = False
    try:
        executor_thread.start()
        if args.launch_mode == "stack":
            launch_processes = [
                _launch_stack(
                    recipe_id=args.recipe_id,
                    use_ui=args.use_ui,
                    bringup_share=bringup_share,
                    cameras_config=cameras_config,
                )
            ]
        else:
            launch_processes = _launch_direct_cameras(recipe_path, cameras_config)
        time.sleep(2.0)
        for process in launch_processes:
            _ensure_process_running(process)

        if args.launch_mode == "stack":
            node.wait_for_action_servers(timeout_sec=20.0)
            start_summary = node.send_start_system(timeout_sec=args.start_timeout_sec)
        else:
            start_summary = "Direct camera launch mode started."

        node.register_topics(record_specs, enabled=True)
        baseline_samples = node.sample(record_specs, duration_sec=args.baseline_sec)

        node.register_topics(preview_specs, enabled=True)
        time.sleep(1.0)
        loaded_samples = node.sample(record_specs + preview_specs, duration_sec=args.loaded_sec)
        loaded_record_samples = {spec.topic: loaded_samples[spec.topic] for spec in record_specs}
        loaded_preview_samples = {spec.topic: loaded_samples[spec.topic] for spec in preview_specs}
        if args.launch_mode == "stack":
            shutdown_summary = node.send_shutdown_system(timeout_sec=args.shutdown_timeout_sec)
        else:
            shutdown_summary = "Direct camera launch mode stopped."

        comparisons, checks = _comparison_report(
            record_specs=record_specs,
            preview_specs=preview_specs,
            baseline=baseline_samples,
            loaded_record=loaded_record_samples,
            loaded_preview=loaded_preview_samples,
            min_record_ratio=args.min_record_ratio,
            max_record_drop_pct=args.max_record_drop_pct,
            min_preview_ratio=args.min_preview_ratio,
            max_preview_age_ms=args.max_preview_age_ms,
        )
        report = {
            "recipe_id": args.recipe_id,
            "cameras_config": str(cameras_config),
            "launch_mode": args.launch_mode,
            "use_ui": args.use_ui,
            "start_summary": start_summary,
            "shutdown_summary": shutdown_summary,
            "checks_passed": all(check.ok for check in checks),
            "comparisons": [asdict(item) for item in comparisons],
            "checks": [asdict(item) for item in checks],
        }
        report_text = json.dumps(report, indent=2, sort_keys=True)
        print(report_text)
        if args.report_json:
            report_path = Path(args.report_json).expanduser()
            report_path.parent.mkdir(parents=True, exist_ok=True)
            report_path.write_text(report_text + "\n", encoding="utf-8")

        if not report["checks_passed"]:
            dump_launch_output = True
            return 1
        return 0
    except Exception:
        dump_launch_output = True
        raise
    finally:
        if launch_processes:
            _terminate_processes(launch_processes)
            process_output = _drain_process_outputs(launch_processes)
            if dump_launch_output and process_output.strip():
                print("\n--- launch output ---", file=sys.stderr)
                print(process_output, file=sys.stderr)
        executor_thread.shutdown()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
