from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
from typing import Any, Callable

from rosbag2_interfaces.srv import Pause, Resume

from .models import (
    ActiveSession,
    RecorderRuntime,
    RecordingPolicy,
    RecipeSpec,
    SessionArtifacts,
    SupervisorConfig,
)


class RecordManager:
    def __init__(self, node) -> None:
        self._node = node
        self._runtime: RecorderRuntime | None = None
        self._process: subprocess.Popen[str] | None = None
        self._pause_client = None
        self._resume_client = None
        self._log_handle = None

    def runtime_summary(self) -> str:
        if self._runtime is None:
            return "Recorder idle."
        return (
            f"Recorder node={self._runtime.node_name} prepared={self._runtime.prepared} "
            f"recording_active={self._runtime.recording_active} pid={self._runtime.pid}"
        )

    def validate_record_topics(self, topics: list[str]) -> list[str]:
        normalized = self._normalize_topics(topics)
        if not normalized:
            raise RuntimeError("Recording topic list is empty after normalization.")
        if self._node.recording_policy().canonical_joint_state_topic not in normalized:
            self._node.get_logger().warn(
                "Recording topic list does not include canonical joint state topic "
                f"{self._node.recording_policy().canonical_joint_state_topic}. "
                "Session can still record, but it will not be training-ready under the plan contract."
            )
        if not any("image_raw" in topic for topic in normalized):
            self._node.get_logger().warn(
                "Recording topic list does not include raw image topics."
            )
        return normalized

    def prepare_session(
        self,
        *,
        active_session: ActiveSession,
        recipe: RecipeSpec,
        supervisor_config: SupervisorConfig,
        recording_policy: RecordingPolicy,
        normalized_topics: list[str],
        event_log_snapshot_source: Callable[[], list[str]],
    ) -> tuple[SessionArtifacts, RecorderRuntime]:
        del event_log_snapshot_source
        session_dir = self._create_session_dir(recording_policy.session_root, active_session.session_id)
        bag_dir = session_dir / recording_policy.bag_directory_name
        artifacts = SessionArtifacts(
            session_dir=session_dir,
            bag_dir=bag_dir,
            metadata_path=session_dir / recording_policy.metadata_file_name,
            event_log_path=session_dir / recording_policy.event_log_file_name,
            timing_report_path=session_dir / recording_policy.timing_report_file_name,
        )
        artifacts.snapshot_paths = self._snapshot_inputs(
            artifacts=artifacts,
            recipe=recipe,
            supervisor_config=supervisor_config,
            recording_policy=recording_policy,
        )
        runtime = RecorderRuntime(
            node_name=f"{recording_policy.recorder_node_name_prefix}_{active_session.session_id}",
            prepared=True,
            log_path=session_dir / "recorder.log",
        )
        active_session.artifacts = artifacts
        active_session.record_topics = list(normalized_topics)
        active_session.recorder_node_name = runtime.node_name
        self._runtime = runtime
        self._write_metadata(
            active_session=active_session,
            recording_policy=recording_policy,
            status="preparing",
            result_success=False,
            result_message="Session directory prepared.",
        )
        return artifacts, runtime

    def start_recording(
        self,
        *,
        active_session: ActiveSession,
        recording_policy: RecordingPolicy,
        sync_health_summary: dict[str, Any],
    ) -> None:
        runtime = self._require_runtime(active_session)
        artifacts = self._require_artifacts(active_session)
        runtime.command = self._build_recorder_command(
            bag_dir=artifacts.bag_dir,
            node_name=runtime.node_name,
            recording_policy=recording_policy,
            topics=active_session.record_topics,
        )
        runtime.sync_health_checked = True
        runtime.sync_health_summary = dict(sync_health_summary)
        active_session.time_sync_summary = dict(sync_health_summary)
        self._log_handle = runtime.log_path.open("a", encoding="utf-8")
        try:
            self._process = subprocess.Popen(
                runtime.command,
                stdout=self._log_handle,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        except Exception:
            self._close_log_handle()
            raise
        runtime.pid = self._process.pid
        try:
            runtime.pgid = os.getpgid(self._process.pid)
        except Exception:
            runtime.pgid = None
        self._create_service_clients(runtime.node_name)
        if recording_policy.use_start_paused:
            self._wait_for_service(self._resume_client, recording_policy.service_timeout_sec, "resume")
            self._wait_for_service(self._pause_client, recording_policy.service_timeout_sec, "pause")
            self._call_empty_service(
                self._resume_client,
                Resume.Request(),
                recording_policy.service_timeout_sec,
                "resume",
            )
        runtime.recording_active = True
        self._write_metadata(
            active_session=active_session,
            recording_policy=recording_policy,
            status="recording",
            result_success=True,
            result_message="Recorder started.",
        )

    def stop_recording(
        self,
        *,
        active_session: ActiveSession,
        recording_policy: RecordingPolicy,
        event_log_entries: list[str],
    ) -> dict[str, Any]:
        runtime = self._require_runtime(active_session)
        artifacts = self._require_artifacts(active_session)
        incomplete = False
        if self._process is not None and self._process.poll() is None:
            if recording_policy.use_start_paused and runtime.recording_active:
                try:
                    self._call_empty_service(
                        self._pause_client,
                        Pause.Request(),
                        recording_policy.service_timeout_sec,
                        "pause",
                    )
                except Exception:
                    incomplete = True
            if not self._request_process_stop(recording_policy.stop_timeout_sec):
                incomplete = True
                self._terminate_process(recording_policy.cleanup_timeout_sec)
        runtime.recording_active = False
        timing_summary = self._write_timing_report(active_session, recording_policy, incomplete)
        active_session.timing_validation_summary = dict(timing_summary)
        artifacts.event_log_path.write_text(self._format_event_log(event_log_entries), encoding="utf-8")
        self._write_metadata(
            active_session=active_session,
            recording_policy=recording_policy,
            status="incomplete" if incomplete else "completed",
            result_success=not incomplete,
            result_message=(
                "Session stopped normally"
                if not incomplete
                else "Recorder required forced cleanup or pause failed; partial artifacts preserved."
            ),
        )
        self._cleanup_runtime()
        return {
            "success": not incomplete,
            "incomplete": incomplete,
            "timing_validation_summary": timing_summary,
        }

    def abort_recording(
        self,
        *,
        active_session: ActiveSession,
        recording_policy: RecordingPolicy,
        message: str,
        event_log_entries: list[str] | None = None,
    ) -> None:
        if self._process is not None and self._process.poll() is None:
            self._terminate_process(recording_policy.cleanup_timeout_sec)
        artifacts = active_session.artifacts
        if artifacts is not None:
            if event_log_entries is not None:
                artifacts.event_log_path.write_text(
                    self._format_event_log(event_log_entries),
                    encoding="utf-8",
                )
            self._write_timing_report(active_session, recording_policy, True)
            self._write_metadata(
                active_session=active_session,
                recording_policy=recording_policy,
                status="failed",
                result_success=False,
                result_message=message,
            )
        self._cleanup_runtime()

    def finalize_failed_session(
        self,
        *,
        active_session: ActiveSession,
        recording_policy: RecordingPolicy,
        message: str,
        event_log_entries: list[str],
    ) -> None:
        artifacts = active_session.artifacts
        if artifacts is not None:
            artifacts.event_log_path.write_text(
                self._format_event_log(event_log_entries),
                encoding="utf-8",
            )
            self._write_timing_report(active_session, recording_policy, True)
            self._write_metadata(
                active_session=active_session,
                recording_policy=recording_policy,
                status="failed",
                result_success=False,
                result_message=message,
            )
        self._cleanup_runtime()

    def reset_runtime(self) -> None:
        self._terminate_process(1.0)
        self._cleanup_runtime()

    def _require_runtime(self, active_session: ActiveSession) -> RecorderRuntime:
        if self._runtime is None:
            raise RuntimeError(f"Recorder runtime not prepared for {active_session.session_id}.")
        return self._runtime

    def _require_artifacts(self, active_session: ActiveSession) -> SessionArtifacts:
        if active_session.artifacts is None:
            raise RuntimeError(f"Session artifacts not prepared for {active_session.session_id}.")
        return active_session.artifacts

    def _normalize_topics(self, topics: list[str]) -> list[str]:
        normalized: list[str] = []
        seen: set[str] = set()
        for topic in topics:
            candidate = str(topic).strip()
            if not candidate or candidate in seen:
                continue
            if not candidate.startswith("/"):
                raise RuntimeError(f"Recording topic must be absolute: {candidate}")
            normalized.append(candidate)
            seen.add(candidate)
        return normalized

    def _build_recorder_command(
        self,
        *,
        bag_dir: Path,
        node_name: str,
        recording_policy: RecordingPolicy,
        topics: list[str],
    ) -> list[str]:
        command = [
            "ros2",
            "bag",
            "record",
            "--disable-keyboard-controls",
            "--node-name",
            node_name,
            "--storage",
            recording_policy.storage_id,
            "-o",
            str(bag_dir),
            "--compression-mode",
            recording_policy.compression_mode,
        ]
        if recording_policy.compression_mode != "none":
            command.extend(["--compression-format", recording_policy.compression_format])
        if recording_policy.use_start_paused:
            command.append("--start-paused")
        command.append("--topics")
        command.extend(topics)
        return command

    def _create_session_dir(self, session_root: Path, session_id: str) -> Path:
        dated_dir = session_root.expanduser() / datetime.now().strftime("%Y-%m-%d")
        session_dir = dated_dir / session_id
        session_dir.mkdir(parents=True, exist_ok=False)
        return session_dir

    def _snapshot_inputs(
        self,
        *,
        artifacts: SessionArtifacts,
        recipe: RecipeSpec,
        supervisor_config: SupervisorConfig,
        recording_policy: RecordingPolicy,
    ) -> dict[str, Path]:
        snapshots: dict[str, Path] = {}
        if recording_policy.snapshot_recipe:
            path = self._snapshot_file(recipe.path, artifacts.session_dir / recipe.path.name)
            if path is not None:
                snapshots["recipe"] = path
        if recording_policy.snapshot_site_config and supervisor_config.cameras_config is not None:
            path = self._snapshot_file(
                supervisor_config.cameras_config,
                artifacts.session_dir / supervisor_config.cameras_config.name,
            )
            if path is not None:
                snapshots["cameras"] = path
        if recording_policy.snapshot_startup_policy and supervisor_config.startup_policy_config is not None:
            path = self._snapshot_file(
                supervisor_config.startup_policy_config,
                artifacts.session_dir / supervisor_config.startup_policy_config.name,
            )
            if path is not None:
                snapshots["startup_policy"] = path
        if recording_policy.snapshot_fault_policy and supervisor_config.fault_policy_config is not None:
            path = self._snapshot_file(
                supervisor_config.fault_policy_config,
                artifacts.session_dir / supervisor_config.fault_policy_config.name,
            )
            if path is not None:
                snapshots["fault_policy"] = path
        if recording_policy.snapshot_time_sync_config and supervisor_config.trackers_config is not None:
            path = self._snapshot_file(
                supervisor_config.trackers_config,
                artifacts.session_dir / supervisor_config.trackers_config.name,
            )
            if path is not None:
                snapshots["time_sync_config"] = path
        if recording_policy.snapshot_camera_calibration and supervisor_config.cameras_config is not None:
            path = self._snapshot_file(
                supervisor_config.cameras_config,
                artifacts.session_dir / f"camera_calibration_{supervisor_config.cameras_config.name}",
            )
            if path is not None:
                snapshots["camera_calibration"] = path
        if recording_policy.snapshot_robot_description:
            for source in (
                supervisor_config.wujihand_identity_file,
                supervisor_config.wujihand_teleop_config,
                supervisor_config.manus_config,
            ):
                if source is None:
                    continue
                path = self._snapshot_file(source, artifacts.session_dir / source.name)
                if path is not None:
                    snapshots.setdefault("robot_description", path)
        return snapshots

    def _snapshot_file(self, source: Path, destination: Path) -> Path | None:
        source = source.expanduser()
        if not source.exists() or not source.is_file():
            return None
        shutil.copy2(source, destination)
        return destination

    def _capture_git_state(self) -> dict[str, Any]:
        revision = ""
        dirty = False
        try:
            revision = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        except Exception:
            revision = ""
        try:
            status = subprocess.run(
                ["git", "status", "--short"],
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
            dirty = bool(status)
        except Exception:
            dirty = False
        return {"revision": revision, "dirty": dirty}

    def _write_metadata(
        self,
        *,
        active_session: ActiveSession,
        recording_policy: RecordingPolicy,
        status: str,
        result_success: bool,
        result_message: str,
    ) -> None:
        artifacts = active_session.artifacts
        if artifacts is None:
            return
        runtime = self._runtime
        metadata = {
            "session_id": active_session.session_id,
            "recipe_id": active_session.recipe_id,
            "operator_id": active_session.operator_id,
            "site_name": active_session.site_name,
            "session_tag": active_session.session_tag,
            "status": status,
            "session_dir": str(artifacts.session_dir),
            "bag_dir": str(artifacts.bag_dir),
            "record_topics": list(active_session.record_topics),
            "recorder": {
                "node_name": "" if runtime is None else runtime.node_name,
                "command": [] if runtime is None else list(runtime.command),
                "pid": None if runtime is None else runtime.pid,
                "pgid": None if runtime is None else runtime.pgid,
                "storage_id": recording_policy.storage_id,
                "compression_mode": recording_policy.compression_mode,
                "compression_format": recording_policy.compression_format,
            },
            "timestamps": {
                "session_start_ros_time_ns": active_session.session_start_ros_time_ns,
                "session_stop_ros_time_ns": active_session.session_stop_ros_time_ns,
                "session_start_wall_time": active_session.session_start_wall_time,
                "session_stop_wall_time": active_session.session_stop_wall_time,
                "session_start_monotonic_sec": active_session.session_start_monotonic_sec,
                "session_stop_monotonic_sec": active_session.session_stop_monotonic_sec,
            },
            "time_sync": {
                "mode": recording_policy.time_sync_mode,
                "healthy": bool(active_session.time_sync_summary.get("healthy", False)),
                "max_offset_ns": recording_policy.time_sync_max_offset_ns,
                **active_session.time_sync_summary,
            },
            "timing_contract": {
                "canonical_timestamp_clock": recording_policy.canonical_timestamp_clock,
                "canonical_timestamp_semantics": dict(recording_policy.canonical_timestamp_semantics),
                "image_timestamp_source": recording_policy.canonical_timestamp_semantics.get("image", ""),
                "joint_state_timestamp_source": recording_policy.canonical_timestamp_semantics.get("joint_states", ""),
                "alignment_anchor": "image_timestamp",
            },
            "training_ready": self._training_ready(active_session, recording_policy, status),
            "snapshots": {key: path.name for key, path in artifacts.snapshot_paths.items()},
            "timing_report": artifacts.timing_report_path.name,
            "git": self._capture_git_state() if recording_policy.capture_git_state else {},
            "result": {
                "success": result_success,
                "message": result_message,
            },
        }
        artifacts.metadata_path.write_text(
            json.dumps(metadata, indent=2, sort_keys=True),
            encoding="utf-8",
        )

    def _training_ready(
        self,
        active_session: ActiveSession,
        recording_policy: RecordingPolicy,
        status: str,
    ) -> bool:
        if status != "completed":
            return False
        if not recording_policy.training_ready_requires_canonical_timestamps:
            return True
        return bool(active_session.timing_validation_summary.get("canonical_timestamps_ok", False))

    def _write_timing_report(
        self,
        active_session: ActiveSession,
        recording_policy: RecordingPolicy,
        incomplete: bool,
    ) -> dict[str, Any]:
        artifacts = active_session.artifacts
        if artifacts is None:
            return {}
        canonical_timestamps_ok = (
            bool(active_session.time_sync_summary.get("canonical_topics_ok", False))
            and not incomplete
        )
        contract_healthy = bool(active_session.time_sync_summary.get("healthy", False)) and not incomplete
        report = {
            "generated_at": datetime.now(timezone.utc).astimezone().isoformat(),
            "session_id": active_session.session_id,
            "time_sync_mode": recording_policy.time_sync_mode,
            "required": recording_policy.post_session_timing_validation,
            "canonical_timestamp_clock": recording_policy.canonical_timestamp_clock,
            "canonical_timestamp_semantics": dict(recording_policy.canonical_timestamp_semantics),
            "pre_session": dict(active_session.time_sync_summary),
            "post_session": {
                "status": (
                    "passed"
                    if contract_healthy
                    else ("incomplete" if incomplete else "failed")
                ),
                "contract_healthy": contract_healthy,
                "canonical_timestamps_ok": canonical_timestamps_ok,
                "joint_state_min_rate_hz": recording_policy.joint_state_min_rate_hz,
                "joint_state_max_jitter_ms": recording_policy.joint_state_max_jitter_ms,
                "time_sync_max_offset_ns": recording_policy.time_sync_max_offset_ns,
            },
        }
        artifacts.timing_report_path.write_text(
            json.dumps(report, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        return dict(report["post_session"])

    def _create_service_clients(self, node_name: str) -> None:
        self._pause_client = self._node.create_client(Pause, f"/{node_name}/pause")
        self._resume_client = self._node.create_client(Resume, f"/{node_name}/resume")

    def _wait_for_service(self, client, timeout_sec: float, label: str) -> None:
        if client is None:
            raise RuntimeError(f"Recorder {label} client unavailable.")
        try:
            ready = client.wait_for_service(timeout_sec=timeout_sec)
        except Exception as exc:
            raise RuntimeError(f"Recorder {label} service wait failed: {exc}") from exc
        if not ready:
            raise RuntimeError(f"Recorder {label} service unavailable.")

    def _call_empty_service(self, client, request, timeout_sec: float, label: str) -> None:
        if client is None:
            raise RuntimeError(f"Recorder {label} client unavailable.")
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if future.done():
                break
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f"Recorder {label} request timeout.")
        if future.exception() is not None:
            raise RuntimeError(f"Recorder {label} request failed: {future.exception()}")

    def _wait_for_process_exit(self, timeout_sec: float) -> bool:
        if self._process is None:
            return True
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._process.poll() is not None:
                return True
            time.sleep(0.05)
        return self._process.poll() is not None

    def _request_process_stop(self, timeout_sec: float) -> bool:
        if self._process is None:
            return True
        process = self._process
        if process.poll() is not None:
            return True
        try:
            os.killpg(process.pid, signal.SIGINT)
        except Exception:
            process.send_signal(signal.SIGINT)
        return self._wait_for_process_exit(timeout_sec)

    def _terminate_process(self, cleanup_timeout_sec: float) -> None:
        if self._process is None:
            return
        process = self._process
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except Exception:
            process.terminate()
        deadline = time.monotonic() + cleanup_timeout_sec
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return
            time.sleep(0.05)
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except Exception:
            process.kill()
        try:
            process.wait(timeout=2.0)
        except Exception:
            pass

    def _cleanup_runtime(self) -> None:
        self._process = None
        self._pause_client = None
        self._resume_client = None
        self._runtime = None
        self._close_log_handle()

    def _close_log_handle(self) -> None:
        if self._log_handle is None:
            return
        try:
            self._log_handle.close()
        except Exception:
            pass
        self._log_handle = None

    def _format_event_log(self, entries: list[str]) -> str:
        text = "\n".join(entries)
        if text:
            text += "\n"
        return text
