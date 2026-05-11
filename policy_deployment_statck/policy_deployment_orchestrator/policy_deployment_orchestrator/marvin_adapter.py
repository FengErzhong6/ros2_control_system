from __future__ import annotations

from pathlib import Path
import os
import shutil
import signal
import subprocess
import tempfile
import time
from typing import Any

from .models import LaunchSpec


def _launch_value_to_string(value: object) -> str:
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


class SubprocessLaunchSession:
    """External ros2 launch process group, matching data_collection's Marvin model."""

    def __init__(
        self,
        *,
        label: str,
        launch_file_path: str,
        adapter: str = "marvin",
        launch_arguments: dict[str, object] | None = None,
        logger=None,
    ) -> None:
        self._label = label
        self._adapter = str(adapter).strip() or "marvin"
        self._launch_file_path = str(launch_file_path)
        self._launch_arguments = dict(launch_arguments or {})
        self._logger = logger
        self._process: subprocess.Popen | None = None
        self._started_monotonic = 0.0
        self._log_dir = Path(tempfile.mkdtemp(prefix=f"{label}_subprocess_launch_"))
        self._log_path = self._log_dir / "launch.log"
        self._log_handle = None

    @property
    def label(self) -> str:
        return self._label

    @property
    def adapter(self) -> str:
        return self._adapter

    @property
    def log_path(self) -> str:
        return str(self._log_path)

    def start(self) -> None:
        if self._process is not None and self._process.poll() is None:
            raise RuntimeError(f"{self._label}: subprocess launch is already running")

        command = self._build_command()
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        self._log_handle = self._log_path.open("w", encoding="utf-8")
        self._write_log_line("[command] " + " ".join(command))
        try:
            self._process = subprocess.Popen(
                command,
                stdout=self._log_handle,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=env,
            )
        except OSError:
            self._close_log_handle()
            raise
        self._started_monotonic = time.monotonic()
        if self._logger is not None:
            self._logger.info(
                f"{self._label}: started external launch pid={self._process.pid} "
                f"log={self._log_path}"
            )

    def wait_started(self, timeout_sec: float) -> bool:
        process = self._process
        if process is None:
            return False
        deadline = time.monotonic() + max(0.0, timeout_sec)
        settle_until = self._started_monotonic + min(0.5, max(0.0, timeout_sec))
        while time.monotonic() < deadline:
            if process.poll() is not None:
                self._close_log_handle()
                return False
            if time.monotonic() >= settle_until:
                return True
            time.sleep(0.05)
        return process.poll() is None

    def is_running(self) -> bool:
        process = self._process
        return process is not None and process.poll() is None

    def launch_exit_code(self) -> int | None:
        process = self._process
        if process is None:
            return None
        return process.poll()

    def metadata(self) -> dict:
        process = self._process
        pgid = None
        if process is not None:
            try:
                pgid = os.getpgid(process.pid)
            except OSError:
                pgid = None
        return {
            "pid": None if process is None else process.pid,
            "pgid": pgid,
            "command": self._build_command(),
            "log_path": str(self._log_path),
            "launch_file_path": self._launch_file_path,
            "launch_arguments": dict(self._launch_arguments),
            "process_model": "external_subprocess_group",
        }

    def shutdown(self, timeout_sec: float) -> bool:
        process = self._process
        if process is None:
            self._close_log_handle()
            return True
        if process.poll() is not None:
            self._close_log_handle()
            return True

        if timeout_sec <= 0.0:
            self._signal_process_group(signal.SIGINT, timeout_sec=0.0)
            return process.poll() is not None

        if not self._signal_process_group(signal.SIGINT, timeout_sec=min(timeout_sec, 12.0)):
            self._signal_process_group(signal.SIGTERM, timeout_sec=4.0)
        if process.poll() is None:
            self._signal_process_group(signal.SIGTERM, timeout_sec=4.0)
        if process.poll() is None:
            self._signal_process_group(signal.SIGKILL, timeout_sec=2.0)
        stopped = process.poll() is not None
        if stopped:
            self._close_log_handle()
        return stopped

    def close(self) -> None:
        self.shutdown(timeout_sec=0.0)
        self._close_log_handle()

    def _build_command(self) -> list[str]:
        command = ["ros2", "launch", self._launch_file_path]
        for key, value in self._launch_arguments.items():
            command.append(f"{key}:={_launch_value_to_string(value)}")
        return command

    def _signal_process_group(self, sig: signal.Signals, timeout_sec: float) -> bool:
        process = self._process
        if process is None or process.poll() is not None:
            return True
        try:
            pgid = os.getpgid(process.pid)
        except OSError:
            return process.poll() is not None
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return True

        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return True
            time.sleep(0.2)
        return process.poll() is not None

    def _write_log_line(self, line: str) -> None:
        if self._log_handle is None:
            return
        timestamp = time.time()
        self._log_handle.write(f"[{timestamp:.6f}] {line}\n")
        self._log_handle.flush()

    def _close_log_handle(self) -> None:
        if self._log_handle is not None:
            try:
                self._log_handle.flush()
            except OSError:
                pass
            try:
                self._log_handle.close()
            except OSError:
                pass
            self._log_handle = None


class PolicyMarvinAdapter:
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
        if not Path(self._launch_path).exists():
            raise RuntimeError(f"Marvin launch file does not exist: {self._launch_path}")

    def bringup(self, *, startup_timeout: float) -> SubprocessLaunchSession:
        self._supervisor._ensure_marvin_graph_cleanup(
            timeout_sec=12.0,
            context_label=f"{self._launch_spec.device_id} prelaunch",
            allow_external_cleanup=True,
        )
        session = SubprocessLaunchSession(
            label=self._launch_spec.device_id,
            adapter=self._launch_spec.adapter,
            launch_file_path=self._launch_path,
            launch_arguments=self._launch_arguments,
            logger=self._supervisor.get_logger(),
        )
        self._supervisor._managed_sessions.append(session)
        try:
            session.start()
            if not session.wait_started(max(startup_timeout, 1.0)):
                raise RuntimeError(
                    f"Launch {self._launch_spec.device_id} did not start. "
                    f"log={session.log_path}"
                )
            self._supervisor._runtime_manifest.register_device(
                device_id=self._launch_spec.device_id,
                adapter="marvin",
                metadata=session.metadata(),
            )
            return session
        except Exception:
            self._remove_session(session)
            raise

    def wait_ready(self, *, session: SubprocessLaunchSession, timeout_sec: float) -> None:
        self._supervisor._wait_for_launch_ready(
            session,
            self._launch_spec,
            timeout_sec,
        )

    def shutdown_session(
        self,
        session: SubprocessLaunchSession,
        *,
        context_label: str,
    ) -> None:
        try:
            self._supervisor._prepare_marvin_safe_shutdown(context_label)
            session.shutdown(timeout_sec=18.0)
            session.close()
        finally:
            self._remove_session(session)
            try:
                self._supervisor._runtime_manifest.unregister_device(
                    self._launch_spec.device_id
                )
            except Exception:
                pass
        self._supervisor._ensure_marvin_graph_cleanup(
            timeout_sec=12.0,
            context_label=f"{self._launch_spec.device_id} retry cleanup",
            allow_external_cleanup=True,
        )

    def _remove_session(self, session: SubprocessLaunchSession) -> None:
        self._supervisor._managed_sessions = [
            item for item in self._supervisor._managed_sessions if item is not session
        ]
