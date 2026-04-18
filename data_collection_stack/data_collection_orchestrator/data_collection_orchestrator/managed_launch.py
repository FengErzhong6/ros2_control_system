from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import signal
import tempfile
import threading
import time
from typing import Callable

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource


def _normalize_launch_arguments(arguments: dict[str, object] | None) -> list[tuple[str, str]]:
    if not arguments:
        return []
    normalized = []
    for key, value in arguments.items():
        if isinstance(value, bool):
            normalized.append((str(key), "true" if value else "false"))
        else:
            normalized.append((str(key), str(value)))
    return normalized


@dataclass
class ManagedProcessInfo:
    pid: int
    process_name: str
    cmd: list[str]
    returncode: int | None = None
    started_at_monotonic: float = 0.0
    exited_at_monotonic: float = 0.0


class ManagedLaunchSession:
    def __init__(
        self,
        *,
        launch_manager: "LaunchManager",
        label: str,
        launch_file_path: str,
        launch_arguments: dict[str, object] | None = None,
        process_matcher: Callable[[str, list[str]], bool],
        logger=None,
    ) -> None:
        self._launch_manager = launch_manager
        self._label = label
        self._launch_file_path = str(launch_file_path)
        self._launch_arguments = dict(launch_arguments or {})
        self._process_matcher = process_matcher
        self._logger = logger

        self._lock = threading.RLock()
        self._started_event = threading.Event()
        self._finished_event = threading.Event()
        self._processes: dict[int, ManagedProcessInfo] = {}
        self._shutdown_requested = False
        self._included = False
        self._close_requested = False
        self._launch_exit_code: int | None = None

        log_dir = Path(tempfile.mkdtemp(prefix=f"{label}_launch_"))
        self._log_path = log_dir / "launch.log"
        self._log_handle = self._log_path.open("w", encoding="utf-8")

    @property
    def log_path(self) -> str:
        return str(self._log_path)

    def start(self) -> None:
        with self._lock:
            if self._included:
                raise RuntimeError(f"{self._label}: managed launch is already running")
            self._started_event.clear()
            self._finished_event.clear()
            self._processes = {}
            self._shutdown_requested = False
            self._close_requested = False
            self._launch_exit_code = None
            self._included = True
        self._launch_manager.include_session(self)

    def build_launch_description(self) -> LaunchDescription:
        include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(self._launch_file_path),
            launch_arguments=_normalize_launch_arguments(self._launch_arguments),
        )
        return LaunchDescription([include])

    def matches_process(self, process_name: str, cmd: list[str]) -> bool:
        with self._lock:
            if self._shutdown_requested and not self._processes:
                return False
        return self._process_matcher(process_name, cmd)

    def on_process_start(self, event) -> None:
        info = ManagedProcessInfo(
            pid=int(event.pid),
            process_name=str(event.process_name),
            cmd=[str(item) for item in event.cmd],
            started_at_monotonic=time.monotonic(),
        )
        with self._lock:
            self._processes[info.pid] = info
        self._started_event.set()
        self._write_log_line(
            f"[process-start] pid={info.pid} name={info.process_name} cmd={' '.join(info.cmd)}"
        )

    def on_process_exit(self, event) -> None:
        with self._lock:
            info = self._processes.get(int(event.pid))
            if info is None:
                return
            info.returncode = int(event.returncode)
            info.exited_at_monotonic = time.monotonic()
            all_exited = self._all_processes_exited_locked()
            if all_exited:
                self._launch_exit_code = 0 if all(
                    proc.returncode == 0 for proc in self._processes.values()
                ) else 1
                self._finished_event.set()
                if self._close_requested:
                    self._close_log_handle_locked()
        self._write_log_line(
            f"[process-exit] pid={event.pid} name={event.process_name} returncode={event.returncode}"
        )

    def on_process_io(self, event, *, stream: str) -> None:
        with self._lock:
            if int(event.pid) not in self._processes:
                return
        text = event.text.decode("utf-8", errors="replace")
        for raw_line in text.splitlines():
            self._write_log_line(
                f"[{stream}] pid={event.pid} name={event.process_name} {raw_line}"
            )

    def on_launch_complete(self, exit_code: int) -> None:
        with self._lock:
            if self._launch_exit_code is None:
                self._launch_exit_code = int(exit_code)
            if not self._processes or self._all_processes_exited_locked():
                self._finished_event.set()
                if self._close_requested:
                    self._close_log_handle_locked()
        self._write_log_line(f"[launch-complete] exit_code={exit_code}")

    def wait_started(self, timeout_sec: float) -> bool:
        return self._started_event.wait(timeout=timeout_sec)

    def is_running(self) -> bool:
        with self._lock:
            if self._finished_event.is_set():
                return False
            if self._processes:
                return any(info.returncode is None for info in self._processes.values())
            return self._included and self._launch_exit_code is None

    def shutdown(self, timeout_sec: float) -> bool:
        with self._lock:
            self._shutdown_requested = True
            pids = list(self._processes.keys())

        if not pids:
            self._finished_event.set()
            return True

        pgids = []
        for pid in pids:
            try:
                pgids.append(os.getpgid(pid))
            except OSError:
                continue

        unique_pgids = sorted({pgid for pgid in pgids if pgid not in {os.getpid(), os.getpgrp()}})
        if not unique_pgids:
            self._finished_event.set()
            return True

        for sig, wait_s in (
            (signal.SIGINT, min(timeout_sec, 6.0)),
            (signal.SIGTERM, 4.0),
            (signal.SIGKILL, 2.0),
        ):
            for pgid in unique_pgids:
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    continue
            if self.wait_finished(wait_s):
                return True

        return self.wait_finished(1.0)

    def wait_finished(self, timeout_sec: float) -> bool:
        return self._finished_event.wait(timeout=timeout_sec)

    def launch_exit_code(self) -> int | None:
        with self._lock:
            return self._launch_exit_code

    def metadata(self) -> dict:
        with self._lock:
            child_pids = sorted(self._processes.keys())
        child_pgids = []
        for pid in child_pids:
            try:
                child_pgids.append(os.getpgid(pid))
            except OSError:
                continue

        unique_pgids = sorted(set(child_pgids))
        return {
            "log_path": str(self._log_path),
            "pid": None if not child_pids else child_pids[0],
            "pgid": None if not unique_pgids else unique_pgids[0],
            "child_pids": child_pids,
            "child_pgids": unique_pgids,
            "launch_file_path": self._launch_file_path,
            "launch_arguments": dict(self._launch_arguments),
        }

    def close(self) -> None:
        self.shutdown(timeout_sec=0.0)
        with self._lock:
            self._close_requested = True
            if self._finished_event.is_set():
                self._close_log_handle_locked()

    def _all_processes_exited_locked(self) -> bool:
        return bool(self._processes) and all(
            info.returncode is not None for info in self._processes.values()
        )

    def _write_log_line(self, line: str) -> None:
        timestamp = time.time()
        with self._lock:
            if self._log_handle.closed:
                return
            self._log_handle.write(f"{timestamp:.6f} {line}\n")
            self._log_handle.flush()

    def _close_log_handle_locked(self) -> None:
        if not self._log_handle.closed:
            self._log_handle.close()


class LaunchManager:
    DEFAULT_LAUNCH_LOG_ROOT = Path("/tmp/data_collection_launch_logs")

    def __init__(self) -> None:
        self._ensure_launch_logging_root()
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._sessions: list[ManagedLaunchSession] = []
        self._pending_sessions: list[ManagedLaunchSession] = []
        self._active_sessions: list[ManagedLaunchSession] = []
        self._running_event = threading.Event()
        self._launch_service: LaunchService | None = None
        self._shutdown_requested = False

    def include_session(self, session: ManagedLaunchSession) -> None:
        with self._lock:
            self._sessions.append(session)
            if self._launch_service is None:
                self._pending_sessions.append(session)
                self._condition.notify_all()
                return
            self._active_sessions.append(session)
            launch_service = self._launch_service
        launch_service.include_launch_description(session.build_launch_description())

    def run_forever(self) -> int:
        last_exit_code = 0
        while True:
            with self._condition:
                while not self._shutdown_requested and not self._pending_sessions:
                    self._condition.wait()
                if self._shutdown_requested and not self._pending_sessions:
                    break
                pending_sessions = list(self._pending_sessions)
                self._pending_sessions = []
                self._active_sessions = list(pending_sessions)
                launch_service = LaunchService(noninteractive=True)
                launch_service.include_launch_description(self._event_bridge_description())
                self._launch_service = launch_service
                self._running_event.set()

            for session in pending_sessions:
                launch_service.include_launch_description(session.build_launch_description())

            try:
                last_exit_code = launch_service.run(shutdown_when_idle=True)
            finally:
                with self._condition:
                    batch_sessions = list(self._active_sessions)
                    self._active_sessions = []
                    self._launch_service = None
                    self._running_event.clear()
                    should_break = self._shutdown_requested and not self._pending_sessions
                for session in batch_sessions:
                    session.on_launch_complete(last_exit_code)
                with self._condition:
                    if should_break:
                        break
        return last_exit_code

    def shutdown(self) -> None:
        with self._condition:
            self._shutdown_requested = True
            launch_service = self._launch_service
            self._condition.notify_all()
        if launch_service is not None:
            launch_service.shutdown(force_sync=True)

    def wait_until_running(self, timeout_sec: float) -> bool:
        return self._running_event.wait(timeout=timeout_sec)

    def _event_bridge_description(self) -> LaunchDescription:
        return LaunchDescription(
            [
                RegisterEventHandler(
                    OnProcessStart(on_start=self._handle_process_start)
                ),
                RegisterEventHandler(
                    OnProcessExit(on_exit=self._handle_process_exit)
                ),
                RegisterEventHandler(
                    OnProcessIO(
                        on_stdout=self._handle_process_stdout,
                        on_stderr=self._handle_process_stderr,
                    )
                ),
            ]
        )

    def _handle_process_start(self, event, context):
        del context
        process_name = str(event.process_name)
        cmd = [str(item) for item in event.cmd]
        for session in self._session_snapshot():
            if session.matches_process(process_name, cmd):
                session.on_process_start(event)
        return None

    def _handle_process_exit(self, event, context):
        del context
        for session in self._session_snapshot():
            session.on_process_exit(event)
        return None

    def _handle_process_stdout(self, event):
        for session in self._session_snapshot():
            session.on_process_io(event, stream="stdout")
        return None

    def _handle_process_stderr(self, event):
        for session in self._session_snapshot():
            session.on_process_io(event, stream="stderr")
        return None

    def _session_snapshot(self) -> list[ManagedLaunchSession]:
        with self._lock:
            return list(self._sessions)

    def _ensure_launch_logging_root(self) -> None:
        if "ROS_LOG_DIR" in os.environ and os.environ["ROS_LOG_DIR"].strip():
            return
        self.DEFAULT_LAUNCH_LOG_ROOT.mkdir(parents=True, exist_ok=True)
        os.environ["ROS_LOG_DIR"] = str(self.DEFAULT_LAUNCH_LOG_ROOT)
