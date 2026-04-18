from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import time


class RuntimeManifest:
    DEFAULT_PATH = Path("/tmp/data_collection_supervisor_runtime.json")

    def __init__(self, path: Path | None = None) -> None:
        self._path = self.DEFAULT_PATH if path is None else path
        self._supervisor_pid = os.getpid()
        self._recipe_id = ""
        self._system_state = "IDLE"
        self._devices: dict[str, dict] = {}

    @property
    def path(self) -> Path:
        return self._path

    def cleanup_stale_runtime(self) -> list[str]:
        if not self._path.exists():
            return []

        try:
            data = self._load_file()
        except RuntimeError as exc:
            self.delete()
            return [f"Removed unreadable runtime manifest: {exc}"]

        old_supervisor_pid = int(data.get("supervisor_pid", 0) or 0)
        if old_supervisor_pid > 0 and self._pid_exists(old_supervisor_pid):
            return [
                "Runtime manifest belongs to a still-running supervisor "
                f"(pid={old_supervisor_pid}); stale cleanup skipped."
            ]

        notes: list[str] = []
        for device_id, entry in data.get("devices", {}).items():
            notes.extend(self._cleanup_device_entry(device_id, entry))

        self.delete()
        return notes

    def cleanup_known_orphans(self) -> list[str]:
        notes: list[str] = []
        notes.extend(self._cleanup_orphaned_stack_process_groups())
        return notes

    def set_context(self, *, recipe_id: str, system_state: str) -> None:
        self._recipe_id = recipe_id
        self._system_state = system_state
        self._sync()

    def register_device(
        self,
        *,
        device_id: str,
        adapter: str,
        metadata: dict | None,
    ) -> None:
        metadata = {} if metadata is None else dict(metadata)
        raw_pid = metadata.get("pid")
        try:
            pid = int(raw_pid)
        except (TypeError, ValueError):
            return

        if pid <= 0:
            return

        self._devices[device_id] = {
            "device_id": device_id,
            "adapter": adapter,
            "pid": pid,
            "pgid": int(metadata.get("pgid", pid)),
            "command": metadata.get("command", []),
            "log_path": metadata.get("log_path", ""),
            "registered_at_sec": time.time(),
        }
        self._sync()

    def unregister_device(self, device_id: str) -> None:
        if device_id not in self._devices:
            return
        self._devices.pop(device_id, None)
        self._sync()

    def clear(self) -> None:
        self._devices = {}
        self.delete()

    def delete(self) -> None:
        try:
            self._path.unlink()
        except FileNotFoundError:
            pass

    def _sync(self) -> None:
        if not self._devices:
            self.delete()
            return

        payload = {
            "supervisor_pid": self._supervisor_pid,
            "recipe_id": self._recipe_id,
            "system_state": self._system_state,
            "updated_at_sec": time.time(),
            "devices": self._devices,
        }

        self._path.parent.mkdir(parents=True, exist_ok=True)
        temp_path = self._path.with_suffix(f"{self._path.suffix}.tmp")
        temp_path.write_text(
            json.dumps(payload, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        os.replace(temp_path, self._path)

    def _load_file(self) -> dict:
        try:
            raw_text = self._path.read_text(encoding="utf-8")
        except OSError as exc:
            raise RuntimeError(f"{self._path}: {exc}") from exc

        try:
            data = json.loads(raw_text)
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"{self._path}: invalid JSON: {exc}") from exc

        if not isinstance(data, dict):
            raise RuntimeError(f"{self._path}: expected top-level JSON object")
        return data

    def _cleanup_device_entry(self, device_id: str, entry: object) -> list[str]:
        if not isinstance(entry, dict):
            return [f"Skipped stale cleanup for {device_id}: invalid manifest entry."]

        pid = self._coerce_positive_int(entry.get("pid"))
        pgid = self._coerce_positive_int(entry.get("pgid"))
        command = entry.get("command", [])

        if pid is None and pgid is None:
            return [f"Skipped stale cleanup for {device_id}: no pid/pgid recorded."]

        notes = []
        if pgid is not None and pgid not in {os.getpid(), os.getpgrp()}:
            notes.extend(
                self._signal_process_group(
                    pgid,
                    device_id=device_id,
                    command=command,
                    pid=pid,
                )
            )
            return notes

        if pid is not None and pid != os.getpid():
            notes.extend(
                self._signal_process(
                    pid,
                    device_id=device_id,
                    command=command,
                )
            )
            return notes

        return [f"Skipped stale cleanup for {device_id}: pid/pgid refers to current process."]

    def _signal_process_group(
        self,
        pgid: int,
        *,
        device_id: str,
        command: object,
        pid: int | None,
    ) -> list[str]:
        description = self._command_summary(command)
        notes = [
            f"Cleaning stale process group for {device_id}: pgid={pgid} command={description}"
        ]
        for sig, timeout_sec in (
            (signal.SIGINT, 8.0),
            (signal.SIGTERM, 4.0),
            (signal.SIGKILL, 2.0),
        ):
            if not self._process_group_exists(pgid):
                return notes
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return notes
            except PermissionError as exc:
                notes.append(f"Permission denied while signaling pgid={pgid}: {exc}")
                return notes
            if self._wait_for_process_group_exit(pgid, timeout_sec, pid=pid):
                return notes

        if self._process_group_exists(pgid) and not self._pid_is_gone(pid):
            notes.append(f"Process group {pgid} for {device_id} is still alive after cleanup.")
        return notes

    def _signal_process(self, pid: int, *, device_id: str, command: object) -> list[str]:
        description = self._command_summary(command)
        notes = [
            f"Cleaning stale process for {device_id}: pid={pid} command={description}"
        ]
        for sig, timeout_sec in (
            (signal.SIGINT, 8.0),
            (signal.SIGTERM, 4.0),
            (signal.SIGKILL, 2.0),
        ):
            if not self._pid_exists(pid):
                return notes
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                return notes
            except PermissionError as exc:
                notes.append(f"Permission denied while signaling pid={pid}: {exc}")
                return notes
            if self._wait_for_pid_exit(pid, timeout_sec):
                return notes

        if self._pid_exists(pid):
            notes.append(f"Process {pid} for {device_id} is still alive after cleanup.")
        return notes

    def _wait_for_process_group_exit(self, pgid: int, timeout_sec: float, *, pid: int | None) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self._process_group_exists(pgid) or self._pid_is_gone(pid):
                return True
            time.sleep(0.2)
        return (not self._process_group_exists(pgid)) or self._pid_is_gone(pid)

    def _wait_for_pid_exit(self, pid: int, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not self._pid_exists(pid):
                return True
            time.sleep(0.2)
        return not self._pid_exists(pid)

    def _pid_exists(self, pid: int) -> bool:
        status_path = Path(f"/proc/{pid}/status")
        if status_path.exists():
            try:
                status_text = status_path.read_text(encoding="utf-8", errors="replace")
            except OSError:
                status_text = ""
            if "\nState:\tZ" in status_text or status_text.startswith("State:\tZ"):
                return False
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _process_group_exists(self, pgid: int) -> bool:
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _coerce_positive_int(self, raw_value: object) -> int | None:
        try:
            parsed = int(raw_value)
        except (TypeError, ValueError):
            return None
        if parsed <= 0:
            return None
        return parsed

    def _pid_is_gone(self, pid: int | None) -> bool:
        if pid is None:
            return False
        return not self._pid_exists(pid)

    def _command_summary(self, command: object) -> str:
        if isinstance(command, list):
            return " ".join(str(item) for item in command)
        if isinstance(command, str):
            return command
        return "<unknown>"

    def _cleanup_orphaned_marvin_robot_state_publishers(self) -> list[str]:
        notes: list[str] = []
        cleaned_pgids: set[int] = set()
        cleaned_pids: set[int] = set()

        for proc_entry in Path("/proc").iterdir():
            if not proc_entry.name.isdigit():
                continue

            pid = int(proc_entry.name)
            if pid in {0, self._supervisor_pid, os.getpid()}:
                continue

            command = self._read_proc_cmdline(pid)
            if not command:
                continue
            if Path(command[0]).name != "robot_state_publisher":
                continue

            params_file = self._extract_params_file(command)
            if params_file is None or not self._params_file_contains_marvin_dual(params_file):
                continue

            ppid = self._read_proc_ppid(pid)
            if not self._process_looks_orphaned(ppid):
                continue

            pgid = self._safe_getpgid(pid)
            if pgid is not None and pgid not in {os.getpid(), os.getpgrp()}:
                if pgid in cleaned_pgids:
                    continue
                cleaned_pgids.add(pgid)
                notes.extend(
                    self._signal_process_group(
                        pgid,
                        device_id="orphaned_marvin_robot_state_publisher",
                        command=command,
                        pid=pid,
                    )
                )
                continue

            if pid in cleaned_pids:
                continue
            cleaned_pids.add(pid)
            notes.extend(
                self._signal_process(
                    pid,
                    device_id="orphaned_marvin_robot_state_publisher",
                    command=command,
                )
            )

        return notes

    def _cleanup_orphaned_stack_process_groups(self) -> list[str]:
        notes: list[str] = []
        cleaned_pgids: set[int] = set()
        cleaned_pids: set[int] = set()

        for proc_entry in Path("/proc").iterdir():
            if not proc_entry.name.isdigit():
                continue

            pid = int(proc_entry.name)
            if pid in {0, self._supervisor_pid, os.getpid()}:
                continue

            command = self._read_proc_cmdline(pid)
            if not command:
                continue

            device_id = self._stack_orphan_device_id(command)
            if not device_id:
                continue

            ppid = self._read_proc_ppid(pid)
            if not self._process_looks_orphaned(ppid):
                continue

            pgid = self._safe_getpgid(pid)
            if pgid is not None and pgid not in {os.getpid(), os.getpgrp()}:
                if pgid in cleaned_pgids:
                    continue
                cleaned_pgids.add(pgid)
                notes.extend(
                    self._signal_process_group(
                        pgid,
                        device_id=device_id,
                        command=command,
                        pid=pid,
                    )
                )
                continue

            if pid in cleaned_pids:
                continue
            cleaned_pids.add(pid)
            notes.extend(
                self._signal_process(
                    pid,
                    device_id=device_id,
                    command=command,
                )
            )

        return notes

    def _stack_orphan_device_id(self, command: list[str]) -> str:
        executable = Path(command[0]).name
        full_command = " ".join(command)

        if "ros2 launch camera_system single_realsense.launch.py" in full_command:
            return "orphaned_camera_stack"
        if "ros2 launch camera_system single_orbbec.launch.py" in full_command:
            return "orphaned_camera_stack"
        if "ros2 run htc_system tracker_publisher" in full_command:
            return "orphaned_htc_tracker"
        if "ros2 launch htc_system tracker_publisher.launch.py" in full_command:
            return "orphaned_htc_tracker"
        if "ros2 launch manus_system manus_raw_publisher.launch.py" in full_command:
            return "orphaned_manus_glove"
        if "ros2 launch marvin_system marvin_tracker_teleop.launch.py" in full_command:
            return "orphaned_marvin_stack"

        if executable == "camera_preview_bridge":
            return "orphaned_camera_stack"
        if executable == "tracker_publisher":
            return "orphaned_htc_tracker"
        if executable == "manus_raw_publisher_node":
            return "orphaned_manus_glove"
        if executable == "manus_gripper_node":
            return "orphaned_manus_gripper"
        if executable == "motion_server":
            return "orphaned_marvin_stack"
        if executable == "move_group_wrapper.py":
            return "orphaned_marvin_move_group"
        if executable == "ui_stub":
            return "orphaned_data_collection_ui"
        if executable == "robot_state_publisher":
            params_file = self._extract_params_file(command)
            if params_file is not None and self._params_file_contains_marvin_dual(params_file):
                return "orphaned_marvin_robot_state_publisher"
            return ""
        if executable == "ros2_control_node":
            for params_file in self._extract_params_files(command):
                params_path = Path(params_file)
                if (
                    self._params_file_contains_marvin_dual(params_path)
                    or "marvin_tracker_teleop_controllers.yaml" in str(params_path)
                    or "marvin_dual_trajectory_controllers.yaml" in str(params_path)
                ):
                    return "orphaned_marvin_ros2_control"
            return ""
        return ""

    def _read_proc_cmdline(self, pid: int) -> list[str]:
        cmdline_path = Path(f"/proc/{pid}/cmdline")
        try:
            raw_data = cmdline_path.read_bytes()
        except OSError:
            return []
        return [chunk.decode("utf-8", errors="replace") for chunk in raw_data.split(b"\0") if chunk]

    def _read_proc_ppid(self, pid: int) -> int | None:
        status_path = Path(f"/proc/{pid}/status")
        try:
            status_text = status_path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return None

        for line in status_text.splitlines():
            if not line.startswith("PPid:"):
                continue
            _, _, raw_value = line.partition(":")
            return self._coerce_positive_int(raw_value.strip())
        return None

    def _extract_params_file(self, command: list[str]) -> Path | None:
        for index, item in enumerate(command):
            if item != "--params-file":
                continue
            if index + 1 >= len(command):
                return None
            return Path(command[index + 1])
        return None

    def _extract_params_files(self, command: list[str]) -> list[Path]:
        paths: list[Path] = []
        for index, item in enumerate(command):
            if item != "--params-file":
                continue
            if index + 1 >= len(command):
                continue
            paths.append(Path(command[index + 1]))
        return paths

    def _params_file_contains_marvin_dual(self, path: Path) -> bool:
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError:
            return False
        return "marvin_dual.urdf" in text

    def _process_looks_orphaned(self, ppid: int | None) -> bool:
        if ppid is None or ppid <= 1:
            return True

        parent_command = self._read_proc_cmdline(ppid)
        if not parent_command:
            return True

        parent_executable = Path(parent_command[0]).name
        return parent_executable == "systemd" and "--user" in parent_command

    def _safe_getpgid(self, pid: int) -> int | None:
        try:
            return os.getpgid(pid)
        except ProcessLookupError:
            return None
        except PermissionError:
            return None
