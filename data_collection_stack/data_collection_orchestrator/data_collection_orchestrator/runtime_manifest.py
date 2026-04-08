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
