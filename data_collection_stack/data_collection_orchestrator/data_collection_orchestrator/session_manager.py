from __future__ import annotations

from pathlib import Path
import re

from .models import ActiveSession
from .session_paths import sanitize_session_name


_SESSION_ID_PATTERN = re.compile(r"^session_(\d{6})$")


class SessionManager:
    def __init__(self, *, session_root: Path | None = None) -> None:
        self._default_session_root = session_root
        self._session_counters: dict[str, int] = {}
        self._active_session: ActiveSession | None = None

    def _root_key(self, session_root: Path | None) -> str:
        if session_root is None:
            return "__default__"
        return str(session_root.expanduser().resolve())

    def _session_key(self, session_root: Path | None, session_name: str) -> str:
        return f"{self._root_key(session_root)}::{sanitize_session_name(session_name)}"

    def _session_name_directories(self, session_root: Path | None, session_name: str) -> list[Path]:
        if session_root is None:
            return []
        root = session_root.expanduser()
        if not root.exists() or not root.is_dir():
            return []

        sanitized_session_name = sanitize_session_name(session_name)
        session_name_dirs: list[Path] = []

        direct_dir = root / sanitized_session_name
        if direct_dir.exists() and direct_dir.is_dir():
            session_name_dirs.append(direct_dir)

        for child in root.iterdir():
            if not child.is_dir():
                continue
            legacy_dir = child / sanitized_session_name
            if legacy_dir.exists() and legacy_dir.is_dir():
                session_name_dirs.append(legacy_dir)

        return session_name_dirs

    def _discover_existing_counter(self, session_root: Path | None, session_name: str) -> int:
        if session_root is None:
            return 0

        counter = 0
        for session_name_dir in self._session_name_directories(session_root, session_name):
            for child in session_name_dir.iterdir():
                if not child.is_dir():
                    continue
                match = _SESSION_ID_PATTERN.match(child.name)
                if match is None:
                    continue
                counter = max(counter, int(match.group(1)))
        return counter

    def refresh_counter_from_disk(self, session_root: Path | None = None, session_name: str = "") -> None:
        effective_root = self._default_session_root if session_root is None else session_root
        key = self._session_key(effective_root, session_name)
        self._session_counters[key] = max(
            self._session_counters.get(key, 0),
            self._discover_existing_counter(effective_root, session_name),
        )

    def reserve_next_session_id(self, session_root: Path | None = None, session_name: str = "") -> str:
        effective_root = self._default_session_root if session_root is None else session_root
        key = self._session_key(effective_root, session_name)
        if key not in self._session_counters:
            self._session_counters[key] = self._discover_existing_counter(effective_root, session_name)
        self._session_counters[key] += 1
        return f"session_{self._session_counters[key]:06d}"

    def begin_session(
        self,
        recipe_id: str,
        operator_id: str,
        session_name: str,
        *,
        session_root: Path | None = None,
        site_name: str = "",
    ) -> ActiveSession:
        effective_root = self._default_session_root if session_root is None else session_root
        session = ActiveSession(
            session_id=self.reserve_next_session_id(effective_root, session_name),
            recipe_id=recipe_id,
            operator_id=operator_id or "unknown",
            session_name=session_name,
            session_root=effective_root,
            site_name=site_name,
        )
        self._active_session = session
        return session

    @property
    def active_session(self) -> ActiveSession | None:
        return self._active_session

    def end_session(self) -> ActiveSession | None:
        session = self._active_session
        self._active_session = None
        return session

    def summary(self) -> str:
        if self._active_session is None:
            return "No active session."
        return (
            f"Active session: {self._active_session.session_id} "
            f"({self._active_session.session_name})"
        )
