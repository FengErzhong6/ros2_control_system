from __future__ import annotations

from datetime import datetime
from pathlib import Path
import re

from .models import ActiveSession


_SESSION_ID_PATTERN = re.compile(r"^session_(\d{6})$")


class SessionManager:
    def __init__(self, *, session_root: Path | None = None) -> None:
        self._session_root = session_root
        self._session_counter = self._discover_existing_counter()
        self._active_session: ActiveSession | None = None

    def _discover_existing_counter(self) -> int:
        if self._session_root is None:
            return 0
        dated_dir = self._session_root.expanduser() / datetime.now().strftime("%Y-%m-%d")
        if not dated_dir.exists() or not dated_dir.is_dir():
            return 0

        counter = 0
        for child in dated_dir.iterdir():
            if not child.is_dir():
                continue
            match = _SESSION_ID_PATTERN.match(child.name)
            if match is None:
                continue
            counter = max(counter, int(match.group(1)))
        return counter

    def refresh_counter_from_disk(self) -> None:
        self._session_counter = max(self._session_counter, self._discover_existing_counter())

    def reserve_next_session_id(self) -> str:
        self.refresh_counter_from_disk()
        self._session_counter += 1
        return f"session_{self._session_counter:06d}"

    def begin_session(
        self,
        recipe_id: str,
        operator_id: str,
        session_tag: str,
        *,
        site_name: str = "",
    ) -> ActiveSession:
        session = ActiveSession(
            session_id=self.reserve_next_session_id(),
            recipe_id=recipe_id,
            operator_id=operator_id or "unknown",
            session_tag=session_tag,
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
        return f"Active session: {self._active_session.session_id}"
