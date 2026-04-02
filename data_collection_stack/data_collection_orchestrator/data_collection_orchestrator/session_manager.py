from __future__ import annotations

from .models import ActiveSession


class SessionManager:
    def __init__(self) -> None:
        self._session_counter = 0
        self._active_session: ActiveSession | None = None

    @property
    def active_session(self) -> ActiveSession | None:
        return self._active_session

    def begin_session(self, recipe_id: str, operator_id: str, session_tag: str) -> ActiveSession:
        self._session_counter += 1
        session = ActiveSession(
            session_id=f"session_{self._session_counter:06d}",
            recipe_id=recipe_id,
            operator_id=operator_id or "unknown",
            session_tag=session_tag,
        )
        self._active_session = session
        return session

    def end_session(self) -> ActiveSession | None:
        session = self._active_session
        self._active_session = None
        return session

    def summary(self) -> str:
        if self._active_session is None:
            return "No active session."
        return f"Active session: {self._active_session.session_id}"
