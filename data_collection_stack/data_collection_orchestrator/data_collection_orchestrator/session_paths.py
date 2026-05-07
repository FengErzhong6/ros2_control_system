from __future__ import annotations

import re


def sanitize_session_name(session_name: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9._-]+", "_", session_name.strip())
    sanitized = sanitized.strip("._-")
    return sanitized or "session"
