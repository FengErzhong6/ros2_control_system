from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path


@dataclass
class EventLog:
    entries: list[str] = field(default_factory=list)

    def record(self, message: str) -> None:
        timestamp = datetime.now(timezone.utc).astimezone().isoformat()
        self.entries.append(f"{timestamp} {message}")

    def snapshot(self) -> list[str]:
        return list(self.entries)

    def write_to(self, path: Path) -> None:
        text = "\n".join(self.entries)
        if text:
            text += "\n"
        path.write_text(text, encoding="utf-8")
