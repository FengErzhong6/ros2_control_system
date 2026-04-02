from dataclasses import dataclass, field


@dataclass
class EventLog:
    entries: list[str] = field(default_factory=list)

    def record(self, message: str) -> None:
        self.entries.append(message)
