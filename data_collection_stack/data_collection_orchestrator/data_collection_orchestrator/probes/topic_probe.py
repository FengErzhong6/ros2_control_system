from dataclasses import dataclass


@dataclass
class TopicProbe:
    topic_name: str

    def describe(self) -> str:
        return f"Topic probe placeholder for {self.topic_name}"
