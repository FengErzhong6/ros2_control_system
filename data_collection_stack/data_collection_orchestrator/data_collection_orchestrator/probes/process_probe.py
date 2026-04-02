from dataclasses import dataclass


@dataclass
class ProcessProbe:
    process_name: str

    def describe(self) -> str:
        return f"Process probe placeholder for {self.process_name}"
