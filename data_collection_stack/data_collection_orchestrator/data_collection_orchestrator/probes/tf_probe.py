from dataclasses import dataclass


@dataclass
class TfProbe:
    frame_id: str

    def describe(self) -> str:
        return f"TF probe placeholder for {self.frame_id}"
