from dataclasses import dataclass


@dataclass
class RosbagProbe:
    directory: str

    def describe(self) -> str:
        return f"Rosbag probe placeholder for {self.directory}"
