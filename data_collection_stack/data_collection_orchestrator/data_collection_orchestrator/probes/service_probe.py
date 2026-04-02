from dataclasses import dataclass


@dataclass
class ServiceProbe:
    service_name: str

    def describe(self) -> str:
        return f"Service probe placeholder for {self.service_name}"
