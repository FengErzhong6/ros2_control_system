from dataclasses import dataclass

from ..models import DeviceSpec


@dataclass
class AdapterResult:
    status: str
    summary: str
    metadata: dict | None = None

    @classmethod
    def ok(cls, summary: str, metadata: dict | None = None) -> "AdapterResult":
        return cls(status="OK", summary=summary, metadata=metadata)

    @classmethod
    def failed(cls, summary: str, metadata: dict | None = None) -> "AdapterResult":
        return cls(status="FAILED", summary=summary, metadata=metadata)

    @classmethod
    def degraded(cls, summary: str, metadata: dict | None = None) -> "AdapterResult":
        return cls(status="DEGRADED", summary=summary, metadata=metadata)

    @classmethod
    def unsupported(
        cls, summary: str = "Not implemented", metadata: dict | None = None
    ) -> "AdapterResult":
        return cls(status="UNSUPPORTED", summary=summary, metadata=metadata)

    def is_failure(self) -> bool:
        return self.status == "FAILED"


class AdapterBase:
    def __init__(self, device: DeviceSpec, node=None) -> None:
        self.device = device
        self.node = node

    def precheck(self) -> AdapterResult:
        return AdapterResult.ok(f"{self.device.device_id}: precheck passed (no-op)")

    def bringup(self) -> AdapterResult:
        return AdapterResult.ok(f"{self.device.device_id}: bringup completed (no-op)")

    def wait_ready(self, timeout_sec: float) -> AdapterResult:
        del timeout_sec
        return AdapterResult.ok(f"{self.device.device_id}: ready (no-op)")

    def shutdown(self) -> AdapterResult:
        return AdapterResult.ok(f"{self.device.device_id}: shutdown completed (no-op)")

    def diagnose(self) -> AdapterResult:
        return AdapterResult.unsupported()

    def dump_metadata(self) -> dict:
        return {
            "device_id": self.device.device_id,
            "adapter": self.device.adapter,
            "required": self.device.required,
        }

    def record_topics(self) -> list[str]:
        return []

    def preview_topics(self) -> list[str]:
        return []

    def arm(self) -> AdapterResult:
        return AdapterResult.unsupported(f"{self.device.device_id}: arm unsupported")

    def disarm(self) -> AdapterResult:
        return AdapterResult.unsupported(f"{self.device.device_id}: disarm unsupported")

    def home(self) -> AdapterResult:
        return AdapterResult.unsupported(f"{self.device.device_id}: home unsupported")

    def before_session(self) -> AdapterResult:
        return AdapterResult.ok(f"{self.device.device_id}: before_session completed (no-op)")

    def after_session(self) -> AdapterResult:
        return AdapterResult.ok(f"{self.device.device_id}: after_session completed (no-op)")
