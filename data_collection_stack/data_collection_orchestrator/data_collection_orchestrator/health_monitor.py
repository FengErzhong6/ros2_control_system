from __future__ import annotations

from dataclasses import dataclass

from data_collection_interfaces.msg import DeviceState

from .adapters.base import AdapterResult
from .models import DeviceSpec


@dataclass(frozen=True)
class DeviceHealthReport:
    device_id: str
    status: str
    health_state: int
    summary: str
    consecutive_unhealthy: int
    should_fault: bool
    metadata: dict | None = None


class HealthMonitor:
    def __init__(self, *, fatal_after: int = 2) -> None:
        self._fatal_after = max(1, fatal_after)
        self._consecutive_unhealthy: dict[str, int] = {}

    def reset(self) -> None:
        self._consecutive_unhealthy = {}

    def evaluate(
        self,
        devices: list[DeviceSpec],
        adapters: dict[str, object],
        *,
        fault_on_unhealthy: bool,
    ) -> dict[str, DeviceHealthReport]:
        reports: dict[str, DeviceHealthReport] = {}
        active_device_ids = {device.device_id for device in devices}
        self._consecutive_unhealthy = {
            device_id: count
            for device_id, count in self._consecutive_unhealthy.items()
            if device_id in active_device_ids
        }

        for device in devices:
            adapter = adapters.get(device.device_id)
            if adapter is None:
                diagnosis = AdapterResult.failed(
                    f"{device.device_id}: adapter missing from runtime."
                )
            else:
                try:
                    diagnosis = adapter.diagnose()
                except Exception as exc:  # pragma: no cover - defensive runtime path
                    diagnosis = AdapterResult.failed(
                        f"{device.device_id}: runtime health check raised {exc!r}."
                    )

            consecutive_unhealthy = self._update_unhealthy_count(device.device_id, diagnosis.status)
            health_state = self._map_health_state(diagnosis.status)
            should_fault = (
                fault_on_unhealthy
                and device.required
                and diagnosis.status in {"DEGRADED", "FAILED"}
                and consecutive_unhealthy >= self._fatal_after
            )
            summary = diagnosis.summary
            if diagnosis.status in {"DEGRADED", "FAILED"}:
                summary = f"{summary} [streak={consecutive_unhealthy}]"

            reports[device.device_id] = DeviceHealthReport(
                device_id=device.device_id,
                status=diagnosis.status,
                health_state=health_state,
                summary=summary,
                consecutive_unhealthy=consecutive_unhealthy,
                should_fault=should_fault,
                metadata=diagnosis.metadata,
            )

        return reports

    def summary(self) -> str:
        if not self._consecutive_unhealthy:
            return "No health streaks recorded."
        return f"Tracked runtime health for {len(self._consecutive_unhealthy)} devices."

    def _update_unhealthy_count(self, device_id: str, status: str) -> int:
        if status in {"DEGRADED", "FAILED"}:
            next_count = self._consecutive_unhealthy.get(device_id, 0) + 1
            self._consecutive_unhealthy[device_id] = next_count
            return next_count

        self._consecutive_unhealthy[device_id] = 0
        return 0

    def _map_health_state(self, status: str) -> int:
        if status == "OK":
            return DeviceState.HEALTH_OK
        if status == "DEGRADED":
            return DeviceState.HEALTH_DEGRADED
        if status == "FAILED":
            return DeviceState.HEALTH_FAILED
        return DeviceState.HEALTH_UNKNOWN
