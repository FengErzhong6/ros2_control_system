from .base import AdapterBase, AdapterResult


class WujihandAdapter(AdapterBase):
    def bringup(self) -> AdapterResult:
        return AdapterResult.ok(f"{self.device.device_id}: Wujihand bringup placeholder ready")
