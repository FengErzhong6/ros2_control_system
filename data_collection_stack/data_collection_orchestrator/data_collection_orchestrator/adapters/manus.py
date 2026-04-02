from .base import AdapterBase, AdapterResult


class ManusAdapter(AdapterBase):
    def bringup(self) -> AdapterResult:
        return AdapterResult.ok(f"{self.device.device_id}: Manus bringup placeholder ready")
