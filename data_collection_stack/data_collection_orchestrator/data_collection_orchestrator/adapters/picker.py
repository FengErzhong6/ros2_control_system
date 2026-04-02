from .base import AdapterBase, AdapterResult


class PickerAdapter(AdapterBase):
    def bringup(self) -> AdapterResult:
        return AdapterResult.ok(f"{self.device.device_id}: Picker bringup placeholder ready")
