from .base import AdapterBase, AdapterResult
from .camera import CameraAdapter
from .htc import HtcAdapter
from .manus import ManusAdapter
from .marvin import MarvinAdapter
from .picker import PickerAdapter
from .wujihand import WujihandAdapter


ADAPTER_TYPES = {
    "camera": CameraAdapter,
    "htc": HtcAdapter,
    "manus": ManusAdapter,
    "marvin": MarvinAdapter,
    "picker": PickerAdapter,
    "wujihand": WujihandAdapter,
}


def create_adapter(device_spec, node=None):
    adapter_cls = ADAPTER_TYPES.get(device_spec.adapter, AdapterBase)
    return adapter_cls(device_spec, node=node)

__all__ = [
    "AdapterBase",
    "AdapterResult",
    "ADAPTER_TYPES",
    "CameraAdapter",
    "HtcAdapter",
    "ManusAdapter",
    "MarvinAdapter",
    "PickerAdapter",
    "WujihandAdapter",
    "create_adapter",
]
