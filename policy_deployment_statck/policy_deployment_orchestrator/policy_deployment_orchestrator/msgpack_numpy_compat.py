from __future__ import annotations

import struct
from typing import Any

import numpy as np


class MsgpackError(RuntimeError):
    pass


def pack_array(obj: Any) -> Any:
    if isinstance(obj, (np.ndarray, np.generic)) and obj.dtype.kind in ("V", "O", "c"):
        raise ValueError(f"Unsupported dtype: {obj.dtype}")

    if isinstance(obj, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": obj.tobytes(),
            b"dtype": obj.dtype.str,
            b"shape": obj.shape,
        }

    if isinstance(obj, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": obj.item(),
            b"dtype": obj.dtype.str,
        }

    return obj


def unpack_array(obj: dict[Any, Any]) -> Any:
    ndarray_key = b"__ndarray__" if b"__ndarray__" in obj else "__ndarray__"
    generic_key = b"__npgeneric__" if b"__npgeneric__" in obj else "__npgeneric__"
    if ndarray_key in obj:
        data_key = b"data" if b"data" in obj else "data"
        dtype_key = b"dtype" if b"dtype" in obj else "dtype"
        shape_key = b"shape" if b"shape" in obj else "shape"
        return np.ndarray(
            buffer=obj[data_key],
            dtype=np.dtype(obj[dtype_key]),
            shape=tuple(obj[shape_key]),
        )
    if generic_key in obj:
        data_key = b"data" if b"data" in obj else "data"
        dtype_key = b"dtype" if b"dtype" in obj else "dtype"
        return np.dtype(obj[dtype_key]).type(obj[data_key])
    return obj


class Packer:
    def pack(self, obj: Any) -> bytes:
        return packb(obj)


def packb(obj: Any) -> bytes:
    encoder = _Encoder()
    return encoder.pack(obj)


def unpackb(data: bytes | bytearray | memoryview) -> Any:
    decoder = _Decoder(bytes(data))
    value = decoder.unpack()
    if decoder.offset != len(decoder.data):
        raise MsgpackError("Trailing bytes after msgpack object")
    return value


class _Encoder:
    def pack(self, obj: Any) -> bytes:
        obj = pack_array(obj)

        if obj is None:
            return b"\xc0"
        if obj is False:
            return b"\xc2"
        if obj is True:
            return b"\xc3"
        if isinstance(obj, np.bool_):
            return self.pack(bool(obj))
        if isinstance(obj, int):
            return self._pack_int(obj)
        if isinstance(obj, float):
            return b"\xcb" + struct.pack(">d", obj)
        if isinstance(obj, str):
            data = obj.encode("utf-8")
            return self._pack_raw(data, str_type=True)
        if isinstance(obj, (bytes, bytearray, memoryview)):
            return self._pack_raw(bytes(obj), str_type=False)
        if isinstance(obj, (list, tuple)):
            return self._pack_array(obj)
        if isinstance(obj, dict):
            return self._pack_map(obj)

        raise TypeError(f"Cannot msgpack object of type {type(obj).__name__}")

    def _pack_int(self, value: int) -> bytes:
        if 0 <= value <= 0x7F:
            return bytes([value])
        if -32 <= value < 0:
            return bytes([0x100 + value])
        if 0 <= value <= 0xFF:
            return b"\xcc" + struct.pack(">B", value)
        if 0 <= value <= 0xFFFF:
            return b"\xcd" + struct.pack(">H", value)
        if 0 <= value <= 0xFFFFFFFF:
            return b"\xce" + struct.pack(">I", value)
        if 0 <= value <= 0xFFFFFFFFFFFFFFFF:
            return b"\xcf" + struct.pack(">Q", value)
        if -0x80 <= value < 0:
            return b"\xd0" + struct.pack(">b", value)
        if -0x8000 <= value < 0:
            return b"\xd1" + struct.pack(">h", value)
        if -0x80000000 <= value < 0:
            return b"\xd2" + struct.pack(">i", value)
        if -0x8000000000000000 <= value < 0:
            return b"\xd3" + struct.pack(">q", value)
        raise OverflowError(f"Integer out of msgpack range: {value}")

    def _pack_raw(self, data: bytes, *, str_type: bool) -> bytes:
        length = len(data)
        if str_type and length <= 31:
            return bytes([0xA0 | length]) + data
        if str_type:
            if length <= 0xFF:
                return b"\xd9" + struct.pack(">B", length) + data
            if length <= 0xFFFF:
                return b"\xda" + struct.pack(">H", length) + data
            return b"\xdb" + struct.pack(">I", length) + data
        if length <= 0xFF:
            return b"\xc4" + struct.pack(">B", length) + data
        if length <= 0xFFFF:
            return b"\xc5" + struct.pack(">H", length) + data
        return b"\xc6" + struct.pack(">I", length) + data

    def _pack_array(self, items: list[Any] | tuple[Any, ...]) -> bytes:
        length = len(items)
        if length <= 15:
            header = bytes([0x90 | length])
        elif length <= 0xFFFF:
            header = b"\xdc" + struct.pack(">H", length)
        else:
            header = b"\xdd" + struct.pack(">I", length)
        return header + b"".join(self.pack(item) for item in items)

    def _pack_map(self, mapping: dict[Any, Any]) -> bytes:
        length = len(mapping)
        if length <= 15:
            header = bytes([0x80 | length])
        elif length <= 0xFFFF:
            header = b"\xde" + struct.pack(">H", length)
        else:
            header = b"\xdf" + struct.pack(">I", length)
        body = bytearray()
        for key, value in mapping.items():
            body.extend(self.pack(key))
            body.extend(self.pack(value))
        return header + bytes(body)


class _Decoder:
    def __init__(self, data: bytes) -> None:
        self.data = data
        self.offset = 0

    def unpack(self) -> Any:
        code = self._read_u8()

        if code <= 0x7F:
            return code
        if code >= 0xE0:
            return code - 0x100
        if 0xA0 <= code <= 0xBF:
            return self._read_bytes(code & 0x1F).decode("utf-8")
        if 0x90 <= code <= 0x9F:
            return [self.unpack() for _ in range(code & 0x0F)]
        if 0x80 <= code <= 0x8F:
            return self._unpack_map(code & 0x0F)

        if code == 0xC0:
            return None
        if code == 0xC2:
            return False
        if code == 0xC3:
            return True
        if code == 0xC4:
            return self._read_bytes(self._read_u8())
        if code == 0xC5:
            return self._read_bytes(self._read(">H"))
        if code == 0xC6:
            return self._read_bytes(self._read(">I"))
        if code == 0xCA:
            return self._read(">f")
        if code == 0xCB:
            return self._read(">d")
        if code == 0xCC:
            return self._read_u8()
        if code == 0xCD:
            return self._read(">H")
        if code == 0xCE:
            return self._read(">I")
        if code == 0xCF:
            return self._read(">Q")
        if code == 0xD0:
            return self._read(">b")
        if code == 0xD1:
            return self._read(">h")
        if code == 0xD2:
            return self._read(">i")
        if code == 0xD3:
            return self._read(">q")
        if code == 0xD9:
            return self._read_bytes(self._read_u8()).decode("utf-8")
        if code == 0xDA:
            return self._read_bytes(self._read(">H")).decode("utf-8")
        if code == 0xDB:
            return self._read_bytes(self._read(">I")).decode("utf-8")
        if code == 0xDC:
            return [self.unpack() for _ in range(self._read(">H"))]
        if code == 0xDD:
            return [self.unpack() for _ in range(self._read(">I"))]
        if code == 0xDE:
            return self._unpack_map(self._read(">H"))
        if code == 0xDF:
            return self._unpack_map(self._read(">I"))

        raise MsgpackError(f"Unsupported msgpack code 0x{code:02x}")

    def _unpack_map(self, length: int) -> Any:
        mapping = {}
        for _ in range(length):
            key = self.unpack()
            value = self.unpack()
            mapping[key] = value
        return unpack_array(mapping)

    def _read_u8(self) -> int:
        if self.offset >= len(self.data):
            raise MsgpackError("Unexpected end of msgpack data")
        value = self.data[self.offset]
        self.offset += 1
        return value

    def _read(self, fmt: str) -> Any:
        size = struct.calcsize(fmt)
        raw = self._read_bytes(size)
        return struct.unpack(fmt, raw)[0]

    def _read_bytes(self, length: int) -> bytes:
        end = self.offset + length
        if end > len(self.data):
            raise MsgpackError("Unexpected end of msgpack data")
        raw = self.data[self.offset:end]
        self.offset = end
        return raw
