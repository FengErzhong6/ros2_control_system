from __future__ import annotations

import asyncio
import logging
import time
from typing import Any

try:
    import msgpack  # type: ignore
except Exception:  # pragma: no cover - optional runtime dependency
    msgpack = None
import numpy as np
try:
    import websockets.sync.client as _sync_client  # type: ignore
except Exception:  # pragma: no cover - depends on distro websockets version
    _sync_client = None
    import websockets.legacy.client as _legacy_client  # type: ignore

from . import msgpack_numpy_compat


def _pack_array(obj: Any) -> Any:
    if (isinstance(obj, (np.ndarray, np.generic))) and obj.dtype.kind in ("V", "O", "c"):
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


def _unpack_array(obj: dict[Any, Any]) -> Any:
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


class _Codec:
    def __init__(self) -> None:
        if msgpack is not None:
            self._packer = msgpack.Packer(default=_pack_array)
        else:
            self._packer = msgpack_numpy_compat.Packer()

    def pack(self, obj: Any) -> bytes:
        if msgpack is None:
            return self._packer.pack(obj)
        return self._packer.pack(obj)

    def unpack(self, data: bytes) -> Any:
        if msgpack is None:
            return msgpack_numpy_compat.unpackb(data)
        return msgpack.unpackb(data, object_hook=_unpack_array)


class WebsocketPolicyClient:
    def __init__(
        self,
        *,
        host: str,
        port: int,
        connect_timeout_sec: float = 10.0,
        api_key: str = "",
    ) -> None:
        if host.startswith("ws"):
            self._uri = host
        else:
            self._uri = f"ws://{host}"
        if port is not None:
            self._uri += f":{int(port)}"
        self._connect_timeout_sec = max(0.1, float(connect_timeout_sec))
        self._api_key = api_key
        self._codec = _Codec()
        self._ws = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._metadata: dict[str, Any] = {}

    @property
    def metadata(self) -> dict[str, Any]:
        return dict(self._metadata)

    def connect(self) -> None:
        deadline = time.monotonic() + self._connect_timeout_sec
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                headers = {"Authorization": f"Api-Key {self._api_key}"} if self._api_key else None
                self._ws = self._connect_once(headers)
                metadata_raw = self._recv()
                if isinstance(metadata_raw, str):
                    raise RuntimeError(metadata_raw)
                metadata = self._codec.unpack(metadata_raw)
                self._metadata = metadata if isinstance(metadata, dict) else {}
                return
            except Exception as exc:  # pragma: no cover - network timing
                last_error = exc
                logging.info("Waiting for policy server at %s: %s", self._uri, exc)
                time.sleep(0.25)
        raise RuntimeError(f"Policy server unavailable at {self._uri}: {last_error}")

    def infer(self, observation: dict[str, Any]) -> dict[str, Any]:
        if self._ws is None:
            self.connect()
        assert self._ws is not None
        self._send(self._codec.pack(observation))
        response = self._recv()
        if isinstance(response, str):
            raise RuntimeError(f"Error in policy server:\n{response}")
        result = self._codec.unpack(response)
        if not isinstance(result, dict):
            raise RuntimeError(f"Policy server returned {type(result).__name__}, expected dict")
        return result

    def close(self) -> None:
        if self._ws is not None:
            try:
                if _sync_client is not None:
                    self._ws.close()
                elif self._loop is not None:
                    self._loop.run_until_complete(self._ws.close())
            except Exception:
                pass
            self._ws = None
        if self._loop is not None:
            try:
                self._loop.close()
            except Exception:
                pass
            self._loop = None

    def _connect_once(self, headers: dict[str, str] | None):
        if _sync_client is not None:
            return _sync_client.connect(
                self._uri,
                compression=None,
                max_size=None,
                additional_headers=headers,
            )
        if self._loop is None:
            self._loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self._loop)
        return self._loop.run_until_complete(
            _legacy_client.connect(
                self._uri,
                compression=None,
                max_size=None,
                extra_headers=headers,
            )
        )

    def _send(self, data: bytes) -> None:
        assert self._ws is not None
        if _sync_client is not None:
            self._ws.send(data)
            return
        assert self._loop is not None
        self._loop.run_until_complete(self._ws.send(data))

    def _recv(self):
        assert self._ws is not None
        if _sync_client is not None:
            return self._ws.recv()
        assert self._loop is not None
        return self._loop.run_until_complete(self._ws.recv())
