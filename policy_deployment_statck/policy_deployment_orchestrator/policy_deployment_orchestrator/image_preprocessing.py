from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
from PIL import Image as PILImage

try:
    PIL_RESAMPLE_LANCZOS = PILImage.Resampling.LANCZOS
except AttributeError:  # pragma: no cover - older Pillow compatibility
    PIL_RESAMPLE_LANCZOS = PILImage.LANCZOS


@dataclass(frozen=True)
class ImagePreprocessSpec:
    square_crop_anchor: str | None = None
    resize_size: tuple[int, int] | None = None


def decode_image_message(msg: Any) -> np.ndarray:
    encoding = str(msg.encoding).lower()
    channel_count = image_channel_count(encoding)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected_bytes = int(msg.height) * int(msg.step)
    if raw.size < expected_bytes:
        raise RuntimeError(
            f"Image data is shorter than expected: got {raw.size} bytes, expected {expected_bytes}"
        )
    rows = raw[:expected_bytes].reshape(int(msg.height), int(msg.step))
    pixel_bytes = int(msg.width) * channel_count
    if pixel_bytes > int(msg.step):
        raise RuntimeError(
            f"Image step is too small for {msg.width}x{msg.height} {msg.encoding}: step={msg.step}"
        )
    packed = rows[:, :pixel_bytes]
    if channel_count == 1:
        mono = packed.reshape(int(msg.height), int(msg.width))
        return np.repeat(mono[:, :, None], 3, axis=2)

    image = packed.reshape(int(msg.height), int(msg.width), channel_count)
    if encoding in {"rgb8"}:
        return image[:, :, :3]
    if encoding in {"bgr8", "8uc3"}:
        return image[:, :, [2, 1, 0]]
    if encoding in {"rgba8", "8uc4"}:
        return image[:, :, :3]
    if encoding == "bgra8":
        return image[:, :, [2, 1, 0]]
    raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")


def image_channel_count(encoding: str) -> int:
    if encoding in {"mono8", "8uc1"}:
        return 1
    if encoding in {"rgb8", "bgr8", "8uc3"}:
        return 3
    if encoding in {"rgba8", "bgra8", "8uc4"}:
        return 4
    raise RuntimeError(f"Unsupported image encoding: {encoding}")


def preprocess_image_for_topic(
    image: np.ndarray,
    topic: str,
    *,
    square_crop_anchor: str | None,
    resize_size: tuple[int, int] | None,
) -> np.ndarray:
    if square_crop_anchor:
        image = crop_square(image, anchor=_resolve_anchor(topic, square_crop_anchor))
    if resize_size is not None:
        image = resize_image(image, resize_size)
    return np.ascontiguousarray(image)


def crop_square(image: np.ndarray, *, anchor: str) -> np.ndarray:
    height, width = image.shape[:2]
    side = min(height, width)
    if side <= 0:
        raise RuntimeError(f"Image has invalid shape: {image.shape}")
    if anchor == "right":
        left = width - side
        top = 0
    else:
        left = (width - side) // 2
        top = (height - side) // 2
    return image[top : top + side, left : left + side]


def resize_image(image: np.ndarray, resize_size: tuple[int, int]) -> np.ndarray:
    target_width, target_height = resize_size
    if image.shape[1] == target_width and image.shape[0] == target_height:
        return np.ascontiguousarray(image)
    resized = PILImage.fromarray(np.ascontiguousarray(image), mode="RGB").resize(
        (target_width, target_height),
        resample=PIL_RESAMPLE_LANCZOS,
    )
    return np.asarray(resized, dtype=np.uint8)


def _resolve_anchor(topic: str, square_crop_anchor: str) -> str:
    anchor = square_crop_anchor.strip().lower()
    if anchor in {"center", "right"}:
        return anchor
    if anchor == "topic":
        topic_lower = topic.lower()
        return "right" if "cam_high" in topic_lower else "center"
    raise RuntimeError(f"Unsupported square crop anchor: {square_crop_anchor!r}")
