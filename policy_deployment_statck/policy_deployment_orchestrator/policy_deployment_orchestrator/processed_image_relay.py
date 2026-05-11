from __future__ import annotations

import json
import time
from typing import Any

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .image_preprocessing import decode_image_message, preprocess_image_for_topic


def _normalize_topic(value: str) -> str:
    text = str(value).strip()
    if not text:
        raise RuntimeError("Topic must not be empty.")
    return text if text.startswith("/") else f"/{text}"


class ProcessedImageRelay(Node):
    def __init__(self) -> None:
        super().__init__("policy_deployment_processed_image_relay")

        raw_specs = self.declare_parameter("image_specs", "[]").value
        self._specs = self._parse_specs(raw_specs)
        self._preview_publishers: dict[str, Any] = {}
        self._preview_subscriptions = []
        self._last_publish_monotonic: dict[str, float] = {}
        self._specs_by_source: dict[str, list[dict[str, Any]]] = {}

        for spec in self._specs:
            publisher = self.create_publisher(Image, spec["output_topic"], qos_profile_sensor_data)
            self._preview_publishers[spec["output_topic"]] = publisher
            self._specs_by_source.setdefault(spec["source_topic"], []).append(spec)

        for source_topic, specs in self._specs_by_source.items():
            self._preview_subscriptions.append(
                self.create_subscription(
                    Image,
                    source_topic,
                    self._make_callback(source_topic, specs),
                    qos_profile_sensor_data,
                )
            )

        joined = ", ".join(
            f"{spec['source_topic']}->{spec['output_topic']}" for spec in self._specs
        )
        self.get_logger().info(
            f"Processed image relay active for {len(self._specs)} streams: {joined}"
        )

    def _parse_specs(self, raw_specs: object) -> list[dict[str, Any]]:
        parsed = json.loads(raw_specs) if isinstance(raw_specs, str) else raw_specs
        if not isinstance(parsed, list):
            raise RuntimeError("image_specs must be a JSON array.")

        specs: list[dict[str, Any]] = []
        seen_outputs: set[str] = set()
        for item in parsed:
            if not isinstance(item, dict):
                raise RuntimeError("Each image spec must be a JSON object.")
            source_topic = _normalize_topic(str(item.get("source_topic", "")))
            output_topic = _normalize_topic(str(item.get("output_topic", "")))
            if output_topic in seen_outputs:
                raise RuntimeError(f"Duplicate processed image output topic: {output_topic}")
            seen_outputs.add(output_topic)

            resize_size = item.get("resize_size")
            if resize_size is not None:
                if not isinstance(resize_size, list) or len(resize_size) != 2:
                    raise RuntimeError(f"resize_size must be [width, height]: {resize_size}")
                resize_size = (int(resize_size[0]), int(resize_size[1]))

            specs.append(
                {
                    "camera_id": str(item.get("camera_id", "")),
                    "title": str(item.get("title", item.get("camera_id", ""))),
                    "source_topic": source_topic,
                    "output_topic": output_topic,
                    "square_crop_anchor": item.get("square_crop_anchor"),
                    "resize_size": resize_size,
                    "max_fps": max(0.0, float(item.get("max_fps", 10.0))),
                }
            )
        return specs

    def _make_callback(self, source_topic: str, specs: list[dict[str, Any]]):

        def _callback(message: Image) -> None:
            try:
                raw_image = decode_image_message(message)
                for spec in specs:
                    if not self._should_publish(spec):
                        continue
                    try:
                        publisher = self._preview_publishers[spec["output_topic"]]
                        processed = preprocess_image_for_topic(
                            raw_image,
                            source_topic,
                            square_crop_anchor=spec["square_crop_anchor"],
                            resize_size=spec["resize_size"],
                        )
                        publisher.publish(
                            self._to_image_msg(processed, frame_id=message.header.frame_id)
                        )
                    except Exception as exc:
                        self.get_logger().warning(
                            f"Failed to publish preview {spec['output_topic']}: {exc}"
                        )
            except Exception as exc:
                self.get_logger().warning(f"Failed to process image {source_topic}: {exc}")

        return _callback

    def _should_publish(self, spec: dict[str, Any]) -> bool:
        max_fps = float(spec["max_fps"])
        if max_fps <= 0.0:
            return True
        now = time.monotonic()
        output_topic = spec["output_topic"]
        last = self._last_publish_monotonic.get(output_topic, 0.0)
        if now - last < 1.0 / max_fps:
            return False
        self._last_publish_monotonic[output_topic] = now
        return True

    def _to_image_msg(self, image: np.ndarray, *, frame_id: str) -> Image:
        if image.ndim != 3 or image.shape[2] != 3:
            raise RuntimeError(f"Expected RGB image HxWx3, got {image.shape}")
        if image.dtype != np.uint8:
            image = image.astype(np.uint8, copy=False)
        image = np.ascontiguousarray(image)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.height = int(image.shape[0])
        msg.width = int(image.shape[1])
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = int(msg.width * 3)
        msg.data = image.tobytes()
        return msg


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ProcessedImageRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
