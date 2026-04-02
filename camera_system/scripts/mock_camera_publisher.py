#!/usr/bin/env python3

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def bytes_per_pixel(encoding: str) -> int:
    if encoding in {"rgb8", "bgr8"}:
        return 3
    if encoding in {"mono8"}:
        return 1
    raise RuntimeError(f"Unsupported mock camera encoding: {encoding}")


class MockCameraPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mock_camera_publisher")

        self._camera_name = str(self.declare_parameter("camera_name", "mock_camera").value)
        self._frame_id = str(
            self.declare_parameter("frame_id", "camera_color_optical_frame").value
        )
        self._image_topic = str(self.declare_parameter("image_topic", "image_raw").value)
        self._camera_info_topic = str(
            self.declare_parameter("camera_info_topic", "camera_info").value
        )
        self._publish_camera_info = bool(
            self.declare_parameter("publish_camera_info", True).value
        )
        self._encoding = str(self.declare_parameter("encoding", "rgb8").value)
        self._width = int(self.declare_parameter("width", 640).value)
        self._height = int(self.declare_parameter("height", 480).value)
        self._publish_rate = float(self.declare_parameter("publish_rate", 30.0).value)
        self._counter = 0

        self._image_publisher = self.create_publisher(
            Image, self._image_topic, qos_profile_sensor_data
        )
        self._camera_info_publisher = None
        if self._publish_camera_info:
            self._camera_info_publisher = self.create_publisher(
                CameraInfo, self._camera_info_topic, qos_profile_sensor_data
            )

        period_sec = 1.0 / max(self._publish_rate, 1.0)
        self.create_timer(period_sec, self._publish_frame)
        self.get_logger().info(
            f"Mock camera publisher ready. camera={self._camera_name} "
            f"image_topic={self._image_topic} camera_info_topic={self._camera_info_topic} "
            f"publish_camera_info={self._publish_camera_info}"
        )

    def _publish_frame(self) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._frame_id
        msg.height = self._height
        msg.width = self._width
        msg.encoding = self._encoding
        msg.is_bigendian = False
        bpp = bytes_per_pixel(self._encoding)
        msg.step = self._width * bpp
        msg.data = self._image_bytes(self._width, self._height, self._counter, self._encoding)
        self._image_publisher.publish(msg)

        if self._camera_info_publisher is not None:
            info = CameraInfo()
            info.header.stamp = stamp
            info.header.frame_id = self._frame_id
            info.width = self._width
            info.height = self._height
            info.k = [
                float(self._width),
                0.0,
                float(self._width) / 2.0,
                0.0,
                float(self._height),
                float(self._height) / 2.0,
                0.0,
                0.0,
                1.0,
            ]
            info.p = [
                float(self._width),
                0.0,
                float(self._width) / 2.0,
                0.0,
                0.0,
                float(self._height),
                float(self._height) / 2.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]
            self._camera_info_publisher.publish(info)

        self._counter += 1

    def _image_bytes(self, width: int, height: int, counter: int, encoding: str) -> list[int]:
        phase = counter % 255
        if encoding == "mono8":
            return [phase] * (width * height)

        accent = int((127.5 * (1.0 + math.sin(counter / 10.0)))) % 255
        if encoding == "rgb8":
            pixel = [phase, 255 - phase, accent]
        else:
            pixel = [accent, 255 - phase, phase]
        return pixel * (width * height)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
