#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
import yaml


DEFAULT_PARENT_FRAME = "world"
DEFAULT_RATE_HZ = 60.0


class MockTrackerPublisher(Node):
    def __init__(self) -> None:
        super().__init__("mock_tracker_publisher")

        self._config_file = Path(
            str(self.declare_parameter("config_file", "").value)
        ).expanduser()
        self._publish_rate = float(
            self.declare_parameter("publish_rate", DEFAULT_RATE_HZ).value
        )
        self._broadcaster = TransformBroadcaster(self)
        self._parent_frame, self._frame_ids = self._load_config()
        self._start_time = self.get_clock().now()

        period_sec = 1.0 / max(self._publish_rate, 1.0)
        self.create_timer(period_sec, self._publish_mock_tf)
        self.get_logger().info(
            f"Mock tracker publisher ready. parent_frame={self._parent_frame} "
            f"frames={self._frame_ids}"
        )

    def _load_config(self) -> tuple[str, list[str]]:
        if not self._config_file.exists():
            self.get_logger().warn(
                f"Config file does not exist: {self._config_file}. Using defaults."
            )
            return DEFAULT_PARENT_FRAME, self._default_frame_ids()

        with self._config_file.open("r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        params = data.get("tracker_publisher", {}).get("ros__parameters", {})
        parent_frame = str(params.get("parent_frame", DEFAULT_PARENT_FRAME))
        frame_ids = [
            str(frame_id)
            for frame_id in params.get("trackers", {}).get("frame_ids", [])
            if str(frame_id).strip()
        ]
        if not frame_ids:
            frame_ids = self._default_frame_ids()
        return parent_frame, frame_ids

    def _default_frame_ids(self) -> list[str]:
        return [
            "tracker_left_hand",
            "tracker_right_hand",
            "tracker_left_upper_arm",
            "tracker_right_upper_arm",
            "tracker_torso",
        ]

    def _publish_mock_tf(self) -> None:
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        stamp = self.get_clock().now().to_msg()

        transforms: list[TransformStamped] = []
        for index, frame_id in enumerate(self._frame_ids):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._parent_frame
            transform.child_frame_id = frame_id

            x, y, z = self._pose_for_frame(frame_id, index, elapsed)
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = z
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            transforms.append(transform)

        self._broadcaster.sendTransform(transforms)

    def _pose_for_frame(self, frame_id: str, index: int, elapsed: float) -> tuple[float, float, float]:
        sway = 0.02 * math.sin(elapsed * 1.2)
        lift = 0.01 * math.cos(elapsed * 0.8)

        if "torso" in frame_id:
            return 0.0, 0.0, 1.20
        if "left_hand" in frame_id:
            return 0.32 + sway, 0.28, 0.95 + lift
        if "right_hand" in frame_id:
            return 0.32 + sway, -0.28, 0.95 + lift
        if "left_upper_arm" in frame_id:
            return 0.12 + 0.5 * sway, 0.18, 1.05
        if "right_upper_arm" in frame_id:
            return 0.12 + 0.5 * sway, -0.18, 1.05

        return 0.10 + 0.02 * index, -0.10 + 0.05 * index, 1.00


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MockTrackerPublisher()
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
