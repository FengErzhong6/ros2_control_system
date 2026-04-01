#!/usr/bin/env python3

from collections import deque
import signal
import sys
import time
from dataclasses import dataclass

import rclpy
from cv_bridge import CvBridge
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QFrame, QApplication, QGridLayout, QLabel, QVBoxLayout, QWidget
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


@dataclass(frozen=True)
class CameraStream:
    key: str
    title: str
    topic: str


class TrackerCameraDashboard(Node):
    def __init__(self) -> None:
        super().__init__("tracker_camera_dashboard")

        self.bridge = CvBridge()
        self.window_title = str(
            self.declare_parameter("window_title", "Marvin Tracker Cameras").value
        )
        self.streams = [
            CameraStream(
                "high",
                str(self.declare_parameter("high_camera_title", "High Camera").value),
                str(
                    self.declare_parameter(
                        "high_image_topic",
                        "/cam_high/cam_high/color/image_raw",
                    ).value
                ),
            ),
            CameraStream(
                "left",
                str(self.declare_parameter("left_camera_title", "Left Wrist").value),
                str(self.declare_parameter("left_image_topic", "/cam_left_wrist/image_raw").value),
            ),
            CameraStream(
                "right",
                str(self.declare_parameter("right_camera_title", "Right Wrist").value),
                str(self.declare_parameter("right_image_topic", "/cam_right_wrist/image_raw").value),
            ),
        ]
        self.frames = {stream.key: None for stream in self.streams}
        self._frame_times = {stream.key: deque(maxlen=60) for stream in self.streams}
        self._warned_topics = set()

        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        for stream in self.streams:
            self.create_subscription(
                Image,
                stream.topic,
                lambda msg, key=stream.key: self._on_image(key, msg),
                image_qos,
            )
            self.get_logger().info(f"Subscribing to {stream.topic} ({stream.title})")

    def _on_image(self, key: str, msg: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8").copy()
        except Exception as exc:
            if key not in self._warned_topics:
                self._warned_topics.add(key)
                self.get_logger().warn(f"Failed to decode {key} image: {exc}")
            return

        if key in self._warned_topics:
            self._warned_topics.remove(key)

        received_monotonic = time.monotonic()
        frame_times = self._frame_times[key]
        frame_times.append(received_monotonic)
        fps = 0.0
        if len(frame_times) >= 2:
            duration = frame_times[-1] - frame_times[0]
            if duration > 0.0:
                fps = (len(frame_times) - 1) / duration

        previous = self.frames[key]
        seq = 1 if previous is None else previous["seq"] + 1
        self.frames[key] = {
            "seq": seq,
            "image": image,
            "encoding": msg.encoding,
            "stamp": msg.header.stamp,
            "received_monotonic": received_monotonic,
            "fps": fps,
        }


class CameraPanel(QFrame):
    def __init__(self, stream: CameraStream) -> None:
        super().__init__()
        self._pixmap = None
        self._placeholder = "Waiting for images..."

        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet(
            """
            QFrame {
                background: #101214;
                border: 1px solid #2c3138;
                border-radius: 14px;
            }
            """
        )

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(8)

        title = QLabel(stream.title)
        title.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        title.setStyleSheet("color: #f4f7fb; font-size: 20px; font-weight: 700;")
        root.addWidget(title)

        topic = QLabel(stream.topic)
        topic.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        topic.setStyleSheet("color: #8c97a6; font-size: 12px;")
        topic.setTextInteractionFlags(Qt.TextSelectableByMouse)
        root.addWidget(topic)

        self.fps = QLabel("FPS --")
        self.fps.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.fps.setStyleSheet("color: #7ed0ff; font-size: 13px; font-weight: 700;")
        root.addWidget(self.fps)

        self.image = QLabel(self._placeholder)
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumSize(320, 240)
        self.image.setStyleSheet(
            """
            QLabel {
                background: #050607;
                border-radius: 12px;
                color: #7b8694;
                font-size: 16px;
                font-weight: 600;
            }
            """
        )
        root.addWidget(self.image, stretch=1)

        self.meta = QLabel("No frames received")
        self.meta.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.meta.setStyleSheet("color: #aab4c0; font-size: 12px;")
        root.addWidget(self.meta)

    def set_frame(self, pixmap: QPixmap, fps_text: str, meta_text: str) -> None:
        self._pixmap = pixmap
        self._render_pixmap()
        self.fps.setText(fps_text)
        self.meta.setText(meta_text)

    def set_meta(self, fps_text: str, meta_text: str) -> None:
        self.fps.setText(fps_text)
        self.meta.setText(meta_text)

    def _render_pixmap(self) -> None:
        if self._pixmap is None:
            self.image.setPixmap(QPixmap())
            self.image.setText(self._placeholder)
            return

        scaled = self._pixmap.scaled(
            self.image.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.image.setText("")
        self.image.setPixmap(scaled)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._render_pixmap()


class TrackerCameraDashboardWindow(QWidget):
    def __init__(self, node: TrackerCameraDashboard) -> None:
        super().__init__()
        self.node = node
        self._last_seq = {stream.key: 0 for stream in self.node.streams}
        self.panels = {}

        self.setWindowTitle(self.node.window_title)
        self.resize(1560, 980)
        self.setStyleSheet("QWidget { background: #181b1f; color: #f4f7fb; }")

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(12)

        header = QLabel("Live Camera Dashboard")
        header.setStyleSheet("font-size: 28px; font-weight: 800; color: #f7fafc;")
        root.addWidget(header)

        subheader = QLabel("Top camera spans the full row; wrist cameras stay visible below.")
        subheader.setStyleSheet("font-size: 13px; color: #9aa6b2;")
        root.addWidget(subheader)

        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)
        root.addLayout(grid, stretch=1)

        for stream in self.node.streams:
            self.panels[stream.key] = CameraPanel(stream)

        grid.addWidget(self.panels["high"], 0, 0, 1, 2)
        grid.addWidget(self.panels["left"], 1, 0)
        grid.addWidget(self.panels["right"], 1, 1)
        grid.setRowStretch(0, 3)
        grid.setRowStretch(1, 2)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        self._ros_spin_timer = QTimer(self)
        self._ros_spin_timer.timeout.connect(self._spin_ros)
        self._ros_spin_timer.start(10)

        self._refresh_timer = QTimer(self)
        self._refresh_timer.timeout.connect(self._refresh_panels)
        self._refresh_timer.start(33)

    def _spin_ros(self) -> None:
        if not rclpy.ok():
            self._ros_spin_timer.stop()
            return

        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception:
            if not rclpy.ok():
                self._ros_spin_timer.stop()
                return
            raise

    def _refresh_panels(self) -> None:
        now = time.monotonic()
        for stream in self.node.streams:
            panel = self.panels[stream.key]
            frame = self.node.frames[stream.key]
            if frame is None:
                continue

            fps_text, meta_text = self._format_meta(frame, now)
            if frame["seq"] != self._last_seq[stream.key]:
                image = frame["image"]
                height, width, _ = image.shape
                qimage = QImage(
                    image.data,
                    width,
                    height,
                    image.strides[0],
                    QImage.Format_RGB888,
                ).copy()
                panel.set_frame(QPixmap.fromImage(qimage), fps_text, meta_text)
                self._last_seq[stream.key] = frame["seq"]
            else:
                panel.set_meta(fps_text, meta_text)

    def _format_meta(self, frame: dict, now: float) -> tuple[str, str]:
        image = frame["image"]
        height, width, _ = image.shape
        age_sec = max(0.0, now - frame["received_monotonic"])
        age_ms = int(age_sec * 1000.0)
        fps = frame.get("fps", 0.0)
        if age_sec > 1.0:
            fps = 0.0
        if frame["stamp"].sec == 0 and frame["stamp"].nanosec == 0:
            stamp_text = "stamp unavailable"
        else:
            stamp_text = f"stamp {frame['stamp'].sec}.{frame['stamp'].nanosec:09d}"
        fps_text = f"FPS {fps:.1f}"
        meta_text = f"{width}x{height} | {frame['encoding']} | {age_ms} ms ago | {stamp_text}"
        return fps_text, meta_text


def main() -> None:
    rclpy.init()
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QApplication(sys.argv)
    node = TrackerCameraDashboard()
    window = TrackerCameraDashboardWindow(node)
    window.show()

    exit_code = app.exec_()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
