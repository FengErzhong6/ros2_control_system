from __future__ import annotations

from dataclasses import dataclass
import time

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (
    QFrame,
    QGridLayout,
    QLabel,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from ..ros_client import CameraStreamSnapshot
from ..view_model import CameraStreamConfig


@dataclass
class CameraPanelModel:
    title: str = "Cameras"


def _pixmap_from_image_message(image_msg) -> QPixmap | None:
    if image_msg is None:
        return None

    if image_msg.encoding == "rgb8":
        image_format = QImage.Format_RGB888
        payload = bytes(image_msg.data)
        image = QImage(payload, image_msg.width, image_msg.height, image_msg.step, image_format)
    elif image_msg.encoding == "bgr8":
        payload = bytes(image_msg.data)
        image = QImage(
            payload,
            image_msg.width,
            image_msg.height,
            image_msg.step,
            QImage.Format_BGR888,
        )
    elif image_msg.encoding == "mono8":
        payload = bytes(image_msg.data)
        image = QImage(
            payload,
            image_msg.width,
            image_msg.height,
            image_msg.step,
            QImage.Format_Grayscale8,
        )
    else:
        return None

    if image.isNull():
        return None
    return QPixmap.fromImage(image)


class CameraTileWidget(QFrame):
    def __init__(self, stream_config: CameraStreamConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._stream_config = stream_config
        self._last_generation = -1
        self._current_pixmap: QPixmap | None = None
        self._display_window_start = time.monotonic()
        self._display_window_frames = 0
        self._display_fps = 0.0

        self.setFrameShape(QFrame.StyledPanel)
        self.setLineWidth(1)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self._title_label = QLabel(stream_config.title, self)
        self._title_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._title_label.setStyleSheet("font-weight: 600;")
        layout.addWidget(self._title_label)

        self._image_label = QLabel("Waiting for preview...", self)
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setMinimumSize(320, 240)
        self._image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._image_label.setStyleSheet("background: #101418; color: #d0d7de;")
        layout.addWidget(self._image_label, 1)

        self._stats_label = QLabel(self)
        self._stats_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self._stats_label.setWordWrap(True)
        layout.addWidget(self._stats_label)
        self._set_waiting_text()

    def refresh(self, snapshot: CameraStreamSnapshot, now_wall_time: float) -> None:
        self._roll_display_window()
        if snapshot.generation != self._last_generation and snapshot.image_msg is not None:
            pixmap = _pixmap_from_image_message(snapshot.image_msg)
            if pixmap is not None:
                self._current_pixmap = pixmap
                self._last_generation = snapshot.generation
                self._display_window_frames += 1
                self._apply_pixmap()
            else:
                self._image_label.setText(f"Unsupported preview encoding: {snapshot.image_msg.encoding}")

        self._update_stats(snapshot, now_wall_time)

    def resizeEvent(self, event) -> None:  # noqa: N802
        self._apply_pixmap()
        super().resizeEvent(event)

    def _roll_display_window(self) -> None:
        now = time.monotonic()
        elapsed = now - self._display_window_start
        if elapsed >= 1.0:
            self._display_fps = self._display_window_frames / elapsed
            self._display_window_frames = 0
            self._display_window_start = now

    def _apply_pixmap(self) -> None:
        if self._current_pixmap is None:
            return

        scaled = self._current_pixmap.scaled(
            self._image_label.size(),
            Qt.KeepAspectRatio,
            Qt.FastTransformation,
        )
        self._image_label.setPixmap(scaled)

    def _set_waiting_text(self) -> None:
        self._stats_label.setText(
            f"Capture Topic: {self._capture_topic_text()}\n"
            f"Capture FPS Target: {self._capture_fps_target_text()}\n"
            f"Preview Topic: {self._stream_config.preview_topic}\n"
            "Preview RX FPS: 0.0\n"
            "UI Display FPS: 0.0\n"
            f"Preview Publish Cap: {self._preview_cap_text()}\n"
            "Frame Age: n/a\n"
            "Resolution: waiting"
        )

    def _update_stats(self, snapshot: CameraStreamSnapshot, now_wall_time: float) -> None:
        if snapshot.image_msg is None:
            self._set_waiting_text()
            return

        frame_age_text = "n/a"
        if snapshot.header_stamp_sec is not None:
            frame_age_sec = max(0.0, now_wall_time - snapshot.header_stamp_sec)
            frame_age_text = f"{frame_age_sec * 1000.0:.1f} ms"

        self._stats_label.setText(
            f"Capture Topic: {self._capture_topic_text()}\n"
            f"Capture FPS Target: {self._capture_fps_target_text()}\n"
            f"Preview Topic: {snapshot.preview_topic}\n"
            f"Preview RX FPS: {snapshot.rx_fps:.1f}\n"
            f"UI Display FPS: {self._display_fps:.1f}\n"
            f"Preview Publish Cap: {self._preview_cap_text()}\n"
            f"Frame Age: {frame_age_text}\n"
            f"Resolution: {snapshot.image_msg.width}x{snapshot.image_msg.height}"
        )

    def _capture_topic_text(self) -> str:
        capture_topic = self._stream_config.capture_topic
        if capture_topic is None or not capture_topic.strip():
            return "n/a"
        return capture_topic

    def _capture_fps_target_text(self) -> str:
        capture_fps_target = self._stream_config.capture_fps_target
        if capture_fps_target is None:
            return "auto"
        return f"{capture_fps_target:.1f} Hz"

    def _preview_cap_text(self) -> str:
        preview_fps_limit = self._stream_config.preview_fps_limit
        if preview_fps_limit is None:
            return "auto"
        return f"{preview_fps_limit:.1f} Hz"


class CameraPanel(QWidget):
    def __init__(
        self,
        model: CameraPanelModel,
        streams: tuple[CameraStreamConfig, ...],
        columns: int,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self._tiles: dict[str, CameraTileWidget] = {}
        self._empty_label: QLabel | None = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title_label = QLabel(model.title, self)
        title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        layout.addWidget(title_label)

        if not streams:
            self._empty_label = QLabel("No camera preview streams configured.", self)
            self._empty_label.setAlignment(Qt.AlignCenter)
            self._empty_label.setMinimumHeight(200)
            self._empty_label.setStyleSheet("background: #101418; color: #d0d7de;")
            layout.addWidget(self._empty_label, 1)
            return

        grid = QGridLayout()
        grid.setSpacing(8)
        for index, stream in enumerate(streams):
            row = index // columns
            column = index % columns
            tile = CameraTileWidget(stream, self)
            grid.addWidget(tile, row, column)
            self._tiles[stream.camera_id] = tile
        layout.addLayout(grid, 1)

    def refresh(self, snapshots: list[CameraStreamSnapshot], now_wall_time: float) -> None:
        for snapshot in snapshots:
            tile = self._tiles.get(snapshot.camera_id)
            if tile is not None:
                tile.refresh(snapshot, now_wall_time)
