from __future__ import annotations

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

from ..ros_client import PreviewImageSnapshot
from ..view_model import PreviewStreamConfig


def _pixmap_from_image_message(image_msg) -> QPixmap | None:
    if image_msg is None:
        return None
    payload = bytes(image_msg.data)
    if image_msg.encoding == "rgb8":
        image = QImage(payload, image_msg.width, image_msg.height, image_msg.step, QImage.Format_RGB888)
    elif image_msg.encoding == "bgr8":
        image = QImage(payload, image_msg.width, image_msg.height, image_msg.step, QImage.Format_BGR888)
    elif image_msg.encoding == "mono8":
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


class PreviewTileWidget(QFrame):
    def __init__(self, stream_config: PreviewStreamConfig, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._stream_config = stream_config
        self._last_generation = -1
        self._current_pixmap: QPixmap | None = None
        self._display_window_start = time.monotonic()
        self._display_window_frames = 0
        self._display_fps = 0.0

        self.setFrameShape(QFrame.StyledPanel)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        self._title_label = QLabel(stream_config.title, self)
        self._title_label.setStyleSheet("font-weight: 600;")
        layout.addWidget(self._title_label)

        self._image_label = QLabel("Waiting for processed preview...", self)
        self._image_label.setAlignment(Qt.AlignCenter)
        self._image_label.setMinimumSize(260, 220)
        self._image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._image_label.setStyleSheet("background: #101418; color: #d0d7de;")
        layout.addWidget(self._image_label, 1)

        self._stats_label = QLabel(self)
        self._stats_label.setWordWrap(True)
        layout.addWidget(self._stats_label)
        self._set_waiting_text()

    def refresh(self, snapshot: PreviewImageSnapshot, now_wall_time: float) -> None:
        self._roll_display_window()
        if snapshot.generation != self._last_generation and snapshot.image_msg is not None:
            pixmap = _pixmap_from_image_message(snapshot.image_msg)
            if pixmap is not None:
                self._current_pixmap = pixmap
                self._last_generation = snapshot.generation
                self._display_window_frames += 1
                self._apply_pixmap()
            else:
                self._current_pixmap = None
                self._image_label.setPixmap(QPixmap())
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
        self._image_label.setPixmap(QPixmap())
        self._image_label.setText("Waiting for processed preview...")
        self._stats_label.setText(
            f"Preview Topic: {self._stream_config.preview_topic}\n"
            "Preview RX FPS: 0.0\n"
            "UI Display FPS: 0.0\n"
            "Frame Age: n/a\n"
            "Resolution: waiting"
        )

    def _update_stats(self, snapshot: PreviewImageSnapshot, now_wall_time: float) -> None:
        if snapshot.image_msg is None:
            self._set_waiting_text()
            return
        frame_age_text = "n/a"
        if snapshot.header_stamp_sec is not None:
            frame_age_sec = max(0.0, now_wall_time - snapshot.header_stamp_sec)
            frame_age_text = f"{frame_age_sec * 1000.0:.1f} ms"
        self._stats_label.setText(
            f"Preview Topic: {snapshot.config.preview_topic}\n"
            f"Preview RX FPS: {snapshot.rx_fps:.1f}\n"
            f"UI Display FPS: {self._display_fps:.1f}\n"
            f"Frame Age: {frame_age_text}\n"
            f"Resolution: {snapshot.image_msg.width}x{snapshot.image_msg.height}"
        )


class PreviewPanel(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._tiles: dict[str, PreviewTileWidget] = {}
        self._empty_label: QLabel | None = None

        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(0, 0, 0, 0)
        self._layout.setSpacing(8)

        title_label = QLabel("Processed Policy Images", self)
        title_label.setStyleSheet("font-size: 16px; font-weight: 600;")
        self._layout.addWidget(title_label)

        self._grid = QGridLayout()
        self._grid.setSpacing(8)
        self._layout.addLayout(self._grid, 1)

    def set_streams(self, streams: list[PreviewStreamConfig], columns: int = 2) -> None:
        self._clear_grid()
        self._tiles = {}
        if not streams:
            self._empty_label = QLabel("No processed image previews configured.", self)
            self._empty_label.setAlignment(Qt.AlignCenter)
            self._empty_label.setMinimumHeight(180)
            self._empty_label.setStyleSheet("background: #101418; color: #d0d7de;")
            self._grid.addWidget(self._empty_label, 0, 0)
            return

        self._empty_label = None
        for index, stream in enumerate(streams):
            tile = PreviewTileWidget(stream, self)
            self._grid.addWidget(tile, index // columns, index % columns)
            self._tiles[stream.preview_topic] = tile

    def refresh(self, snapshots: list[PreviewImageSnapshot], now_wall_time: float) -> None:
        for snapshot in snapshots:
            tile = self._tiles.get(snapshot.config.preview_topic)
            if tile is not None:
                tile.refresh(snapshot, now_wall_time)

    def _clear_grid(self) -> None:
        while self._grid.count():
            item = self._grid.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()
