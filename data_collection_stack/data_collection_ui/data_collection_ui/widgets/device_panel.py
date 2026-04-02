from __future__ import annotations

from dataclasses import dataclass

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QHeaderView,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from data_collection_interfaces.msg import DeviceState

from ..ros_client import DeviceSnapshot


@dataclass
class DevicePanelModel:
    title: str = "Devices"


_LIFECYCLE_LABELS = {
    DeviceState.LIFECYCLE_UNKNOWN: "UNKNOWN",
    DeviceState.LIFECYCLE_IDLE: "IDLE",
    DeviceState.LIFECYCLE_STARTING: "STARTING",
    DeviceState.LIFECYCLE_READY: "READY",
    DeviceState.LIFECYCLE_ACTIVE: "ACTIVE",
    DeviceState.LIFECYCLE_STOPPING: "STOPPING",
    DeviceState.LIFECYCLE_FAULT: "FAULT",
}

_HEALTH_LABELS = {
    DeviceState.HEALTH_UNKNOWN: "UNKNOWN",
    DeviceState.HEALTH_OK: "OK",
    DeviceState.HEALTH_DEGRADED: "DEGRADED",
    DeviceState.HEALTH_FAILED: "FAILED",
}

_HEALTH_COLORS = {
    DeviceState.HEALTH_UNKNOWN: QColor("#6e7681"),
    DeviceState.HEALTH_OK: QColor("#2da44e"),
    DeviceState.HEALTH_DEGRADED: QColor("#d29922"),
    DeviceState.HEALTH_FAILED: QColor("#cf222e"),
}


class DevicePanel(QWidget):
    def __init__(self, model: DevicePanelModel, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title_label = QLabel(model.title, self)
        title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        layout.addWidget(title_label)

        self._table = QTableWidget(0, 6, self)
        self._table.setHorizontalHeaderLabels(
            ["Device", "Class", "Lifecycle", "Health", "Ready", "Summary"]
        )
        self._table.verticalHeader().setVisible(False)
        self._table.setAlternatingRowColors(True)
        self._table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._table.setSelectionMode(QAbstractItemView.NoSelection)
        self._table.setFocusPolicy(Qt.NoFocus)
        self._table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self._table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        layout.addWidget(self._table, 1)

    def refresh(self, devices: tuple[DeviceSnapshot, ...], now_wall_time: float) -> None:
        self._table.setRowCount(len(devices))
        for row, device in enumerate(devices):
            self._set_item(row, 0, f"{device.device_id}{' *' if device.is_required else ''}")
            self._set_item(row, 1, device.device_class)
            self._set_item(
                row,
                2,
                _LIFECYCLE_LABELS.get(device.lifecycle_state, str(device.lifecycle_state)),
            )
            health_item = self._set_item(
                row,
                3,
                _HEALTH_LABELS.get(device.health_state, str(device.health_state)),
            )
            health_item.setForeground(_HEALTH_COLORS.get(device.health_state, QColor("#6e7681")))

            ready_text = "-"
            if device.last_ready_stamp_sec is not None:
                ready_age_sec = max(0.0, now_wall_time - device.last_ready_stamp_sec)
                ready_text = f"{ready_age_sec:.1f}s ago"
            self._set_item(row, 4, ready_text)
            self._set_item(row, 5, device.summary)

    def _set_item(self, row: int, column: int, text: str) -> QTableWidgetItem:
        item = self._table.item(row, column)
        if item is None:
            item = QTableWidgetItem()
            self._table.setItem(row, column, item)
        item.setText(text)
        return item
