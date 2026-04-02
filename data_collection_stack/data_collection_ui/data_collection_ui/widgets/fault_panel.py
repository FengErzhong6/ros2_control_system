from __future__ import annotations

from dataclasses import dataclass

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QVBoxLayout,
    QWidget,
)

from data_collection_interfaces.msg import FaultEvent

from ..ros_client import FaultSnapshot


@dataclass
class FaultPanelModel:
    title: str = "Faults"


_SEVERITY_LABELS = {
    FaultEvent.SEVERITY_DEGRADED: "DEGRADED",
    FaultEvent.SEVERITY_RECOVERABLE: "RECOVERABLE",
    FaultEvent.SEVERITY_BLOCKING: "BLOCKING",
    FaultEvent.SEVERITY_FATAL: "FATAL",
}

_SEVERITY_COLORS = {
    FaultEvent.SEVERITY_DEGRADED: QColor("#9a6700"),
    FaultEvent.SEVERITY_RECOVERABLE: QColor("#bc4c00"),
    FaultEvent.SEVERITY_BLOCKING: QColor("#cf222e"),
    FaultEvent.SEVERITY_FATAL: QColor("#a40e26"),
}


class FaultPanel(QWidget):
    def __init__(self, model: FaultPanelModel, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title_label = QLabel(model.title, self)
        title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        layout.addWidget(title_label)

        self._list = QListWidget(self)
        self._list.setAlternatingRowColors(True)
        self._list.setSelectionMode(QAbstractItemView.NoSelection)
        self._list.setFocusPolicy(Qt.NoFocus)
        layout.addWidget(self._list, 1)

    def refresh(self, faults: tuple[FaultSnapshot, ...], now_wall_time: float) -> None:
        self._list.clear()
        if not faults:
            item = QListWidgetItem("No active faults.")
            item.setForeground(QColor("#2da44e"))
            self._list.addItem(item)
            return

        for fault in faults:
            severity = _SEVERITY_LABELS.get(fault.severity, str(fault.severity))
            age_text = "n/a"
            if fault.stamp_sec is not None:
                age_sec = max(0.0, now_wall_time - fault.stamp_sec)
                age_text = f"{age_sec:.1f}s ago"

            device_suffix = f" [{fault.device_id}]" if fault.device_id else ""
            text = f"{severity}{device_suffix}: {fault.summary} ({age_text})"
            if fault.detail and fault.detail != fault.summary:
                text += f"\n{fault.detail}"

            item = QListWidgetItem(text)
            item.setForeground(_SEVERITY_COLORS.get(fault.severity, QColor("#cf222e")))
            self._list.addItem(item)
