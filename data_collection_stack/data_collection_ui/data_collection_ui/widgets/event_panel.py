from dataclasses import dataclass
from datetime import datetime

from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import QLabel, QListWidget, QListWidgetItem, QVBoxLayout, QWidget

from ..ros_client import UiEventEntry


@dataclass
class EventPanelModel:
    title: str = "Events"


_LEVEL_COLORS = {
    "INFO": QColor("#1f6feb"),
    "STATE": QColor("#2da44e"),
    "WARN": QColor("#9a6700"),
    "ERROR": QColor("#cf222e"),
}


class EventPanel(QWidget):
    def __init__(self, model: EventPanelModel, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title_label = QLabel(model.title, self)
        title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        layout.addWidget(title_label)

        self._list = QListWidget(self)
        layout.addWidget(self._list, 1)

    def refresh(self, events: tuple[UiEventEntry, ...]) -> None:
        self._list.clear()
        if not events:
            self._list.addItem("No events yet.")
            return

        for event in events[-80:]:
            timestamp = datetime.fromtimestamp(event.stamp_sec).strftime("%H:%M:%S")
            item = QListWidgetItem(f"[{timestamp}] {event.level}: {event.message}")
            item.setForeground(_LEVEL_COLORS.get(event.level, QColor("#6e7681")))
            self._list.addItem(item)
        self._list.scrollToBottom()
