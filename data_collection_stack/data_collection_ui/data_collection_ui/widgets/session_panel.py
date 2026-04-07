import os
from dataclasses import dataclass

from PyQt5.QtWidgets import (
    QFormLayout,
    QLabel,
    QLineEdit,
    QVBoxLayout,
    QWidget,
)

from ..ros_client import SystemStateSnapshot


@dataclass
class SessionPanelModel:
    title: str = "Session"


class SessionPanel(QWidget):
    def __init__(self, model: SessionPanelModel, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title_label = QLabel(model.title, self)
        title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        layout.addWidget(title_label)

        form = QFormLayout()
        form.setContentsMargins(0, 0, 0, 0)
        form.setSpacing(8)

        self._operator_input = QLineEdit(self)
        self._operator_input.setText(os.environ.get("USER", "mmlab"))
        form.addRow("Operator", self._operator_input)

        self._site_input = QLineEdit(self)
        self._site_input.setPlaceholderText("default")
        form.addRow("Site", self._site_input)

        self._session_tag_input = QLineEdit(self)
        self._session_tag_input.setPlaceholderText("optional")
        form.addRow("Session Tag", self._session_tag_input)

        self._recipe_value = QLabel("-", self)
        self._recipe_value.setWordWrap(True)
        form.addRow("Recipe", self._recipe_value)

        self._session_value = QLabel("-", self)
        form.addRow("Active Session", self._session_value)

        self._state_value = QLabel("IDLE", self)
        form.addRow("System State", self._state_value)

        layout.addLayout(form)

    def operator_id(self) -> str:
        return self._operator_input.text().strip()

    def site_name(self) -> str:
        return self._site_input.text().strip()

    def session_tag(self) -> str:
        return self._session_tag_input.text().strip()

    def refresh(self, system_state: SystemStateSnapshot, fallback_recipe_id: str) -> None:
        self._recipe_value.setText(system_state.recipe_id or fallback_recipe_id or "-")
        self._session_value.setText(system_state.active_session_id or "-")
        self._state_value.setText(system_state.system_state or "IDLE")
