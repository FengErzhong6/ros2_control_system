import os
from dataclasses import dataclass

from PyQt5.QtWidgets import (
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QStyle,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from ..ros_client import SystemStateSnapshot


@dataclass
class SessionPanelModel:
    title: str = "Session"
    default_save_directory: str = ""


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

        self._session_name_input = QLineEdit(self)
        self._session_name_input.setPlaceholderText("pick_and_place_pi0")
        form.addRow("Session Name", self._session_name_input)

        save_directory_widget = QWidget(self)
        save_directory_layout = QHBoxLayout(save_directory_widget)
        save_directory_layout.setContentsMargins(0, 0, 0, 0)
        save_directory_layout.setSpacing(6)
        self._save_directory_input = QLineEdit(save_directory_widget)
        self._save_directory_input.setText(model.default_save_directory)
        self._save_directory_input.setPlaceholderText("~/.ros/data_collection")
        save_directory_layout.addWidget(self._save_directory_input, 1)
        self._browse_save_directory_button = QToolButton(save_directory_widget)
        self._browse_save_directory_button.setIcon(
            self.style().standardIcon(QStyle.SP_DirOpenIcon)
        )
        self._browse_save_directory_button.setToolTip("Choose save directory")
        self._browse_save_directory_button.clicked.connect(self._choose_save_directory)
        save_directory_layout.addWidget(self._browse_save_directory_button, 0)
        form.addRow("Save Directory", save_directory_widget)

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

    def session_name(self) -> str:
        return self._session_name_input.text().strip()

    def save_directory(self) -> str:
        return self._save_directory_input.text().strip()

    def _choose_save_directory(self) -> None:
        current_directory = os.path.expanduser(self.save_directory() or "~")
        selected_directory = QFileDialog.getExistingDirectory(
            self,
            "Select Save Directory",
            current_directory,
            QFileDialog.ShowDirsOnly,
        )
        if selected_directory:
            self._save_directory_input.setText(selected_directory)

    def refresh(self, system_state: SystemStateSnapshot, fallback_recipe_id: str) -> None:
        self._recipe_value.setText(system_state.recipe_id or fallback_recipe_id or "-")
        self._session_value.setText(system_state.active_session_id or "-")
        self._state_value.setText(system_state.system_state or "IDLE")
