from dataclasses import dataclass

from PyQt5.QtWidgets import (
    QGridLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


@dataclass
class CommandPanelModel:
    title: str = "Commands"


_BUTTON_DEFINITIONS = [
    ("StartSystem", "Connect"),
    ("ShutdownSystem", "Disconnect"),
    ("StartSession", "Start Collection"),
    ("StopSession", "Stop Collection"),
    ("GoHome", "Go Home"),
    ("AcknowledgeFault", "Acknowledge Fault"),
]


class CommandPanel(QWidget):
    def __init__(self, model: CommandPanelModel, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._buttons: dict[str, QPushButton] = {}
        self._shortcut_labels: dict[str, str] = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)

        title_label = QLabel(model.title, self)
        title_label.setStyleSheet("font-size: 18px; font-weight: 600;")
        layout.addWidget(title_label)

        self._status_label = QLabel("No active command.", self)
        self._status_label.setWordWrap(True)
        layout.addWidget(self._status_label)

        button_grid = QGridLayout()
        button_grid.setSpacing(8)
        for index, (command_name, label) in enumerate(_BUTTON_DEFINITIONS):
            button = QPushButton(label, self)
            button.setMinimumHeight(52)
            button_grid.addWidget(button, index // 2, index % 2)
            self._buttons[command_name] = button
        layout.addLayout(button_grid)

    def set_command_handler(self, command_name: str, handler) -> None:
        button = self._buttons.get(command_name)
        if button is None:
            return
        button.clicked.connect(handler)

    def set_shortcut_labels(self, shortcut_labels: dict[str, str]) -> None:
        self._shortcut_labels = dict(shortcut_labels)

    def refresh(
        self,
        *,
        allowed_commands: tuple[str, ...],
        pending_commands: tuple[str, ...],
    ) -> None:
        allowed_command_set = set(allowed_commands)
        pending_command_set = set(pending_commands)
        any_pending = bool(pending_command_set)

        for command_name, label in _BUTTON_DEFINITIONS:
            button = self._buttons[command_name]
            is_pending = command_name in pending_command_set
            primary_label = f"{label}..." if is_pending else label
            shortcut_label = self._shortcut_labels.get(command_name, "")
            if shortcut_label:
                button.setText(f"{primary_label}\n[{shortcut_label}]")
            else:
                button.setText(primary_label)
            button.setEnabled((command_name in allowed_command_set) and not any_pending)

        if pending_commands:
            self._status_label.setText("In Progress: " + ", ".join(pending_commands))
        elif allowed_commands:
            self._status_label.setText("Available: " + ", ".join(allowed_commands))
        else:
            self._status_label.setText("No command is currently available.")
