from __future__ import annotations

from dataclasses import dataclass
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QKeySequence
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QShortcut,
    QVBoxLayout,
    QWidget,
)

from ..ros_client import RosClient
from ..view_model import UiViewModel
from .camera_panel import CameraPanel, CameraPanelModel
from .command_panel import CommandPanel, CommandPanelModel
from .device_panel import DevicePanel, DevicePanelModel
from .event_panel import EventPanel, EventPanelModel
from .fault_panel import FaultPanel, FaultPanelModel
from .session_panel import SessionPanel, SessionPanelModel


@dataclass
class MainWindowModel:
    title: str = "Data Collection Console"


_STATE_STYLE = {
    "IDLE": ("#3d405b", "#f2f3f7"),
    "PREFLIGHT": ("#9a6700", "#fff1c7"),
    "STARTING": ("#1f6feb", "#dff0ff"),
    "READY": ("#2da44e", "#ddfbe6"),
    "ARMED": ("#bc4c00", "#ffe2bb"),
    "RECORDING": ("#cf222e", "#ffd8d3"),
    "PAUSED": ("#8250df", "#efe3ff"),
    "STOPPING": ("#6e7781", "#eaeef2"),
    "FAULT": ("#a40e26", "#ffd7d5"),
}


class MainWindow(QMainWindow):
    def __init__(self, view_model: UiViewModel, ros_client: RosClient) -> None:
        super().__init__()
        self._view_model = view_model
        self._ros_client = ros_client

        self.setWindowTitle(view_model.title)
        self.resize(1920, 1080)

        central_widget = QWidget(self)
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

        self._state_badge = QLabel(self)
        self._state_badge.setStyleSheet(
            "font-size: 28px; font-weight: 700; padding: 8px 14px; border-radius: 10px;"
        )
        layout.addWidget(self._state_badge)

        self._state_label = QLabel(self)
        self._state_label.setStyleSheet("font-size: 16px; font-weight: 600;")
        layout.addWidget(self._state_label)

        self._summary_label = QLabel(self)
        self._summary_label.setWordWrap(True)
        layout.addWidget(self._summary_label)

        self._commands_label = QLabel(self)
        self._commands_label.setWordWrap(True)
        layout.addWidget(self._commands_label)

        content_layout = QHBoxLayout()
        content_layout.setSpacing(12)

        self._camera_panel = CameraPanel(
            model=CameraPanelModel(),
            streams=view_model.config.camera_streams,
            columns=view_model.config.camera_grid_columns,
            parent=self,
        )
        content_layout.addWidget(self._camera_panel, 3)

        sidebar_layout = QVBoxLayout()
        sidebar_layout.setSpacing(12)

        self._session_panel = SessionPanel(model=SessionPanelModel(), parent=self)
        sidebar_layout.addWidget(self._session_panel, 0)

        self._command_panel = CommandPanel(model=CommandPanelModel(), parent=self)
        sidebar_layout.addWidget(self._command_panel, 0)

        self._device_panel = DevicePanel(model=DevicePanelModel(), parent=self)
        sidebar_layout.addWidget(self._device_panel, 2)

        self._fault_panel = FaultPanel(model=FaultPanelModel(), parent=self)
        sidebar_layout.addWidget(self._fault_panel, 1)

        self._event_panel = EventPanel(model=EventPanelModel(), parent=self)
        sidebar_layout.addWidget(self._event_panel, 1)

        content_layout.addLayout(sidebar_layout, 2)
        layout.addLayout(content_layout, 1)

        self.setCentralWidget(central_widget)
        self._bind_commands()
        self._install_hotkeys()

        self._refresh_timer = QTimer(self)
        refresh_interval_ms = max(1, int(1000 / max(1, view_model.config.refresh_hz)))
        self._refresh_timer.setInterval(refresh_interval_ms)
        self._refresh_timer.timeout.connect(self._refresh)
        self._refresh_timer.start()
        self._refresh()

    def _bind_commands(self) -> None:
        self._command_panel.set_command_handler("StartSystem", self._on_connect)
        self._command_panel.set_command_handler("ShutdownSystem", self._on_disconnect)
        self._command_panel.set_command_handler("StartSession", self._on_start_collection)
        self._command_panel.set_command_handler("StopSession", self._on_stop_collection)
        self._command_panel.set_command_handler("GoHome", self._on_go_home)
        self._command_panel.set_command_handler(
            "AcknowledgeFault",
            self._on_acknowledge_fault,
        )

    def _install_hotkeys(self) -> None:
        shortcut_labels = {
            binding.command: binding.key
            for binding in self._view_model.hotkeys
        }
        self._command_panel.set_shortcut_labels(shortcut_labels)
        if not self._view_model.config.enable_hotkeys:
            return

        command_handlers = {
            "StartSystem": self._on_connect,
            "ShutdownSystem": self._on_disconnect,
            "StartSession": self._on_start_collection,
            "StopSession": self._on_stop_collection,
            "GoHome": self._on_go_home,
            "AcknowledgeFault": self._on_acknowledge_fault,
        }

        self._shortcuts: list[QShortcut] = []
        for binding in self._view_model.hotkeys:
            handler = command_handlers.get(binding.command)
            if handler is None:
                continue
            shortcut = QShortcut(QKeySequence(binding.key), self)
            shortcut.activated.connect(handler)
            self._shortcuts.append(shortcut)

    def _on_connect(self) -> None:
        self._ros_client.connect_system(
            operator_id=self._session_panel.operator_id(),
            site_name=self._session_panel.site_name(),
        )

    def _on_disconnect(self) -> None:
        self._ros_client.disconnect_system(force=False)

    def _on_start_collection(self) -> None:
        self._ros_client.start_collection(
            operator_id=self._session_panel.operator_id(),
            session_tag=self._session_panel.session_tag(),
        )

    def _on_stop_collection(self) -> None:
        self._ros_client.stop_collection(reason="ui_stop")

    def _on_go_home(self) -> None:
        self._ros_client.go_home()

    def _on_acknowledge_fault(self) -> None:
        self._ros_client.acknowledge_fault()

    def _refresh(self) -> None:
        system_state = self._ros_client.system_state_snapshot()
        state_foreground, state_background = _STATE_STYLE.get(
            system_state.system_state,
            ("#3d405b", "#f2f3f7"),
        )
        self._state_badge.setStyleSheet(
            "font-size: 28px; "
            "font-weight: 700; "
            "padding: 8px 14px; "
            "border-radius: 10px; "
            f"color: {state_foreground}; "
            f"background: {state_background};"
        )
        self._state_badge.setText(system_state.system_state or "IDLE")
        self._state_label.setText(
            " | ".join(
                [
                    f"Recipe: {system_state.recipe_id or self._view_model.config.recipe_id}",
                    f"Session: {system_state.active_session_id or '-'}",
                ]
            )
        )
        self._summary_label.setText(system_state.summary or "Waiting for system state...")
        allowed_commands = ", ".join(system_state.allowed_commands) if system_state.allowed_commands else "-"
        self._commands_label.setText(f"Allowed Commands: {allowed_commands}")
        self._session_panel.refresh(system_state, self._view_model.config.recipe_id)
        self._command_panel.refresh(
            allowed_commands=system_state.allowed_commands,
            pending_commands=self._ros_client.pending_commands(),
        )
        now_wall_time = time.time()
        self._camera_panel.refresh(
            self._ros_client.camera_stream_snapshots(),
            now_wall_time=now_wall_time,
        )
        self._device_panel.refresh(system_state.devices, now_wall_time=now_wall_time)
        self._fault_panel.refresh(system_state.active_faults, now_wall_time=now_wall_time)
        self._event_panel.refresh(self._ros_client.event_entries())
