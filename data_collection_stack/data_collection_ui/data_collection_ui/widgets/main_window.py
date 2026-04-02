from __future__ import annotations

from dataclasses import dataclass
import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QHBoxLayout, QLabel, QMainWindow, QVBoxLayout, QWidget

from ..ros_client import RosClient
from ..view_model import UiViewModel
from .camera_panel import CameraPanel, CameraPanelModel
from .device_panel import DevicePanel, DevicePanelModel
from .fault_panel import FaultPanel, FaultPanelModel


@dataclass
class MainWindowModel:
    title: str = "Data Collection Console"


class MainWindow(QMainWindow):
    def __init__(self, view_model: UiViewModel, ros_client: RosClient) -> None:
        super().__init__()
        self._view_model = view_model
        self._ros_client = ros_client

        self.setWindowTitle(view_model.title)
        self.resize(1600, 900)

        central_widget = QWidget(self)
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(10)

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

        self._device_panel = DevicePanel(model=DevicePanelModel(), parent=self)
        sidebar_layout.addWidget(self._device_panel, 2)

        self._fault_panel = FaultPanel(model=FaultPanelModel(), parent=self)
        sidebar_layout.addWidget(self._fault_panel, 1)

        content_layout.addLayout(sidebar_layout, 2)
        layout.addLayout(content_layout, 1)

        self.setCentralWidget(central_widget)

        self._refresh_timer = QTimer(self)
        refresh_interval_ms = max(1, int(1000 / max(1, view_model.config.refresh_hz)))
        self._refresh_timer.setInterval(refresh_interval_ms)
        self._refresh_timer.timeout.connect(self._refresh)
        self._refresh_timer.start()
        self._refresh()

    def _refresh(self) -> None:
        system_state = self._ros_client.system_state_snapshot()
        self._state_label.setText(
            " | ".join(
                [
                    f"System: {system_state.system_state}",
                    f"Recipe: {system_state.recipe_id or self._view_model.config.recipe_id}",
                    f"Session: {system_state.active_session_id or '-'}",
                ]
            )
        )
        self._summary_label.setText(system_state.summary or "Waiting for system state...")
        allowed_commands = ", ".join(system_state.allowed_commands) if system_state.allowed_commands else "-"
        self._commands_label.setText(f"Allowed Commands: {allowed_commands}")
        now_wall_time = time.time()
        self._camera_panel.refresh(
            self._ros_client.camera_stream_snapshots(),
            now_wall_time=now_wall_time,
        )
        self._device_panel.refresh(system_state.devices, now_wall_time=now_wall_time)
        self._fault_panel.refresh(system_state.active_faults, now_wall_time=now_wall_time)
