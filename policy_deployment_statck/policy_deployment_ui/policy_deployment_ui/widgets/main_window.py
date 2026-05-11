from __future__ import annotations

import time

from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QPushButton,
    QPlainTextEdit,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from ..ros_client import RosClient
from ..view_model import UiViewModel
from .preview_panel import PreviewPanel


_STATE_STYLE = {
    "IDLE": ("#3d405b", "#f2f3f7"),
    "STARTING": ("#1f6feb", "#dff0ff"),
    "READY": ("#2da44e", "#ddfbe6"),
    "EXECUTING": ("#bc4c00", "#ffe2bb"),
    "HOMING": ("#8250df", "#efe3ff"),
    "STOPPING": ("#6e7781", "#eaeef2"),
    "FAULT": ("#a40e26", "#ffd7d5"),
}


class MainWindow(QMainWindow):
    def __init__(self, *, view_model: UiViewModel, ros_client: RosClient) -> None:
        super().__init__()
        self._view_model = view_model
        self._ros_client = ros_client

        self.setWindowTitle(view_model.title)
        self.resize(1760, 1080)
        central = QWidget(self)
        self.setCentralWidget(central)

        root = QVBoxLayout(central)

        self._state_badge = QLabel("IDLE", self)
        self._state_badge.setStyleSheet(
            "font-size: 28px; font-weight: 700; padding: 8px 14px; border-radius: 10px;"
        )
        root.addWidget(self._state_badge)

        self._state_label = QLabel(self)
        self._state_label.setWordWrap(True)
        self._state_label.setStyleSheet("font-size: 16px; font-weight: 600;")
        root.addWidget(self._state_label)

        self._summary_label = QLabel(self)
        self._summary_label.setWordWrap(True)
        root.addWidget(self._summary_label)

        self._commands_label = QLabel(self)
        self._commands_label.setWordWrap(True)
        root.addWidget(self._commands_label)

        self._motion_status_label = QLabel(self)
        self._motion_status_label.setWordWrap(True)
        self._motion_status_label.setStyleSheet(
            "font-size: 14px; padding: 8px 12px; border-radius: 8px;"
        )
        root.addWidget(self._motion_status_label)

        form = QFormLayout()

        self.profile_combo = QComboBox()
        self.profile_combo.setEditable(True)
        self.profile_combo.setCurrentText(view_model.profile_id)
        form.addRow("Policy", self.profile_combo)

        self.prompt_edit = QLineEdit()
        self.prompt_edit.setText(view_model.prompt)
        form.addRow("Prompt", self.prompt_edit)

        self.max_steps_spin = QSpinBox()
        self.max_steps_spin.setRange(0, 1_000_000)
        self.max_steps_spin.setValue(view_model.max_steps)
        form.addRow("Max steps", self.max_steps_spin)

        self.open_loop_spin = QSpinBox()
        self.open_loop_spin.setRange(0, 1_000_000)
        self.open_loop_spin.setValue(view_model.open_loop_horizon)
        form.addRow("Open-loop", self.open_loop_spin)

        root.addLayout(form)

        button_row = QHBoxLayout()
        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")
        self.execute_button = QPushButton("Execute Policy")
        self.stop_button = QPushButton("Stop Policy")
        self.home_button = QPushButton("Go Home")
        button_row.addWidget(self.connect_button)
        button_row.addWidget(self.disconnect_button)
        button_row.addWidget(self.execute_button)
        button_row.addWidget(self.stop_button)
        button_row.addWidget(self.home_button)
        root.addLayout(button_row)

        self._preview_panel = PreviewPanel(self)
        root.addWidget(self._preview_panel, 1)

        grid = QGridLayout()
        self.system_state_value = QLabel("-")
        self.rollout_state_value = QLabel("-")
        self.recipe_value = QLabel("-")
        self.profile_value = QLabel("-")
        self.prompt_value = QLabel("-")
        self.chunk_value = QLabel("-")
        self.step_value = QLabel("-")
        self.infer_value = QLabel("-")
        self.summary_value = QLabel("-")
        self._known_profile_ids: list[str] = []
        self._preview_profile_id = ""

        grid.addWidget(QLabel("System"), 0, 0)
        grid.addWidget(self.system_state_value, 0, 1)
        grid.addWidget(QLabel("Rollout"), 1, 0)
        grid.addWidget(self.rollout_state_value, 1, 1)
        grid.addWidget(QLabel("Recipe"), 2, 0)
        grid.addWidget(self.recipe_value, 2, 1)
        grid.addWidget(QLabel("Active profile"), 3, 0)
        grid.addWidget(self.profile_value, 3, 1)
        grid.addWidget(QLabel("Prompt"), 4, 0)
        grid.addWidget(self.prompt_value, 4, 1)
        grid.addWidget(QLabel("Chunk"), 5, 0)
        grid.addWidget(self.chunk_value, 5, 1)
        grid.addWidget(QLabel("Step"), 6, 0)
        grid.addWidget(self.step_value, 6, 1)
        grid.addWidget(QLabel("Infer ms"), 7, 0)
        grid.addWidget(self.infer_value, 7, 1)
        grid.addWidget(QLabel("Summary"), 8, 0)
        grid.addWidget(self.summary_value, 8, 1)
        root.addLayout(grid)

        self.events = QPlainTextEdit()
        self.events.setReadOnly(True)
        root.addWidget(self.events)

        self.connect_button.clicked.connect(self._on_connect)
        self.disconnect_button.clicked.connect(self._on_disconnect)
        self.execute_button.clicked.connect(self._on_execute)
        self.stop_button.clicked.connect(self._on_stop)
        self.home_button.clicked.connect(self._on_home)

        self._timer = QTimer(self)
        self._timer.setInterval(200)
        self._timer.timeout.connect(self._refresh)
        self._timer.start()

        self._refresh_profiles()
        self._refresh()

    def _refresh_profiles(self) -> None:
        profiles = self._ros_client.profiles
        profile_ids = [profile.profile_id for profile in profiles]
        if profile_ids == self._known_profile_ids:
            return

        current = self.profile_combo.currentText()
        self.profile_combo.blockSignals(True)
        self.profile_combo.clear()
        for profile_id in profile_ids:
            self.profile_combo.addItem(profile_id)
        if current:
            idx = self.profile_combo.findText(current)
            if idx >= 0:
                self.profile_combo.setCurrentIndex(idx)
            else:
                self.profile_combo.setEditText(current)
        elif profile_ids:
            self.profile_combo.setCurrentIndex(0)
        self.profile_combo.blockSignals(False)
        self._known_profile_ids = profile_ids

    def _refresh(self) -> None:
        self._refresh_profiles()
        state = self._ros_client.state
        motion_status = self._ros_client.motion_status_snapshot()
        selected_profile_id = (
            self._current_profile_id()
            or state.active_profile_id
            or self._view_model.profile_id
        )
        self._refresh_status_header(state, motion_status, selected_profile_id)
        self._refresh_buttons(state)
        self._refresh_preview_panel(selected_profile_id)
        self.system_state_value.setText(state.system_state)
        self.rollout_state_value.setText(state.rollout_state)
        self.recipe_value.setText(state.recipe_id or self._view_model.recipe_id)
        self.profile_value.setText(state.active_profile_id or selected_profile_id)
        self.prompt_value.setText(state.active_prompt or self.prompt_edit.text().strip())
        self.chunk_value.setText(str(state.chunk_index))
        self.step_value.setText(str(state.step_index))
        self.infer_value.setText(f"{state.last_infer_ms:.1f}")
        self.summary_value.setText(state.summary)
        self.events.setPlainText(
            "\n".join(f"[{entry.level}] {entry.message}" for entry in self._ros_client.events[-40:])
        )

    def _refresh_preview_panel(self, profile_id: str) -> None:
        profile_id = profile_id.strip()
        if profile_id != self._preview_profile_id:
            self._preview_panel.set_streams(self._ros_client.preview_stream_configs(profile_id))
            self._preview_profile_id = profile_id
        self._preview_panel.refresh(
            self._ros_client.preview_stream_snapshots(profile_id),
            now_wall_time=time.time(),
        )

    def _refresh_status_header(self, state, motion_status, selected_profile_id: str) -> None:
        state_name = state.system_state or "IDLE"
        state_foreground, state_background = _STATE_STYLE.get(
            state_name,
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
        self._state_badge.setText(state_name)
        recipe_id = state.recipe_id or self._view_model.recipe_id or "-"
        profile_id = state.active_profile_id or selected_profile_id or self._view_model.profile_id or "-"
        prompt = state.active_prompt or self.prompt_edit.text().strip() or "-"
        self._state_label.setText(
            " | ".join(
                [
                    f"Recipe: {recipe_id}",
                    f"Profile: {profile_id}",
                    f"Prompt: {prompt}",
                    f"Rollout: {state.rollout_state or '-'}",
                    f"Devices: {state.device_count}",
                    f"Faults: {state.fault_count}",
                ]
            )
        )
        self._summary_label.setText(state.summary or "Waiting for system state...")
        allowed_commands = ", ".join(state.allowed_commands) if state.allowed_commands else "-"
        self._commands_label.setText(f"Allowed Commands: {allowed_commands}")

        if motion_status.available and motion_status.controller_interlock_ok:
            motion_fg = "#0f5132"
            motion_bg = "#d1e7dd"
        elif motion_status.available:
            motion_fg = "#842029"
            motion_bg = "#f8d7da"
        else:
            motion_fg = "#664d03"
            motion_bg = "#fff3cd"
        self._motion_status_label.setStyleSheet(
            "font-size: 14px; "
            "padding: 8px 12px; "
            "border-radius: 8px; "
            f"color: {motion_fg}; "
            f"background: {motion_bg};"
        )
        if motion_status.available:
            self._motion_status_label.setText(
                "Marvin Motion: "
                f"mode={motion_status.mode} | "
                f"teleop={motion_status.teleop_state} | "
                f"busy={'ON' if motion_status.motion_busy else 'OFF'} | "
                f"interlock={'OK' if motion_status.controller_interlock_ok else 'FAULT'} | "
                f"primary={motion_status.primary_controller_state} | "
                f"trajectory={motion_status.trajectory_controller_state} | "
                f"impedance={'OK' if motion_status.hardware_joint_impedance_ok else 'FAULT'}"
            )
        else:
            self._motion_status_label.setText(
                "Marvin Motion: unavailable"
                + (f" | {motion_status.message}" if motion_status.message else "")
            )

    def _refresh_buttons(self, state) -> None:
        allowed = set(state.allowed_commands)
        if not allowed and state.system_state == "IDLE":
            allowed.add("ConnectSystem")
        self.connect_button.setEnabled("ConnectSystem" in allowed)
        self.disconnect_button.setEnabled("DisconnectSystem" in allowed)
        self.execute_button.setEnabled("ExecutePolicy" in allowed)
        self.stop_button.setEnabled("StopPolicy" in allowed)
        self.home_button.setEnabled("GoHome" in allowed)

    def _current_profile_id(self) -> str:
        return self.profile_combo.currentText().strip()

    def _on_connect(self) -> None:
        self._ros_client.connect_system(
            profile_id=self._current_profile_id(),
            prompt=self.prompt_edit.text().strip(),
        )

    def _on_disconnect(self) -> None:
        self._ros_client.disconnect_system()

    def _on_execute(self) -> None:
        self._ros_client.execute_policy(
            profile_id=self._current_profile_id(),
            prompt=self.prompt_edit.text().strip(),
            max_steps=int(self.max_steps_spin.value()),
            open_loop_horizon=int(self.open_loop_spin.value()),
        )

    def _on_stop(self) -> None:
        self._ros_client.stop_policy("ui_stop_button")

    def _on_home(self) -> None:
        self._ros_client.go_home(True)
