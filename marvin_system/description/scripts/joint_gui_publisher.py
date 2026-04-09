#!/usr/bin/env python3
"""
Joint state publisher GUI with sliders AND text-input spin boxes.

Drop-in replacement for joint_state_publisher_gui that:
  - Shows real radian values on every control
  - Lets the user drag a slider OR type a value directly
  - Handles mimic joints automatically
  - Supports configurable initial (home) positions via ROS parameters

Parameters (all optional, loaded from YAML):
  initial_positions.<joint_name>: float   – start-up position for that joint
  publish_rate: int                        – publish frequency in Hz (default 20)
"""

import math
from pathlib import Path
import signal
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

SLIDER_STEPS = 10000
LEFT_ARM_JOINT_NAMES = [f"Joint{i}_L" for i in range(1, 8)]
RIGHT_ARM_JOINT_NAMES = [f"Joint{i}_R" for i in range(1, 8)]


def _load_home_from_config(path: Path, pose_id: str) -> dict[str, float]:
    if not path.is_file():
        raise FileNotFoundError(f"Home config file not found: {path}")

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    home: dict[str, float] = {}

    motion_params = data.get("marvin_motion_server", {}).get("ros__parameters", {})
    named_pose = motion_params.get("named_poses", {}).get(pose_id, {})
    if isinstance(named_pose, dict):
        left = named_pose.get("left", [])
        right = named_pose.get("right", [])
        if isinstance(left, list) and isinstance(right, list):
            if len(left) == len(LEFT_ARM_JOINT_NAMES) and len(right) == len(RIGHT_ARM_JOINT_NAMES):
                for joint_name, value in zip(LEFT_ARM_JOINT_NAMES, left):
                    home[joint_name] = float(value)
                for joint_name, value in zip(RIGHT_ARM_JOINT_NAMES, right):
                    home[joint_name] = float(value)
                return home

    initial_positions = (
        data.get("joint_gui_publisher", {})
        .get("ros__parameters", {})
        .get("initial_positions", {})
    )
    if isinstance(initial_positions, dict):
        return {
            str(name): float(value)
            for name, value in initial_positions.items()
            if str(name).strip()
        }

    raise RuntimeError(f"Unsupported home config schema in {path}")


class _JointRow:
    """One row of widgets: label | slider | spin-box | range."""

    def __init__(self, name, lower, upper, initial, layout, on_change):
        self.name = name
        self.lower = lower
        self.upper = upper
        self._on_change = on_change
        self._guard = False
        self._is_normalized = name.startswith("gripper_")

        if self._is_normalized:
            lo_display = lower * 100.0
            hi_display = upper * 100.0
            suffix = " %"
        else:
            lo_display = math.degrees(lower)
            hi_display = math.degrees(upper)
            suffix = " °"

        row = QHBoxLayout()

        lbl = QLabel(name)
        lbl.setFixedWidth(110)
        row.addWidget(lbl)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.slider.valueChanged.connect(self._slider_moved)
        row.addWidget(self.slider, stretch=3)

        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(2)
        self.spin.setSingleStep(0.5)
        self.spin.setRange(lo_display, hi_display)
        self.spin.setSuffix(suffix)
        self.spin.setFixedWidth(150)
        self.spin.valueChanged.connect(self._spin_edited)
        row.addWidget(self.spin)

        rng = QLabel(f"[{lo_display:.1f}{suffix.strip()}, {hi_display:.1f}{suffix.strip()}]")
        rng.setFixedWidth(150)
        rng.setStyleSheet("color: grey;")
        row.addWidget(rng)

        layout.addLayout(row)
        self.set_value(initial)

    # ---- conversion helpers ----
    def _to_slider(self, v):
        if self.upper <= self.lower:
            return 0
        return round((v - self.lower) / (self.upper - self.lower) * SLIDER_STEPS)

    def _from_slider(self, s):
        return self.lower + s / SLIDER_STEPS * (self.upper - self.lower)

    # ---- callbacks (guarded against circular updates) ----
    def _slider_moved(self, s):
        if self._guard:
            return
        self._guard = True
        value = self._from_slider(s)
        if self._is_normalized:
            self.spin.setValue(value * 100.0)
        else:
            self.spin.setValue(math.degrees(value))
        self._on_change()
        self._guard = False

    def _spin_edited(self, displayed_value):
        if self._guard:
            return
        self._guard = True
        if self._is_normalized:
            value = displayed_value / 100.0
        else:
            value = math.radians(displayed_value)
        self.slider.setValue(self._to_slider(value))
        self._on_change()
        self._guard = False

    # ---- public API ----
    def value(self):
        """Return current command value in controller units."""
        if self._is_normalized:
            return self.spin.value() / 100.0
        return math.radians(self.spin.value())

    def set_value(self, v):
        """Set position from a value in controller units."""
        self._guard = True
        v = max(self.lower, min(self.upper, v))
        if self._is_normalized:
            self.spin.setValue(v * 100.0)
        else:
            self.spin.setValue(math.degrees(v))
        self.slider.setValue(self._to_slider(v))
        self._guard = False

    def set_enabled(self, enabled: bool):
        self.slider.setEnabled(enabled)
        self.spin.setEnabled(enabled)


class _MainWindow(QWidget):
    def __init__(self, node, free_joints, mimic_joints, initial_positions, home):
        super().__init__()
        self.node = node
        self.mimic_joints = mimic_joints
        self.initial_positions = initial_positions
        self.home = home
        self.go_home_service = str(node.declare_parameter("go_home_service", "").value).strip()
        self.go_home_timeout_sec = float(node.declare_parameter("go_home_timeout_sec", 20.0).value)
        self.local_home_on_service_failure = bool(
            node.declare_parameter("local_home_on_service_failure", False).value
        )
        self.go_home_client = None
        if self.go_home_service:
            self.go_home_client = node.create_client(Trigger, self.go_home_service)
        self.setWindowTitle("Joint GUI Publisher")
        self.setMinimumWidth(750)

        root = QVBoxLayout()

        # toolbar
        bar = QHBoxLayout()
        self._toolbar_buttons = []
        for text, slot in [
            ("Home", self._go_home),
            ("Center", self._center),
            ("Zero", self._zero),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(slot)
            bar.addWidget(btn)
            self._toolbar_buttons.append(btn)
        root.addLayout(bar)

        self._status_label = QLabel("")
        self._status_label.setWordWrap(True)
        self._status_label.setStyleSheet("color: #555;")
        root.addWidget(self._status_label)

        # scrollable joint list
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        panel = QWidget()
        self._rows_layout = QVBoxLayout()
        panel.setLayout(self._rows_layout)
        scroll.setWidget(panel)
        root.addWidget(scroll)

        self.setLayout(root)

        # create one row per free joint
        self.rows = []
        for j in free_joints:
            init = initial_positions.get(j["name"], home.get(j["name"], 0.0))
            r = _JointRow(
                j["name"], j["lower"], j["upper"], init,
                self._rows_layout, self._publish,
            )
            self.rows.append(r)

        # periodic publish
        rate = node.declare_parameter("publish_rate", 20).value
        self._timer = QTimer()
        self._timer.timeout.connect(self._publish)
        self._timer.start(max(1, 1000 // rate))

        self.set_interaction_enabled(True)

    def set_interaction_enabled(self, enabled: bool):
        for button in self._toolbar_buttons:
            button.setEnabled(enabled)
        for row in self.rows:
            row.set_enabled(enabled)

    def set_startup_status(self, text: str, *, ready: bool):
        self._status_label.setText(text)
        self._status_label.setStyleSheet(
            "color: #0b7a29;" if ready else "color: #8a5a00;"
        )

    def apply_feedback_positions(self, feedback_positions: dict[str, float]):
        for row in self.rows:
            if row.name in feedback_positions:
                row.set_value(feedback_positions[row.name])

    # ---- publish ----
    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        positions = {}
        for r in self.rows:
            v = r.value()
            msg.name.append(r.name)
            msg.position.append(v)
            positions[r.name] = v
        for m in self.mimic_joints:
            parent_v = positions.get(m["parent"], 0.0)
            msg.name.append(m["name"])
            msg.position.append(m["multiplier"] * parent_v + m["offset"])
        self.node.pub.publish(msg)

    # ---- toolbar actions ----
    def _go_home(self):
        if self.go_home_client is not None:
            if self._call_motion_go_home():
                self._apply_local_home()
                return
            if not self.local_home_on_service_failure:
                return
        self._apply_local_home()

    def _apply_local_home(self):
        for r in self.rows:
            if r.name in self.home:
                r.set_value(self.home[r.name])

    def _call_motion_go_home(self) -> bool:
        service_name = self.go_home_service or "/marvin_motion/go_home"
        deadline = time.monotonic() + max(1.0, self.go_home_timeout_sec)
        while time.monotonic() < deadline:
            if self.go_home_client.service_is_ready():
                break
            if self.go_home_client.wait_for_service(timeout_sec=0.1):
                break
            rclpy.spin_once(self.node, timeout_sec=0.0)
            QApplication.processEvents()
        else:
            self.node.get_logger().error(f"Home service unavailable: {service_name}")
            return False

        future = self.go_home_client.call_async(Trigger.Request())
        while time.monotonic() < deadline:
            if future.done():
                break
            rclpy.spin_once(self.node, timeout_sec=0.05)
            QApplication.processEvents()

        if not future.done():
            self.node.get_logger().error(f"Home request timed out: {service_name}")
            return False

        if future.exception() is not None:
            self.node.get_logger().error(
                f"Home request failed via {service_name}: {future.exception()}"
            )
            return False

        response = future.result()
        if not response.success:
            self.node.get_logger().error(
                f"Home request rejected via {service_name}: {response.message}"
            )
            return False

        self.node.get_logger().info(
            f"Home request accepted via {service_name}: {response.message}"
        )
        return True

    def _center(self):
        for r in self.rows:
            r.set_value((r.lower + r.upper) / 2.0)

    def _zero(self):
        for r in self.rows:
            r.set_value(0.0)


def _parse_urdf(xml_string):
    root = ET.fromstring(xml_string)
    free, mimic = [], []
    for j in root.findall(".//joint"):
        jtype = j.get("type", "fixed")
        if jtype not in ("revolute", "prismatic", "continuous"):
            continue
        name = j.get("name")
        limit = j.find("limit")
        if jtype == "continuous":
            lo, hi = -3.14159265, 3.14159265
        elif limit is not None:
            lo = float(limit.get("lower", "0"))
            hi = float(limit.get("upper", "0"))
        else:
            lo, hi = -3.14159265, 3.14159265

        m = j.find("mimic")
        if m is not None:
            mimic.append({
                "name": name,
                "parent": m.get("joint"),
                "multiplier": float(m.get("multiplier", "1")),
                "offset": float(m.get("offset", "0")),
            })
        else:
            free.append({"name": name, "lower": lo, "upper": hi})
    return free, mimic


def main():
    rclpy.init()
    node = Node("joint_gui_publisher")
    publish_topic = str(node.declare_parameter("publish_topic", "joint_states").value).strip()
    if not publish_topic:
        publish_topic = "joint_states"
    node.pub = node.create_publisher(JointState, publish_topic, 10)
    node.get_logger().info(f"Publishing GUI joint states on {publish_topic}.")

    # wait for robot_description (published by robot_state_publisher)
    desc_holder = [None]
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def _on_desc(msg):
        desc_holder[0] = msg.data

    node.create_subscription(String, "robot_description", _on_desc, qos)

    deadline = time.monotonic() + 15.0
    while desc_holder[0] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if desc_holder[0] is None:
        node.get_logger().fatal("Timed out waiting for /robot_description")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    free_joints, mimic_joints = _parse_urdf(desc_holder[0])
    node.get_logger().info(
        f"Parsed URDF joints for GUI: free={len(free_joints)} mimic={len(mimic_joints)}"
    )

    home_pose_id = str(node.declare_parameter("home_pose_id", "home").value).strip() or "home"
    home = {}
    home_config_path = str(node.declare_parameter("home_config_path", "").value).strip()
    if home_config_path:
        try:
            home = _load_home_from_config(Path(home_config_path).expanduser(), home_pose_id)
            node.get_logger().info(
                f"Loaded home positions for pose '{home_pose_id}' from {home_config_path}."
            )
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error(f"Failed to load home positions from {home_config_path}: {exc}")

    # Allow per-joint initial_positions to override named poses when explicitly provided.
    for j in free_joints:
        pname = f"initial_positions.{j['name']}"
        had_param = node.has_parameter(pname)
        if not had_param:
            node.declare_parameter(pname, 0.0)
        value = node.get_parameter(pname).value
        if had_param or j["name"] not in home:
            home[j["name"]] = value

    initial_positions = dict(home)
    feedback_topic = str(node.declare_parameter("feedback_topic", "/joint_states").value).strip()
    sync_from_feedback = bool(
        node.declare_parameter("sync_from_feedback_on_startup", True).value
    )
    feedback_timeout_sec = float(node.declare_parameter("feedback_timeout_sec", 10.0).value)
    feedback_positions = {}
    required_joint_names = {joint["name"] for joint in free_joints}
    feedback_sub = None
    feedback_start_time = time.monotonic()
    feedback_last_log_time = 0.0
    feedback_timeout_reported = False

    # Qt
    node.get_logger().info("Creating QApplication for joint GUI.")
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal.signal(signal.SIGTERM, lambda *_args: app.quit())

    node.get_logger().info("Constructing joint GUI main window.")
    win = _MainWindow(node, free_joints, mimic_joints, initial_positions, home)
    node.get_logger().info("Calling win.show() for joint GUI.")
    win.show()
    node.get_logger().info(
        "Joint GUI window state after show: "
        f"visible={win.isVisible()} hidden={win.isHidden()} winId={int(win.winId())}"
    )

    if sync_from_feedback and feedback_topic:
        node.get_logger().info(
            f"Waiting for startup feedback from {feedback_topic} "
            f"for {len(required_joint_names)} GUI joints."
        )
        node.get_logger().info(
            f"Required GUI joints: {', '.join(sorted(required_joint_names))}"
        )
        win.set_interaction_enabled(False)
        win.set_startup_status(
            f"Waiting for startup feedback from {feedback_topic}...", ready=False
        )

        def _on_feedback(msg: JointState):
            for name, position in zip(msg.name, msg.position):
                feedback_positions[str(name)] = float(position)

        feedback_sub = node.create_subscription(
            JointState,
            feedback_topic,
            _on_feedback,
            qos_profile_sensor_data,
        )
    else:
        win.set_startup_status("GUI ready.", ready=True)

    ros_spin = QTimer()
    ros_spin.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_spin.start(10)

    feedback_check_timer = QTimer()

    def _check_feedback_readiness():
        nonlocal feedback_last_log_time, feedback_timeout_reported, feedback_sub
        if not sync_from_feedback or not feedback_topic:
            feedback_check_timer.stop()
            return

        missing = sorted(required_joint_names.difference(feedback_positions))
        if not missing:
            for joint in free_joints:
                joint_name = joint["name"]
                initial_positions[joint_name] = feedback_positions[joint_name]
            win.apply_feedback_positions(feedback_positions)
            win.set_interaction_enabled(True)
            win.set_startup_status(
                f"Synced startup feedback from {feedback_topic}.", ready=True
            )
            node.get_logger().info(
                f"Seeded GUI initial positions from {feedback_topic}."
            )
            if feedback_sub is not None:
                node.destroy_subscription(feedback_sub)
                feedback_sub = None
            feedback_check_timer.stop()
            return

        now = time.monotonic()
        if now - feedback_last_log_time >= 2.0:
            received = sorted(
                name for name in feedback_positions if name in required_joint_names
            )
            node.get_logger().info(
                f"Startup feedback progress: received={len(received)}/"
                f"{len(required_joint_names)} missing="
                f"{', '.join(missing) if missing else '<none>'}"
            )
            feedback_last_log_time = now

        if (
            feedback_timeout_sec > 0.0
            and not feedback_timeout_reported
            and now - feedback_start_time >= feedback_timeout_sec
        ):
            feedback_timeout_reported = True
            node.get_logger().error(
                f"Timed out waiting for complete {feedback_topic} feedback. "
                f"Missing joints: {', '.join(missing)}"
            )

        win.set_startup_status(
            f"Waiting for {feedback_topic}: received "
            f"{len(required_joint_names) - len(missing)}/{len(required_joint_names)} joints.",
            ready=False,
        )

    feedback_check_timer.timeout.connect(_check_feedback_readiness)
    feedback_check_timer.start(100)

    app.exec_()
    if feedback_sub is not None:
        node.destroy_subscription(feedback_sub)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
