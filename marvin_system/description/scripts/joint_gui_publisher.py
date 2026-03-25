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
import signal
import sys
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

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


class _JointRow:
    """One row of widgets: label | slider | spin-box | range."""

    def __init__(self, name, lower, upper, initial, layout, on_change):
        self.name = name
        self.lower = lower
        self.upper = upper
        self._on_change = on_change
        self._guard = False

        lo_deg = math.degrees(lower)
        hi_deg = math.degrees(upper)

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
        self.spin.setRange(lo_deg, hi_deg)
        self.spin.setSuffix(" °")
        self.spin.setFixedWidth(150)
        self.spin.valueChanged.connect(self._spin_edited)
        row.addWidget(self.spin)

        rng = QLabel(f"[{lo_deg:.1f}°, {hi_deg:.1f}°]")
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
        self.spin.setValue(math.degrees(self._from_slider(s)))
        self._on_change()
        self._guard = False

    def _spin_edited(self, v_deg):
        if self._guard:
            return
        self._guard = True
        self.slider.setValue(self._to_slider(math.radians(v_deg)))
        self._on_change()
        self._guard = False

    # ---- public API ----
    def value(self):
        """Return current position in radians."""
        return math.radians(self.spin.value())

    def set_value(self, v):
        """Set position from a value in radians."""
        self._guard = True
        v = max(self.lower, min(self.upper, v))
        self.spin.setValue(math.degrees(v))
        self.slider.setValue(self._to_slider(v))
        self._guard = False


class _MainWindow(QWidget):
    def __init__(self, node, free_joints, mimic_joints, home):
        super().__init__()
        self.node = node
        self.mimic_joints = mimic_joints
        self.home = home
        self.setWindowTitle("Joint GUI Publisher")
        self.setMinimumWidth(750)

        root = QVBoxLayout()

        # toolbar
        bar = QHBoxLayout()
        for text, slot in [
            ("Home", self._go_home),
            ("Center", self._center),
            ("Zero", self._zero),
        ]:
            btn = QPushButton(text)
            btn.clicked.connect(slot)
            bar.addWidget(btn)
        root.addLayout(bar)

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
            init = home.get(j["name"], 0.0)
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
        for r in self.rows:
            r.set_value(self.home.get(r.name, 0.0))

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
    node.pub = node.create_publisher(JointState, "joint_states", 10)

    # wait for robot_description (published by robot_state_publisher)
    desc_holder = [None]
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def _on_desc(msg):
        desc_holder[0] = msg.data

    node.create_subscription(String, "robot_description", _on_desc, qos)

    import time
    deadline = time.monotonic() + 15.0
    while desc_holder[0] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if desc_holder[0] is None:
        node.get_logger().fatal("Timed out waiting for /robot_description")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    free_joints, mimic_joints = _parse_urdf(desc_holder[0])

    # read optional initial_positions.<joint> parameters
    home = {}
    for j in free_joints:
        pname = f"initial_positions.{j['name']}"
        node.declare_parameter(pname, 0.0)
        home[j["name"]] = node.get_parameter(pname).value

    # Qt
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    win = _MainWindow(node, free_joints, mimic_joints, home)
    win.show()

    ros_spin = QTimer()
    ros_spin.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_spin.start(10)

    app.exec_()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
