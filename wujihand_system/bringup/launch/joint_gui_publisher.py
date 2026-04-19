#!/usr/bin/env python3

import math
import signal
import sys
import time
import xml.etree.ElementTree as ET

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QHBoxLayout,
    QLabel,
    QScrollArea,
    QSlider,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String


SLIDER_STEPS = 10000


class JointRow:
    def __init__(self, name, lower, upper, initial, layout, on_change):
        self.name = name
        self.lower = lower
        self.upper = upper
        self._on_change = on_change
        self._guard = False

        row = QHBoxLayout()

        label = QLabel(name)
        label.setFixedWidth(140)
        row.addWidget(label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, SLIDER_STEPS)
        self.slider.valueChanged.connect(self._slider_changed)
        row.addWidget(self.slider, stretch=3)

        self.spin = QDoubleSpinBox()
        self.spin.setDecimals(2)
        self.spin.setSingleStep(1.0)
        self.spin.setRange(math.degrees(lower), math.degrees(upper))
        self.spin.setSuffix(" deg")
        self.spin.setFixedWidth(130)
        self.spin.valueChanged.connect(self._spin_changed)
        row.addWidget(self.spin)

        range_label = QLabel(
            f"[{math.degrees(lower):.1f} deg, {math.degrees(upper):.1f} deg]"
        )
        range_label.setFixedWidth(200)
        range_label.setStyleSheet("color: grey;")
        row.addWidget(range_label)

        layout.addLayout(row)
        self.set_value(initial)

    def _to_slider(self, value):
        if self.upper <= self.lower:
            return 0
        return round((value - self.lower) / (self.upper - self.lower) * SLIDER_STEPS)

    def _from_slider(self, slider_value):
        return self.lower + slider_value / SLIDER_STEPS * (self.upper - self.lower)

    def _slider_changed(self, slider_value):
        if self._guard:
            return
        self._guard = True
        value = self._from_slider(slider_value)
        self.spin.setValue(math.degrees(value))
        self._on_change()
        self._guard = False

    def _spin_changed(self, displayed_value):
        if self._guard:
            return
        self._guard = True
        value = math.radians(displayed_value)
        value = max(self.lower, min(self.upper, value))
        self.slider.setValue(self._to_slider(value))
        self._on_change()
        self._guard = False

    def value(self):
        return math.radians(self.spin.value())

    def set_value(self, value):
        self._guard = True
        value = max(self.lower, min(self.upper, value))
        self.spin.setValue(math.degrees(value))
        self.slider.setValue(self._to_slider(value))
        self._guard = False


class MainWindow(QWidget):
    def __init__(self, node, free_joints):
        super().__init__()
        self.node = node
        self.rows = []

        self.setWindowTitle("Wuji Hand Joint GUI")
        self.setMinimumWidth(850)

        root = QVBoxLayout()

        status = QLabel("UI shows degrees; published JointState positions are radians.")
        status.setStyleSheet("color: #0b7a29;")
        root.addWidget(status)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        panel = QWidget()
        rows_layout = QVBoxLayout()
        panel.setLayout(rows_layout)
        scroll.setWidget(panel)
        root.addWidget(scroll)
        self.setLayout(root)

        for joint in free_joints:
            row = JointRow(
                joint["name"],
                joint["lower"],
                joint["upper"],
                0.0,
                rows_layout,
                self.publish,
            )
            self.rows.append(row)

        timer = QTimer(self)
        timer.timeout.connect(self.publish)
        timer.start(50)

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        for row in self.rows:
            msg.name.append(row.name)
            msg.position.append(row.value())
        self.node.publisher_.publish(msg)


def parse_urdf(xml_string):
    root = ET.fromstring(xml_string)
    free = []
    for joint in root.findall(".//joint"):
        joint_type = joint.get("type", "fixed")
        if joint_type not in ("revolute", "prismatic", "continuous"):
            continue
        if joint.find("mimic") is not None:
            continue

        name = joint.get("name")
        if not name:
            continue

        if joint_type == "continuous":
            lower, upper = -math.pi, math.pi
        else:
            limit = joint.find("limit")
            if limit is None:
                lower, upper = -math.pi, math.pi
            else:
                lower = float(limit.get("lower", "-3.141592653589793"))
                upper = float(limit.get("upper", "3.141592653589793"))

        free.append({"name": name, "lower": lower, "upper": upper})
    return free


def main():
    rclpy.init()
    node = Node("wujihand_joint_gui_publisher")

    publish_topic = str(node.declare_parameter("publish_topic", "gui_joint_states").value).strip()
    if not publish_topic:
        publish_topic = "gui_joint_states"

    node.publisher_ = node.create_publisher(JointState, publish_topic, 10)
    node.get_logger().info(
        f"Publishing GUI joint states on {publish_topic} in radians (UI displays degrees)."
    )

    robot_description = {"xml": None}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def _on_description(msg):
        robot_description["xml"] = msg.data

    node.create_subscription(String, "robot_description", _on_description, qos)

    deadline = time.monotonic() + 15.0
    while robot_description["xml"] is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    if robot_description["xml"] is None:
        node.get_logger().fatal("Timed out waiting for /robot_description")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    free_joints = parse_urdf(robot_description["xml"])
    node.get_logger().info(f"Parsed {len(free_joints)} controllable joints from URDF.")

    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, lambda *_args: app.quit())
    signal.signal(signal.SIGTERM, lambda *_args: app.quit())

    window = MainWindow(node, free_joints)
    window.show()

    ros_spin = QTimer()
    ros_spin.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0))
    ros_spin.start(10)

    exit_code = app.exec_()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
