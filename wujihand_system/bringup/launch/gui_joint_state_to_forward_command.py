#!/usr/bin/env python3

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String


def parse_joint_order(xml_string: str) -> list[str]:
    root = ET.fromstring(xml_string)
    order = []
    for joint in root.findall(".//joint"):
        joint_type = joint.get("type", "fixed")
        if joint_type not in ("revolute", "prismatic", "continuous"):
            continue
        if joint.find("mimic") is not None:
            continue
        name = joint.get("name")
        if name:
            order.append(name)
    return order


class GuiJointStateToForwardCommand(Node):
    def __init__(self) -> None:
        super().__init__("gui_joint_state_to_forward_command")

        self._input_topic = self.declare_parameter("input_topic", "gui_joint_states").value
        self._output_topic = self.declare_parameter(
            "output_topic", "/forward_position_controller/commands"
        ).value

        self._joint_order: list[str] = []
        self._pub = self.create_publisher(Float64MultiArray, self._output_topic, 10)
        self._sub = self.create_subscription(JointState, self._input_topic, self._on_js, 10)
        self._robot_description_sub = self.create_subscription(
            String,
            "robot_description",
            self._on_robot_description,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

        self._last_cmd: list[float] = []

        # Publish initial all-zeros command once so the forward controller has a defined target.
        self.create_timer(0.2, self._publish_initial_once)
        self._initial_published = False

        self.get_logger().info(
            f"Bridging {self._input_topic} -> {self._output_topic}; waiting for robot_description "
            "to determine joint order."
        )

    def _on_robot_description(self, msg: String) -> None:
        if self._joint_order:
            return
        self._joint_order = parse_joint_order(msg.data)
        self._last_cmd = [0.0] * len(self._joint_order)
        self.get_logger().info(
            "Resolved forward command joint order from URDF: "
            + ", ".join(self._joint_order)
        )

    def _publish_initial_once(self) -> None:
        if self._initial_published or not self._joint_order:
            return
        self._publish_cmd(self._last_cmd)
        self._initial_published = True

    def _publish_cmd(self, cmd: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(cmd)
        self._pub.publish(msg)

    def _on_js(self, msg: JointState) -> None:
        if not self._joint_order:
            return
        name_to_index = {name: i for i, name in enumerate(msg.name)}

        cmd = [0.0] * len(self._joint_order)
        for out_i, joint_name in enumerate(self._joint_order):
            src_i = name_to_index.get(joint_name)
            if src_i is None:
                cmd[out_i] = self._last_cmd[out_i]
                continue
            if src_i >= len(msg.position):
                cmd[out_i] = self._last_cmd[out_i]
                continue
            cmd[out_i] = float(msg.position[src_i])

        self._last_cmd = cmd
        self._publish_cmd(cmd)


def main() -> None:
    rclpy.init()
    node = GuiJointStateToForwardCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
