#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_ORDER = [
    "finger0_joint0",
    "finger0_joint1",
    "finger0_joint2",
    "finger0_joint3",
    "finger1_joint0",
    "finger1_joint1",
    "finger1_joint2",
    "finger1_joint3",
    "finger2_joint0",
    "finger2_joint1",
    "finger2_joint2",
    "finger2_joint3",
    "finger3_joint0",
    "finger3_joint1",
    "finger3_joint2",
    "finger3_joint3",
    "finger4_joint0",
    "finger4_joint1",
    "finger4_joint2",
    "finger4_joint3",
]


class GuiJointStateToForwardCommand(Node):
    def __init__(self) -> None:
        super().__init__("gui_joint_state_to_forward_command")

        self._input_topic = self.declare_parameter("input_topic", "gui_joint_states").value
        self._output_topic = self.declare_parameter(
            "output_topic", "/forward_position_controller/commands"
        ).value

        self._pub = self.create_publisher(Float64MultiArray, self._output_topic, 10)
        self._sub = self.create_subscription(JointState, self._input_topic, self._on_js, 10)

        self._last_cmd = [0.0] * len(JOINT_ORDER)

        # Publish initial all-zeros command once so the forward controller has a defined target.
        self.create_timer(0.2, self._publish_initial_once)
        self._initial_published = False

        self.get_logger().info(
            f"Bridging {self._input_topic} -> {self._output_topic} (initial command: all 0.0)"
        )

    def _publish_initial_once(self) -> None:
        if self._initial_published:
            return
        self._publish_cmd(self._last_cmd)
        self._initial_published = True

    def _publish_cmd(self, cmd: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(cmd)
        self._pub.publish(msg)

    def _on_js(self, msg: JointState) -> None:
        name_to_index = {name: i for i, name in enumerate(msg.name)}

        cmd = [0.0] * len(JOINT_ORDER)
        for out_i, joint_name in enumerate(JOINT_ORDER):
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
