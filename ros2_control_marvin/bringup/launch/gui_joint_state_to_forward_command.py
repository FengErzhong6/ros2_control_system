#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_ORDER = [
    "Joint1_L",
    "Joint2_L",
    "Joint3_L",
    "Joint4_L",
    "Joint5_L",
    "Joint6_L",
    "Joint7_L",
    "Joint1_R",
    "Joint2_R",
    "Joint3_R",
    "Joint4_R",
    "Joint5_R",
    "Joint6_R",
    "Joint7_R",
]


def _extract_positions(msg: JointState) -> dict[str, float]:
    out: dict[str, float] = {}
    name_to_index = {name: i for i, name in enumerate(msg.name)}
    for name in JOINT_ORDER:
        idx = name_to_index.get(name)
        if idx is None or idx >= len(msg.position):
            continue
        out[name] = float(msg.position[idx])
    return out


class GuiJointStateToForwardCommand(Node):
    """Bridge GUI JointState -> forward position controller commands.

    Safety goal: avoid startup jump to zero.

    We treat the GUI slider values as *relative* offsets around the real robot pose:

        cmd = q_real0 + (q_gui - q_gui0)

    where q_real0 is the first received real /joint_states pose, and q_gui0 is the first
    received GUI pose.
    """

    def __init__(self) -> None:
        super().__init__("gui_joint_state_to_forward_command")

        self._gui_topic = self.declare_parameter("gui_topic", "gui_joint_states").value
        self._real_topic = self.declare_parameter("real_topic", "/joint_states").value
        self._output_topic = self.declare_parameter(
            "output_topic", "/forward_position_controller/commands"
        ).value

        self._pub = self.create_publisher(Float64MultiArray, self._output_topic, 10)

        self._sub_gui = self.create_subscription(JointState, self._gui_topic, self._on_gui, 10)
        self._sub_real = self.create_subscription(JointState, self._real_topic, self._on_real, 10)

        self._q_real0: dict[str, float] | None = None
        self._q_gui0: dict[str, float] | None = None
        self._last_cmd = [0.0] * len(JOINT_ORDER)

        self._initialized = False
        self._initial_published = False

        self.create_timer(0.2, self._publish_initial_once)

        self.get_logger().info(
            f"Bridging GUI '{self._gui_topic}' + REAL '{self._real_topic}' -> '{self._output_topic}'"
        )

    def _maybe_init(self) -> None:
        if self._initialized:
            return
        if self._q_real0 is None or self._q_gui0 is None:
            return

        # Publish an initial hold command equal to the current real pose.
        cmd = []
        for name in JOINT_ORDER:
            cmd.append(float(self._q_real0.get(name, 0.0)))
        self._last_cmd = cmd
        self._publish_cmd(cmd)
        self._initialized = True

        self.get_logger().info("Initialized offset mapping; initial hold command published.")

    def _publish_initial_once(self) -> None:
        if not self._initialized:
            self._maybe_init()
            return
        if self._initial_published:
            return
        self._publish_cmd(self._last_cmd)
        self._initial_published = True

    def _publish_cmd(self, cmd: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(cmd)
        self._pub.publish(msg)

    def _on_real(self, msg: JointState) -> None:
        if self._q_real0 is None:
            q = _extract_positions(msg)
            if len(q) >= 1:
                self._q_real0 = q
                self.get_logger().info("Captured initial real joint pose.")
                self._maybe_init()

    def _on_gui(self, msg: JointState) -> None:
        if self._q_gui0 is None:
            q = _extract_positions(msg)
            if len(q) >= 1:
                self._q_gui0 = q
                self.get_logger().info("Captured initial GUI joint pose.")
                self._maybe_init()
            return

        if not self._initialized or self._q_real0 is None or self._q_gui0 is None:
            return

        q_gui = _extract_positions(msg)

        cmd = []
        for name in JOINT_ORDER:
            q0_real = float(self._q_real0.get(name, 0.0))
            q0_gui = float(self._q_gui0.get(name, 0.0))
            q_now_gui = float(q_gui.get(name, q0_gui))
            cmd.append(q0_real + (q_now_gui - q0_gui))

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
