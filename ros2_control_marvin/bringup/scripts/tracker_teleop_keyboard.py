#!/usr/bin/env python3

import atexit
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class TrackerTeleopKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("tracker_teleop_keyboard")

        self.state_topic = self.declare_parameter(
            "state_topic", "/tracker_teleop_controller/teleop_state").value
        self.arm_service_name = self.declare_parameter(
            "arm_service", "/tracker_teleop_controller/set_armed").value
        self.enable_service_name = self.declare_parameter(
            "enable_service", "/tracker_teleop_controller/set_enabled").value
        self.debounce_sec = float(self.declare_parameter("debounce_sec", 0.25).value)

        self.current_state = "UNKNOWN"
        self.last_key_time = 0.0
        self.stop_event = threading.Event()
        self.input_file = None

        self.arm_client = self.create_client(SetBool, self.arm_service_name)
        self.enable_client = self.create_client(SetBool, self.enable_service_name)
        state_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, self.state_topic, self.on_state, state_qos)

        self.stdin_fd = None
        self.stdin_settings = None
        input_file, input_label = self.open_input_tty()
        if input_file is not None:
            self.input_file = input_file
            self.stdin_fd = input_file.fileno()
            self.stdin_settings = termios.tcgetattr(self.stdin_fd)
            tty.setcbreak(self.stdin_fd)
            atexit.register(self.restore_terminal)
            self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
            self.key_thread.start()
            self.get_logger().info(f"Keyboard input attached to {input_label}.")
            self.print_help()
        else:
            self.get_logger().error(
                "No TTY available; keyboard gate is unavailable in this launch context.")

    def open_input_tty(self):
        if sys.stdin.isatty():
            return sys.stdin, "stdin"

        try:
            return open("/dev/tty", "rb", buffering=0), "/dev/tty"
        except OSError as exc:
            self.get_logger().warn(f"Failed to open /dev/tty: {exc}")
            return None, None

    def restore_terminal(self) -> None:
        if self.stdin_fd is not None and self.stdin_settings is not None:
            termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.stdin_settings)
            self.stdin_fd = None
            self.stdin_settings = None
        if self.input_file is not None and self.input_file is not sys.stdin:
            self.input_file.close()
            self.input_file = None

    def destroy_node(self) -> bool:
        self.stop_event.set()
        self.restore_terminal()
        return super().destroy_node()

    def print_help(self) -> None:
        self.get_logger().info(
            "Keyboard gate ready: [Space]=start/pause teleop")

    def on_state(self, msg: String) -> None:
        state = msg.data.split("|", 1)[0].strip()
        if state and state != self.current_state:
            self.current_state = state
            self.get_logger().info(f"Tracker teleop state: {self.current_state}")

    def keyboard_loop(self) -> None:
        while rclpy.ok() and not self.stop_event.is_set():
            ready, _, _ = select.select([self.stdin_fd], [], [], 0.1)
            if not ready:
                continue

            ch = os.read(self.stdin_fd, 1).decode(errors="ignore")
            if not ch:
                continue

            now = time.monotonic()
            if now - self.last_key_time < self.debounce_sec:
                continue
            self.last_key_time = now

            if ch == " ":
                self.handle_space()

    def ensure_service(self, client, label: str, service_name: str) -> bool:
        if client.service_is_ready():
            return True
        if client.wait_for_service(timeout_sec=0.2):
            return True
        self.get_logger().warn(f"{label} service not ready: {service_name}")
        return False

    def call_set_bool(self, client, label: str, service_name: str, value: bool) -> bool:
        if not self.ensure_service(client, label, service_name):
            return False

        request = SetBool.Request()
        request.data = value
        future = client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline and rclpy.ok():
            time.sleep(0.01)

        if not future.done() or future.result() is None:
            self.get_logger().warn(f"{label} request timed out.")
            return False

        response = future.result()
        if not response.success:
            self.get_logger().warn(f"{label} rejected: {response.message}")
            return False

        self.get_logger().info(f"{label}: {response.message}")
        return True

    def handle_space(self) -> None:
        if self.current_state == "ENABLED":
            if not self.call_set_bool(self.enable_client, "disable", self.enable_service_name, False):
                return
            self.call_set_bool(self.arm_client, "disarm", self.arm_service_name, False)
            return

        if not self.call_set_bool(self.arm_client, "arm", self.arm_service_name, True):
            return
        self.call_set_bool(self.enable_client, "enable", self.enable_service_name, True)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrackerTeleopKeyboard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
