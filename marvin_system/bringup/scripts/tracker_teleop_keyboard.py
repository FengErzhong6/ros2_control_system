#!/usr/bin/env python3

import atexit
from datetime import datetime
import os
import select
import shutil
import signal
import subprocess
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rosbag2_interfaces.srv import Pause, Resume, Stop
from std_msgs.msg import String
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger


DEFAULT_TRAJECTORY_ROOT = os.path.expanduser("~/.ros/marvin_trajectory")
DEFAULT_TRAJECTORY_BAG_PREFIX = "marvin_joint_trajectory"
DEFAULT_TRAJECTORY_TOPICS = ["/joint_states"]


class TrackerTeleopRosbagRecorder:
    def __init__(
        self,
        node: Node,
        *,
        enabled: bool,
        trajectory_root: str,
        bag_prefix: str,
        bag_topics,
    ) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._enabled = enabled
        self._trajectory_root = os.path.expanduser(trajectory_root)
        self._bag_prefix = bag_prefix
        self._bag_topics = [str(topic) for topic in bag_topics if str(topic).strip()]
        self._recorder_node_name = "tracker_teleop_joint_recorder"
        self._resume_client = None
        self._pause_client = None
        self._stop_client = None
        self._process = None
        self._current_bag_dir = None
        self._recording_active = False
        self._session_started = False

        if not self._enabled:
            return

        recorder_ns = f"/{self._recorder_node_name}"
        self._resume_client = self._node.create_client(Resume, f"{recorder_ns}/resume")
        self._pause_client = self._node.create_client(Pause, f"{recorder_ns}/pause")
        self._stop_client = self._node.create_client(Stop, f"{recorder_ns}/stop")

    def start(self) -> None:
        if not self._enabled:
            return
        if not self._spawn_paused_session():
            self._logger.error(
                "Joint trajectory recorder is disabled for this run because ros2 bag "
                "could not be prepared."
            )
            self._enabled = False

    def shutdown(self) -> None:
        if not self._enabled or self._process is None:
            return

        if self._recording_active:
            self.finish_session(prepare_next=False)
            return

        self._terminate_process(remove_bag_dir=not self._session_started)

    def begin_recording(self) -> bool:
        if not self._enabled:
            return True

        if self._process is None or self._process.poll() is not None:
            if not self._spawn_paused_session():
                return False

        if self._recording_active:
            return True

        if not self._call_empty_service(
            self._resume_client,
            Resume.Request(),
            "resume recorder",
            timeout_sec=5.0,
        ):
            return False

        self._recording_active = True
        self._session_started = True
        self._logger.info(f"Recording joint trajectory to: {self._current_bag_dir}")
        return True

    def finish_session(self, prepare_next: bool = True) -> bool:
        if not self._enabled or self._process is None:
            return True

        if self._recording_active:
            if not self._call_empty_service(
                self._pause_client,
                Pause.Request(),
                "pause recorder",
                timeout_sec=5.0,
            ):
                return False
            self._recording_active = False

        if not self._call_empty_service(
            self._stop_client,
            Stop.Request(),
            "stop recorder",
            timeout_sec=5.0,
        ):
            self._terminate_process()
            return False

        bag_dir = self._current_bag_dir
        process = self._process
        try:
            process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self._logger.error("Recorder stop timed out; terminating ros2 bag.")
            self._terminate_process()
            return False

        self._process = None
        self._current_bag_dir = None
        self._logger.info(f"Joint trajectory bag saved: {bag_dir}")

        if prepare_next:
            return self._spawn_paused_session()
        return True

    def _spawn_paused_session(self) -> bool:
        os.makedirs(self._trajectory_root, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_dir = os.path.join(self._trajectory_root, f"{self._bag_prefix}_{timestamp}")
        cmd = [
            "ros2",
            "bag",
            "record",
            "--start-paused",
            "--disable-keyboard-controls",
            "--node-name",
            self._recorder_node_name,
            "-o",
            bag_dir,
            "--topics",
            *self._bag_topics,
        ]

        try:
            self._process = subprocess.Popen(
                cmd,
                stdout=None,
                stderr=None,
                start_new_session=True,
            )
        except OSError as exc:
            self._logger.error(f"Failed to start ros2 bag recorder: {exc}")
            self._process = None
            self._current_bag_dir = None
            return False

        self._current_bag_dir = bag_dir
        self._recording_active = False
        self._session_started = False

        if not self._wait_for_service(self._resume_client, "recorder resume", timeout_sec=10.0):
            self._terminate_process(remove_bag_dir=True)
            return False
        if not self._wait_for_service(self._pause_client, "recorder pause", timeout_sec=10.0):
            self._terminate_process(remove_bag_dir=True)
            return False
        self._logger.info(f"Joint trajectory recorder ready: {bag_dir}")
        return True

    def _wait_for_service(self, client, label: str, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok():
            if self._process is not None and self._process.poll() is not None:
                self._logger.error(f"{label} unavailable because ros2 bag exited.")
                return False
            if client.service_is_ready() or client.wait_for_service(timeout_sec=0.2):
                return True
            if time.monotonic() >= deadline:
                break
        self._logger.error(f"{label} service timeout")
        return False

    def _call_empty_service(self, client, request, label: str, timeout_sec: float) -> bool:
        if not self._wait_for_service(client, label, timeout_sec):
            return False

        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and rclpy.ok() and time.monotonic() < deadline:
            time.sleep(0.01)

        if not future.done():
            self._logger.error(f"{label} request timed out")
            return False

        if future.exception() is not None:
            self._logger.error(f"{label} request failed: {future.exception()}")
            return False

        return True

    def _terminate_process(self, remove_bag_dir: bool = False) -> None:
        process = self._process
        bag_dir = self._current_bag_dir
        if process is None:
            return

        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5.0)

        self._process = None
        self._current_bag_dir = None
        self._recording_active = False
        self._session_started = False
        if remove_bag_dir and bag_dir:
            shutil.rmtree(bag_dir, ignore_errors=True)


class TrackerTeleopKeyboard(Node):
    def __init__(self) -> None:
        super().__init__("tracker_teleop_keyboard")

        self.state_topic = self.declare_parameter(
            "state_topic", "/tracker_teleop_controller/teleop_state").value
        self.arm_service_name = self.declare_parameter(
            "arm_service", "/tracker_teleop_controller/set_armed").value
        self.enable_service_name = self.declare_parameter(
            "enable_service", "/tracker_teleop_controller/set_enabled").value
        self.home_service_name = self.declare_parameter(
            "home_service", "/tracker_teleop_controller/go_home").value
        self.debounce_sec = float(self.declare_parameter("debounce_sec", 0.25).value)
        self.auto_enable_without_tty = bool(
            self.declare_parameter("auto_enable_without_tty", False).value)
        self.enable_joint_recording = bool(
            self.declare_parameter("enable_joint_recording", False).value)
        self.trajectory_root = os.path.expanduser(
            self.declare_parameter("trajectory_root", DEFAULT_TRAJECTORY_ROOT).value)
        self.trajectory_bag_prefix = self.declare_parameter(
            "trajectory_bag_prefix", DEFAULT_TRAJECTORY_BAG_PREFIX).value
        self.trajectory_bag_topics = self.normalize_topic_list(
            self.declare_parameter("trajectory_bag_topics", DEFAULT_TRAJECTORY_TOPICS).value)

        self.current_state = "UNKNOWN"
        self.last_key_time = 0.0
        self.stop_event = threading.Event()
        self.input_file = None
        self.session_active = False

        self.arm_client = self.create_client(SetBool, self.arm_service_name)
        self.enable_client = self.create_client(SetBool, self.enable_service_name)
        self.home_client = self.create_client(Trigger, self.home_service_name)
        state_qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, self.state_topic, self.on_state, state_qos)
        self.recorder = TrackerTeleopRosbagRecorder(
            self,
            enabled=self.enable_joint_recording,
            trajectory_root=self.trajectory_root,
            bag_prefix=self.trajectory_bag_prefix,
            bag_topics=self.trajectory_bag_topics,
        )
        self.recorder.start()

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
            if self.auto_enable_without_tty:
                self.get_logger().warn(
                    "No TTY available; keyboard gate is unavailable in this launch context. "
                    "Falling back to one-shot auto-enable.")
                self.auto_enable_thread = threading.Thread(
                    target=self.auto_enable_once, daemon=True)
                self.auto_enable_thread.start()
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
        if self.recorder is not None:
            self.recorder.shutdown()
        return super().destroy_node()

    @staticmethod
    def normalize_topic_list(value):
        if isinstance(value, (list, tuple)):
            return [str(topic) for topic in value if str(topic).strip()]
        if isinstance(value, str):
            return [value] if value.strip() else []
        return []

    def print_help(self) -> None:
        self.get_logger().info(
            "Keyboard gate ready: [Space]=start/pause teleop, [n]=next sample (go home)")
        if self.enable_joint_recording:
            self.get_logger().info(
                "Joint trajectory recording is enabled for this run."
            )

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
            elif ch in ("n", "N"):
                self.handle_next()

    def ensure_service(self, client, label: str, service_name: str) -> bool:
        if client.service_is_ready():
            return True
        if client.wait_for_service(timeout_sec=0.2):
            return True
        self.get_logger().warn(f"{label} service not ready: {service_name}")
        return False

    def wait_for_service(self, client, label: str, service_name: str, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not self.stop_event.is_set():
            if client.service_is_ready() or client.wait_for_service(timeout_sec=0.2):
                return True
            if time.monotonic() >= deadline:
                break
        self.get_logger().warn(f"{label} service not ready before timeout: {service_name}")
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

    def call_trigger(self, client, label: str, service_name: str) -> bool:
        if not self.ensure_service(client, label, service_name):
            return False

        request = Trigger.Request()
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
        if self.session_active or self.current_state == "ENABLED":
            if self.call_set_bool(self.enable_client, "stop", self.enable_service_name, False):
                if self.session_active and self.recorder is not None:
                    self.recorder.finish_session()
                self.session_active = False
            return

        if self.recorder is not None and not self.recorder.begin_recording():
            self.get_logger().warn("Start aborted because ros2 bag is not ready.")
            return
        if not self.call_set_bool(self.arm_client, "start-arm", self.arm_service_name, True):
            if self.recorder is not None:
                self.recorder.finish_session()
            return
        if self.call_set_bool(self.enable_client, "start-enable", self.enable_service_name, True):
            self.session_active = True
            return
        if self.recorder is not None:
            self.recorder.finish_session()

    def handle_next(self) -> None:
        if self.session_active and self.recorder is not None:
            self.recorder.finish_session()
        self.session_active = False
        self.call_trigger(self.home_client, "next-sample", self.home_service_name)

    def auto_enable_once(self) -> None:
        time.sleep(0.2)
        if not self.wait_for_service(self.arm_client, "arm", self.arm_service_name, 10.0):
            return
        if not self.wait_for_service(self.enable_client, "enable", self.enable_service_name, 10.0):
            return

        self.get_logger().warn(
            "No interactive TTY detected; sending one-shot arm+enable requests.")
        if self.recorder is not None and not self.recorder.begin_recording():
            self.get_logger().warn("Auto-enable aborted because ros2 bag is not ready.")
            return
        if not self.call_set_bool(self.arm_client, "start-arm", self.arm_service_name, True):
            if self.recorder is not None:
                self.recorder.finish_session()
            return
        if self.call_set_bool(self.enable_client, "start-enable", self.enable_service_name, True):
            self.session_active = True
            return
        if self.recorder is not None:
            self.recorder.finish_session()


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
