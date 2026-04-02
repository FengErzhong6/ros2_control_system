#!/usr/bin/env python3

import atexit
import os
import select
import threading
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple
import termios

from controller_manager_msgs.srv import ListControllers
import rclpy
import rosbag2_py
import yaml
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


DEFAULT_JOINT_NAMES = [
    "Joint1_L", "Joint2_L", "Joint3_L", "Joint4_L",
    "Joint5_L", "Joint6_L", "Joint7_L",
    "Joint1_R", "Joint2_R", "Joint3_R", "Joint4_R",
    "Joint5_R", "Joint6_R", "Joint7_R",
]
ACTIVE_CONTROLLER_NAMES = ("joint_state_broadcaster", "forward_position_controller")


def load_storage_id(bag_dir: Path) -> str:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"metadata.yaml not found in bag directory: {bag_dir}")

    with metadata_path.open("r", encoding="utf-8") as handle:
        metadata = yaml.safe_load(handle)

    return metadata["rosbag2_bagfile_information"]["storage_identifier"]


def load_home_positions(path: Path) -> dict[str, float]:
    if not path.is_file():
        raise FileNotFoundError(f"Home positions file not found: {path}")

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}

    params = data.get("joint_gui_publisher", {}).get("ros__parameters", {})
    initial_positions = params.get("initial_positions", {})
    if not isinstance(initial_positions, dict):
        raise RuntimeError(f"Invalid initial_positions in {path}")
    return {
        str(name): float(value)
        for name, value in initial_positions.items()
        if str(name).strip()
    }


class JointTrajectoryBagReplay(Node):
    def __init__(self) -> None:
        super().__init__("joint_trajectory_bag_replay")

        bag_path_value = self.declare_parameter("trajectory_bag", "").value
        if not isinstance(bag_path_value, str) or not bag_path_value.strip():
            raise RuntimeError("Parameter 'trajectory_bag' must be a rosbag directory path.")

        self._bag_path = Path(bag_path_value).expanduser().resolve()
        if not self._bag_path.is_dir():
            raise RuntimeError(f"Trajectory bag directory not found: {self._bag_path}")

        self._bag_topic = str(self.declare_parameter("bag_topic", "/joint_states").value)
        self._command_topic = str(
            self.declare_parameter(
                "command_topic", "/forward_position_controller/commands"
            ).value
        )
        self._joint_names = [
            str(name)
            for name in self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES).value
            if str(name).strip()
        ]
        if not self._joint_names:
            raise RuntimeError("Parameter 'joint_names' must contain at least one joint.")

        self._rate_scale = float(self.declare_parameter("rate_scale", 1.0).value)
        if self._rate_scale <= 0.0:
            raise RuntimeError("Parameter 'rate_scale' must be > 0.")

        self._start_delay_sec = float(self.declare_parameter("start_delay_sec", 1.0).value)
        if self._start_delay_sec < 0.0:
            raise RuntimeError("Parameter 'start_delay_sec' must be >= 0.")

        self._wait_for_feedback_timeout_sec = float(
            self.declare_parameter("wait_for_feedback_timeout_sec", 10.0).value
        )
        if self._wait_for_feedback_timeout_sec < 0.0:
            raise RuntimeError("Parameter 'wait_for_feedback_timeout_sec' must be >= 0.")

        self._wait_for_command_sub_timeout_sec = float(
            self.declare_parameter("wait_for_command_sub_timeout_sec", 10.0).value
        )
        if self._wait_for_command_sub_timeout_sec < 0.0:
            raise RuntimeError("Parameter 'wait_for_command_sub_timeout_sec' must be >= 0.")

        self._wait_for_controllers_timeout_sec = float(
            self.declare_parameter("wait_for_controllers_timeout_sec", 15.0).value
        )
        if self._wait_for_controllers_timeout_sec < 0.0:
            raise RuntimeError("Parameter 'wait_for_controllers_timeout_sec' must be >= 0.")

        self._list_controllers_service = str(
            self.declare_parameter(
                "list_controllers_service", "/controller_manager/list_controllers"
            ).value
        )
        home_positions_file_value = str(
            self.declare_parameter("home_positions_file", "").value
        ).strip()
        if not home_positions_file_value:
            raise RuntimeError("Parameter 'home_positions_file' must be set.")
        self._home_positions_file = Path(home_positions_file_value).expanduser().resolve()
        self._home_positions = load_home_positions(self._home_positions_file)
        self._home_tolerance_deg = float(self.declare_parameter("home_tolerance_deg", 0.5).value)
        self._home_stage_timeout_sec = float(
            self.declare_parameter("home_stage_timeout_sec", 15.0).value
        )

        self._command_pub = self.create_publisher(Float64MultiArray, self._command_topic, 10)
        self._feedback_lock = threading.Lock()
        self._feedback_positions: List[Optional[float]] = [None] * len(self._joint_names)
        self._feedback_received = threading.Event()
        self._stop_event = threading.Event()
        self._tty_file = None
        self._tty_fd = None
        self._tty_settings = None
        self._list_controllers_client = self.create_client(
            ListControllers, self._list_controllers_service
        )
        self.create_subscription(JointState, "joint_states", self._on_joint_state, 10)
        atexit.register(self._restore_terminal)

        self._samples = self._load_samples()
        first_time_ns = self._samples[0][0]
        last_time_ns = self._samples[-1][0]
        duration_sec = max(0.0, (last_time_ns - first_time_ns) / 1_000_000_000.0)
        self.get_logger().info(
            f"Loaded {len(self._samples)} replay samples from {self._bag_path} "
            f"on {self._bag_topic} (duration {duration_sec:.3f} s)."
        )

        self._worker = threading.Thread(target=self._run_playback, daemon=True)
        self._worker.start()

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._restore_terminal()
        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=1.0)
        return super().destroy_node()

    def _on_joint_state(self, msg: JointState) -> None:
        index_by_name = {name: idx for idx, name in enumerate(msg.name)}
        updated = False
        with self._feedback_lock:
            for joint_index, joint_name in enumerate(self._joint_names):
                msg_index = index_by_name.get(joint_name)
                if msg_index is None or msg_index >= len(msg.position):
                    continue
                self._feedback_positions[joint_index] = msg.position[msg_index]
                updated = True
            if updated and all(value is not None for value in self._feedback_positions):
                self._feedback_received.set()

    def _load_samples(self) -> List[Tuple[int, List[Optional[float]]]]:
        storage_id = load_storage_id(self._bag_path)
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(
            uri=str(self._bag_path), storage_id=storage_id
        )
        converter_options = rosbag2_py.ConverterOptions("", "")
        reader.open(storage_options, converter_options)

        topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
        if self._bag_topic not in topic_types:
            raise RuntimeError(
                f"Topic '{self._bag_topic}' not found in bag. "
                f"Available topics: {sorted(topic_types)}"
            )
        if topic_types[self._bag_topic] != "sensor_msgs/msg/JointState":
            raise RuntimeError(
                f"Topic '{self._bag_topic}' has unsupported type "
                f"'{topic_types[self._bag_topic]}'. Expected sensor_msgs/msg/JointState."
            )

        reader.set_filter(rosbag2_py.StorageFilter(topics=[self._bag_topic]))

        samples: List[Tuple[int, List[Optional[float]]]] = []
        while reader.has_next():
            topic_name, serialized_data, timestamp_ns = reader.read_next()
            if topic_name != self._bag_topic:
                continue

            message = deserialize_message(serialized_data, JointState)
            sample = self._extract_sample(message)
            if any(value is not None for value in sample):
                samples.append((timestamp_ns, sample))

        if not samples:
            raise RuntimeError(
                f"No joint samples matching {self._joint_names} were found in {self._bag_path}."
            )
        return samples

    def _extract_sample(self, msg: JointState) -> List[Optional[float]]:
        index_by_name = {name: idx for idx, name in enumerate(msg.name)}
        sample: List[Optional[float]] = [None] * len(self._joint_names)
        for joint_index, joint_name in enumerate(self._joint_names):
            msg_index = index_by_name.get(joint_name)
            if msg_index is None or msg_index >= len(msg.position):
                continue
            sample[joint_index] = msg.position[msg_index]
        return sample

    def _wait_for_command_subscriber(self) -> bool:
        deadline = time.monotonic() + self._wait_for_command_sub_timeout_sec
        while not self._stop_event.is_set() and rclpy.ok():
            if self._command_pub.get_subscription_count() > 0:
                return True
            if time.monotonic() >= deadline:
                break
            time.sleep(0.05)
        return False

    def _wait_for_active_controllers(self) -> bool:
        deadline = time.monotonic() + self._wait_for_controllers_timeout_sec
        request = ListControllers.Request()

        while not self._stop_event.is_set() and rclpy.ok():
            if not self._list_controllers_client.service_is_ready():
                if not self._list_controllers_client.wait_for_service(timeout_sec=0.2):
                    if time.monotonic() >= deadline:
                        break
                    continue

            future = self._list_controllers_client.call_async(request)
            call_deadline = time.monotonic() + 2.0
            while not future.done() and not self._stop_event.is_set() and rclpy.ok():
                if time.monotonic() >= call_deadline:
                    break
                time.sleep(0.01)

            if future.done() and future.exception() is None and future.result() is not None:
                states = {controller.name: controller.state for controller in future.result().controller}
                if all(states.get(name) == "active" for name in ACTIVE_CONTROLLER_NAMES):
                    self.get_logger().info(
                        "Replay gating confirmed: joint_state_broadcaster and "
                        "forward_position_controller are active."
                    )
                    return True

            if time.monotonic() >= deadline:
                break
            time.sleep(0.1)

        return False

    def _wait_for_feedback(self) -> bool:
        if self._feedback_received.wait(timeout=self._wait_for_feedback_timeout_sec):
            return True
        return False

    def _snapshot_feedback(self) -> List[float]:
        with self._feedback_lock:
            if not all(value is not None for value in self._feedback_positions):
                missing = [
                    self._joint_names[index]
                    for index, value in enumerate(self._feedback_positions)
                    if value is None
                ]
                raise RuntimeError(
                    "Joint feedback missing for replay startup: "
                    + ", ".join(missing)
                )
            return [float(value) for value in self._feedback_positions]

    def _current_feedback(self) -> List[Optional[float]]:
        with self._feedback_lock:
            return list(self._feedback_positions)

    def _publish_command(self, command: Sequence[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(command)
        self._command_pub.publish(msg)

    def _sleep_until(self, target_time: float) -> bool:
        while not self._stop_event.is_set() and rclpy.ok():
            remaining = target_time - time.monotonic()
            if remaining <= 0.0:
                return True
            time.sleep(min(remaining, 0.01))
        return False

    def _open_tty_for_ctrl_c(self) -> bool:
        if self._tty_fd is not None:
            return True

        try:
            input_file = open("/dev/tty", "rb", buffering=0)
        except OSError as exc:
            self.get_logger().error(f"Failed to open /dev/tty for Ctrl+C capture: {exc}")
            return False

        self._tty_file = input_file
        self._tty_fd = input_file.fileno()
        self._tty_settings = termios.tcgetattr(self._tty_fd)
        raw_settings = termios.tcgetattr(self._tty_fd)
        raw_settings[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
        raw_settings[6][termios.VMIN] = 1
        raw_settings[6][termios.VTIME] = 0
        termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, raw_settings)
        return True

    def _restore_terminal(self) -> None:
        if self._tty_fd is not None and self._tty_settings is not None:
            termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._tty_settings)
            self._tty_fd = None
            self._tty_settings = None
        if self._tty_file is not None:
            self._tty_file.close()
            self._tty_file = None

    def _wait_for_ctrl_c(self) -> bool:
        if not self._open_tty_for_ctrl_c():
            return False

        self.get_logger().info("Replay complete. Press Ctrl+C to return home and exit.")
        while not self._stop_event.is_set() and rclpy.ok():
            ready, _, _ = select.select([self._tty_fd], [], [], 0.1)
            if not ready:
                continue
            ch = os.read(self._tty_fd, 1)
            if ch == b"\x03":
                self.get_logger().info("Ctrl+C captured. Returning to home position.")
                return True
        return False

    def _build_home_stages(self) -> List[List[int]]:
        stages: List[List[int]] = []
        used_indices: set[int] = set()
        name_to_index = {name: index for index, name in enumerate(self._joint_names)}

        for joint_id in range(7, 0, -1):
            stage: List[int] = []
            for suffix in ("_L", "_R"):
                joint_name = f"Joint{joint_id}{suffix}"
                index = name_to_index.get(joint_name)
                if index is None or joint_name not in self._home_positions:
                    continue
                stage.append(index)
                used_indices.add(index)
            if stage:
                stages.append(stage)

        for index in reversed(range(len(self._joint_names))):
            if index in used_indices:
                continue
            if self._joint_names[index] in self._home_positions:
                stages.append([index])

        return stages

    def _run_go_home_sequence(self) -> None:
        stages = self._build_home_stages()
        if not stages:
            self.get_logger().warn("No home positions available for replay joints; exiting in place.")
            return

        tolerance_rad = self._home_tolerance_deg * 3.141592653589793 / 180.0
        current_command = self._snapshot_feedback()
        for stage in stages:
            stage_names = [self._joint_names[index] for index in stage]
            for index in stage:
                current_command[index] = self._home_positions[self._joint_names[index]]

            deadline = time.monotonic() + self._home_stage_timeout_sec
            while not self._stop_event.is_set() and rclpy.ok():
                self._publish_command(current_command)
                feedback = self._current_feedback()
                stage_reached = True
                for index in stage:
                    feedback_value = feedback[index]
                    if feedback_value is None:
                        stage_reached = False
                        break
                    if abs(feedback_value - current_command[index]) > tolerance_rad:
                        stage_reached = False
                        break
                if stage_reached:
                    break
                if time.monotonic() >= deadline:
                    self.get_logger().warn(
                        f"Home stage timeout for joints: {', '.join(stage_names)}"
                    )
                    break
                time.sleep(0.02)

        for _ in range(10):
            if self._stop_event.is_set() or not rclpy.ok():
                break
            self._publish_command(current_command)
            time.sleep(0.02)
        self.get_logger().info("Home position sequence finished.")

    def _run_playback(self) -> None:
        try:
            if not self._wait_for_active_controllers():
                self.get_logger().error(
                    "Replay aborted: joint_state_broadcaster and "
                    "forward_position_controller did not both reach active state."
                )
                return

            if not self._wait_for_command_subscriber():
                self.get_logger().error(
                    f"Replay aborted: {self._command_topic} has no subscribers after "
                    f"{self._wait_for_command_sub_timeout_sec:.1f} s."
                )
                return

            if not self._wait_for_feedback():
                self.get_logger().error(
                    "Replay aborted: did not receive complete joint_states feedback "
                    f"within {self._wait_for_feedback_timeout_sec:.1f} s."
                )
                return

            current_command = self._snapshot_feedback()
            if self._start_delay_sec > 0.0:
                self.get_logger().info(
                    f"Replay will start in {self._start_delay_sec:.2f} s."
                )
                if not self._sleep_until(time.monotonic() + self._start_delay_sec):
                    return

            first_timestamp_ns = self._samples[0][0]
            playback_start = time.monotonic()
            self.get_logger().info(
                f"Starting replay of {len(self._samples)} samples to "
                f"{self._command_topic} at {self._rate_scale:.3f}x speed."
            )

            for timestamp_ns, sample in self._samples:
                target_time = playback_start + (
                    (timestamp_ns - first_timestamp_ns) / 1_000_000_000.0 / self._rate_scale
                )
                if not self._sleep_until(target_time):
                    return

                for joint_index, value in enumerate(sample):
                    if value is not None:
                        current_command[joint_index] = value
                self._publish_command(current_command)

            self.get_logger().info(f"Replay finished for bag {self._bag_path}.")
            if self._wait_for_ctrl_c():
                self._run_go_home_sequence()
                self._restore_terminal()
                rclpy.shutdown()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Replay failed: {exc}")
        finally:
            self._restore_terminal()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointTrajectoryBagReplay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
