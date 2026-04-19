from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
import traceback
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray, Float64MultiArray

from manus_system.msg import ManusGloveRawArray

from .config_types import WujihandManusPipelineConfig, load_dataclass_from_yaml
from .manus_input_adapter import ManusInputAdapter
from .manus_mapping import is_valid_hand_keypoints
from .profiling import PeriodicProfiler
from .wujihand_retarget_bridge import WujihandRetargetBridge


def get_sensor_data_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class WujihandManusPipelineNode(Node):
    def __init__(self, config: WujihandManusPipelineConfig) -> None:
        super().__init__("wujihand_manus_pipeline")
        self._config = config
        self._adapter = ManusInputAdapter(config.adapter_config())
        self._bridge = WujihandRetargetBridge(config.bridge_config())
        self._profiler = PeriodicProfiler(config.profiling_enabled, config.profiling_log_period_sec, self.get_logger())
        self._last_input_monotonic = 0.0
        self._timeout_action_sent = False

        qos = get_sensor_data_qos()
        self._left_pub = self.create_publisher(Float64MultiArray, config.left_command_topic, qos)
        self._right_pub = self.create_publisher(Float64MultiArray, config.right_command_topic, qos)
        self._hand_input_pub = None
        self._debug_left_pub = None
        self._debug_right_pub = None
        self._left_qpos_debug_pub = None
        self._right_qpos_debug_pub = None

        if config.publish_hand_input:
            self._hand_input_pub = self.create_publisher(Float32MultiArray, config.output_topic, qos)
        if config.publish_debug_topics:
            self._debug_left_pub = self.create_publisher(Float32MultiArray, config.debug_left_topic, qos)
            self._debug_right_pub = self.create_publisher(Float32MultiArray, config.debug_right_topic, qos)
        if config.publish_debug_qpos:
            self._left_qpos_debug_pub = self.create_publisher(Float64MultiArray, config.left_debug_qpos_topic, qos)
            self._right_qpos_debug_pub = self.create_publisher(Float64MultiArray, config.right_debug_qpos_topic, qos)

        self.create_subscription(ManusGloveRawArray, config.input_topic, self._on_gloves_raw, qos)
        self.create_timer(max(config.command_timeout_sec / 2.0, 0.05), self._on_timeout_timer)
        self.get_logger().info(
            "Wujihand Manus pipeline ready. "
            f"raw={config.input_topic}, "
            f"left_cmd={config.left_command_topic}, "
            f"right_cmd={config.right_command_topic}, "
            f"publish_hand_input={str(config.publish_hand_input).lower()}"
        )

    def _publish_float32(self, publisher, values: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = np.asarray(values, dtype=np.float32).reshape(-1).tolist()
        publisher.publish(msg)

    def _publish_float64(self, publisher, values: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = np.asarray(values, dtype=np.float64).reshape(-1).tolist()
        publisher.publish(msg)

    def _publish_zero_commands(self) -> None:
        zeros = np.zeros(20, dtype=np.float64)
        if self._config.enable_left_hand:
            self._publish_float64(self._left_pub, zeros)
        if self._config.enable_right_hand:
            self._publish_float64(self._right_pub, zeros)

    def _on_gloves_raw(self, msg: ManusGloveRawArray) -> None:
        try:
            self._last_input_monotonic = time.monotonic()
            self._timeout_action_sent = False

            with self._profiler.measure("adapt_total"):
                adapted = self._adapter.adapt_message(msg)

            if self._hand_input_pub is not None and adapted.hand_input is not None:
                self._publish_float32(self._hand_input_pub, adapted.hand_input)
            if self._debug_left_pub is not None and adapted.left_keypoints is not None:
                self._publish_float32(self._debug_left_pub, adapted.left_keypoints)
            if self._debug_right_pub is not None and adapted.right_keypoints is not None:
                self._publish_float32(self._debug_right_pub, adapted.right_keypoints)

            left_keypoints = adapted.left_keypoints if is_valid_hand_keypoints(adapted.left_keypoints) else None
            right_keypoints = adapted.right_keypoints if is_valid_hand_keypoints(adapted.right_keypoints) else None

            if self._config.require_both_hands:
                if self._config.enable_left_hand and left_keypoints is None:
                    self._profiler.maybe_log()
                    return
                if self._config.enable_right_hand and right_keypoints is None:
                    self._profiler.maybe_log()
                    return

            with self._profiler.measure("retarget_total"):
                output = self._bridge.retarget(left_keypoints, right_keypoints)

            if self._config.enable_left_hand and output.left_command is not None:
                self._publish_float64(self._left_pub, output.left_command)
                if self._left_qpos_debug_pub is not None:
                    self._publish_float64(self._left_qpos_debug_pub, output.left_command)
            if self._config.enable_right_hand and output.right_command is not None:
                self._publish_float64(self._right_pub, output.right_command)
                if self._right_qpos_debug_pub is not None:
                    self._publish_float64(self._right_qpos_debug_pub, output.right_command)

            self._profiler.maybe_log()
        except Exception:  # pragma: no cover - runtime guard
            self.get_logger().error(
                "Pipeline callback failed:\n" + traceback.format_exc()
            )

    def _on_timeout_timer(self) -> None:
        if self._config.hold_last_command_on_missing_input:
            return
        if self._config.command_timeout_sec <= 0.0:
            return
        if self._last_input_monotonic <= 0.0:
            return
        if self._timeout_action_sent:
            return
        if (time.monotonic() - self._last_input_monotonic) < self._config.command_timeout_sec:
            return
        self._publish_zero_commands()
        self._timeout_action_sent = True


def _default_config_path() -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("wujihand_system")) / "config" / "manus_input.yaml"


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="High-performance Manus -> Wuji hand teleop pipeline.")
    parser.add_argument("-c", "--config", default=str(_default_config_path()))
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "wujihand_manus_pipeline"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)
    config = load_dataclass_from_yaml(WujihandManusPipelineConfig, args.config)

    rclpy.init(args=raw_argv)
    node = WujihandManusPipelineNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
