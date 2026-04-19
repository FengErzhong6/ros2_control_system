from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32MultiArray

from ament_index_python.packages import get_package_share_directory
from manus_system.msg import ManusGloveRawArray

from .config_types import ManusInputAdapterConfig, load_dataclass_from_yaml
from .manus_mapping import MappingOptions, extract_hands_from_gloves, flatten_hand_input


def get_sensor_data_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


@dataclass(slots=True)
class AdaptedFrame:
    left_keypoints: Optional[np.ndarray]
    right_keypoints: Optional[np.ndarray]
    hand_input: Optional[np.ndarray]


class ManusInputAdapter:
    def __init__(self, config: ManusInputAdapterConfig) -> None:
        self.config = config
        self.mapping_options = MappingOptions(
            flip_x=config.flip_x,
            flip_y=config.flip_y,
            flip_z=config.flip_z,
            expected_left_side_name=config.expected_left_side_name,
            expected_right_side_name=config.expected_right_side_name,
        )

    def adapt_message(self, msg: ManusGloveRawArray) -> AdaptedFrame:
        left_keypoints, right_keypoints = extract_hands_from_gloves(msg.gloves, self.mapping_options)
        hand_input = flatten_hand_input(
            left_keypoints,
            right_keypoints,
            include_left_hand=self.config.include_left_hand,
            include_right_hand=self.config.include_right_hand,
            drop_if_missing_hand=self.config.drop_if_missing_hand,
            right_first=self.config.right_first,
        )
        return AdaptedFrame(
            left_keypoints=left_keypoints,
            right_keypoints=right_keypoints,
            hand_input=hand_input,
        )


class ManusInputAdapterNode(Node):
    def __init__(self, config: ManusInputAdapterConfig) -> None:
        super().__init__("manus_input_adapter")
        self._adapter = ManusInputAdapter(config)
        self._config = config

        qos = get_sensor_data_qos()
        self._output_pub = self.create_publisher(Float32MultiArray, config.output_topic, qos)
        self._debug_left_pub = None
        self._debug_right_pub = None
        if config.publish_debug_topics:
            self._debug_left_pub = self.create_publisher(Float32MultiArray, config.debug_left_topic, qos)
            self._debug_right_pub = self.create_publisher(Float32MultiArray, config.debug_right_topic, qos)

        self.create_subscription(ManusGloveRawArray, config.input_topic, self._on_gloves_raw, qos)
        self.get_logger().info(
            "Manus input adapter ready. "
            f"input={config.input_topic}, output={config.output_topic}, "
            f"include_left={str(config.include_left_hand).lower()}, "
            f"include_right={str(config.include_right_hand).lower()}"
        )

    def _publish_array(self, publisher, values: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = np.asarray(values, dtype=np.float32).reshape(-1).tolist()
        publisher.publish(msg)

    def _on_gloves_raw(self, msg: ManusGloveRawArray) -> None:
        adapted = self._adapter.adapt_message(msg)
        if adapted.hand_input is not None and self._config.publish_hand_input:
            self._publish_array(self._output_pub, adapted.hand_input)
        if self._debug_left_pub is not None and adapted.left_keypoints is not None:
            self._publish_array(self._debug_left_pub, adapted.left_keypoints)
        if self._debug_right_pub is not None and adapted.right_keypoints is not None:
            self._publish_array(self._debug_right_pub, adapted.right_keypoints)


def _default_config_path() -> Path:
    return Path(get_package_share_directory("wujihand_system")) / "config" / "manus_input.yaml"


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Adapt Manus raw gloves to /hand_input.")
    parser.add_argument("-c", "--config", default=str(_default_config_path()))
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "manus_input_adapter"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)
    config = load_dataclass_from_yaml(ManusInputAdapterConfig, args.config)

    rclpy.init(args=raw_argv)
    node = ManusInputAdapterNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
