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
from .keypoint_markers import KeypointMarkerConfig, KeypointMarkerPublisher, RawManusMarkerPublisher
from .manus_mapping import HandMetadata, MappingOptions, extract_hands_from_gloves, flatten_hand_input


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
    left_metadata: HandMetadata
    right_metadata: HandMetadata
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
        left_keypoints, right_keypoints, left_metadata, right_metadata = extract_hands_from_gloves(
            msg.gloves,
            self.mapping_options,
        )
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
            left_metadata=left_metadata,
            right_metadata=right_metadata,
            hand_input=hand_input,
        )


class ManusInputAdapterNode(Node):
    def __init__(self, config: ManusInputAdapterConfig) -> None:
        super().__init__("manus_input_adapter")
        self._adapter = ManusInputAdapter(config)
        self._config = config
        self._latest_raw_msg: Optional[ManusGloveRawArray] = None

        qos = get_sensor_data_qos()
        self._output_pub = self.create_publisher(Float32MultiArray, config.output_topic, qos)
        self._debug_left_pub = None
        self._debug_right_pub = None
        self._marker_pub = None
        self._raw_marker_pub = None
        if config.publish_debug_topics:
            self._debug_left_pub = self.create_publisher(Float32MultiArray, config.debug_left_topic, qos)
            self._debug_right_pub = self.create_publisher(Float32MultiArray, config.debug_right_topic, qos)
        if config.publish_debug_markers:
            marker_config = KeypointMarkerConfig(
                left_topic=config.left_marker_topic,
                right_topic=config.right_marker_topic,
                left_frame_id=config.left_marker_frame_id,
                right_frame_id=config.right_marker_frame_id,
                align_wrist_to_origin=config.marker_align_wrist_to_origin,
                point_scale=config.marker_point_scale,
                line_width=config.marker_line_width,
                lifetime_sec=config.marker_lifetime_sec,
            )
            self._marker_pub = KeypointMarkerPublisher(self, marker_config)
            self._raw_marker_pub = RawManusMarkerPublisher(self, marker_config)

        self.create_subscription(ManusGloveRawArray, config.input_topic, self._on_gloves_raw, qos)
        publish_rate_hz = max(float(config.publish_rate_hz), 1.0)
        self.create_timer(1.0 / publish_rate_hz, self._publish_latest_frame)
        self.get_logger().info(
            "Manus input adapter ready. "
            f"input={config.input_topic}, output={config.output_topic}, "
            f"include_left={str(config.include_left_hand).lower()}, "
            f"include_right={str(config.include_right_hand).lower()}, "
            f"publish_rate_hz={publish_rate_hz:.1f}"
        )
        if self._marker_pub is not None:
            self.get_logger().info(
                "Keypoint markers enabled. "
                f"left_topic={config.left_marker_topic}, right_topic={config.right_marker_topic}, "
                f"left_frame={config.left_marker_frame_id}, right_frame={config.right_marker_frame_id}, "
                f"align_wrist_to_origin={str(config.marker_align_wrist_to_origin).lower()}"
            )

    def _publish_array(self, publisher, values: np.ndarray) -> None:
        msg = Float32MultiArray()
        msg.data = np.asarray(values, dtype=np.float32).reshape(-1).tolist()
        publisher.publish(msg)

    def _on_gloves_raw(self, msg: ManusGloveRawArray) -> None:
        self._latest_raw_msg = msg

    def _publish_latest_frame(self) -> None:
        if self._latest_raw_msg is None:
            return
        adapted = self._adapter.adapt_message(self._latest_raw_msg)
        hand_input = adapted.hand_input
        if hand_input is not None and self._config.publish_hand_input:
            self._publish_array(self._output_pub, hand_input)
        if self._debug_left_pub is not None and adapted.left_keypoints is not None:
            self._publish_array(self._debug_left_pub, adapted.left_keypoints)
        if self._debug_right_pub is not None and adapted.right_keypoints is not None:
            self._publish_array(self._debug_right_pub, adapted.right_keypoints)
        if self._marker_pub is not None:
            self._marker_pub.publish(
                left_keypoints=adapted.left_keypoints,
                right_keypoints=adapted.right_keypoints,
                left_metadata=adapted.left_metadata,
                right_metadata=adapted.right_metadata,
            )
        if self._raw_marker_pub is not None:
            self._raw_marker_pub.publish(self._latest_raw_msg.gloves)


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
