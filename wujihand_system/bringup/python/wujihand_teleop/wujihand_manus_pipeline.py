from __future__ import annotations

import argparse
from dataclasses import asdict
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
from std_srvs.srv import SetBool

from manus_system.msg import ManusGloveRawArray

from .config_types import WujihandManusPipelineConfig, load_dataclass_from_yaml
from .manus_input_adapter import ManusInputAdapter
from .keypoint_markers import KeypointMarkerConfig, KeypointMarkerPublisher, RawManusMarkerPublisher
from .manus_mapping import is_valid_hand_keypoints
from .profiling import PeriodicProfiler
from .recording import AlignedFrameRecorder, build_aligned_record, raw_message_stamp_ns
from .wujihand_retarget_bridge import WujihandRetargetBridge


def get_sensor_data_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def get_command_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class WujihandManusPipelineNode(Node):
    def __init__(self, config: WujihandManusPipelineConfig) -> None:
        super().__init__("wujihand_manus_pipeline")
        self._config = config
        self._enabled = bool(self.declare_parameter("start_enabled", False).value)
        self._adapter = ManusInputAdapter(config.adapter_config())
        self._bridge = WujihandRetargetBridge(config.bridge_config())
        self._profiler = PeriodicProfiler(
            config.profiling_enabled,
            config.profiling_log_period_sec,
            self.get_logger(),
        )
        self._last_input_monotonic = 0.0
        self._timeout_action_sent = False
        self._latest_raw_msg: Optional[ManusGloveRawArray] = None
        self._frame_index = 0
        self._last_recorded_raw_stamp_ns: Optional[int] = None
        self._recorder: Optional[AlignedFrameRecorder] = None
        if config.record_aligned_data:
            self._recorder = AlignedFrameRecorder(
                output_root=config.record_output_dir,
                chunk_size=config.record_chunk_size,
                queue_size=config.record_queue_size,
                flush_period_sec=config.record_flush_period_sec,
                logger=self.get_logger(),
                config_snapshot=asdict(config),
            )

        sensor_qos = get_sensor_data_qos()
        command_qos = get_command_qos()
        self._left_pub = self.create_publisher(
            Float64MultiArray,
            config.left_command_topic,
            command_qos,
        )
        self._right_pub = self.create_publisher(
            Float64MultiArray,
            config.right_command_topic,
            command_qos,
        )
        self._hand_input_pub = None
        self._debug_left_pub = None
        self._debug_right_pub = None
        self._marker_pub = None
        self._raw_marker_pub = None
        self._left_qpos_debug_pub = None
        self._right_qpos_debug_pub = None

        if config.publish_hand_input:
            self._hand_input_pub = self.create_publisher(
                Float32MultiArray,
                config.output_topic,
                sensor_qos,
            )
        if config.publish_debug_topics:
            self._debug_left_pub = self.create_publisher(
                Float32MultiArray,
                config.debug_left_topic,
                sensor_qos,
            )
            self._debug_right_pub = self.create_publisher(
                Float32MultiArray,
                config.debug_right_topic,
                sensor_qos,
            )
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
        if config.publish_debug_qpos:
            self._left_qpos_debug_pub = self.create_publisher(
                Float64MultiArray,
                config.left_debug_qpos_topic,
                command_qos,
            )
            self._right_qpos_debug_pub = self.create_publisher(
                Float64MultiArray,
                config.right_debug_qpos_topic,
                command_qos,
            )

        self.create_service(SetBool, "~/set_enabled", self._handle_set_enabled)
        self.create_subscription(
            ManusGloveRawArray,
            config.input_topic,
            self._on_gloves_raw,
            sensor_qos,
        )
        publish_rate_hz = max(float(config.publish_rate_hz), 1.0)
        self.create_timer(1.0 / publish_rate_hz, self._process_latest_frame)
        self.create_timer(max(config.command_timeout_sec / 2.0, 0.05), self._on_timeout_timer)
        self.get_logger().info(
            "Wujihand Manus pipeline ready. "
            f"raw={config.input_topic}, "
            f"left_cmd={config.left_command_topic}, "
            f"right_cmd={config.right_command_topic}, "
            f"enabled={str(self._enabled).lower()}, "
            f"publish_rate_hz={publish_rate_hz:.1f}"
        )
        if self._marker_pub is not None:
            self.get_logger().info(
                "Keypoint markers enabled. "
                f"left_topic={config.left_marker_topic}, right_topic={config.right_marker_topic}, "
                f"left_frame={config.left_marker_frame_id}, right_frame={config.right_marker_frame_id}, "
                f"align_wrist_to_origin={str(config.marker_align_wrist_to_origin).lower()}"
            )

    def _handle_set_enabled(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        self._enabled = bool(request.data)
        self._timeout_action_sent = False
        response.success = True
        response.message = "enabled" if self._enabled else "disabled"
        self.get_logger().info(f"Pipeline {response.message}.")
        return response

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
            self._latest_raw_msg = msg
        except Exception:
            self.get_logger().error("Pipeline callback failed:\n" + traceback.format_exc())

    def _process_latest_frame(self) -> None:
        try:
            if not self._enabled or self._latest_raw_msg is None:
                return

            with self._profiler.measure("adapt_total"):
                adapted = self._adapter.adapt_message(self._latest_raw_msg)
            hand_input = adapted.hand_input

            if self._hand_input_pub is not None and hand_input is not None:
                self._publish_float32(self._hand_input_pub, hand_input)
            if self._debug_left_pub is not None and adapted.left_keypoints is not None:
                self._publish_float32(self._debug_left_pub, adapted.left_keypoints)
            if self._debug_right_pub is not None and adapted.right_keypoints is not None:
                self._publish_float32(self._debug_right_pub, adapted.right_keypoints)
            if self._marker_pub is not None:
                self._marker_pub.publish(
                    left_keypoints=adapted.left_keypoints,
                    right_keypoints=adapted.right_keypoints,
                    left_metadata=adapted.left_metadata,
                    right_metadata=adapted.right_metadata,
                )
            if self._raw_marker_pub is not None:
                self._raw_marker_pub.publish(self._latest_raw_msg.gloves)

            left_keypoints = adapted.left_keypoints
            right_keypoints = adapted.right_keypoints
            if self._config.validate_keypoints:
                left_keypoints = left_keypoints if is_valid_hand_keypoints(left_keypoints) else None
                right_keypoints = right_keypoints if is_valid_hand_keypoints(right_keypoints) else None

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

            raw_stamp_ns = raw_message_stamp_ns(self._latest_raw_msg)
            if self._recorder is not None and raw_stamp_ns != self._last_recorded_raw_stamp_ns:
                self._recorder.record(
                    build_aligned_record(
                        frame_index=self._frame_index,
                        raw_msg=self._latest_raw_msg,
                        hand_input=hand_input,
                        left_keypoints=adapted.left_keypoints,
                        right_keypoints=adapted.right_keypoints,
                        left_qpos=output.left_command,
                        right_qpos=output.right_command,
                    )
                )
                self._frame_index += 1
                self._last_recorded_raw_stamp_ns = raw_stamp_ns

            self._profiler.maybe_log()
        except Exception:
            self.get_logger().error("Pipeline processing failed:\n" + traceback.format_exc())

    def _on_timeout_timer(self) -> None:
        if not self._enabled:
            return
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

    def destroy_node(self) -> bool:
        if self._recorder is not None:
            self._recorder.close()
            self._recorder = None
        return super().destroy_node()


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
        if rclpy.ok():
            rclpy.shutdown()
