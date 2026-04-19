from __future__ import annotations

import argparse
from dataclasses import dataclass
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

from ament_index_python.packages import get_package_share_directory

from .config_types import WujihandRetargetBridgeConfig, load_dataclass_from_yaml
from .manus_mapping import is_valid_hand_keypoints, split_hand_input


def get_sensor_data_qos() -> QoSProfile:
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


def _load_retargeter_class():
    try:
        from wuji_retargeting import Retargeter
    except Exception as exc:  # pragma: no cover - runtime dependency check
        raise RuntimeError(
            "Failed to import wuji_retargeting. Ensure vendored third_party and its "
            "runtime dependencies (numpy, nlopt, pin, scipy, yaml) are available."
        ) from exc
    return Retargeter


def _default_retarget_config(side: str) -> Path:
    share_dir = Path(get_package_share_directory("wujihand_system"))
    return share_dir / "config" / f"retarget_manus_{side}.yaml"


@dataclass(slots=True)
class RetargetOutput:
    left_command: Optional[np.ndarray]
    right_command: Optional[np.ndarray]


class WujihandRetargetBridge:
    def __init__(self, config: WujihandRetargetBridgeConfig) -> None:
        self.config = config
        self._left_retargeter = self._create_retargeter("left") if config.enable_left_hand else None
        self._right_retargeter = self._create_retargeter("right") if config.enable_right_hand else None
        self._last_left_command: Optional[np.ndarray] = None
        self._last_right_command: Optional[np.ndarray] = None

    def _create_retargeter(self, side: str):
        config_path = self.config.left_retarget_config if side == "left" else self.config.right_retarget_config
        resolved = Path(config_path).expanduser() if config_path else _default_retarget_config(side)
        Retargeter = _load_retargeter_class()
        return Retargeter.from_yaml(str(resolved), side)

    def retarget(
        self,
        left_keypoints: Optional[np.ndarray],
        right_keypoints: Optional[np.ndarray],
    ) -> RetargetOutput:
        left_command = None
        right_command = None

        if left_keypoints is not None and self._left_retargeter is not None:
            left_command = np.asarray(self._left_retargeter.retarget(left_keypoints), dtype=np.float64).reshape(-1)
            self._last_left_command = left_command
        elif left_keypoints is None and self.config.hold_last_command_on_missing_input:
            left_command = self._last_left_command

        if right_keypoints is not None and self._right_retargeter is not None:
            right_command = np.asarray(self._right_retargeter.retarget(right_keypoints), dtype=np.float64).reshape(-1)
            self._last_right_command = right_command
        elif right_keypoints is None and self.config.hold_last_command_on_missing_input:
            right_command = self._last_right_command

        return RetargetOutput(left_command=left_command, right_command=right_command)


class WujihandRetargetBridgeNode(Node):
    def __init__(self, config: WujihandRetargetBridgeConfig) -> None:
        super().__init__("wujihand_retarget_bridge")
        self._config = config
        self._bridge = WujihandRetargetBridge(config)
        self._last_input_monotonic = 0.0
        self._timeout_action_sent = False

        qos = get_sensor_data_qos()
        self._left_pub = self.create_publisher(Float64MultiArray, config.left_command_topic, qos)
        self._right_pub = self.create_publisher(Float64MultiArray, config.right_command_topic, qos)
        self._left_debug_pub = None
        self._right_debug_pub = None
        if config.publish_debug_qpos:
            self._left_debug_pub = self.create_publisher(Float64MultiArray, config.left_debug_qpos_topic, qos)
            self._right_debug_pub = self.create_publisher(Float64MultiArray, config.right_debug_qpos_topic, qos)

        self.create_subscription(Float32MultiArray, config.hand_input_topic, self._on_hand_input, qos)
        self.create_timer(max(config.command_timeout_sec / 2.0, 0.05), self._on_timeout_timer)
        self.get_logger().info(
            "Wujihand retarget bridge ready. "
            f"input={config.hand_input_topic}, "
            f"left_cmd={config.left_command_topic}, "
            f"right_cmd={config.right_command_topic}"
        )

    def _publish_command(self, publisher, values: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = np.asarray(values, dtype=np.float64).reshape(-1).tolist()
        publisher.publish(msg)

    def _publish_debug(self, publisher, values: np.ndarray) -> None:
        if publisher is not None:
            self._publish_command(publisher, values)

    def _publish_zero_commands(self) -> None:
        zeros = np.zeros(20, dtype=np.float64)
        if self._config.enable_left_hand:
            self._publish_command(self._left_pub, zeros)
        if self._config.enable_right_hand:
            self._publish_command(self._right_pub, zeros)

    def _on_hand_input(self, msg: Float32MultiArray) -> None:
        try:
            self._last_input_monotonic = time.monotonic()
            self._timeout_action_sent = False

            left_keypoints, right_keypoints = split_hand_input(
                msg.data,
                enable_left_hand=self._config.enable_left_hand,
                enable_right_hand=self._config.enable_right_hand,
                single_hand_fallback_side=self._config.single_hand_fallback_side,
            )

            if not is_valid_hand_keypoints(left_keypoints):
                left_keypoints = None
            if not is_valid_hand_keypoints(right_keypoints):
                right_keypoints = None

            if self._config.require_both_hands:
                if self._config.enable_left_hand and left_keypoints is None:
                    return
                if self._config.enable_right_hand and right_keypoints is None:
                    return

            output = self._bridge.retarget(left_keypoints, right_keypoints)

            if self._config.enable_left_hand and output.left_command is not None:
                self._publish_command(self._left_pub, output.left_command)
                self._publish_debug(self._left_debug_pub, output.left_command)
            if self._config.enable_right_hand and output.right_command is not None:
                self._publish_command(self._right_pub, output.right_command)
                self._publish_debug(self._right_debug_pub, output.right_command)
        except Exception:  # pragma: no cover - runtime guard
            self.get_logger().error(
                "Retarget bridge callback failed:\n" + traceback.format_exc()
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
    return Path(get_package_share_directory("wujihand_system")) / "config" / "manus_input.yaml"


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Retarget /hand_input to Wuji hand commands.")
    parser.add_argument("-c", "--config", default=str(_default_config_path()))
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "wujihand_retarget_bridge"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)
    config = load_dataclass_from_yaml(WujihandRetargetBridgeConfig, args.config)

    rclpy.init(args=raw_argv)
    node = WujihandRetargetBridgeNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
