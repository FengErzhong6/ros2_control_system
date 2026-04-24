from __future__ import annotations

import json

from data_collection_interfaces.msg import StampedFloat64MultiArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray

from .topic_relay_utils import TopicRelaySpec, clone_message_with_stamp, normalize_topic


_MESSAGE_TYPES = {
    "sensor_msgs/msg/Image": (Image, None),
    "sensor_msgs/msg/JointState": (JointState, None),
    "std_msgs/msg/Float64MultiArray": (Float64MultiArray, StampedFloat64MultiArray),
}


class TimestampRelayNode(Node):
    def __init__(self) -> None:
        super().__init__("data_collection_timestamp_relay")

        raw_specs = self.declare_parameter("topic_specs", "[]").value
        self._topic_specs = self._parse_specs(raw_specs)
        self._relay_publishers = {}
        self._relay_subscriptions = []

        for spec in self._topic_specs:
            publisher_type = spec.msg_type if spec.rewrite_header_stamp else spec.output_msg_type
            publisher = self.create_publisher(
                publisher_type,
                spec.output_topic,
                qos_profile_sensor_data,
            )
            self._relay_publishers[spec.output_topic] = publisher
            subscription = self.create_subscription(
                spec.msg_type,
                spec.source_topic,
                self._make_callback(spec),
                qos_profile_sensor_data,
            )
            self._relay_subscriptions.append(subscription)

        joined = ", ".join(
            f"{spec.source_topic}->{spec.output_topic}" for spec in self._topic_specs
        )
        self.get_logger().info(
            f"Timestamp relay active for {len(self._topic_specs)} topics: {joined}"
        )

    def _parse_specs(self, raw_specs: object) -> list[TopicRelaySpec]:
        if isinstance(raw_specs, str):
            parsed = json.loads(raw_specs)
        else:
            parsed = raw_specs
        if not isinstance(parsed, list):
            raise RuntimeError("topic_specs must be a JSON array.")

        specs: list[TopicRelaySpec] = []
        seen_outputs: set[str] = set()
        for item in parsed:
            if not isinstance(item, dict):
                raise RuntimeError("Each topic spec must be a JSON object.")
            source_topic = normalize_topic(str(item.get("source_topic", "")))
            output_topic = normalize_topic(str(item.get("output_topic", "")))
            msg_type_name = str(item.get("msg_type", "")).strip()
            message_info = _MESSAGE_TYPES.get(msg_type_name)
            if message_info is None:
                raise RuntimeError(f"Unsupported relay message type: {msg_type_name}")
            msg_type, output_msg_type = message_info
            if output_topic in seen_outputs:
                raise RuntimeError(f"Duplicate relay output topic: {output_topic}")
            seen_outputs.add(output_topic)
            specs.append(
                TopicRelaySpec(
                    source_topic=source_topic,
                    output_topic=output_topic,
                    msg_type=msg_type,
                    output_msg_type=output_msg_type,
                    rewrite_header_stamp=output_msg_type is None,
                )
            )
        if not specs:
            raise RuntimeError("topic_specs must not be empty.")
        return specs

    def _make_callback(self, spec: TopicRelaySpec):
        publisher = self._relay_publishers[spec.output_topic]

        def _callback(message) -> None:
            stamp = self.get_clock().now().to_msg()
            if spec.rewrite_header_stamp:
                publisher.publish(clone_message_with_stamp(message, stamp))
                return
            wrapped = spec.output_msg_type()
            wrapped.header.stamp = stamp
            wrapped.data = list(message.data)
            publisher.publish(wrapped)

        return _callback


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = TimestampRelayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
