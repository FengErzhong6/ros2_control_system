from __future__ import annotations

from dataclasses import dataclass

from .topic_relay_utils import normalize_topic, relay_topic_name


@dataclass(frozen=True)
class RelayTopicMapping:
    source_topic: str
    output_topic: str
    source_msg_type: str


_EXPLICIT_SOURCE_TYPES = {
    "/cam_high/cam_high/color/image_raw": "sensor_msgs/msg/Image",
    "/cam_left_wrist/image_raw": "sensor_msgs/msg/Image",
    "/cam_right_wrist/image_raw": "sensor_msgs/msg/Image",
    "/marvin/joint_states": "sensor_msgs/msg/JointState",
    "/left_hand/joint_states": "sensor_msgs/msg/JointState",
    "/right_hand/joint_states": "sensor_msgs/msg/JointState",
    "/tracker_teleop_controller/joint_command": "sensor_msgs/msg/JointState",
    "/left_hand/forward_position_controller/commands": "std_msgs/msg/Float64MultiArray",
    "/right_hand/forward_position_controller/commands": "std_msgs/msg/Float64MultiArray",
}


def resolve_source_msg_type(topic: str) -> str:
    normalized = normalize_topic(topic)
    explicit = _EXPLICIT_SOURCE_TYPES.get(normalized)
    if explicit is not None:
        return explicit
    if normalized.endswith("/image_raw") or "/color/image_raw" in normalized:
        return "sensor_msgs/msg/Image"
    if normalized.endswith("/joint_states") or normalized.endswith("/joint_command"):
        return "sensor_msgs/msg/JointState"
    if normalized.endswith("/commands"):
        return "std_msgs/msg/Float64MultiArray"
    raise RuntimeError(f"Unsupported record topic for relay mapping: {normalized}")


def relay_output_topic(source_topic: str, prefix: str) -> str:
    normalized = normalize_topic(source_topic)
    output = relay_topic_name(normalized, prefix)
    if resolve_source_msg_type(normalized) == "std_msgs/msg/Float64MultiArray":
        return f"{output}_stamped"
    return output


def build_relay_topic_mapping(source_topic: str, prefix: str) -> RelayTopicMapping:
    normalized = normalize_topic(source_topic)
    return RelayTopicMapping(
        source_topic=normalized,
        output_topic=relay_output_topic(normalized, prefix),
        source_msg_type=resolve_source_msg_type(normalized),
    )
