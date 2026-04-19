from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Dict, Mapping, Type, TypeVar

import yaml

T = TypeVar("T")


def load_yaml_mapping(path: str | Path) -> Dict[str, Any]:
    resolved = Path(path).expanduser().resolve()
    with resolved.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    if not isinstance(raw, dict):
        raise ValueError(f"Expected a mapping at the root of config file: {resolved}")
    return raw


def load_dataclass_from_mapping(cls: Type[T], raw: Mapping[str, Any]) -> T:
    valid_fields = {field.name for field in fields(cls)}
    data = {key: value for key, value in raw.items() if key in valid_fields}
    return cls(**data)


def load_dataclass_from_yaml(cls: Type[T], path: str | Path) -> T:
    return load_dataclass_from_mapping(cls, load_yaml_mapping(path))


@dataclass(slots=True)
class ManusInputAdapterConfig:
    input_topic: str = "/manus_raw_publisher_node/gloves_raw"
    output_topic: str = "/hand_input"
    publish_hand_input: bool = True
    publish_debug_topics: bool = False
    debug_left_topic: str = "/hand_input_debug/left"
    debug_right_topic: str = "/hand_input_debug/right"
    include_left_hand: bool = True
    include_right_hand: bool = True
    drop_if_missing_hand: bool = True
    right_first: bool = True
    flip_x: bool = False
    flip_y: bool = True
    flip_z: bool = False
    expected_left_side_name: str = "left"
    expected_right_side_name: str = "right"
    single_hand_fallback_side: str = "right"


@dataclass(slots=True)
class WujihandRetargetBridgeConfig:
    hand_input_topic: str = "/hand_input"
    left_command_topic: str = "/left/forward_position_controller/commands"
    right_command_topic: str = "/right/forward_position_controller/commands"
    left_retarget_config: str = ""
    right_retarget_config: str = ""
    enable_left_hand: bool = True
    enable_right_hand: bool = True
    require_both_hands: bool = False
    hold_last_command_on_missing_input: bool = True
    command_timeout_sec: float = 0.25
    publish_debug_qpos: bool = False
    left_debug_qpos_topic: str = "/left/wujihand_retarget/qpos_debug"
    right_debug_qpos_topic: str = "/right/wujihand_retarget/qpos_debug"
    single_hand_fallback_side: str = "right"


@dataclass(slots=True)
class WujihandManusPipelineConfig:
    input_topic: str = "/manus_raw_publisher_node/gloves_raw"
    output_topic: str = "/hand_input"
    publish_hand_input: bool = True
    publish_debug_topics: bool = False
    debug_left_topic: str = "/hand_input_debug/left"
    debug_right_topic: str = "/hand_input_debug/right"
    include_left_hand: bool = True
    include_right_hand: bool = True
    drop_if_missing_hand: bool = True
    right_first: bool = True
    flip_x: bool = False
    flip_y: bool = True
    flip_z: bool = False
    expected_left_side_name: str = "left"
    expected_right_side_name: str = "right"
    single_hand_fallback_side: str = "right"
    left_command_topic: str = "/left/forward_position_controller/commands"
    right_command_topic: str = "/right/forward_position_controller/commands"
    left_retarget_config: str = ""
    right_retarget_config: str = ""
    enable_left_hand: bool = True
    enable_right_hand: bool = True
    require_both_hands: bool = False
    hold_last_command_on_missing_input: bool = True
    command_timeout_sec: float = 0.25
    publish_debug_qpos: bool = False
    left_debug_qpos_topic: str = "/left/wujihand_retarget/qpos_debug"
    right_debug_qpos_topic: str = "/right/wujihand_retarget/qpos_debug"
    profiling_enabled: bool = True
    profiling_log_period_sec: float = 5.0

    def adapter_config(self) -> ManusInputAdapterConfig:
        return ManusInputAdapterConfig(
            input_topic=self.input_topic,
            output_topic=self.output_topic,
            publish_hand_input=self.publish_hand_input,
            publish_debug_topics=self.publish_debug_topics,
            debug_left_topic=self.debug_left_topic,
            debug_right_topic=self.debug_right_topic,
            include_left_hand=self.include_left_hand,
            include_right_hand=self.include_right_hand,
            drop_if_missing_hand=self.drop_if_missing_hand,
            right_first=self.right_first,
            flip_x=self.flip_x,
            flip_y=self.flip_y,
            flip_z=self.flip_z,
            expected_left_side_name=self.expected_left_side_name,
            expected_right_side_name=self.expected_right_side_name,
            single_hand_fallback_side=self.single_hand_fallback_side,
        )

    def bridge_config(self) -> WujihandRetargetBridgeConfig:
        return WujihandRetargetBridgeConfig(
            hand_input_topic=self.output_topic,
            left_command_topic=self.left_command_topic,
            right_command_topic=self.right_command_topic,
            left_retarget_config=self.left_retarget_config,
            right_retarget_config=self.right_retarget_config,
            enable_left_hand=self.enable_left_hand,
            enable_right_hand=self.enable_right_hand,
            require_both_hands=self.require_both_hands,
            hold_last_command_on_missing_input=self.hold_last_command_on_missing_input,
            command_timeout_sec=self.command_timeout_sec,
            publish_debug_qpos=self.publish_debug_qpos,
            left_debug_qpos_topic=self.left_debug_qpos_topic,
            right_debug_qpos_topic=self.right_debug_qpos_topic,
            single_hand_fallback_side=self.single_hand_fallback_side,
        )
