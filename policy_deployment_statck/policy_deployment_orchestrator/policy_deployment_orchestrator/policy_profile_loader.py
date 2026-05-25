from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

from policy_deployment_interfaces.msg import PolicyProfile

from .models import (
    ActionSliceSpec,
    CommandTargetSpec,
    ImageInputSpec,
    JointGroupSpec,
    PolicyProfileSpec,
)


def discover_policy_profiles(path: Path | None) -> dict[str, PolicyProfileSpec]:
    if path is None or not path.exists():
        return {}
    data = _load_yaml_mapping(path)
    raw_profiles = data.get("profiles", {})
    if isinstance(raw_profiles, list):
        profile_items = []
        for item in raw_profiles:
            if not isinstance(item, dict) or "profile_id" not in item:
                raise RuntimeError(f"Expected profile_id in each profile entry: {path}")
            profile_items.append((str(item["profile_id"]), item))
    elif isinstance(raw_profiles, dict):
        profile_items = [(str(profile_id), raw) for profile_id, raw in raw_profiles.items()]
    else:
        raise RuntimeError(f"profiles must be a mapping or list in {path}")

    profiles: dict[str, PolicyProfileSpec] = {}
    for profile_id, raw_profile in profile_items:
        if not isinstance(raw_profile, dict):
            raise RuntimeError(f"Profile {profile_id!r} must be a mapping in {path}")
        profiles[profile_id] = _parse_profile(profile_id, raw_profile)
    return profiles


def to_msg(profile: PolicyProfileSpec) -> PolicyProfile:
    msg = PolicyProfile()
    msg.profile_id = profile.profile_id
    msg.title = profile.title
    msg.server_host = profile.server_host
    msg.server_port = int(profile.server_port)
    msg.default_prompt = profile.default_prompt
    msg.action_dim = int(profile.action_dim)
    msg.action_horizon = int(profile.action_horizon)
    msg.action_space = profile.action_space
    msg.image_inputs = [f"{item.name}:{item.topic}" for item in profile.image_inputs]
    msg.state_source = "right_arm+right_hand joint_states"
    msg.frozen_groups = list(profile.frozen_groups)
    msg.summary = profile.summary
    return msg


def _parse_profile(profile_id: str, raw: dict[str, Any]) -> PolicyProfileSpec:
    state = _mapping(raw.get("state", {}), f"profile {profile_id}.state")
    commands = _mapping(raw.get("commands", {}), f"profile {profile_id}.commands")
    action_mapping = _mapping(raw.get("action_mapping", {}), f"profile {profile_id}.action_mapping")

    image_inputs = tuple(
        _build_image_input_spec(profile_id, item)
        for item in _sequence(raw.get("image_inputs", []), f"profile {profile_id}.image_inputs")
    )

    action_slices = tuple(
        ActionSliceSpec(
            name=str(name),
            start=int(_mapping(spec, f"profile {profile_id}.action_mapping.{name}")["start"]),
            length=int(_mapping(spec, f"profile {profile_id}.action_mapping.{name}")["length"]),
            mode=str(_mapping(spec, f"profile {profile_id}.action_mapping.{name}").get("mode", "absolute")),
        )
        for name, spec in action_mapping.items()
    )

    return PolicyProfileSpec(
        profile_id=profile_id,
        title=str(raw.get("title", profile_id)),
        server_host=str(raw.get("server_host", "127.0.0.1")),
        server_port=int(raw.get("server_port", 8000)),
        default_prompt=str(raw.get("default_prompt", "")),
        action_dim=int(raw.get("action_dim", 27)),
        action_horizon=int(raw.get("action_horizon", 10)),
        action_space=str(raw.get("action_space", "joint_position")),
        open_loop_horizon=int(raw.get("open_loop_horizon", 8)),
        control_rate_hz=float(raw.get("control_rate_hz", 10.0)),
        image_inputs=image_inputs,
        right_arm_state=_parse_joint_group(state, "right_arm"),
        right_hand_state=_parse_joint_group(state, "right_hand"),
        marvin_command=_parse_command_target(commands, "marvin"),
        right_hand_command=_parse_command_target(commands, "right_hand"),
        action_slices=action_slices,
        frozen_groups=tuple(str(item) for item in raw.get("frozen_groups", [])),
        summary=str(raw.get("summary", "")),
        use_local_inference=bool(raw.get("use_local_inference", False)),
        local_policy_config_name=str(raw.get("local_policy_config_name", "")),
        local_policy_checkpoint_dir=str(raw.get("local_policy_checkpoint_dir", "")),
    )


def _build_image_input_spec(profile_id: str, item: dict[str, Any]) -> ImageInputSpec:
    anchor = _optional_text(item.get("square_crop_anchor"))
    if anchor is not None:
        anchor = anchor.lower()
        if anchor not in {"center", "right", "topic"}:
            raise RuntimeError(
                f"Unsupported square_crop_anchor for profile {profile_id}: {anchor!r}"
            )
    return ImageInputSpec(
        name=str(item["name"]),
        topic=str(item["topic"]),
        required=bool(item.get("required", True)),
        square_crop_anchor=anchor,
        resize_size=_parse_size(
            item.get("resize_size"),
            f"profile {profile_id}.image_inputs.{item.get('name', 'image')}.resize_size",
        ),
    )


def _parse_joint_group(raw_state: dict[str, Any], name: str) -> JointGroupSpec:
    raw_group = _mapping(raw_state.get(name, {}), f"state.{name}")
    return JointGroupSpec(
        name=name,
        topic=str(raw_group["topic"]),
        joint_names=tuple(str(item) for item in raw_group.get("joint_names", [])),
    )


def _parse_command_target(raw_commands: dict[str, Any], name: str) -> CommandTargetSpec:
    raw_target = _mapping(raw_commands.get(name, {}), f"commands.{name}")
    return CommandTargetSpec(
        name=name,
        topic=str(raw_target["topic"]),
        joint_names=tuple(str(item) for item in raw_target.get("joint_names", [])),
        home=tuple(float(item) for item in raw_target.get("home", [])),
    )


def _load_yaml_mapping(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    return _mapping(data, str(path))


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise RuntimeError(f"Expected mapping for {label}, got {type(value).__name__}")
    return value


def _sequence(value: Any, label: str) -> list[dict[str, Any]]:
    if value is None:
        return []
    if not isinstance(value, list):
        raise RuntimeError(f"Expected sequence for {label}, got {type(value).__name__}")
    for item in value:
        if not isinstance(item, dict):
            raise RuntimeError(f"Expected mapping entries for {label}")
    return value


def _optional_text(value: Any) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _parse_size(value: Any, label: str) -> tuple[int, int] | None:
    if value is None:
        return None
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise RuntimeError(f"Expected [width, height] for {label}")
    width = int(value[0])
    height = int(value[1])
    if width <= 0 or height <= 0:
        raise RuntimeError(f"Expected positive size for {label}")
    return (width, height)
