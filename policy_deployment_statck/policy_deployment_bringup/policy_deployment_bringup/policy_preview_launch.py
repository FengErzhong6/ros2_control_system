from __future__ import annotations

import json
from pathlib import Path

import yaml

from launch_ros.actions import Node


def _load_yaml_map(path: str) -> dict:
    with Path(path).expanduser().open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected YAML mapping at root: {path}")
    return data


def _load_all_profiles(path: str) -> dict[str, dict]:
    data = _load_yaml_map(path)
    profiles = data.get("profiles", {})
    if isinstance(profiles, list):
        result: dict[str, dict] = {}
        for item in profiles:
            if not isinstance(item, dict):
                continue
            profile_id = str(item.get("profile_id", "")).strip()
            if profile_id:
                result[profile_id] = item
        return result
    if not isinstance(profiles, dict):
        raise RuntimeError(f"profiles must be a mapping or list in {path}")
    result: dict[str, dict] = {}
    for profile_id, profile in profiles.items():
        if isinstance(profile, dict):
            result[str(profile_id)] = profile
    return result


def _normalized_preview_topic(prefix: str, profile_id: str, camera_name: str) -> str:
    prefix = prefix.strip()
    if not prefix.startswith("/"):
        prefix = f"/{prefix}"
    return f"{prefix}/{profile_id}/{camera_name}/image_raw"


def build_preview_spec(profile: dict, *, profile_id: str, prefix: str, max_fps: float) -> list[dict]:
    specs: list[dict] = []
    for item in profile.get("image_inputs", []):
        if not isinstance(item, dict):
            continue
        name = str(item.get("name", "")).strip()
        topic = str(item.get("topic", "")).strip()
        if not name or not topic:
            continue
        specs.append(
            {
                "camera_id": name,
                "title": name,
                "source_topic": topic,
                "output_topic": _normalized_preview_topic(prefix, profile_id, name),
                "square_crop_anchor": item.get("square_crop_anchor"),
                "resize_size": item.get("resize_size"),
                "max_fps": max_fps,
            }
        )
    return specs


def build_all_preview_specs(*, policy_profiles_config: str, prefix: str, max_fps: float) -> list[dict]:
    image_specs: list[dict] = []
    for profile_id, profile in _load_all_profiles(policy_profiles_config).items():
        image_specs.extend(
            build_preview_spec(profile, profile_id=profile_id, prefix=prefix, max_fps=max_fps)
        )
    return image_specs


def make_preview_relay_node(*, policy_profiles_config: str, prefix: str, max_fps: float) -> Node:
    image_specs = build_all_preview_specs(
        policy_profiles_config=policy_profiles_config,
        prefix=prefix,
        max_fps=max_fps,
    )
    if not image_specs:
        raise RuntimeError("No preview image specs found.")
    return Node(
        package="policy_deployment_orchestrator",
        executable="processed_image_relay",
        name="policy_deployment_processed_image_relay",
        output="screen",
        parameters=[{"image_specs": json.dumps(image_specs)}],
    )
