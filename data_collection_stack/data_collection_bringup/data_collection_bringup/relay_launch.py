from __future__ import annotations

import json
from pathlib import Path

import yaml

from .relay_mapping import build_relay_topic_mapping
from .topic_relay_utils import normalize_topic


def _load_yaml_map(path: str) -> dict:
    with Path(path).expanduser().open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected YAML mapping at root: {path}")
    return data


def _load_recipe(path: str, recipe_id: str) -> dict:
    recipe_dir = Path(path).expanduser()
    for candidate in sorted(recipe_dir.glob("*.yaml")):
        data = _load_yaml_map(str(candidate))
        candidate_recipe_id = str(data.get("recipe_id", candidate.stem))
        if candidate_recipe_id == recipe_id:
            return data
    raise RuntimeError(f"Recipe '{recipe_id}' not found in {recipe_dir}")


def _build_relay_specs(record_topics: list[str], prefix: str) -> list[dict[str, str]]:
    specs: list[dict[str, str]] = []
    seen_outputs: set[str] = set()
    for raw_topic in record_topics:
        mapping = build_relay_topic_mapping(raw_topic, prefix)
        if mapping.output_topic in seen_outputs:
            raise RuntimeError(f"Duplicate relay output topic: {mapping.output_topic}")
        seen_outputs.add(mapping.output_topic)
        specs.append(
            {
                "source_topic": mapping.source_topic,
                "output_topic": mapping.output_topic,
                "msg_type": mapping.source_msg_type,
            }
        )
    return specs


def relay_setup(context, *args, **kwargs):
    del args, kwargs

    from launch.substitutions import LaunchConfiguration
    from launch_ros.actions import Node

    recipe_id = LaunchConfiguration("recipe_id").perform(context)
    recipe_directory = LaunchConfiguration("recipe_directory").perform(context)
    recording_config = LaunchConfiguration("recording_config").perform(context)

    recipe = _load_recipe(recipe_directory, recipe_id)
    recording_policy = _load_yaml_map(recording_config)
    relay_enabled = bool(recording_policy.get("relay_record_topics", False))
    if not relay_enabled:
        return []

    relay_prefix = normalize_topic(
        str(recording_policy.get("relay_record_topic_prefix", "/record"))
    )
    record_topics = [str(topic) for topic in recipe.get("record_topics", [])]
    relay_specs = _build_relay_specs(record_topics, relay_prefix)
    if not relay_specs:
        raise RuntimeError("relay_record_topics is enabled but no relay specs were produced.")

    return [
        Node(
            package="data_collection_bringup",
            executable="timestamp_relay",
            name="data_collection_timestamp_relay",
            output="screen",
            parameters=[{"topic_specs": json.dumps(relay_specs)}],
        )
    ]
