#!/usr/bin/env python3
from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import importlib
import inspect
import json
import math
from pathlib import Path
import re
import shutil
import sys
from typing import Any

import numpy as np
from PIL import Image as PILImage
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rosidl_runtime_py.utilities import get_message

try:
    PIL_RESAMPLE_LANCZOS = PILImage.Resampling.LANCZOS
except AttributeError:  # pragma: no cover - older Pillow compatibility
    PIL_RESAMPLE_LANCZOS = PILImage.LANCZOS


IMAGE_TYPE = "sensor_msgs/msg/Image"
JOINT_STATE_TYPE = "sensor_msgs/msg/JointState"
FLOAT_ARRAY_TYPE = "data_collection_interfaces/msg/StampedFloat64MultiArray"

OBS_STATE_KEY = "observation.state"
ACTION_KEY = "action"
IMAGE_PREFIX = "observation.images"


@dataclass(frozen=True)
class TopicSelection:
    image_topics: list[str]
    state_topics: list[str]
    action_topics: list[str]
    anchor_image_topic: str

    @property
    def all_topics(self) -> list[str]:
        seen: set[str] = set()
        result: list[str] = []
        for topic in [*self.image_topics, *self.state_topics, *self.action_topics]:
            if topic not in seen:
                result.append(topic)
                seen.add(topic)
        return result


@dataclass
class TopicProbe:
    topic: str
    msg_type: str
    count: int = 0
    first_ns: int | None = None
    last_ns: int | None = None
    image_shape: tuple[int, int, int] | None = None
    vector_dim: int | None = None
    vector_names: list[str] = field(default_factory=list)
    anchor_timestamps_ns: list[int] = field(default_factory=list)

    def update_time(self, timestamp_ns: int) -> None:
        self.count += 1
        if self.first_ns is None:
            self.first_ns = timestamp_ns
        self.last_ns = timestamp_ns


@dataclass
class SessionPlan:
    session_dir: Path
    bag_dir: Path
    metadata: dict[str, Any]
    storage_id: str
    topic_types: dict[str, str]
    topic_classes: dict[str, type]
    selection: TopicSelection
    probes: dict[str, TopicProbe]
    fps: int
    start_ns: int
    end_ns: int
    crop_size: tuple[int, int] | None
    resize_size: tuple[int, int] | None
    joint_name_filter: str | None
    feature_by_image_topic: dict[str, str]
    features: dict[str, dict[str, Any]]
    state_names: list[str]
    action_names: list[str]


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    sessions = discover_sessions(args.input)
    if args.max_episodes is not None:
        sessions = sessions[: args.max_episodes]
    if not sessions:
        raise RuntimeError(f"No session directories found under {args.input}")

    first_plan = build_session_plan(sessions[0], args)
    plans = [first_plan]
    for session_dir in sessions[1:]:
        plans.append(build_session_plan(session_dir, args, expected_features=first_plan.features))

    if args.dry_run:
        print_dry_run(plans, args)
        return 0

    output = args.output.resolve()
    LeRobotDataset = import_lerobot_dataset()
    prepare_output_dir(args.input.resolve(), output, args.overwrite)

    dataset = create_lerobot_dataset(
        LeRobotDataset=LeRobotDataset,
        output=output,
        repo_id=args.repo_id or default_repo_id(output),
        robot_type=args.robot_type,
        fps=first_plan.fps,
        features=first_plan.features,
        image_writer_threads=args.image_writer_threads,
        video_backend=args.video_backend,
    )

    conversion_summary: dict[str, Any] = {
        "input": str(args.input),
        "output": str(output),
        "repo_id": args.repo_id or default_repo_id(output),
        "fps": first_plan.fps,
        "crop": {
            "enabled": bool(args.crop),
            "size": list(args.crop_size) if args.crop_size else None,
        },
        "resize": {
            "enabled": bool(args.resize_size),
            "size": list(args.resize_size) if args.resize_size else None,
            "square_crop_policy": "cam_high:right, wrist:center, other:center" if args.resize_size else None,
        },
        "joint_name_filter": args.joint_name_filter,
        "features": first_plan.features,
        "episodes": [],
    }

    total_frames = 0
    for episode_index, plan in enumerate(plans):
        frame_count = convert_session_to_episode(
            dataset=dataset,
            plan=plan,
            task=args.task or default_task(plan),
            frame_limit=None if args.limit_frames is None else args.limit_frames - total_frames,
        )
        if frame_count <= 0:
            raise RuntimeError(f"No frames were written for {plan.session_dir}")
        dataset.save_episode()
        total_frames += frame_count
        conversion_summary["episodes"].append(
            {
                "episode_index": episode_index,
                "session_dir": str(plan.session_dir),
                "bag_dir": str(plan.bag_dir),
                "task": args.task or default_task(plan),
                "frames": frame_count,
                "start_ns": plan.start_ns,
                "end_ns": plan.end_ns,
                "image_topics": plan.selection.image_topics,
                "state_topics": plan.selection.state_topics,
                "action_topics": plan.selection.action_topics,
            }
        )
        print(f"Wrote episode {episode_index}: {frame_count} frames from {plan.session_dir}")
        if args.limit_frames is not None and total_frames >= args.limit_frames:
            break

    finalize_dataset(dataset)
    (output / "conversion_manifest.json").write_text(
        json.dumps(conversion_summary, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(f"Finished LeRobot dataset at {output}")
    return 0


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert data_collection_stack session rosbag directories to a LeRobot dataset.",
    )
    parser.add_argument("--input", required=True, type=Path, help="Session directory or root containing session_* directories.")
    parser.add_argument("--output", required=True, type=Path, help="Output LeRobot dataset root.")
    parser.add_argument("--repo-id", help="LeRobot/Hugging Face repo id. Defaults to local/<output-name>.")
    parser.add_argument("--robot-type", default="unknown", help="Robot type stored in the LeRobot metadata.")
    parser.add_argument("--task", help="Task label for every frame. Defaults to the session recipe_id.")
    parser.add_argument("--fps", type=int, help="Output dataset FPS. Defaults to the rounded anchor camera FPS.")
    parser.add_argument("--storage-id", help="rosbag2 storage id. Defaults to the bag metadata storage_identifier.")
    parser.add_argument("--image-topic", action="append", default=[], help="Image topic to include. Repeatable. Defaults to all Image topics.")
    parser.add_argument("--state-topic", action="append", default=[], help="State vector topic to include. Repeatable. Defaults to */joint_states.")
    parser.add_argument("--action-topic", action="append", default=[], help="Action vector topic to include. Repeatable. Defaults to command topics.")
    parser.add_argument("--anchor-image-topic", help="Image topic used to infer FPS. Defaults to the first selected image topic.")
    parser.add_argument("--crop", action="store_true", help="Center-crop every image.")
    parser.add_argument(
        "--crop-size",
        nargs=2,
        type=int,
        metavar=("WIDTH", "HEIGHT"),
        help="Center crop size. Providing this implies --crop.",
    )
    parser.add_argument(
        "--resize-size",
        nargs=2,
        type=int,
        metavar=("WIDTH", "HEIGHT"),
        help="Resize every image after a topic-aware square crop. cam_high keeps the right square; wrist cameras keep the centered square.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Remove an existing output directory before writing.")
    parser.add_argument("--dry-run", action="store_true", help="Scan sessions and print the inferred LeRobot features without writing.")
    parser.add_argument("--max-episodes", type=int, help="Convert at most this many sessions.")
    parser.add_argument("--limit-frames", type=int, help="Stop after writing this many frames across all episodes.")
    parser.add_argument(
        "--right-side-only",
        action="store_true",
        help="Keep only JointState entries whose names look like right-side joints: names ending in _R or starting with right_.",
    )
    parser.add_argument(
        "--joint-name-filter",
        help="Regex used to keep JointState entries by raw joint name. Applies to state and action JointState topics only.",
    )
    parser.add_argument("--image-writer-threads", type=int, default=4, help="Passed to LeRobotDataset.create when supported.")
    parser.add_argument("--video-backend", help="Passed to LeRobotDataset.create when supported, for example pyav.")
    args = parser.parse_args(argv)

    if args.right_side_only:
        if args.joint_name_filter:
            parser.error("--right-side-only cannot be combined with --joint-name-filter.")
        args.joint_name_filter = r"(^right_|_R$)"
    if args.joint_name_filter:
        try:
            re.compile(args.joint_name_filter)
        except re.error as exc:
            parser.error(f"--joint-name-filter is not a valid regex: {exc}")

    if args.crop_size is not None:
        args.crop = True
        width, height = args.crop_size
        if width <= 0 or height <= 0:
            parser.error("--crop-size WIDTH HEIGHT must be positive.")
    if args.resize_size is not None:
        width, height = args.resize_size
        if width <= 0 or height <= 0:
            parser.error("--resize-size WIDTH HEIGHT must be positive.")
    if args.fps is not None and args.fps <= 0:
        parser.error("--fps must be positive.")
    if args.max_episodes is not None and args.max_episodes <= 0:
        parser.error("--max-episodes must be positive.")
    if args.limit_frames is not None and args.limit_frames <= 0:
        parser.error("--limit-frames must be positive.")
    return args


def discover_sessions(input_path: Path) -> list[Path]:
    root = input_path.expanduser().resolve()
    if is_session_dir(root):
        return [root]
    if not root.exists():
        raise RuntimeError(f"Input path does not exist: {root}")
    sessions = [path for path in sorted(root.iterdir()) if path.is_dir() and is_session_dir(path)]
    return sessions


def is_session_dir(path: Path) -> bool:
    return (path / "metadata.json").is_file() and ((path / "rosbag" / "metadata.yaml").is_file() or (path / "rosbag").is_dir())


def load_session_metadata(session_dir: Path) -> dict[str, Any]:
    metadata_path = session_dir / "metadata.json"
    if not metadata_path.is_file():
        return {}
    with metadata_path.open("r", encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected metadata.json to contain an object: {metadata_path}")
    return data


def session_bag_dir(session_dir: Path, metadata: dict[str, Any]) -> Path:
    local_bag = session_dir / "rosbag"
    if local_bag.exists():
        return local_bag
    raw_bag_dir = metadata.get("bag_dir")
    if raw_bag_dir:
        bag_dir = Path(str(raw_bag_dir)).expanduser()
        if bag_dir.exists():
            return bag_dir
    raise RuntimeError(f"Could not find rosbag directory for {session_dir}")


def detect_storage_id(bag_dir: Path, requested_storage_id: str | None) -> str:
    if requested_storage_id:
        return requested_storage_id
    metadata_path = bag_dir / "metadata.yaml"
    if metadata_path.is_file():
        try:
            import yaml

            data = yaml.safe_load(metadata_path.read_text(encoding="utf-8")) or {}
            info = data.get("rosbag2_bagfile_information", {})
            storage_id = info.get("storage_identifier")
            if storage_id:
                return str(storage_id)
        except Exception:
            pass
    return "mcap"


def open_bag_reader(bag_dir: Path, storage_id: str) -> SequentialReader:
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    return reader


def build_session_plan(
    session_dir: Path,
    args: argparse.Namespace,
    expected_features: dict[str, dict[str, Any]] | None = None,
) -> SessionPlan:
    metadata = load_session_metadata(session_dir)
    bag_dir = session_bag_dir(session_dir, metadata)
    storage_id = detect_storage_id(bag_dir, args.storage_id)
    reader = open_bag_reader(bag_dir, storage_id)
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    topic_classes = {topic: get_message(msg_type) for topic, msg_type in topic_types.items()}
    ordered_topics = metadata_ordered_topics(metadata, topic_types)
    selection = infer_topic_selection(topic_types, ordered_topics, args)
    crop_size = tuple(args.crop_size) if args.crop else None
    resize_size = tuple(args.resize_size) if args.resize_size else None
    probes = scan_topics(
        bag_dir=bag_dir,
        storage_id=storage_id,
        topic_types=topic_types,
        topic_classes=topic_classes,
        selection=selection,
        crop_size=crop_size,
        resize_size=resize_size,
        joint_name_filter=args.joint_name_filter,
    )
    fps = args.fps or infer_fps(probes[selection.anchor_image_topic].anchor_timestamps_ns)
    start_ns, end_ns = common_time_window(probes, selection.all_topics)
    if end_ns <= start_ns:
        raise RuntimeError(f"No overlapping time window across selected topics for {session_dir}")
    feature_by_image_topic = build_image_feature_names(selection.image_topics)
    features, state_names, action_names = build_lerobot_features(
        probes=probes,
        selection=selection,
        feature_by_image_topic=feature_by_image_topic,
    )
    if expected_features is not None and features != expected_features:
        raise RuntimeError(
            f"Feature layout differs for {session_dir}. "
            "Use explicit --image-topic/--state-topic/--action-topic or convert sessions separately."
        )
    return SessionPlan(
        session_dir=session_dir,
        bag_dir=bag_dir,
        metadata=metadata,
        storage_id=storage_id,
        topic_types=topic_types,
        topic_classes=topic_classes,
        selection=selection,
        probes=probes,
        fps=fps,
        start_ns=start_ns,
        end_ns=end_ns,
        crop_size=crop_size,
        resize_size=resize_size,
        joint_name_filter=args.joint_name_filter,
        feature_by_image_topic=feature_by_image_topic,
        features=features,
        state_names=state_names,
        action_names=action_names,
    )


def metadata_ordered_topics(metadata: dict[str, Any], topic_types: dict[str, str]) -> list[str]:
    ordered: list[str] = []
    seen: set[str] = set()
    for raw_topic in metadata.get("record_topics", []):
        topic = normalize_topic(str(raw_topic))
        if topic in topic_types and topic not in seen:
            ordered.append(topic)
            seen.add(topic)
    for topic in sorted(topic_types):
        if topic not in seen:
            ordered.append(topic)
    return ordered


def infer_topic_selection(
    topic_types: dict[str, str],
    ordered_topics: list[str],
    args: argparse.Namespace,
) -> TopicSelection:
    image_topics = resolve_explicit_topics(args.image_topic, topic_types)
    if not image_topics:
        image_topics = [topic for topic in ordered_topics if topic_types.get(topic) == IMAGE_TYPE]
    if not image_topics:
        raise RuntimeError("No image topics found. Pass --image-topic explicitly if needed.")

    state_topics = resolve_explicit_topics(args.state_topic, topic_types)
    if not state_topics:
        state_topics = [
            topic
            for topic in ordered_topics
            if topic_types.get(topic) == JOINT_STATE_TYPE and "/joint_states" in topic
        ]
    if not state_topics:
        raise RuntimeError("No state topics found. Pass --state-topic explicitly.")

    action_topics = resolve_explicit_topics(args.action_topic, topic_types)
    if not action_topics:
        action_topics = [
            topic
            for topic in ordered_topics
            if topic_types.get(topic) in {JOINT_STATE_TYPE, FLOAT_ARRAY_TYPE}
            and ("command" in topic or "commands" in topic)
        ]
    if not action_topics:
        raise RuntimeError("No action topics found. Pass --action-topic explicitly.")

    image_topics = dedupe_topics(image_topics)
    state_topics = dedupe_topics(state_topics)
    action_topics = dedupe_topics(action_topics)

    anchor_image_topic = normalize_topic(args.anchor_image_topic) if args.anchor_image_topic else image_topics[0]
    if anchor_image_topic not in image_topics:
        raise RuntimeError(f"--anchor-image-topic must be one of the selected image topics: {anchor_image_topic}")

    unsupported_state = [topic for topic in state_topics if topic_types.get(topic) not in {JOINT_STATE_TYPE, FLOAT_ARRAY_TYPE}]
    unsupported_action = [topic for topic in action_topics if topic_types.get(topic) not in {JOINT_STATE_TYPE, FLOAT_ARRAY_TYPE}]
    if unsupported_state:
        raise RuntimeError(f"Unsupported state topic types: {unsupported_state}")
    if unsupported_action:
        raise RuntimeError(f"Unsupported action topic types: {unsupported_action}")

    return TopicSelection(
        image_topics=image_topics,
        state_topics=state_topics,
        action_topics=action_topics,
        anchor_image_topic=anchor_image_topic,
    )


def resolve_explicit_topics(raw_topics: list[str], topic_types: dict[str, str]) -> list[str]:
    result: list[str] = []
    for raw_topic in raw_topics:
        topic = normalize_topic(raw_topic)
        if topic not in topic_types:
            available = "\n  ".join(sorted(topic_types))
            raise RuntimeError(f"Topic not found in bag: {topic}\nAvailable topics:\n  {available}")
        result.append(topic)
    return result


def dedupe_topics(topics: list[str]) -> list[str]:
    seen: set[str] = set()
    result: list[str] = []
    for topic in topics:
        if topic in seen:
            continue
        result.append(topic)
        seen.add(topic)
    return result


def scan_topics(
    *,
    bag_dir: Path,
    storage_id: str,
    topic_types: dict[str, str],
    topic_classes: dict[str, type],
    selection: TopicSelection,
    crop_size: tuple[int, int] | None,
    resize_size: tuple[int, int] | None,
    joint_name_filter: str | None,
) -> dict[str, TopicProbe]:
    selected_topics = set(selection.all_topics)
    probes = {
        topic: TopicProbe(topic=topic, msg_type=topic_types[topic])
        for topic in selection.all_topics
    }
    reader = open_bag_reader(bag_dir, storage_id)
    while reader.has_next():
        topic, raw_data, timestamp_ns = reader.read_next()
        if topic not in selected_topics:
            continue
        probe = probes[topic]
        probe.update_time(timestamp_ns)
        if topic == selection.anchor_image_topic:
            probe.anchor_timestamps_ns.append(timestamp_ns)

        needs_image_shape = probe.msg_type == IMAGE_TYPE and probe.image_shape is None
        needs_vector_shape = probe.msg_type != IMAGE_TYPE and probe.vector_dim is None
        if not needs_image_shape and not needs_vector_shape:
            continue

        msg = deserialize_message(raw_data, topic_classes[topic])
        if probe.msg_type == IMAGE_TYPE:
            image = decode_image_message(msg)
            image = preprocess_image_for_topic(
                image,
                topic,
                crop_size=crop_size,
                resize_size=resize_size,
            )
            probe.image_shape = tuple(int(value) for value in image.shape)
        else:
            vector, names = vector_from_message(msg, topic, joint_name_filter=joint_name_filter)
            probe.vector_dim = int(vector.shape[0])
            probe.vector_names = names

    missing = [topic for topic, probe in probes.items() if probe.count == 0]
    if missing:
        raise RuntimeError(f"Selected topics had no messages: {missing}")
    for topic, probe in probes.items():
        if probe.msg_type == IMAGE_TYPE and probe.image_shape is None:
            raise RuntimeError(f"Could not determine image shape for {topic}")
        if probe.msg_type != IMAGE_TYPE and probe.vector_dim is None:
            raise RuntimeError(f"Could not determine vector shape for {topic}")
    return probes


def infer_fps(anchor_timestamps_ns: list[int]) -> int:
    if len(anchor_timestamps_ns) < 2:
        raise RuntimeError("Need at least two anchor image messages to infer FPS. Pass --fps explicitly.")
    duration_ns = anchor_timestamps_ns[-1] - anchor_timestamps_ns[0]
    if duration_ns <= 0:
        raise RuntimeError("Anchor image timestamps are not increasing. Pass --fps explicitly.")
    fps = int(round((len(anchor_timestamps_ns) - 1) * 1_000_000_000.0 / float(duration_ns)))
    if fps <= 0:
        raise RuntimeError("Inferred a non-positive FPS. Pass --fps explicitly.")
    return fps


def common_time_window(probes: dict[str, TopicProbe], topics: list[str]) -> tuple[int, int]:
    starts: list[int] = []
    ends: list[int] = []
    for topic in topics:
        probe = probes[topic]
        if probe.first_ns is None or probe.last_ns is None:
            raise RuntimeError(f"Topic did not have a valid time range: {topic}")
        starts.append(probe.first_ns)
        ends.append(probe.last_ns)
    return max(starts), min(ends)


def build_image_feature_names(image_topics: list[str]) -> dict[str, str]:
    used: set[str] = set()
    feature_by_topic: dict[str, str] = {}
    for topic in image_topics:
        base = sanitize_topic_key(topic)
        key = base
        suffix = 2
        while key in used:
            key = f"{base}_{suffix}"
            suffix += 1
        used.add(key)
        feature_by_topic[topic] = f"{IMAGE_PREFIX}.{key}"
    return feature_by_topic


def build_lerobot_features(
    *,
    probes: dict[str, TopicProbe],
    selection: TopicSelection,
    feature_by_image_topic: dict[str, str],
) -> tuple[dict[str, dict[str, Any]], list[str], list[str]]:
    features: dict[str, dict[str, Any]] = {}

    for topic in selection.image_topics:
        shape = probes[topic].image_shape
        if shape is None:
            raise RuntimeError(f"Missing image shape for {topic}")
        height, width, channels = shape
        if channels != 3:
            raise RuntimeError(f"Expected RGB image with 3 channels for {topic}; got shape {shape}")
        features[feature_by_image_topic[topic]] = {
            "dtype": "video",
            "shape": (height, width, channels),
            "names": ["height", "width", "channels"],
        }

    state_names = flatten_vector_names(probes, selection.state_topics)
    action_names = flatten_vector_names(probes, selection.action_topics)
    features[OBS_STATE_KEY] = {
        "dtype": "float32",
        "shape": (len(state_names),),
        "names": state_names,
    }
    features[ACTION_KEY] = {
        "dtype": "float32",
        "shape": (len(action_names),),
        "names": action_names,
    }
    return features, state_names, action_names


def flatten_vector_names(probes: dict[str, TopicProbe], topics: list[str]) -> list[str]:
    names: list[str] = []
    for topic in topics:
        probe = probes[topic]
        if probe.vector_dim is None:
            raise RuntimeError(f"Missing vector dimension for {topic}")
        if probe.vector_names:
            names.extend(probe.vector_names)
        else:
            topic_key = sanitize_topic_key(topic)
            names.extend(f"{topic_key}.value_{index}" for index in range(probe.vector_dim))
    return names


def convert_session_to_episode(
    *,
    dataset: Any,
    plan: SessionPlan,
    task: str,
    frame_limit: int | None,
) -> int:
    reader = open_bag_reader(plan.bag_dir, plan.storage_id)
    selected_topics = set(plan.selection.all_topics)
    latest: dict[str, np.ndarray] = {}
    target_ns = plan.start_ns
    step_ns = int(round(1_000_000_000.0 / float(plan.fps)))
    frame_count = 0

    def can_write_more() -> bool:
        return frame_limit is None or frame_count < frame_limit

    while reader.has_next() and target_ns <= plan.end_ns and can_write_more():
        topic, raw_data, timestamp_ns = reader.read_next()
        if topic not in selected_topics:
            continue

        while target_ns < timestamp_ns and target_ns <= plan.end_ns and can_write_more():
            if all(topic in latest for topic in selected_topics):
                dataset.add_frame(build_lerobot_frame(plan, latest, task))
                frame_count += 1
            target_ns += step_ns

        msg = deserialize_message(raw_data, plan.topic_classes[topic])
        latest[topic] = decode_selected_message(plan, topic, msg)

    while target_ns <= plan.end_ns and can_write_more():
        if all(topic in latest for topic in selected_topics):
            dataset.add_frame(build_lerobot_frame(plan, latest, task))
            frame_count += 1
        target_ns += step_ns

    return frame_count


def decode_selected_message(plan: SessionPlan, topic: str, msg: Any) -> np.ndarray:
    msg_type = plan.topic_types[topic]
    if msg_type == IMAGE_TYPE:
        image = decode_image_message(msg)
        image = preprocess_image_for_topic(
            image,
            topic,
            crop_size=plan.crop_size,
            resize_size=plan.resize_size,
        )
        expected_shape = plan.probes[topic].image_shape
        if expected_shape is None:
            raise RuntimeError(f"Missing expected image shape for {topic}")
        if tuple(image.shape) != expected_shape:
            raise RuntimeError(f"Image shape changed for {topic}: {image.shape} != {expected_shape}")
        return image

    vector, _ = vector_from_message(msg, topic, joint_name_filter=plan.joint_name_filter)
    expected_dim = plan.probes[topic].vector_dim
    if expected_dim is not None and vector.shape[0] != expected_dim:
        raise RuntimeError(f"Vector dimension changed for {topic}: {vector.shape[0]} != {expected_dim}")
    return vector


def build_lerobot_frame(plan: SessionPlan, latest: dict[str, np.ndarray], task: str) -> dict[str, Any]:
    frame: dict[str, Any] = {"task": task}
    frame[OBS_STATE_KEY] = concat_vectors(latest, plan.selection.state_topics)
    frame[ACTION_KEY] = concat_vectors(latest, plan.selection.action_topics)
    for topic in plan.selection.image_topics:
        frame[plan.feature_by_image_topic[topic]] = latest[topic]
    return frame


def concat_vectors(latest: dict[str, np.ndarray], topics: list[str]) -> np.ndarray:
    arrays = [latest[topic].astype(np.float32, copy=False).reshape(-1) for topic in topics]
    if len(arrays) == 1:
        return arrays[0]
    return np.concatenate(arrays).astype(np.float32, copy=False)


def decode_image_message(msg: Any) -> np.ndarray:
    encoding = str(msg.encoding).lower()
    channel_count = image_channel_count(encoding)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    expected_bytes = int(msg.height) * int(msg.step)
    if raw.size < expected_bytes:
        raise RuntimeError(
            f"Image data is shorter than expected: got {raw.size} bytes, expected {expected_bytes}"
        )
    rows = raw[:expected_bytes].reshape(int(msg.height), int(msg.step))
    pixel_bytes = int(msg.width) * channel_count
    if pixel_bytes > int(msg.step):
        raise RuntimeError(
            f"Image step is too small for {msg.width}x{msg.height} {msg.encoding}: step={msg.step}"
        )
    packed = rows[:, :pixel_bytes]
    if channel_count == 1:
        mono = packed.reshape(int(msg.height), int(msg.width))
        return np.repeat(mono[:, :, None], 3, axis=2).copy()

    image = packed.reshape(int(msg.height), int(msg.width), channel_count)
    if encoding in {"bgr8", "8uc3"}:
        return image[:, :, [2, 1, 0]].copy()
    if encoding == "rgb8":
        return image[:, :, :3].copy()
    if encoding == "rgba8":
        return image[:, :, :3].copy()
    if encoding == "bgra8":
        return image[:, :, [2, 1, 0]].copy()
    if encoding == "8uc4":
        return image[:, :, :3].copy()
    raise RuntimeError(f"Unsupported image encoding: {msg.encoding}")


def image_channel_count(encoding: str) -> int:
    if encoding in {"mono8", "8uc1"}:
        return 1
    if encoding in {"rgb8", "bgr8", "8uc3"}:
        return 3
    if encoding in {"rgba8", "bgra8", "8uc4"}:
        return 4
    raise RuntimeError(f"Unsupported image encoding: {encoding}")


def center_crop(image: np.ndarray, crop_size: tuple[int, int]) -> np.ndarray:
    crop_width, crop_height = crop_size
    height, width = image.shape[:2]
    if crop_width > width or crop_height > height:
        raise RuntimeError(
            f"Crop size {crop_width}x{crop_height} is larger than image {width}x{height}"
        )
    left = (width - crop_width) // 2
    top = (height - crop_height) // 2
    return image[top : top + crop_height, left : left + crop_width].copy()


def preprocess_image_for_topic(
    image: np.ndarray,
    topic: str,
    *,
    crop_size: tuple[int, int] | None,
    resize_size: tuple[int, int] | None,
) -> np.ndarray:
    if crop_size is not None:
        image = center_crop(image, crop_size)
    elif resize_size is not None:
        image = crop_square_for_topic(image, topic)
    if resize_size is not None:
        image = resize_image(image, resize_size)
    return image


def crop_square_for_topic(image: np.ndarray, topic: str) -> np.ndarray:
    topic_lower = normalize_topic(topic).lower()
    if "cam_high" in topic_lower:
        return crop_square(image, anchor="right")
    return crop_square(image, anchor="center")


def crop_square(image: np.ndarray, *, anchor: str) -> np.ndarray:
    height, width = image.shape[:2]
    side = min(height, width)
    if side <= 0:
        raise RuntimeError(f"Image has invalid shape: {image.shape}")
    if anchor == "right":
        left = width - side
        top = 0
    else:
        left = (width - side) // 2
        top = (height - side) // 2
    return image[top : top + side, left : left + side].copy()


def resize_image(image: np.ndarray, resize_size: tuple[int, int]) -> np.ndarray:
    target_width, target_height = resize_size
    if image.shape[1] == target_width and image.shape[0] == target_height:
        return image.copy()
    resized = PILImage.fromarray(image, mode="RGB").resize(
        (target_width, target_height),
        resample=PIL_RESAMPLE_LANCZOS,
    )
    return np.asarray(resized, dtype=np.uint8).copy()


def vector_from_message(
    msg: Any,
    topic: str,
    *,
    joint_name_filter: str | None = None,
) -> tuple[np.ndarray, list[str]]:
    if hasattr(msg, "position") and hasattr(msg, "name"):
        return vector_from_joint_state(msg, topic, joint_name_filter=joint_name_filter)

    if hasattr(msg, "data"):
        values = list(msg.data)
        topic_key = sanitize_topic_key(topic)
        names = [f"{topic_key}.value_{index}" for index in range(len(values))]
        return np.asarray(values, dtype=np.float32), names

    raise RuntimeError(f"Unsupported vector message on {topic}: {type(msg)!r}")


def vector_from_joint_state(
    msg: Any,
    topic: str,
    *,
    joint_name_filter: str | None = None,
) -> tuple[np.ndarray, list[str]]:
    values = list(msg.position)
    if not values:
        values = list(msg.velocity)
    if not values:
        values = list(msg.effort)
    if not values:
        raise RuntimeError(f"JointState topic has no position/velocity/effort values: {topic}")

    raw_names = [str(name) for name in msg.name]
    topic_key = sanitize_topic_key(topic)

    if len(raw_names) != len(values):
        raw_names = [f"value_{index}" for index in range(len(values))]
    if joint_name_filter is not None:
        kept = [
            (name, value)
            for name, value in zip(raw_names, values, strict=True)
            if re.search(joint_name_filter, name)
        ]
        if not kept:
            raise RuntimeError(
                f"JointState filter {joint_name_filter!r} removed every entry from {topic}. "
                f"Available names: {raw_names}"
            )
        raw_names = [name for name, _ in kept]
        values = [value for _, value in kept]
    names = [f"{topic_key}.{sanitize_component(name)}" for name in raw_names]
    return np.asarray(values, dtype=np.float32), names


def normalize_topic(topic: str) -> str:
    topic = topic.strip()
    if not topic:
        raise RuntimeError("Topic name must not be empty.")
    if not topic.startswith("/"):
        topic = f"/{topic}"
    return topic


def sanitize_topic_key(topic: str) -> str:
    topic = normalize_topic(topic)
    if topic.startswith("/record/"):
        topic = topic[len("/record/") :]
    else:
        topic = topic.lstrip("/")
    parts = [part for part in topic.split("/") if part and part not in {"image_raw"}]
    compact_parts: list[str] = []
    for part in parts:
        if compact_parts and compact_parts[-1] == part:
            continue
        compact_parts.append(part)
    if compact_parts and compact_parts[-1] == "color":
        compact_parts = compact_parts[:-1]
    key = "_".join(sanitize_component(part) for part in compact_parts)
    key = re.sub(r"_+", "_", key).strip("_")
    return key or "topic"


def sanitize_component(value: str) -> str:
    sanitized = re.sub(r"[^A-Za-z0-9._-]+", "_", str(value).strip())
    sanitized = sanitized.strip("._-")
    return sanitized or "value"


def default_task(plan: SessionPlan) -> str:
    return str(plan.metadata.get("recipe_id") or plan.metadata.get("session_tag") or plan.session_dir.name)


def default_repo_id(output: Path) -> str:
    name = sanitize_component(output.name).lower()
    return f"local/{name or 'dataset'}"


def prepare_output_dir(input_root: Path, output: Path, overwrite: bool) -> None:
    if output.exists():
        if not overwrite:
            raise RuntimeError(f"Output already exists. Pass --overwrite to replace it: {output}")
        if output == input_root or output in input_root.parents:
            raise RuntimeError(f"Refusing to overwrite input path or one of its parents: {output}")
        shutil.rmtree(output)


def import_lerobot_dataset() -> type:
    import_errors: list[str] = []
    for module_name in (
        "lerobot.datasets.lerobot_dataset",
        "lerobot.common.datasets.lerobot_dataset",
    ):
        try:
            module = importlib.import_module(module_name)
            return module.LeRobotDataset
        except Exception as exc:
            import_errors.append(f"{module_name}: {exc}")
    raise RuntimeError(
        "Could not import LeRobotDataset. Install LeRobot in this Python environment first.\n"
        "Tried:\n  "
        + "\n  ".join(import_errors)
    )


def create_lerobot_dataset(
    *,
    LeRobotDataset: type,
    output: Path,
    repo_id: str,
    robot_type: str,
    fps: int,
    features: dict[str, dict[str, Any]],
    image_writer_threads: int,
    video_backend: str | None,
) -> Any:
    kwargs: dict[str, Any] = {
        "repo_id": repo_id,
        "root": output,
        "robot_type": robot_type,
        "fps": fps,
        "features": features,
        "use_videos": True,
        "image_writer_processes": 0,
        "image_writer_threads": image_writer_threads,
        "video_backend": video_backend,
    }
    signature = inspect.signature(LeRobotDataset.create)
    filtered_kwargs = {
        key: value
        for key, value in kwargs.items()
        if key in signature.parameters and value is not None
    }
    return LeRobotDataset.create(**filtered_kwargs)


def finalize_dataset(dataset: Any) -> None:
    if hasattr(dataset, "finalize"):
        dataset.finalize()
        return
    if hasattr(dataset, "consolidate"):
        dataset.consolidate(run_compute_stats=True)


def print_dry_run(plans: list[SessionPlan], args: argparse.Namespace) -> None:
    first_plan = plans[0]
    print("LeRobot conversion dry run")
    print(f"episodes: {len(plans)}")
    print(f"fps: {first_plan.fps}")
    print(f"crop_size: {first_plan.crop_size}")
    print(f"resize_size: {first_plan.resize_size}")
    print(f"joint_name_filter: {args.joint_name_filter}")
    print("features:")
    print(json.dumps(first_plan.features, indent=2, ensure_ascii=False))
    for index, plan in enumerate(plans):
        duration_sec = (plan.end_ns - plan.start_ns) / 1_000_000_000.0
        approx_frames = max(0, math.floor(duration_sec * plan.fps))
        print(f"\nepisode {index}: {plan.session_dir}")
        print(f"  bag: {plan.bag_dir}")
        print(f"  task: {default_task(plan)}")
        print(f"  duration_sec: {duration_sec:.3f}")
        print(f"  approx_frames: {approx_frames}")
        print(f"  image_topics: {plan.selection.image_topics}")
        print(f"  state_topics: {plan.selection.state_topics}")
        print(f"  action_topics: {plan.selection.action_topics}")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
