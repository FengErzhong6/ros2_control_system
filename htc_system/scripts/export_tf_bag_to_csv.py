#!/usr/bin/env python3

import argparse
import csv
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


DEFAULT_TOPICS = ("/tf", "/tf_static")
CSV_COLUMNS = (
    "row_index",
    "topic",
    "bag_timestamp_ns",
    "bag_timestamp_s",
    "message_index",
    "transform_index",
    "header_stamp_ns",
    "header_stamp_s",
    "frame_id",
    "child_frame_id",
    "translation_x",
    "translation_y",
    "translation_z",
    "rotation_x",
    "rotation_y",
    "rotation_z",
    "rotation_w",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export TF transforms from a rosbag2 directory into a flat CSV. "
            "Each transform in a TFMessage becomes one CSV row."
        )
    )
    parser.add_argument(
        "bag_dir",
        help="Path to the rosbag2 directory, for example tracker_trajectory_20260328_211117.",
    )
    parser.add_argument(
        "-o",
        "--output",
        help=(
            "Output CSV path. "
            "Default: <bag_dir>/<bag_dir_name>_tf.csv"
        ),
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        default=list(DEFAULT_TOPICS),
        help="TF topics to export. Default: /tf /tf_static",
    )
    parser.add_argument(
        "--storage-id",
        default=None,
        help="Override rosbag storage id. Default: read from metadata.yaml",
    )
    return parser.parse_args()


def load_storage_id(bag_dir: Path, override: str | None) -> str:
    if override:
        return override

    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"metadata.yaml not found in bag directory: {bag_dir}")

    with metadata_path.open("r", encoding="utf-8") as handle:
        metadata = yaml.safe_load(handle)

    try:
        return metadata["rosbag2_bagfile_information"]["storage_identifier"]
    except KeyError as exc:
        raise KeyError(
            f"storage_identifier missing from metadata: {metadata_path}"
        ) from exc


def resolve_output_path(bag_dir: Path, output_arg: str | None) -> Path:
    if output_arg:
        return Path(output_arg).expanduser().resolve()
    return (bag_dir / f"{bag_dir.name}_tf.csv").resolve()


def build_topic_type_map(reader: rosbag2_py.SequentialReader) -> Dict[str, str]:
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def pick_topics(
    requested_topics: Sequence[str], topic_type_map: Dict[str, str]
) -> List[Tuple[str, str]]:
    selected: List[Tuple[str, str]] = []
    for topic in requested_topics:
        if topic not in topic_type_map:
            continue
        selected.append((topic, topic_type_map[topic]))
    return selected


def stamp_to_ns(sec: int, nanosec: int) -> int:
    return sec * 1_000_000_000 + nanosec


def iter_tf_rows(
    reader: rosbag2_py.SequentialReader,
    topic_type_map: Dict[str, str],
) -> Iterable[Dict[str, object]]:
    message_index = 0
    row_index = 0

    while reader.has_next():
        topic_name, serialized_data, bag_timestamp_ns = reader.read_next()
        message_index += 1

        msg_type_name = topic_type_map.get(topic_name)
        if not msg_type_name:
            continue

        msg_type = get_message(msg_type_name)
        message = deserialize_message(serialized_data, msg_type)

        for transform_index, transform in enumerate(message.transforms):
            header_stamp_ns = stamp_to_ns(
                transform.header.stamp.sec, transform.header.stamp.nanosec
            )
            row_index += 1
            yield {
                "row_index": row_index,
                "topic": topic_name,
                "bag_timestamp_ns": bag_timestamp_ns,
                "bag_timestamp_s": f"{bag_timestamp_ns / 1_000_000_000.0:.9f}",
                "message_index": message_index,
                "transform_index": transform_index,
                "header_stamp_ns": header_stamp_ns,
                "header_stamp_s": f"{header_stamp_ns / 1_000_000_000.0:.9f}",
                "frame_id": transform.header.frame_id,
                "child_frame_id": transform.child_frame_id,
                "translation_x": transform.transform.translation.x,
                "translation_y": transform.transform.translation.y,
                "translation_z": transform.transform.translation.z,
                "rotation_x": transform.transform.rotation.x,
                "rotation_y": transform.transform.rotation.y,
                "rotation_z": transform.transform.rotation.z,
                "rotation_w": transform.transform.rotation.w,
            }


def main() -> int:
    args = parse_args()
    bag_dir = Path(args.bag_dir).expanduser().resolve()

    if not bag_dir.is_dir():
        print(f"Bag directory not found: {bag_dir}", file=sys.stderr)
        return 1

    try:
        storage_id = load_storage_id(bag_dir, args.storage_id)
    except Exception as exc:
        print(f"Failed to determine storage id: {exc}", file=sys.stderr)
        return 2

    output_path = resolve_output_path(bag_dir, args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_dir), storage_id=storage_id
    )
    converter_options = rosbag2_py.ConverterOptions("", "")

    try:
        reader.open(storage_options, converter_options)
    except RuntimeError as exc:
        print(f"Failed to open bag '{bag_dir}': {exc}", file=sys.stderr)
        return 3

    topic_type_map = build_topic_type_map(reader)
    selected_topics = pick_topics(args.topics, topic_type_map)
    if not selected_topics:
        print(
            "None of the requested topics are present in the bag. "
            f"Requested={list(args.topics)} available={sorted(topic_type_map)}",
            file=sys.stderr,
        )
        return 4

    for topic_name, topic_type in selected_topics:
        if topic_type != "tf2_msgs/msg/TFMessage":
            print(
                f"Skipping topic '{topic_name}' with unsupported type '{topic_type}'.",
                file=sys.stderr,
            )

    tf_topic_names = [
        topic_name
        for topic_name, topic_type in selected_topics
        if topic_type == "tf2_msgs/msg/TFMessage"
    ]
    if not tf_topic_names:
        print("No TFMessage topics selected for export.", file=sys.stderr)
        return 5

    reader.set_filter(rosbag2_py.StorageFilter(topics=tf_topic_names))

    with output_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_COLUMNS)
        writer.writeheader()

        row_count = 0
        for row in iter_tf_rows(reader, topic_type_map):
            writer.writerow(row)
            row_count += 1

    print(f"Bag: {bag_dir}")
    print(f"Storage id: {storage_id}")
    print(f"Topics exported: {', '.join(tf_topic_names)}")
    print(f"CSV rows written: {row_count}")
    print(f"Output: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
