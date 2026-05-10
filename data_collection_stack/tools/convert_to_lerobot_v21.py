#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image as PILImage


def bootstrap_pyarrow() -> None:
    try:
        import pyarrow  # noqa: F401

        return
    except ModuleNotFoundError:
        pass

    suffix = f"cpython-{sys.version_info.major}{sys.version_info.minor}"
    search_roots = [Path("/tmp/uv-cache/archive-v0"), Path.home() / ".cache/uv/archive-v0"]
    for root in search_roots:
        if not root.exists():
            continue
        for candidate in root.glob("*/pyarrow"):
            if not candidate.is_dir():
                continue
            if any(candidate.glob(f"*.{suffix}-*.so")):
                sys.path.insert(0, str(candidate.parent))
                return

    raise ModuleNotFoundError(
        "pyarrow is required for LeRobot v2.1 export. Install it in the Python environment that runs this script."
    )


bootstrap_pyarrow()
import pyarrow as pa
import pyarrow.parquet as pq

from convert_to_lerobot import (  # noqa: E402
    ACTION_KEY,
    IMAGE_TYPE,
    OBS_STATE_KEY,
    SessionPlan,
    build_session_plan,
    default_task,
    discover_sessions,
    open_bag_reader,
)


CODEBASE_VERSION = "v2.1"
DEFAULT_CHUNK_SIZE = 1000
DEFAULT_FEATURES = {
    "timestamp": {"dtype": "float32", "shape": (1,), "names": None},
    "frame_index": {"dtype": "int64", "shape": (1,), "names": None},
    "episode_index": {"dtype": "int64", "shape": (1,), "names": None},
    "index": {"dtype": "int64", "shape": (1,), "names": None},
    "task_index": {"dtype": "int64", "shape": (1,), "names": None},
}

DATA_PATH = "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet"
VIDEO_PATH = "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
IMAGE_PATH = "images/{image_key}/episode_{episode_index:06d}/frame_{frame_index:06d}.png"


def estimate_num_samples(dataset_len: int, min_num_samples: int = 100, max_num_samples: int = 10_000, power: float = 0.75) -> int:
    if dataset_len < min_num_samples:
        min_num_samples = dataset_len
    return max(min_num_samples, min(int(dataset_len**power), max_num_samples))


def sample_indices(data_len: int) -> list[int]:
    num_samples = estimate_num_samples(data_len)
    return np.round(np.linspace(0, data_len - 1, num_samples)).astype(int).tolist()


def auto_downsample_height_width(img: np.ndarray, target_size: int = 150, max_size_threshold: int = 300) -> np.ndarray:
    _, height, width = img.shape
    if max(width, height) < max_size_threshold:
        return img
    downsample_factor = int(width / target_size) if width > height else int(height / target_size)
    return img[:, ::downsample_factor, ::downsample_factor]


def load_image_as_numpy(fpath: str | Path, dtype: np.dtype = np.uint8, channel_first: bool = True) -> np.ndarray:
    img = PILImage.open(fpath).convert("RGB")
    img_array = np.array(img, dtype=dtype)
    if channel_first:
        img_array = np.transpose(img_array, (2, 0, 1))
    if np.issubdtype(dtype, np.floating):
        img_array /= 255.0
    return img_array


def sample_images(image_paths: list[str]) -> np.ndarray:
    sampled_indices = sample_indices(len(image_paths))
    images = None
    for i, idx in enumerate(sampled_indices):
        path = image_paths[idx]
        img = load_image_as_numpy(path, dtype=np.uint8, channel_first=True)
        img = auto_downsample_height_width(img)
        if images is None:
            images = np.empty((len(sampled_indices), *img.shape), dtype=np.uint8)
        images[i] = img
    return images


def get_feature_stats(array: np.ndarray, axis: tuple[int, ...] | int, keepdims: bool) -> dict[str, np.ndarray]:
    return {
        "min": np.min(array, axis=axis, keepdims=keepdims),
        "max": np.max(array, axis=axis, keepdims=keepdims),
        "mean": np.mean(array, axis=axis, keepdims=keepdims),
        "std": np.std(array, axis=axis, keepdims=keepdims),
        "count": np.array([len(array)]),
    }


def compute_episode_stats(episode_data: dict[str, list[str] | np.ndarray], features: dict[str, dict[str, Any]]) -> dict[str, dict[str, np.ndarray]]:
    ep_stats: dict[str, dict[str, np.ndarray]] = {}
    for key, data in episode_data.items():
        if features[key]["dtype"] == "string":
            continue
        elif features[key]["dtype"] in ["image", "video"]:
            ep_ft_array = sample_images(data)  # type: ignore[arg-type]
            axes_to_reduce = (0, 2, 3)
            keepdims = True
        else:
            ep_ft_array = np.asarray(data)
            axes_to_reduce = 0
            keepdims = ep_ft_array.ndim == 1

        ep_stats[key] = get_feature_stats(ep_ft_array, axis=axes_to_reduce, keepdims=keepdims)

        if features[key]["dtype"] in ["image", "video"]:
            ep_stats[key] = {
                stat_key: stat_value if stat_key == "count" else np.squeeze(stat_value / 255.0, axis=0)
                for stat_key, stat_value in ep_stats[key].items()
            }

    return ep_stats


def aggregate_feature_stats(stats_ft_list: list[dict[str, np.ndarray]]) -> dict[str, np.ndarray]:
    means = np.stack([stats["mean"] for stats in stats_ft_list])
    variances = np.stack([stats["std"] ** 2 for stats in stats_ft_list])
    counts = np.stack([stats["count"] for stats in stats_ft_list])
    total_count = counts.sum(axis=0)

    while counts.ndim < means.ndim:
        counts = np.expand_dims(counts, axis=-1)

    weighted_means = means * counts
    total_mean = weighted_means.sum(axis=0) / total_count

    delta_means = means - total_mean
    weighted_variances = (variances + delta_means**2) * counts
    total_variance = weighted_variances.sum(axis=0) / total_count

    return {
        "min": np.min(np.stack([stats["min"] for stats in stats_ft_list]), axis=0),
        "max": np.max(np.stack([stats["max"] for stats in stats_ft_list]), axis=0),
        "mean": total_mean,
        "std": np.sqrt(total_variance),
        "count": total_count,
    }


def aggregate_stats(stats_list: list[dict[str, dict[str, np.ndarray]]]) -> dict[str, dict[str, np.ndarray]]:
    if not stats_list:
        return {}

    data_keys = {key for stats in stats_list for key in stats}
    aggregated_stats = {key: {} for key in data_keys}
    for key in data_keys:
        stats_with_key = [stats[key] for stats in stats_list if key in stats]
        aggregated_stats[key] = aggregate_feature_stats(stats_with_key)
    return aggregated_stats


def serialize_stats(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: serialize_stats(v) for k, v in value.items()}
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (int, float, str, bool)) or value is None:
        return value
    raise TypeError(f"Unsupported stats value: {type(value)!r}")


def write_json(data: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=4, ensure_ascii=False), encoding="utf-8")


def append_jsonline(data: dict[str, Any], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(data, ensure_ascii=False))
        handle.write("\n")


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    if isinstance(value, tuple):
        return [json_safe(v) for v in value]
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    return value


def video_info_from_feature(feature: dict[str, Any], fps: int) -> dict[str, Any]:
    height, width, channels = feature["shape"]
    return {
        "video.height": int(height),
        "video.width": int(width),
        "video.codec": "h264",
        "video.pix_fmt": "yuv420p",
        "video.is_depth_map": False,
        "video.fps": int(fps),
        "video.channels": int(channels),
        "has_audio": False,
    }


def build_frame(plan: SessionPlan, latest: dict[str, np.ndarray], task: str, timestamp_s: float) -> dict[str, Any]:
    frame: dict[str, Any] = {
        "task": task,
        "timestamp": np.float32(timestamp_s),
        OBS_STATE_KEY: concat_vectors(latest, plan.selection.state_topics),
        ACTION_KEY: concat_vectors(latest, plan.selection.action_topics),
    }
    for topic in plan.selection.image_topics:
        frame[plan.feature_by_image_topic[topic]] = latest[topic]
    return frame


def concat_vectors(latest: dict[str, np.ndarray], topics: list[str]) -> np.ndarray:
    arrays = [latest[topic].astype(np.float32, copy=False).reshape(-1) for topic in topics]
    if not arrays:
        raise RuntimeError("Cannot build a vector from zero topics.")
    if len(arrays) == 1:
        return arrays[0]
    return np.concatenate(arrays).astype(np.float32, copy=False)


def prepare_output_dir(input_root: Path, output: Path, overwrite: bool) -> None:
    if output.exists():
        if not overwrite:
            raise RuntimeError(f"Output already exists. Pass --overwrite to replace it: {output}")
        if output == input_root or output in input_root.parents:
            raise RuntimeError(f"Refusing to overwrite input path or one of its parents: {output}")
        shutil.rmtree(output)
    output.mkdir(parents=True, exist_ok=True)


def arrow_dtype(dtype: str) -> pa.DataType:
    return pa.from_numpy_dtype(np.dtype(dtype))


def write_parquet_episode(output_path: Path, episode_data: dict[str, Any], features: dict[str, dict[str, Any]]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    columns = []
    names = []
    for key, feature in features.items():
        if feature["dtype"] == "video":
            continue

        values = episode_data[key]
        if feature["shape"] == (1,):
            array = pa.array(np.asarray(values, dtype=np.dtype(feature["dtype"])).reshape(-1), type=arrow_dtype(feature["dtype"]))
        elif len(feature["shape"]) == 1:
            np_values = np.asarray(values, dtype=np.dtype(feature["dtype"]))
            if np_values.ndim == 1:
                np_values = np_values.reshape(-1, 1)
            flat = pa.array(np_values.reshape(-1), type=arrow_dtype(feature["dtype"]))
            array = pa.FixedSizeListArray.from_arrays(flat, list_size=np_values.shape[1])
        else:
            array = pa.array(np.asarray(values).tolist())

        columns.append(array)
        names.append(key)

    table = pa.Table.from_arrays(columns, names=names)
    pq.write_table(table, output_path)


def encode_video_from_images(image_dir: Path, video_path: Path, fps: int) -> None:
    video_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-framerate",
        str(fps),
        "-start_number",
        "0",
        "-i",
        str(image_dir / "frame_%06d.png"),
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-crf",
        "18",
        "-movflags",
        "+faststart",
        str(video_path),
    ]
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError(
            f"ffmpeg failed for {video_path}:\n{result.stderr.strip() or result.stdout.strip()}"
        )


@dataclass
class LeRobotV21Writer:
    root: Path
    repo_id: str
    fps: int
    robot_type: str
    features: dict[str, dict[str, Any]]
    total_frames: int = 0
    total_episodes: int = 0
    total_tasks: int = 0
    total_videos: int = 0
    total_chunks: int = 0
    tasks: dict[int, str] = field(default_factory=dict)
    task_to_index: dict[str, int] = field(default_factory=dict)
    episodes: dict[int, dict[str, Any]] = field(default_factory=dict)
    episodes_stats: dict[int, dict[str, dict[str, np.ndarray]]] = field(default_factory=dict)
    stats: dict[str, dict[str, np.ndarray]] = field(default_factory=dict)
    episode_buffer: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)
        self.video_keys = [key for key, ft in self.features.items() if ft["dtype"] == "video"]
        self.full_features = {**self.features, **DEFAULT_FEATURES}
        self.parquet_keys = [key for key, ft in self.full_features.items() if ft["dtype"] != "video"]
        self.data_path = DATA_PATH
        self.video_path = VIDEO_PATH
        self.image_root = self.root / "images"
        self.data_root = self.root / "data"
        self.video_root = self.root / "videos"
        self.meta_root = self.root / "meta"
        self.info = self._create_info()
        write_json(self.info, self.meta_root / "info.json")
        self.episode_buffer = self.create_episode_buffer()

    def _create_info(self) -> dict[str, Any]:
        info_features = {**self.features, **DEFAULT_FEATURES}
        for key in self.video_keys:
            info_features[key] = {**info_features[key], "info": video_info_from_feature(info_features[key], self.fps)}
        return {
            "codebase_version": CODEBASE_VERSION,
            "robot_type": self.robot_type,
            "total_episodes": 0,
            "total_frames": 0,
            "total_tasks": 0,
            "total_videos": 0,
            "total_chunks": 0,
            "chunks_size": DEFAULT_CHUNK_SIZE,
            "fps": self.fps,
            "splits": {},
            "data_path": self.data_path,
            "video_path": self.video_path,
            "features": info_features,
        }

    def create_episode_buffer(self, episode_index: int | None = None) -> dict[str, Any]:
        current_ep_idx = self.total_episodes if episode_index is None else episode_index
        return {
            "episode_index": current_ep_idx,
            "size": 0,
            "task": [],
            "timestamp": [],
            OBS_STATE_KEY: [],
            ACTION_KEY: [],
            **{key: [] for key in self.video_keys},
        }

    def get_task_index(self, task: str) -> int | None:
        return self.task_to_index.get(task)

    def add_task(self, task: str) -> None:
        if task in self.task_to_index:
            return
        task_index = self.total_tasks
        self.task_to_index[task] = task_index
        self.tasks[task_index] = task
        self.total_tasks += 1
        self.info["total_tasks"] = self.total_tasks
        append_jsonline({"task_index": task_index, "task": task}, self.meta_root / "tasks.jsonl")

    def add_frame(self, frame: dict[str, Any]) -> None:
        if self.episode_buffer is None:
            self.episode_buffer = self.create_episode_buffer()

        episode_index = self.episode_buffer["episode_index"]
        frame_index = self.episode_buffer["size"]
        task = str(frame["task"])
        timestamp = float(frame["timestamp"])

        self.episode_buffer["task"].append(task)
        self.episode_buffer["timestamp"].append(timestamp)
        self.episode_buffer[OBS_STATE_KEY].append(np.asarray(frame[OBS_STATE_KEY], dtype=np.float32))
        self.episode_buffer[ACTION_KEY].append(np.asarray(frame[ACTION_KEY], dtype=np.float32))

        for key in self.video_keys:
            image = np.asarray(frame[key], dtype=np.uint8)
            image_path = self._get_image_file_path(episode_index, key, frame_index)
            image_path.parent.mkdir(parents=True, exist_ok=True)
            PILImage.fromarray(image, mode="RGB").save(image_path)
            self.episode_buffer[key].append(str(image_path))

        self.episode_buffer["size"] += 1

    def _get_image_file_path(self, episode_index: int, image_key: str, frame_index: int) -> Path:
        return self.root / IMAGE_PATH.format(
            image_key=image_key, episode_index=episode_index, frame_index=frame_index
        )

    def _get_parquet_file_path(self, episode_index: int) -> Path:
        return self.root / DATA_PATH.format(
            episode_chunk=episode_index // self.chunks_size,
            episode_index=episode_index,
        )

    def _get_video_file_path(self, episode_index: int, video_key: str) -> Path:
        return self.root / VIDEO_PATH.format(
            episode_chunk=episode_index // self.chunks_size,
            video_key=video_key,
            episode_index=episode_index,
        )

    @property
    def chunks_size(self) -> int:
        return int(self.info["chunks_size"])

    def save_episode(self) -> dict[str, Any]:
        if self.episode_buffer is None:
            raise RuntimeError("No episode buffer available.")

        episode_buffer = self.episode_buffer
        episode_length = int(episode_buffer["size"])
        if episode_length <= 0:
            raise ValueError("Episode buffer is empty.")

        episode_index = int(episode_buffer["episode_index"])
        tasks = episode_buffer.pop("task")
        episode_tasks = list(dict.fromkeys(tasks))

        for task in episode_tasks:
            self.add_task(task)

        episode_data: dict[str, Any] = {
            "timestamp": np.asarray(episode_buffer["timestamp"], dtype=np.float32),
            "frame_index": np.arange(episode_length, dtype=np.int64),
            "episode_index": np.full((episode_length,), episode_index, dtype=np.int64),
            "index": np.arange(self.total_frames, self.total_frames + episode_length, dtype=np.int64),
            "task_index": np.asarray([self.get_task_index(task) for task in tasks], dtype=np.int64),
            OBS_STATE_KEY: np.stack(episode_buffer[OBS_STATE_KEY]).astype(np.float32, copy=False),
            ACTION_KEY: np.stack(episode_buffer[ACTION_KEY]).astype(np.float32, copy=False),
        }
        for key in self.video_keys:
            episode_data[key] = list(episode_buffer[key])

        parquet_data = {}
        for key in self.parquet_keys:
            feature = self.full_features[key]
            value = episode_data[key]
            if feature["shape"] == (1,):
                parquet_data[key] = np.asarray(value, dtype=np.dtype(feature["dtype"])).reshape(-1)
            elif len(feature["shape"]) == 1:
                parquet_data[key] = np.asarray(value, dtype=np.dtype(feature["dtype"]))
            else:
                parquet_data[key] = np.asarray(value)

        parquet_path = self._get_parquet_file_path(episode_index)
        write_parquet_episode(parquet_path, parquet_data, self.full_features)

        episode_stats = compute_episode_stats(episode_data, self.full_features)

        video_paths: dict[str, Path] = {}
        for key in self.video_keys:
            image_dir = self._get_image_file_path(episode_index, key, 0).parent
            video_path = self._get_video_file_path(episode_index, key)
            encode_video_from_images(image_dir, video_path, self.fps)
            video_paths[key] = video_path

        self.info["total_episodes"] += 1
        self.info["total_frames"] += episode_length
        self.info["total_videos"] += len(self.video_keys)
        self.info["splits"] = {"train": f"0:{self.info['total_episodes']}"}
        chunk = episode_index // self.chunks_size
        if chunk >= self.info["total_chunks"]:
            self.info["total_chunks"] = chunk + 1
        write_json(self.info, self.meta_root / "info.json")

        self.episodes[episode_index] = {
            "episode_index": episode_index,
            "tasks": episode_tasks,
            "length": episode_length,
        }
        append_jsonline(self.episodes[episode_index], self.meta_root / "episodes.jsonl")

        self.episodes_stats[episode_index] = episode_stats
        self.stats = aggregate_stats([self.stats, episode_stats]) if self.stats else episode_stats
        append_jsonline(
            {"episode_index": episode_index, "stats": serialize_stats(episode_stats)},
            self.meta_root / "episodes_stats.jsonl",
        )
        write_json(serialize_stats(self.stats), self.meta_root / "stats.json")

        if self.image_root.is_dir():
            shutil.rmtree(self.image_root)

        self.total_episodes += 1
        self.total_frames += episode_length
        self.episode_buffer = self.create_episode_buffer()

        return {
            "episode_index": episode_index,
            "frames": episode_length,
            "tasks": episode_tasks,
            "parquet_path": str(parquet_path.relative_to(self.root)),
            "video_paths": {key: str(path.relative_to(self.root)) for key, path in video_paths.items()},
        }


def convert_session_to_episode_v21(
    *,
    writer: LeRobotV21Writer,
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
                writer.add_frame(build_frame(plan, latest, task, target_ns / 1_000_000_000.0))
                frame_count += 1
            target_ns += step_ns

        msg = decode_selected_message(plan, topic, raw_data)
        latest[topic] = msg

    while target_ns <= plan.end_ns and can_write_more():
        if all(topic in latest for topic in selected_topics):
            writer.add_frame(build_frame(plan, latest, task, target_ns / 1_000_000_000.0))
            frame_count += 1
        target_ns += step_ns

    return frame_count


def decode_selected_message(plan: SessionPlan, topic: str, raw_data: Any) -> np.ndarray:
    from rclpy.serialization import deserialize_message

    msg = deserialize_message(raw_data, plan.topic_classes[topic])
    return _decode_selected_message(plan, topic, msg)


def _decode_selected_message(plan: SessionPlan, topic: str, msg: Any) -> np.ndarray:
    msg_type = plan.topic_types[topic]
    if msg_type == IMAGE_TYPE:
        return decode_selected_message_image(plan, topic, msg)
    from convert_to_lerobot import vector_from_message  # noqa: E402

    vector, _ = vector_from_message(msg, topic, joint_name_filter=plan.joint_name_filter)
    expected_dim = plan.probes[topic].vector_dim
    if expected_dim is not None and vector.shape[0] != expected_dim:
        raise RuntimeError(f"Vector dimension changed for {topic}: {vector.shape[0]} != {expected_dim}")
    return vector


def decode_selected_message_image(plan: SessionPlan, topic: str, msg: Any) -> np.ndarray:
    from convert_to_lerobot import decode_image_message, preprocess_image_for_topic  # noqa: E402

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


def print_dry_run(plans: list[SessionPlan], args: argparse.Namespace) -> None:
    first_plan = plans[0]
    print("LeRobot v2.1 conversion dry run")
    print(f"episodes: {len(plans)}")
    print(f"fps: {first_plan.fps}")
    print(f"crop_size: {first_plan.crop_size}")
    print(f"resize_size: {first_plan.resize_size}")
    print(f"joint_name_filter: {args.joint_name_filter}")
    print("features:")
    print(json.dumps(json_safe({**first_plan.features, **DEFAULT_FEATURES}), indent=2, ensure_ascii=False))
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


def parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert data_collection_stack session rosbag directories to a LeRobot v2.1 dataset.",
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
    args = parser.parse_args(argv)

    if args.right_side_only:
        if args.joint_name_filter:
            parser.error("--right-side-only cannot be combined with --joint-name-filter.")
        args.joint_name_filter = r"(^right_|_R$)"
    if args.joint_name_filter:
        import re

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
    prepare_output_dir(args.input.resolve(), output, args.overwrite)
    writer = LeRobotV21Writer(
        root=output,
        repo_id=args.repo_id or f"local/{output.name}",
        fps=first_plan.fps,
        robot_type=args.robot_type,
        features=first_plan.features,
    )

    conversion_summary: dict[str, Any] = {
        "format": CODEBASE_VERSION,
        "input": str(args.input.resolve()),
        "output": str(output),
        "repo_id": args.repo_id or f"local/{output.name}",
        "fps": first_plan.fps,
        "joint_name_filter": args.joint_name_filter,
        "crop": {
            "enabled": bool(args.crop),
            "size": list(args.crop_size) if args.crop_size else None,
        },
        "resize": {
            "enabled": bool(args.resize_size),
            "size": list(args.resize_size) if args.resize_size else None,
        },
        "features": json_safe({**first_plan.features, **DEFAULT_FEATURES}),
        "episodes": [],
    }

    total_frames = 0
    for episode_index, plan in enumerate(plans):
        frame_count = convert_session_to_episode_v21(
            writer=writer,
            plan=plan,
            task=args.task or default_task(plan),
            frame_limit=None if args.limit_frames is None else args.limit_frames - total_frames,
        )
        if frame_count <= 0:
            raise RuntimeError(f"No frames were written for {plan.session_dir}")

        episode_summary = writer.save_episode()
        total_frames += frame_count
        conversion_summary["episodes"].append(
            {
                **episode_summary,
                "episode_index": episode_index,
                "session_dir": str(plan.session_dir),
                "bag_dir": str(plan.bag_dir),
                "task": args.task or default_task(plan),
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

    write_json(conversion_summary, output / "conversion_manifest.json")
    print(f"Finished LeRobot v2.1 dataset at {output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as exc:
        print(f"error: {exc}", file=sys.stderr)
        raise SystemExit(1)
