#!/usr/bin/env python3

import argparse
import json
import math
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message, serialize_message
from scipy import signal
from sensor_msgs.msg import JointState


DEFAULT_TOPIC = "/joint_states"
DEFAULT_STORAGE_ID = "mcap"


@dataclass
class JointStats:
    name: str
    raw_max_velocity: float
    cleaned_max_velocity: float
    raw_max_acceleration: float
    cleaned_max_acceleration: float
    max_abs_position_delta: float
    rms_position_delta: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Clean a recorded joint-state rosbag into a smoother replay-friendly rosbag. "
            "Pipeline: resample -> Hampel outlier removal -> zero-phase smoothing "
            "-> velocity/acceleration limiting."
        )
    )
    parser.add_argument("input_bag", help="Input rosbag directory.")
    parser.add_argument(
        "-o",
        "--output-bag",
        help="Output rosbag directory. Default: <input_bag>_cleaned",
    )
    parser.add_argument(
        "--topic",
        default=DEFAULT_TOPIC,
        help=f"JointState topic to clean. Default: {DEFAULT_TOPIC}",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=100.0,
        help="Uniform resampling rate in Hz. Default: 100.0",
    )
    parser.add_argument(
        "--filter",
        choices=("savgol", "butter", "none"),
        default="savgol",
        help="Smoothing filter type. Default: savgol",
    )
    parser.add_argument(
        "--savgol-window",
        type=int,
        default=21,
        help="Savitzky-Golay window length in samples (odd). Default: 21",
    )
    parser.add_argument(
        "--savgol-polyorder",
        type=int,
        default=3,
        help="Savitzky-Golay polynomial order. Default: 3",
    )
    parser.add_argument(
        "--butter-cutoff-hz",
        type=float,
        default=4.0,
        help="Butterworth low-pass cutoff frequency in Hz. Default: 4.0",
    )
    parser.add_argument(
        "--butter-order",
        type=int,
        default=3,
        help="Butterworth filter order. Default: 3",
    )
    parser.add_argument(
        "--hampel-window",
        type=int,
        default=9,
        help="Hampel window size in samples. Default: 9",
    )
    parser.add_argument(
        "--hampel-sigma",
        type=float,
        default=3.0,
        help="Hampel outlier threshold in MAD sigmas. Default: 3.0",
    )
    parser.add_argument(
        "--median-window",
        type=int,
        default=5,
        help="Optional median filter window in samples. <=1 disables it. Default: 5",
    )
    parser.add_argument(
        "--max-velocity",
        type=float,
        default=1.5,
        help="Maximum allowed joint velocity in rad/s. <=0 disables it. Default: 1.5",
    )
    parser.add_argument(
        "--max-acceleration",
        type=float,
        default=6.0,
        help="Maximum allowed joint acceleration in rad/s^2. <=0 disables it. Default: 6.0",
    )
    parser.add_argument(
        "--storage-id",
        default=DEFAULT_STORAGE_ID,
        help=f"Output rosbag storage id. Default: {DEFAULT_STORAGE_ID}",
    )
    parser.add_argument(
        "--summary-path",
        default=None,
        help="Optional JSON summary path. Default: <output_bag>/cleaning_summary.json",
    )
    return parser.parse_args()


def load_storage_id(bag_dir: Path) -> str:
    metadata_path = bag_dir / "metadata.yaml"
    if not metadata_path.is_file():
        raise FileNotFoundError(f"metadata.yaml not found in bag directory: {bag_dir}")

    with metadata_path.open("r", encoding="utf-8") as handle:
        metadata = yaml.safe_load(handle) or {}

    return metadata["rosbag2_bagfile_information"]["storage_identifier"]


def make_output_path(input_bag: Path, output_bag: str | None) -> Path:
    if output_bag:
        return Path(output_bag).expanduser().resolve()
    return input_bag.parent / f"{input_bag.name}_cleaned"


def open_reader(bag_dir: Path, topic: str) -> Tuple[rosbag2_py.SequentialReader, Dict[str, str]]:
    storage_id = load_storage_id(bag_dir)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise RuntimeError(
            f"Topic '{topic}' not found in bag. Available topics: {sorted(topic_types)}"
        )
    if topic_types[topic] != "sensor_msgs/msg/JointState":
        raise RuntimeError(
            f"Topic '{topic}' has unsupported type '{topic_types[topic]}'. "
            "Expected sensor_msgs/msg/JointState."
        )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    return reader, topic_types


def read_joint_state_series(
    bag_dir: Path, topic: str
) -> Tuple[np.ndarray, List[str], np.ndarray]:
    reader, _ = open_reader(bag_dir, topic)

    timestamps_ns: List[int] = []
    joint_names: List[str] | None = None
    samples: List[List[float]] = []

    while reader.has_next():
        topic_name, serialized_data, timestamp_ns = reader.read_next()
        if topic_name != topic:
            continue

        message = deserialize_message(serialized_data, JointState)
        if joint_names is None:
            joint_names = [str(name) for name in message.name]
            if not joint_names:
                raise RuntimeError("First JointState message has no joint names.")
        index_by_name = {name: idx for idx, name in enumerate(message.name)}
        sample = []
        for joint_name in joint_names:
            idx = index_by_name.get(joint_name)
            if idx is None or idx >= len(message.position):
                raise RuntimeError(
                    f"Joint '{joint_name}' missing from message at timestamp {timestamp_ns}."
                )
            sample.append(float(message.position[idx]))

        timestamps_ns.append(int(timestamp_ns))
        samples.append(sample)

    if not timestamps_ns or joint_names is None:
        raise RuntimeError(f"No JointState messages found on topic '{topic}'.")

    timestamps = np.asarray(timestamps_ns, dtype=np.int64)
    if len(timestamps) >= 2 and np.any(np.diff(timestamps) <= 0):
        raise RuntimeError("Bag timestamps are not strictly increasing.")

    return timestamps, joint_names, np.asarray(samples, dtype=np.float64)


def build_uniform_timeline(
    timestamps_ns: np.ndarray, sample_rate_hz: float
) -> np.ndarray:
    if sample_rate_hz <= 0.0:
        raise RuntimeError("sample_rate must be > 0.")
    start_ns = int(timestamps_ns[0])
    end_ns = int(timestamps_ns[-1])
    step_ns = int(round(1_000_000_000.0 / sample_rate_hz))
    if step_ns <= 0:
        raise RuntimeError("sample_rate produced an invalid time step.")

    count = max(2, int(math.floor((end_ns - start_ns) / step_ns)) + 1)
    uniform = start_ns + np.arange(count, dtype=np.int64) * step_ns
    if uniform[-1] < end_ns:
        uniform = np.append(uniform, np.int64(end_ns))
    return uniform


def interpolate_series(
    source_times_ns: np.ndarray, source_positions: np.ndarray, target_times_ns: np.ndarray
) -> np.ndarray:
    source_times = source_times_ns.astype(np.float64) / 1.0e9
    target_times = target_times_ns.astype(np.float64) / 1.0e9
    out = np.empty((len(target_times_ns), source_positions.shape[1]), dtype=np.float64)
    for joint_index in range(source_positions.shape[1]):
        out[:, joint_index] = np.interp(target_times, source_times, source_positions[:, joint_index])
    return out


def make_odd_window(value: int, upper_bound: int) -> int:
    if upper_bound <= 0:
        return 0
    window = max(1, int(value))
    if window % 2 == 0:
        window += 1
    if window > upper_bound:
        window = upper_bound if upper_bound % 2 == 1 else upper_bound - 1
    return max(1, window)


def hampel_filter(series: np.ndarray, window_size: int, sigma: float) -> np.ndarray:
    if window_size <= 1 or sigma <= 0.0:
        return series.copy()

    n = len(series)
    radius = window_size // 2
    filtered = series.copy()
    for index in range(n):
        left = max(0, index - radius)
        right = min(n, index + radius + 1)
        window = series[left:right]
        median = np.median(window)
        mad = np.median(np.abs(window - median))
        if mad <= 1e-12:
            continue
        threshold = sigma * 1.4826 * mad
        if abs(series[index] - median) > threshold:
            filtered[index] = median
    return filtered


def median_filter(series: np.ndarray, window_size: int) -> np.ndarray:
    if window_size <= 1:
        return series.copy()
    window = make_odd_window(window_size, len(series))
    if window <= 1:
        return series.copy()
    return signal.medfilt(series, kernel_size=window)


def smooth_series(
    series: np.ndarray,
    *,
    filter_type: str,
    sample_rate_hz: float,
    savgol_window: int,
    savgol_polyorder: int,
    butter_cutoff_hz: float,
    butter_order: int,
) -> np.ndarray:
    if filter_type == "none" or len(series) < 3:
        return series.copy()

    if filter_type == "savgol":
        window = make_odd_window(savgol_window, len(series))
        if window <= savgol_polyorder:
            return series.copy()
        return signal.savgol_filter(series, window_length=window, polyorder=savgol_polyorder, mode="interp")

    if filter_type == "butter":
        nyquist = 0.5 * sample_rate_hz
        if butter_cutoff_hz <= 0.0 or butter_cutoff_hz >= nyquist:
            return series.copy()
        sos = signal.butter(
            butter_order,
            butter_cutoff_hz / nyquist,
            btype="low",
            output="sos",
        )
        return signal.sosfiltfilt(sos, series)

    raise RuntimeError(f"Unsupported filter type: {filter_type}")


def limit_velocity(series: np.ndarray, dt: float, max_velocity: float) -> np.ndarray:
    if max_velocity <= 0.0 or len(series) < 2:
        return series.copy()
    max_delta = max_velocity * dt
    limited = series.copy()
    for index in range(1, len(limited)):
        delta = limited[index] - limited[index - 1]
        if abs(delta) > max_delta:
            limited[index] = limited[index - 1] + math.copysign(max_delta, delta)
    return limited


def limit_acceleration(series: np.ndarray, dt: float, max_acceleration: float) -> np.ndarray:
    if max_acceleration <= 0.0 or len(series) < 3:
        return series.copy()

    velocities = np.diff(series) / dt
    limited_velocities = velocities.copy()
    max_delta_v = max_acceleration * dt
    for index in range(1, len(limited_velocities)):
        delta_v = limited_velocities[index] - limited_velocities[index - 1]
        if abs(delta_v) > max_delta_v:
            limited_velocities[index] = (
                limited_velocities[index - 1] + math.copysign(max_delta_v, delta_v)
            )

    limited = np.empty_like(series)
    limited[0] = series[0]
    for index in range(1, len(series)):
        limited[index] = limited[index - 1] + limited_velocities[index - 1] * dt
    return limited


def compute_velocity_and_acceleration(series: np.ndarray, dt: float) -> Tuple[float, float]:
    if len(series) < 2:
        return 0.0, 0.0
    velocity = np.diff(series) / dt
    max_velocity = float(np.max(np.abs(velocity))) if len(velocity) > 0 else 0.0
    if len(velocity) < 2:
        return max_velocity, 0.0
    acceleration = np.diff(velocity) / dt
    max_acceleration = float(np.max(np.abs(acceleration))) if len(acceleration) > 0 else 0.0
    return max_velocity, max_acceleration


def clean_positions(
    joint_names: Sequence[str],
    uniform_positions: np.ndarray,
    *,
    sample_rate_hz: float,
    filter_type: str,
    savgol_window: int,
    savgol_polyorder: int,
    butter_cutoff_hz: float,
    butter_order: int,
    hampel_window: int,
    hampel_sigma: float,
    median_window: int,
    max_velocity: float,
    max_acceleration: float,
) -> Tuple[np.ndarray, List[JointStats]]:
    dt = 1.0 / sample_rate_hz
    cleaned = np.empty_like(uniform_positions)
    stats: List[JointStats] = []

    for joint_index, joint_name in enumerate(joint_names):
        raw = uniform_positions[:, joint_index]
        filtered = hampel_filter(raw, hampel_window, hampel_sigma)
        filtered = median_filter(filtered, median_window)
        filtered = smooth_series(
            filtered,
            filter_type=filter_type,
            sample_rate_hz=sample_rate_hz,
            savgol_window=savgol_window,
            savgol_polyorder=savgol_polyorder,
            butter_cutoff_hz=butter_cutoff_hz,
            butter_order=butter_order,
        )
        filtered = limit_velocity(filtered, dt, max_velocity)
        filtered = limit_acceleration(filtered, dt, max_acceleration)
        cleaned[:, joint_index] = filtered

        raw_v, raw_a = compute_velocity_and_acceleration(raw, dt)
        clean_v, clean_a = compute_velocity_and_acceleration(filtered, dt)
        delta = filtered - raw
        stats.append(
            JointStats(
                name=joint_name,
                raw_max_velocity=raw_v,
                cleaned_max_velocity=clean_v,
                raw_max_acceleration=raw_a,
                cleaned_max_acceleration=clean_a,
                max_abs_position_delta=float(np.max(np.abs(delta))),
                rms_position_delta=float(np.sqrt(np.mean(delta * delta))),
            )
        )

    return cleaned, stats


def write_cleaned_bag(
    output_bag: Path,
    topic: str,
    joint_names: Sequence[str],
    timestamps_ns: np.ndarray,
    positions: np.ndarray,
    storage_id: str,
) -> None:
    if output_bag.exists():
        raise RuntimeError(f"Output path already exists: {output_bag}")

    output_bag.parent.mkdir(parents=True, exist_ok=True)

    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(output_bag), storage_id=storage_id),
        rosbag2_py.ConverterOptions("", ""),
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            0,
            topic,
            "sensor_msgs/msg/JointState",
            "cdr",
            [],
            "",
        )
    )

    for timestamp_ns, sample in zip(timestamps_ns, positions, strict=True):
        msg = JointState()
        msg.header.stamp.sec = int(timestamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(timestamp_ns % 1_000_000_000)
        msg.name = list(joint_names)
        msg.position = [float(value) for value in sample]
        writer.write(topic, serialize_message(msg), int(timestamp_ns))

    writer.close()


def write_summary(
    summary_path: Path,
    *,
    input_bag: Path,
    output_bag: Path,
    topic: str,
    sample_rate_hz: float,
    filter_type: str,
    stats: Sequence[JointStats],
) -> None:
    summary = {
        "input_bag": str(input_bag),
        "output_bag": str(output_bag),
        "topic": topic,
        "sample_rate_hz": sample_rate_hz,
        "filter_type": filter_type,
        "joints": [
            {
                "name": item.name,
                "raw_max_velocity": item.raw_max_velocity,
                "cleaned_max_velocity": item.cleaned_max_velocity,
                "raw_max_acceleration": item.raw_max_acceleration,
                "cleaned_max_acceleration": item.cleaned_max_acceleration,
                "max_abs_position_delta": item.max_abs_position_delta,
                "rms_position_delta": item.rms_position_delta,
            }
            for item in stats
        ],
    }
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2, ensure_ascii=True)


def print_summary(stats: Sequence[JointStats]) -> None:
    print("Joint cleaning summary:")
    for item in stats:
        print(
            f"  {item.name}: "
            f"vel {item.raw_max_velocity:.3f}->{item.cleaned_max_velocity:.3f} rad/s, "
            f"acc {item.raw_max_acceleration:.3f}->{item.cleaned_max_acceleration:.3f} rad/s^2, "
            f"max|dq|={item.max_abs_position_delta:.4f} rad, "
            f"rms|dq|={item.rms_position_delta:.4f} rad"
        )


def main() -> int:
    args = parse_args()
    input_bag = Path(args.input_bag).expanduser().resolve()
    if not input_bag.is_dir():
        print(f"Input bag not found: {input_bag}")
        return 1

    output_bag = make_output_path(input_bag, args.output_bag)
    summary_path = (
        Path(args.summary_path).expanduser().resolve()
        if args.summary_path
        else output_bag / "cleaning_summary.json"
    )

    timestamps_ns, joint_names, raw_positions = read_joint_state_series(input_bag, args.topic)
    uniform_timestamps_ns = build_uniform_timeline(timestamps_ns, args.sample_rate)
    uniform_positions = interpolate_series(timestamps_ns, raw_positions, uniform_timestamps_ns)
    cleaned_positions, stats = clean_positions(
        joint_names,
        uniform_positions,
        sample_rate_hz=args.sample_rate,
        filter_type=args.filter,
        savgol_window=args.savgol_window,
        savgol_polyorder=args.savgol_polyorder,
        butter_cutoff_hz=args.butter_cutoff_hz,
        butter_order=args.butter_order,
        hampel_window=args.hampel_window,
        hampel_sigma=args.hampel_sigma,
        median_window=args.median_window,
        max_velocity=args.max_velocity,
        max_acceleration=args.max_acceleration,
    )

    try:
        write_cleaned_bag(
            output_bag,
            args.topic,
            joint_names,
            uniform_timestamps_ns,
            cleaned_positions,
            args.storage_id,
        )
        write_summary(
            summary_path,
            input_bag=input_bag,
            output_bag=output_bag,
            topic=args.topic,
            sample_rate_hz=args.sample_rate,
            filter_type=args.filter,
            stats=stats,
        )
    except Exception:
        if output_bag.exists():
            shutil.rmtree(output_bag, ignore_errors=True)
        raise

    print(f"Input bag:  {input_bag}")
    print(f"Output bag: {output_bag}")
    print(f"Summary:    {summary_path}")
    print_summary(stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
