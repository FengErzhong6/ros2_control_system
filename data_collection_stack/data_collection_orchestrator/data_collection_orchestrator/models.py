from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional


@dataclass(frozen=True)
class StartupPolicy:
    startup_timeout_sec: float = 20.0
    ready_stability_window_sec: float = 2.0
    retry_count: int = 2
    retry_backoff_sec: float = 1.0


@dataclass(frozen=True)
class RecordingPolicy:
    session_root: Path = Path("~/.ros/data_collection").expanduser()
    bag_directory_name: str = "rosbag"
    metadata_file_name: str = "metadata.json"
    event_log_file_name: str = "event_log.txt"
    timing_report_file_name: str = "timing_report.json"
    snapshot_recipe: bool = True
    snapshot_site_config: bool = True
    snapshot_startup_policy: bool = True
    snapshot_fault_policy: bool = True
    snapshot_time_sync_config: bool = True
    snapshot_robot_description: bool = True
    snapshot_camera_calibration: bool = True
    prepare_recorder_on_start_system: bool = False
    use_start_paused: bool = True
    recorder_node_name_prefix: str = "data_collection_recorder"
    service_timeout_sec: float = 10.0
    stop_timeout_sec: float = 10.0
    cleanup_timeout_sec: float = 5.0
    storage_id: str = "mcap"
    compression_mode: str = "none"
    compression_format: str = "zstd"
    capture_git_state: bool = True
    preview_drop_policy: str = "latest_only"
    require_time_sync_healthy: bool = True
    time_sync_mode: str = "relay_ros_clock"
    canonical_timestamp_clock: str = "relay_ros_clock"
    canonical_timestamp_semantics: dict[str, str] = field(
        default_factory=lambda: {
            "image": "relay_receive_time",
            "joint_states": "relay_receive_time",
        }
    )
    relay_record_topics: bool = False
    relay_record_topic_prefix: str = "/record"
    canonical_joint_state_topic: str = "/joint_states"
    time_sync_max_offset_ns: int = 1_000_000
    joint_state_min_rate_hz: float = 100.0
    joint_state_max_jitter_ms: float = 2.0
    post_session_timing_validation: bool = True
    training_ready_requires_canonical_timestamps: bool = True


@dataclass(frozen=True)
class SupervisorConfig:
    recipe_id: str
    recipe_directory: Optional[Path]
    startup_policy_config: Optional[Path]
    fault_policy_config: Optional[Path]
    recording_config: Optional[Path]
    cameras_config: Optional[Path]
    trackers_config: Optional[Path]
    manus_config: Optional[Path]
    manus_user_name: Optional[str]
    wujihand_identity_file: Optional[Path]
    wujihand_teleop_config: Optional[Path]
    wujihand_use_mock_hardware: bool = False
    mock_manus: bool = False
    marvin_mock_grippers: bool = False


@dataclass(frozen=True)
class DeviceSpec:
    device_id: str
    adapter: str
    required: bool = True
    depends_on: list[str] = field(default_factory=list)
    config: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class RecipeSpec:
    recipe_id: str
    path: Path
    devices: list[DeviceSpec] = field(default_factory=list)
    record_topics: list[str] = field(default_factory=list)


@dataclass
class SessionArtifacts:
    session_dir: Path
    bag_dir: Path
    metadata_path: Path
    event_log_path: Path
    timing_report_path: Path
    snapshot_paths: dict[str, Path] = field(default_factory=dict)


@dataclass
class RecorderRuntime:
    node_name: str
    command: list[str] = field(default_factory=list)
    pid: int | None = None
    pgid: int | None = None
    log_path: Path | None = None
    prepared: bool = False
    recording_active: bool = False
    sync_health_checked: bool = False
    sync_health_summary: dict[str, Any] = field(default_factory=dict)


@dataclass
class ActiveSession:
    session_id: str
    recipe_id: str
    operator_id: str
    session_name: str
    session_root: Path | None = None
    site_name: str = ""
    artifacts: SessionArtifacts | None = None
    record_topics: list[str] = field(default_factory=list)
    recorder_node_name: str = ""
    session_start_ros_time_ns: int | None = None
    session_stop_ros_time_ns: int | None = None
    session_start_wall_time: str = ""
    session_stop_wall_time: str = ""
    session_start_monotonic_sec: float | None = None
    session_stop_monotonic_sec: float | None = None
    time_sync_summary: dict[str, Any] = field(default_factory=dict)
    timing_validation_summary: dict[str, Any] = field(default_factory=dict)
