# Data Collection Recording Implementation Plan

## 1. Goal

Implement an industrial-grade recording pipeline for `data_collection_stack` so that `StartSession` and `StopSession` control an actual `ros2 bag` recorder for raw camera images and real Marvin/WujiHand joint states, while preserving teleoperation smoothness and controller stability.

The final target is not merely “a bag that exists”, but a dataset whose camera and robot timelines are deterministic, auditable, and reliable enough for production-quality downstream use.

This document is a refinement of the earlier high-level plan. It is intended to be directly actionable by mapping the design to concrete files, APIs, configuration keys, state transitions, and validation steps.

Primary outcomes:

- `Start Collection` creates a real bag and metadata on disk
- recorded topics stay limited to raw camera images and real robot joint feedback
- timestamps come from trusted producer-side time definitions rather than recorder arrival order
- session start is gated by explicit synchronization health checks
- calibration, timing provenance, and validation evidence are saved as session artifacts
- recording overhead stays off the real-time control path
- failure handling is explicit and recoverable

## 2. Confirmed Current Gap

The current collection stack already has:

- a top-level launch entrypoint
- a supervisor state machine
- session actions in UI and orchestrator
- recipe loading with `record_topics`
- session id allocation
- per-device `before_session()` / `after_session()` hooks

But it does **not** have a working recorder pipeline.

Current reality by file:

- `data_collection_bringup/launch/collection_app.launch.py`
  - launches supervisor and optional UI only
- `data_collection_bringup/launch/headless_supervisor.launch.py`
  - launches supervisor only
- `data_collection_orchestrator/supervisor.py`
  - `execute_start_session()` enters `RECORDING`
  - no bag process is spawned
  - no metadata file is written
- `data_collection_orchestrator/record_manager.py`
  - currently a placeholder only
- `data_collection_orchestrator/session_manager.py`
  - allocates `session_id` only
- `data_collection_bringup/config/session/recording.yaml`
  - exists, but is not currently loaded into supervisor config

Conclusion:

The collection stack currently implements a **session state machine**, not a **recording system**.

## 3. Scope and Non-Goals

### 3.1 Scope

This plan covers:

- supervisor-owned rosbag lifecycle
- session output directory management
- metadata persistence
- recipe topic resolution
- recording-related configuration
- timestamp and alignment readiness checks
- validation for recording performance and correctness

### 3.2 Non-Goals

The following are intentionally out of scope for this plan:

- custom dataset export format beyond raw bag + metadata + timing diagnostics
- in-process rosbag writing with `rosbag2_py`
- online compression tuning for every machine profile
- cloud upload or remote archival during collection
- automatic semantic labeling or annotation tools

Implementation can still land incrementally, but the target architecture in this document is the industrial-grade end state rather than a best-effort prototype.

## 4. Industrial-Grade Recording Contract and Immediate Data Gaps

The target dataset is intentionally narrower than the full live teleoperation stack. Even if HTC tracker and Manus glove remain runtime dependencies for some collection flows, the recorder does not need to persist their streams.

The default `record_topics` should remain:

- `/cam_high/cam_high/color/image_raw`
- `/cam_left_wrist/image_raw`
- `/cam_right_wrist/image_raw`
- `/joint_states`

This contract assumes that `/joint_states` contains the real Marvin and WujiHand measured joint feedback needed for reconstruction.

Industrial-grade alignment depends on more than the bag topic list. The session contract must also preserve:

- one canonical timestamp contract shared by every recorded stream in the dataset
- authoritative camera calibration and extrinsic provenance
- robot-description and joint-name provenance
- synchronization health evidence captured before recording starts
- post-session timing validation evidence captured after recording stops

For training datasets, unified timestamp semantics are a hard requirement, not an optional quality improvement.

Canonical dataset rule:

- every recorded stream must map its `header.stamp` into the same dataset time axis
- the dataset time axis must have one declared clock domain and one declared semantic meaning
- bag arrival order must never be used as the training-time alignment reference
- if a producer cannot satisfy the canonical timestamp contract, its stream is not eligible for the training dataset until fixed

Recommended semantic definition:

- image `header.stamp`: the camera source timestamp mapped into the canonical dataset clock
- `/joint_states` `header.stamp`: the controller-cycle measurement timestamp mapped into the canonical dataset clock
- all downstream alignment for training uses these canonical timestamps only

Dataset-quality rule:

- a session that records the right topics but violates the canonical timestamp contract is an invalid training session
- such a session may still be kept for debugging, but it must not be marked as training-ready

Important observations:

1. `/joint_states` should be treated as the single robot-state source of truth for both arms and hands only if its timestamps come from the controller sample cycle and its values reflect measured feedback
2. controller command topics such as `/left_hand/forward_position_controller/commands` and `/right_hand/forward_position_controller/commands` are not measurement streams and should stay out of the default dataset
3. HTC tracker, Manus glove, `/tf`, and `/tf_static` are not required in the default bag if downstream reconstruction can rely on joint states plus snapshotted calibration and robot-description files
4. camera intrinsics and extrinsics must be snapshotted as authoritative session artifacts unless they are themselves versioned and immutable elsewhere
5. the dataset cannot be considered industrial-grade unless the producers expose trustworthy source-side timing semantics for both images and joint states

Immediate recommendation:

- keep the default bag limited to the three raw image topics plus `/joint_states`
- verify under the target launch that `/joint_states` really includes both Marvin and WujiHand measured states
- define and enforce a producer-side timing contract for camera images and `/joint_states`
- snapshot camera calibration, robot description, and time-sync configuration inputs into the session directory
- keep tracker, Manus, TF, and controller-command topics out of the default recording recipe unless a later measured requirement proves they are essential
- explicitly mark sessions as training-ready only when the canonical timestamp contract passed validation

## 5. Design Decisions

### 5.1 Canonical dataset timestamp

All recorded streams in a training dataset must use one canonical dataset timestamp contract.

Why:

- training requires one shared time axis across images and robot state
- per-topic timestamp semantics silently poison alignment quality
- downstream interpolation and sample selection become well-defined only when all streams share one semantic time basis

Rules:

- canonical timestamps are assigned by the original producers, not by a later relay node rewriting messages in bulk
- the canonical clock domain and semantic meaning must be declared in metadata
- any stream whose timestamps cannot be mapped into the canonical dataset clock must be excluded from training datasets

### 5.2 Recorder ownership

The supervisor owns the recording lifecycle, but the recorder itself runs as a separate process.

Why:

- avoids mixing bag writing with teleop/controller execution
- keeps recorder startup, cleanup, and failure handling explicit
- reuses a recorder process pattern already used elsewhere in the workspace

### 5.3 One session, one bag

Each `StartSession` / `StopSession` pair produces one bag directory.

Why:

- simpler ownership of metadata
- easier partial reruns
- easier failure isolation
- easier downstream relabeling and trimming

### 5.4 Recipe-first topic contract

Topic selection should follow this rule:

1. if `recipe.record_topics` is non-empty, use that exact list after normalization
2. if `recipe.record_topics` is empty, fall back to the union of `adapter.record_topics()` for all active devices

This preserves recipe authority while still allowing sensible defaults.

### 5.5 Start the recorder paused

The preferred mode is:

- spawn recorder with `--start-paused`
- wait for recorder services to appear
- resume on `StartSession`
- stop on `StopSession`

This shifts process startup cost away from the critical session boundary.

### 5.6 Stability over online compression

Default behavior:

- no aggressive online compression by default
- write to fast local disk
- compress or repackage later if needed

Compression is a performance tuning problem, not a functional prerequisite for the timing contract.

## 6. File-by-File Implementation Plan

Important observations:

1. `/joint_states` should be treated as the single robot-state source of truth for both arms and hands only if its timestamps come from the controller sample cycle and its values reflect measured feedback
2. controller command topics such as `/left_hand/forward_position_controller/commands` and `/right_hand/forward_position_controller/commands` are not measurement streams and should stay out of the default dataset
3. HTC tracker, Manus glove, `/tf`, and `/tf_static` are not required in the default bag if downstream reconstruction can rely on joint states plus snapshotted calibration and robot-description files
4. camera intrinsics and extrinsics must be snapshotted as authoritative session artifacts unless they are themselves versioned and immutable elsewhere
5. the dataset cannot be considered industrial-grade unless the producers expose trustworthy source-side timing semantics for both images and joint states

Immediate recommendation:

- keep the default bag limited to the three raw image topics plus `/joint_states`
- verify under the target launch that `/joint_states` really includes both Marvin and WujiHand measured states
- define and enforce a producer-side timing contract for camera images and `/joint_states`
- snapshot camera calibration, robot description, and time-sync configuration inputs into the session directory
- keep tracker, Manus, TF, and controller-command topics out of the default recording recipe unless a later measured requirement proves they are essential

## 5. Design Decisions

### 5.1 Recorder ownership

The supervisor owns the recording lifecycle, but the recorder itself runs as a separate process.

Why:

- avoids mixing bag writing with teleop/controller execution
- keeps recorder startup, cleanup, and failure handling explicit
- reuses a recorder process pattern already used elsewhere in the workspace

### 5.2 One session, one bag

Each `StartSession` / `StopSession` pair produces one bag directory.

Why:

- simpler ownership of metadata
- easier partial reruns
- easier failure isolation
- easier downstream relabeling and trimming

### 5.3 Recipe-first topic contract

Topic selection should follow this rule:

1. if `recipe.record_topics` is non-empty, use that exact list after normalization
2. if `recipe.record_topics` is empty, fall back to the union of `adapter.record_topics()` for all active devices

This preserves recipe authority while still allowing sensible defaults.

### 5.4 Start the recorder paused

The preferred mode is:

- spawn recorder with `--start-paused`
- wait for recorder services to appear
- resume on `StartSession`
- stop on `StopSession`

This shifts process startup cost away from the critical session boundary.

### 5.5 Stability over online compression

Default behavior:

- no aggressive online compression by default
- write to fast local disk
- compress or repackage later if needed

Compression is a performance tuning problem, not a functional prerequisite for the timing contract.

## 6. File-by-File Implementation Plan

## 6.1 `data_collection_bringup/config/session/recording.yaml`

Current file is too small to drive the real pipeline. Expand it into the authoritative recording policy.

Recommended target schema:

```yaml
session_root: ~/.ros/data_collection
bag_directory_name: rosbag
metadata_file_name: metadata.json
event_log_file_name: event_log.txt
timing_report_file_name: timing_report.json
snapshot_recipe: true
snapshot_site_config: true
snapshot_startup_policy: true
snapshot_fault_policy: true
snapshot_time_sync_config: true
snapshot_robot_description: true
snapshot_camera_calibration: true
prepare_recorder_on_start_system: false
use_start_paused: true
recorder_node_name_prefix: data_collection_recorder
service_timeout_sec: 10.0
stop_timeout_sec: 10.0
cleanup_timeout_sec: 5.0
storage_id: mcap
compression_mode: none
compression_format: zstd
capture_git_state: true
preview_drop_policy: latest_only
require_time_sync_healthy: true
time_sync_mode: hardware_trigger_or_common_clock
canonical_timestamp_clock: shared_dataset_clock
canonical_timestamp_semantics:
  image: source_capture_time
  joint_states: controller_measurement_time
time_sync_max_offset_ns: 1000000
joint_state_min_rate_hz: 100.0
joint_state_max_jitter_ms: 2.0
post_session_timing_validation: true
training_ready_requires_canonical_timestamps: true
```

Notes:

- `storage_id` should stay configurable because installed rosbag plugins may differ across machines
- `mcap` is the preferred default when available; fall back to `sqlite3` only if required by the target machine
- `compression_mode: none` remains the safe default until throughput is benchmarked under the full camera load
- `require_time_sync_healthy` should block session start when the target timing contract is not satisfied
- `canonical_timestamp_clock` and `canonical_timestamp_semantics` define the one allowed time axis for training datasets
- the exact `time_sync_mode` may map to hardware trigger, controller-distributed clock, or a shared grandmaster clock depending on the deployed hardware
- `training_ready_requires_canonical_timestamps` should prevent a session from being marked training-ready unless every recorded stream satisfied the canonical timestamp contract
- `prepare_recorder_on_start_system` can remain `false` unless startup latency becomes an operator problem

## 6.2 `data_collection_bringup/launch/collection_app.launch.py`

Add a new launch argument and pass it to supervisor:

- `recording_config`

Default value should point to:

- `config/session/recording.yaml`

Required change:

- declare launch argument
- pass `recording_config` into supervisor node parameters

## 6.3 `data_collection_bringup/launch/headless_supervisor.launch.py`

Same change as above:

- declare `recording_config`
- pass it into supervisor parameters

This keeps UI and headless flows behaviorally aligned.

## 6.4 `data_collection_orchestrator/models.py`

Expand models to carry recording configuration and session artifacts.

Recommended additions:

### `RecordingPolicy`

Suggested fields:

- `session_root: Path`
- `bag_directory_name: str`
- `metadata_file_name: str`
- `event_log_file_name: str`
- `timing_report_file_name: str`
- `snapshot_recipe: bool`
- `snapshot_site_config: bool`
- `snapshot_startup_policy: bool`
- `snapshot_fault_policy: bool`
- `snapshot_time_sync_config: bool`
- `snapshot_robot_description: bool`
- `snapshot_camera_calibration: bool`
- `prepare_recorder_on_start_system: bool`
- `use_start_paused: bool`
- `recorder_node_name_prefix: str`
- `service_timeout_sec: float`
- `stop_timeout_sec: float`
- `cleanup_timeout_sec: float`
- `storage_id: str`
- `compression_mode: str`
- `compression_format: str`
- `capture_git_state: bool`
- `require_time_sync_healthy: bool`
- `time_sync_mode: str`
- `canonical_timestamp_clock: str`
- `canonical_timestamp_semantics: dict[str, str]`
- `time_sync_max_offset_ns: int`
- `joint_state_min_rate_hz: float`
- `joint_state_max_jitter_ms: float`
- `post_session_timing_validation: bool`
- `training_ready_requires_canonical_timestamps: bool`

### `SessionArtifacts`

Suggested fields:

- `session_dir: Path`
- `bag_dir: Path`
- `metadata_path: Path`
- `event_log_path: Path`
- `timing_report_path: Path`
- `snapshot_paths: dict[str, Path]`

### `RecorderRuntime`

Suggested fields:

- `node_name: str`
- `command: list[str]`
- `pid: int | None`
- `pgid: int | None`
- `log_path: Path | None`
- `prepared: bool`
- `recording_active: bool`
- `sync_health_checked: bool`
- `sync_health_summary: dict[str, Any]`

### Extend `SupervisorConfig`

Add:

- `recording_config: Optional[Path]`

### Extend `ActiveSession`

Add:

- `site_name: str`
- `artifacts: SessionArtifacts | None`
- `record_topics: list[str]`
- `recorder_node_name: str`
- `session_start_ros_time_ns: int | None`
- `session_stop_ros_time_ns: int | None`
- `session_start_wall_time: str`
- `session_stop_wall_time: str`
- `session_start_monotonic_sec: float | None`
- `session_stop_monotonic_sec: float | None`
- `time_sync_summary: dict[str, Any]`
- `timing_validation_summary: dict[str, Any]`

Do not store bag writer internals inside `ActiveSession`; runtime process objects should stay inside `RecordManager`.

## 6.5 `data_collection_orchestrator/session_manager.py`

Current implementation only allocates `session_id`. Extend it to create richer session objects.

Recommended responsibilities:

- allocate deterministic session ids
- build `ActiveSession`
- retain active session reference
- expose `active_session`
- clear active session on `end_session()`

Do **not** add process lifecycle logic here. That belongs in `RecordManager`.

## 6.6 `data_collection_orchestrator/event_log.py`

Current event log is in-memory only.

Recommended additions:

- `snapshot() -> list[str]`
- optional timestamped record format
- optional `write_to(path: Path)` helper

Phase-1 behavior should be simple:

- continue collecting entries in memory during runtime
- write `event_log.txt` into the session directory at session stop or session failure finalization

## 6.7 `data_collection_orchestrator/record_manager.py`

This file currently contains a placeholder. Replace it with the actual recorder controller.

### Responsibilities

- load and validate recording policy
- normalize topic list
- create session directory structure
- snapshot configs into session directory
- write metadata skeleton before recording starts
- spawn rosbag recorder process
- wait for `/resume`, `/pause`, and `/stop` services if using `--start-paused`
- resume recording
- stop recorder cleanly
- force terminate recorder on failure
- write final metadata and event log

### Recommended public API

```python
class RecordManager:
    def prepare_session(...): ...
    def start_recording(...): ...
    def stop_recording(...): ...
    def abort_recording(...): ...
    def finalize_failed_session(...): ...
    def runtime_summary(...): ...
```

### Suggested method semantics

#### `prepare_session(...)`

Inputs:

- active session
- recipe
- supervisor config
- recording policy
- normalized topic list
- event log snapshot source

Outputs:

- session artifacts
- recorder runtime descriptor

Work:

- create `YYYY-MM-DD/session_xxxxxx/`
- create bag directory path
- copy config snapshots
- write early `metadata.json` with status `preparing`

#### `start_recording(...)`

Work:

- build command:

```bash
ros2 bag record \
  --start-paused \
  --disable-keyboard-controls \
  --node-name <node_name> \
  --storage <storage_id> \
  -o <bag_dir> \
  --topics <topic list>
```

- use `subprocess.Popen(..., start_new_session=True)`
- create rclpy clients for:
  - `/<node_name>/resume`
  - `/<node_name>/pause`
  - `/<node_name>/stop`
- wait for services
- call resume
- update metadata to status `recording`

#### `stop_recording(...)`

Work:

- call pause if recording is active
- call stop
- wait for process exit within timeout
- if timeout expires, terminate then kill as fallback
- preserve partial bag if termination becomes necessary
- update metadata status to `completed` or `incomplete`
- write `event_log.txt`

#### `abort_recording(...)`

Use during failed `StartSession` after partial recorder startup.

Work:

- stop or kill recorder if it exists
- keep session directory for postmortem
- write metadata status `failed`
- never silently delete a bag that may contain partial but useful evidence

### Internal helpers

Recommended helpers:

- `_normalize_topics(topics: list[str]) -> list[str]`
- `_validate_topics(topics: list[str]) -> list[str]`
- `_build_recorder_command(...) -> list[str]`
- `_wait_for_service(...) -> bool`
- `_call_empty_service(...) -> bool`
- `_terminate_process(...) -> None`
- `_write_metadata(...) -> None`
- `_snapshot_file(...) -> Path | None`
- `_capture_git_state(...) -> dict`

## 6.8 `data_collection_orchestrator/supervisor.py`

This is the main integration point.

### Config loading

Extend `_declare_config()` to include:

- `recording_config`

Load `RecordingPolicy` during supervisor initialization.

### Construction

Add:

- `self._record_manager = RecordManager(...)`

Recommended testability improvement:

- allow `DataCollectionSupervisor(..., record_manager=None)` injection for unit tests

### Topic resolution

Add a helper:

```python
def _session_record_topics(self) -> list[str]:
    ...
```

Rules:

- use recipe `record_topics` if present
- otherwise union adapter defaults
- deduplicate while preserving deterministic order
- warn on obvious quality issues

### `execute_start_system()`

Recorder spawn is not required during `StartSystem` unless `prepare_recorder_on_start_system` is enabled.

Responsibilities:

- load recipe
- bring up devices
- load recording policy
- load or resolve time-sync configuration
- verify that mandatory timing prerequisites are observable
- reach `READY`
- keep session-specific recording preparation deferred to `StartSession`

Optional optimization:

- if `prepare_recorder_on_start_system` is true, pre-create session-independent recorder resources, but do not enter recording state

The system should not enter `READY` if the configured industrial timing contract is impossible to satisfy on the current machine or hardware topology.

### `execute_start_session()`

Replace the current state-only behavior with the following sequence:

1. validate command is allowed
2. ensure current recipe exists
3. if current state is `READY`, run arm sequence
4. run `before_session()` hooks
5. allocate `ActiveSession`
6. resolve normalized recording topic list
7. call `record_manager.prepare_session(...)`
8. run pre-session synchronization health checks
9. stamp `session_start_*`
10. call `record_manager.start_recording(...)`
11. if recorder start succeeds:
    - set state to `RECORDING`
    - publish success feedback
    - return `success=True`
12. if recorder start fails:
    - abort recorder runtime
    - run rollback:
      - `after_session()` if needed
      - disarm sequence if needed
    - if rollback succeeds, return to `READY` with failed action result
    - if rollback fails, enter `FAULT`

If synchronization health checks fail, `StartSession` must fail before the recorder is resumed.

This ordering matters. The recorder must be known-good before claiming the session is truly recording.

### `execute_stop_session()`

Replace the current state-only stop with:

1. validate command is allowed
2. ensure active session exists
3. stamp `session_stop_*`
4. call `record_manager.stop_recording(...)`
5. run `after_session()` hooks
6. run disarm sequence
7. finalize active session and clear it
8. if recording stopped cleanly and cleanup succeeded:
   - return to `READY`
9. if recording had to be force-killed but artifacts are preserved:
   - return failure result
   - keep state `READY` with explicit degraded summary, or move to `FAULT` if artifact integrity is unknown
10. if device cleanup fails:
   - enter `FAULT`

### `_stop_active_session_runtime()`

This helper should be extended or split so that it becomes the single place coordinating:

- recorder stop/finalization
- `after_session()`
- disarm
- session object cleanup

### `execute_shutdown_system()`

If a session is active:

- stop session recording first
- then shutdown devices
- preserve any partial bag and metadata if shutdown occurred during a failure

### Fault acknowledgement path

When recovering from `FAULT`:

- do not delete session artifacts
- make sure recorder processes are gone
- leave failed session directories intact for debugging

## 6.9 `data_collection_orchestrator/package.xml`

Add missing runtime dependency for recorder control services:

- `rosbag2_interfaces`

This is required if the orchestrator uses rclpy clients for recorder `Pause`, `Resume`, and `Stop` services.

Optional later dependency, only if needed:

- `rosbag2_py`

Do not add `rosbag2_py` unless it is actually used.

## 6.10 `data_collection_orchestrator/setup.py`

Likely no major structural change is required.

Only update if:

- new test modules or data files require packaging changes
- an additional console script is added

## 6.11 `data_collection_bringup/package.xml`

No required change beyond ensuring launch-time dependencies remain valid.

## 7. Recording Policy and Metadata Schemas

## 7.1 Recording policy example

Recommended example configuration:

```yaml
session_root: ~/.ros/data_collection
bag_directory_name: rosbag
metadata_file_name: metadata.json
event_log_file_name: event_log.txt
timing_report_file_name: timing_report.json
snapshot_recipe: true
snapshot_site_config: true
snapshot_startup_policy: true
snapshot_fault_policy: true
snapshot_time_sync_config: true
snapshot_robot_description: true
snapshot_camera_calibration: true
prepare_recorder_on_start_system: false
use_start_paused: true
recorder_node_name_prefix: data_collection_recorder
service_timeout_sec: 10.0
stop_timeout_sec: 10.0
cleanup_timeout_sec: 5.0
storage_id: mcap
compression_mode: none
compression_format: zstd
capture_git_state: true
preview_drop_policy: latest_only
require_time_sync_healthy: true
time_sync_mode: hardware_trigger_or_common_clock
canonical_timestamp_clock: shared_dataset_clock
canonical_timestamp_semantics:
  image: source_capture_time
  joint_states: controller_measurement_time
time_sync_max_offset_ns: 1000000
joint_state_min_rate_hz: 100.0
joint_state_max_jitter_ms: 2.0
post_session_timing_validation: true
training_ready_requires_canonical_timestamps: true
```

## 7.2 Metadata schema example

Suggested `metadata.json` structure:

```json
{
  "session_id": "session_000123",
  "recipe_id": "marvin_manus_wujihand_collection",
  "operator_id": "alice",
  "site_name": "lab_a",
  "session_tag": "trial_03",
  "status": "recording",
  "session_dir": "...",
  "bag_dir": ".../rosbag",
  "record_topics": ["/cam_high/cam_high/color/image_raw", "/cam_left_wrist/image_raw", "/cam_right_wrist/image_raw", "/joint_states"],
  "recorder": {
    "node_name": "data_collection_recorder_session_000123",
    "command": ["ros2", "bag", "record", "..."],
    "pid": 12345,
    "pgid": 12345,
    "storage_id": "mcap",
    "compression_mode": "none",
    "compression_format": "zstd"
  },
  "timestamps": {
    "session_start_ros_time_ns": 0,
    "session_stop_ros_time_ns": 0,
    "session_start_wall_time": "2026-04-22T12:34:56.123456+08:00",
    "session_stop_wall_time": "2026-04-22T12:35:10.654321+08:00",
    "session_start_monotonic_sec": 12345.678,
    "session_stop_monotonic_sec": 12359.201
  },
  "time_sync": {
    "mode": "hardware_trigger_or_common_clock",
    "healthy": true,
    "max_offset_ns": 1000000,
    "camera_sync_locked": true,
    "joint_state_rate_hz": 250.0,
    "joint_state_jitter_ms": 0.4,
    "checks": {
      "pre_session": "passed",
      "post_session": "passed"
    }
  },
  "timing_contract": {
    "canonical_timestamp_clock": "shared_dataset_clock",
    "canonical_timestamp_semantics": {
      "image": "source_capture_time",
      "joint_states": "controller_measurement_time"
    },
    "image_timestamp_source": "camera_driver_source_timestamp",
    "joint_state_timestamp_source": "controller_cycle_timestamp",
    "alignment_anchor": "image_timestamp"
  },
  "training_ready": true,
  "snapshots": {
    "recipe": "recipe.yaml",
    "cameras": "cameras.yaml",
    "robot_description": "robot_description.urdf",
    "camera_calibration": "camera_calibration.yaml",
    "time_sync_config": "time_sync.yaml",
    "startup_policy": "startup.yaml",
    "fault_policy": "fault.yaml"
  },
  "timing_report": "timing_report.json",
  "git": {
    "revision": "<optional>",
    "dirty": true
  },
  "result": {
    "success": true,
    "message": "Session stopped normally"
  }
}
```

Guideline:

- write metadata early with a provisional status
- update it at every major state change
- record the declared time source for every persisted stream
- never leave a successful bag without metadata and a timing validation report
- treat missing synchronization evidence as a dataset-quality failure, not a documentation gap

## 8. Detailed Session State Behavior

## 8.1 `StartSystem`

Target behavior stays mostly unchanged:

- load recipe
- bring up devices
- enter `READY`

Recording-specific additions:

- load recording policy
- validate that `session_root` is writable when practical
- do not start the recorder during `StartSystem` unless explicitly configured

## 8.2 `StartSession` happy path

Detailed order:

1. validate current system state
2. arm supported devices if state is `READY`
3. run `before_session()`
4. create `ActiveSession`
5. resolve recording topics
6. create session directory and bag path
7. snapshot config files
8. write initial metadata with `status=preparing`
9. spawn rosbag recorder paused
10. wait for resume/pause/stop services
11. stamp session start timestamps
12. resume recorder
13. update metadata to `status=recording`
14. set system state to `RECORDING`
15. return success

## 8.3 `StartSession` failure path

Possible failure points:

- topic normalization invalid
- session directory cannot be created
- recorder process spawn fails
- recorder services never appear
- recorder resume fails

Required rollback policy:

1. stop or kill recorder if partially started
2. preserve session directory and metadata
3. run `after_session()` only if `before_session()` was already run
4. disarm devices if arm sequence succeeded
5. if rollback succeeds:
   - return failed action result
   - return to `READY`
6. if rollback fails:
   - enter `FAULT`

The system must not claim `RECORDING` until recorder resume has succeeded.

## 8.4 `StopSession` happy path

Detailed order:

1. confirm active session exists
2. stamp session stop timestamps
3. pause recorder if needed
4. stop recorder
5. wait for recorder process exit
6. run post-session timing validation and write `timing_report.json`
7. write final metadata and event log
8. run `after_session()`
9. disarm devices
10. clear active session
11. enter `READY`
12. return success

## 8.5 `StopSession` degraded path

If recorder stop hangs:

1. try graceful stop first
2. escalate to process terminate
3. escalate to process kill if necessary
4. mark metadata `status=incomplete`
5. keep bag directory on disk
6. continue with device cleanup
7. return failure result
8. enter `READY` with explicit warning summary, or enter `FAULT` if bag integrity cannot be trusted and operator intervention is needed

Recommended rule:

- recorder force-kill with preserved artifacts should return `success=False`
- device cleanup failure should enter `FAULT`

## 8.6 `ShutdownSystem` during active session

Required behavior:

- stop active session recording first
- preserve partial artifacts if needed
- then shutdown devices
- do not delete incomplete bag directories

## 9. Topic Normalization and Recipe Linting

Add lightweight validation before recorder startup.

## 9.1 Normalization rules

- strip whitespace
- drop empty items
- deduplicate while preserving order
- reject invalid topic strings early

## 9.2 Warnings

Warn, but do not block, when:

- a raw image topic exists without matching calibration provenance in the session snapshots
- a controller command topic appears in `record_topics`
- a preview topic appears in `record_topics`
- duplicate semantic topics exist
- a non-preferred storage backend is selected due to machine constraints

## 9.3 Blocking validation

Block session start when:

- topic list is empty after normalization
- `/joint_states` is missing from the normalized topic list
- no raw image topic remains after normalization
- session output directory cannot be created
- required snapshot file path is malformed in config
- required synchronization health checks fail
- the declared timing contract for image or joint-state timestamps cannot be verified
- mandatory calibration or robot-description provenance is missing

Industrial-grade recording should fail closed when timing provenance is unknown or unhealthy.
## 10. Recorder Process Contract

Phase-1 recorder uses the same high-level pattern already used by the workspace's existing rosbag-based tooling.

### 10.1 Command template

```bash
ros2 bag record \
  --start-paused \
  --disable-keyboard-controls \
  --node-name <recorder_node_name> \
  --storage <storage_id> \
  -o <bag_dir> \
  --topics <normalized topic list>
```

Optional later additions:

- compression flags
- split-bag flags
- storage-specific tuning

These should remain opt-in until benchmarked on the target workstation.

### 10.2 Recorder services

Assuming rosbag recorder node name is `/<node_name>`, the supervisor should create clients for:

- `/<node_name>/resume`
- `/<node_name>/pause`
- `/<node_name>/stop`

Use `rosbag2_interfaces.srv.Resume`, `Pause`, and `Stop`.

### 10.3 Process management rules

- use `start_new_session=True`
- track `pid` and `pgid`
- prefer graceful stop
- never silently discard partially written bags
- log command line and process identifiers into metadata

## 11. Performance Plan

## 11.1 Process separation

Recording must stay outside the main callback path of:

- controller manager
- motion server
- teleop bridges
- UI refresh loop
- health monitor

## 11.2 Disk policy

- write to local fast disk first
- do not write directly to network storage
- perform archival copy after session completion

## 11.3 Compression policy

Default behavior:

- `compression_mode: none`

Benchmark tasks:

- prefer `mcap` and compare against `sqlite3` only where required
- compare no compression vs zstd compression
- compare stop latency and CPU pressure under full camera load
- verify that chosen storage and compression settings do not compromise timing determinism

## 11.4 Preview isolation

Preview topics are presentation-plane only.

Rules:

- UI may drop preview frames
- recorder never subscribes to preview topics unless explicitly configured
- `preview_drop_policy` should not affect recorded topics

## 11.5 Optional CPU isolation

If teleop jitter appears under recording load, add a later optimization phase:

- assign recorder to a dedicated CPU set
- isolate camera drivers if necessary
- keep controller-related nodes on a stable CPU set

This should be treated as machine-specific tuning, not required initial architecture.

## 12. Timestamp and Alignment Work Package

The bag is necessary but not sufficient. Alignment quality depends on timestamps emitted by publishers.

For this scope, the alignment problem is specifically camera-image timestamps versus real robot joint-state timestamps. The industrial target is to make both streams traceable to trusted producer-side time definitions under a shared timing contract.

## 12.1 Streams to verify

| Stream | Required time source | Required check |
|---|---|---|
| `cam_high` image | camera driver source timestamp tied to hardware trigger or shared clock | valid `header.stamp`, stable frame rate, monotonic frame sequence, sync lock healthy |
| `cam_left_wrist` image | camera driver source timestamp tied to hardware trigger or shared clock | valid `header.stamp`, stable frame rate, monotonic frame sequence, sync lock healthy |
| `cam_right_wrist` image | camera driver source timestamp tied to hardware trigger or shared clock | valid `header.stamp`, stable frame rate, monotonic frame sequence, sync lock healthy |
| `/joint_states` | controller-cycle timestamp derived from the robot control loop | recent and meaningful stamp, includes both Marvin and WujiHand measured states, rate and jitter within limits |

## 12.2 Industrial timing contract

The target timing contract is:

- all persisted image timestamps come from source-side camera timing, not recorder receipt time and not an arbitrary downstream relay node
- `/joint_states` timestamps come from the robot control cycle that produced the measured state
- camera streams share a common trigger or common clock
- the robot state timeline and camera timeline are related by a measurable and bounded offset under the configured synchronization mode
- every session records enough evidence to prove which timing mode was active

## 12.3 Post-processing anchor strategy

Choose one anchor timeline per dataset:

- image timestamp for vision-centric datasets
- `/joint_states` timestamp for control-centric datasets

For every anchor timestamp:

- find nearest `/joint_states` sample within tolerance for each image frame, or
- find nearest image for each selected joint-state sample, depending on the downstream task
- reject sample pairs that fall outside the configured tolerance
- report alignment error distributions instead of only checking a few hand-picked samples

## 12.4 Target tolerance envelope

Target values should be driven by downstream task requirements, but the recording stack should expose configurable gates for at least:

- camera-to-camera skew
- image to nearest `/joint_states` skew
- `/joint_states` publish-rate floor
- `/joint_states` jitter ceiling
- synchronization offset to the shared timing reference

These limits must be validated against real collected sessions rather than assumed.

## 12.5 Required synchronization strategies

Preferred order of trust:

1. hardware trigger with camera source timestamps
2. common hardware clock or controller-distributed clock
3. shared grandmaster clock such as PTP/gPTP when the hardware supports it

A software-only `now()` at an arbitrary ROS publisher is not sufficient for the industrial target unless it can be demonstrated to meet the configured tolerance envelope under load.

## 12.6 Required session evidence

Every successful session should preserve:

- the declared timing mode
- pre-session synchronization health results
- per-stream timing provenance
- post-session timing validation report
- any observed skew, jitter, dropped-frame, or dropped-sample counters

A bag without this evidence is operationally useful, but it is not industrial-grade.
## 13. Testing and Validation Plan

## 13.1 Unit tests

Add or update tests for:

- recording config parsing
- topic normalization and linting
- session directory generation
- metadata writing and rewrite behavior
- recorder command generation
- recorder stop timeout behavior using mocked processes

Suggested test files:

- `test_record_manager.py`
- `test_recording_policy.py`
- `test_session_metadata.py`

## 13.2 Supervisor integration tests

Use dependency injection for `RecordManager` to test supervisor state transitions.

Scenarios:

1. `StartSession` succeeds and enters `RECORDING`
2. recorder preparation fails before spawn
3. recorder spawn succeeds but resume fails
4. `StopSession` succeeds and returns to `READY`
5. `StopSession` requires forced recorder termination
6. recorder failure + cleanup failure triggers `FAULT`

## 13.3 System validation on target machine

For the default recipe:

1. launch collection app
2. run `StartSystem`
3. run `StartSession`
4. verify bag directory appears
5. move Marvin, WujiHand, and cameras for a short session
6. run `StopSession`
7. inspect bag topic list
8. inspect metadata and event log
9. confirm no recorder process remains

## 13.4 Performance validation

Run a representative collection session and record:

- CPU usage
- memory usage
- disk throughput
- teleop smoothness
- whether recorder stop latency is acceptable
- whether dropped-message symptoms appear

## 13.5 Dataset readiness validation

For one short sample bag:

- extract image timestamps and frame sequence information
- extract `/joint_states` timestamps and rate statistics
- compute image-to-joint nearest-neighbor timing error histogram
- compute camera-to-camera skew histogram
- confirm that measured skew and jitter stay within the configured envelope
- write a machine-readable `timing_report.json`

The implementation is not done until the resulting bag is not only present, but also accompanied by validation evidence showing that it satisfied the configured timing contract.

## 14. Execution Phases

## Phase 1: Configuration and model wiring

Changes:

- expand `recording.yaml`
- add `recording_config` launch arg in both launch files
- extend `SupervisorConfig`
- add recording-related dataclasses in `models.py`
- add time-sync and timing-validation policy fields

Done when:

- supervisor can load recording and timing policy
- timing requirements are visible in logs/tests even before full enforcement lands

## Phase 2: Replace placeholder `record_manager.py`

Changes:

- implement session directory creation
- implement metadata skeleton writing
- implement timing-report path creation
- implement recorder process spawn/stop logic

Done when:

- a standalone unit test can create and stop a recorder for a trivial topic set
- metadata and timing-report artifacts are emitted deterministically

## Phase 3: Supervisor integration and fail-closed session gating

Changes:

- wire `execute_start_session()` to prepare and start recorder
- wire synchronization health checks into `StartSystem` and `StartSession`
- wire `execute_stop_session()` to stop recorder and finalize metadata
- integrate rollback and failure paths

Done when:

- `Start Collection` produces a real bag only when timing prerequisites pass
- `Stop Collection` closes it cleanly and writes timing evidence

## Phase 4: Producer-side timing contract

Changes:

- document and verify the timestamp source for all persisted streams
- verify that `/joint_states` is sufficient as the only robot-state stream for downstream reconstruction
- identify any publisher still relying on late software-side stamping

Done when:

- every recorded stream has a declared and verified source-side timing contract

## Phase 5: Synchronization validation

Changes:

- implement pre-session synchronization health checks
- implement post-session timing validation
- write `timing_report.json`

Done when:

- every successful session carries explicit evidence that the timing contract was checked and passed

## Phase 6: Runtime performance hardening

Changes:

- benchmark storage backends
- evaluate compression options
- optionally add CPU isolation guidance
- confirm that the timing contract still holds under representative runtime load

Done when:

- recording does not cause unacceptable teleop degradation on the target workstation
- timing gates remain green under representative load

## Phase 7: Hardware-synchronized deployment

Changes:

- integrate the selected industrial synchronization mode into deployment documentation and startup checks
- verify hardware trigger, distributed clock, or shared grandmaster status on the target machine

Done when:

- the deployed system satisfies the selected industrial synchronization architecture end to end

## 15. Definition of Done

The recording plan is implemented when all of the following are true:

- `StartSession` starts a real bag recorder only after synchronization health checks pass
- `StopSession` stops it and finalizes metadata
- session artifacts are written under a deterministic directory layout
- active recipe topics are actually present in the bag
- no recorder process remains after session stop
- failure cases preserve partial artifacts and surface explicit errors
- every persisted stream has documented timing provenance
- every successful session produces a machine-readable timing validation report
- at least one representative session has been checked against the configured timing envelope and passes
- the operator can collect data without noticeable degradation of real-time control behavior
- the deployed hardware and software together satisfy the selected industrial synchronization architecture

## 16. Immediate Next Coding Steps

1. expand `data_collection_bringup/config/session/recording.yaml` with timing-policy fields
2. add `recording_config` to:
   - `data_collection_bringup/launch/collection_app.launch.py`
   - `data_collection_bringup/launch/headless_supervisor.launch.py`
3. extend `SupervisorConfig` and add `RecordingPolicy`
4. replace placeholder `data_collection_orchestrator/record_manager.py`
5. add `rosbag2_interfaces` dependency to `data_collection_orchestrator/package.xml`
6. inject `RecordManager` into `supervisor.py`
7. wire `StartSession` / `StopSession` to recorder lifecycle and synchronization health gates
8. write `metadata.json`, `event_log.txt`, and `timing_report.json`
9. run a small end-to-end capture with:
   - one camera image topic
   - `/joint_states`
10. implement short-bag timing validation tooling
11. expand to the default full recipe and measure runtime impact under the intended synchronization mode

## 17. Open Questions to Resolve During Implementation

These should be answered with measured behavior, not assumptions:

1. Which synchronization architecture will be the deployment target: hardware trigger, distributed control clock, shared grandmaster clock, or a demonstrably sufficient software-only fallback?
2. Is the preferred `mcap` backend stable and fast enough for all three cameras plus `/joint_states` on the target machine, or is `sqlite3` required there?
3. Is `/joint_states` alone sufficient for downstream reconstruction of Marvin and WujiHand state, or is an additional measured-state topic needed?
4. Should wrist camera `camera_info` be added to the default recipe now or kept as a snapshotted artifact only?
5. Is `prepare_recorder_on_start_system` necessary for acceptable UI responsiveness, or is deferred startup during `StartSession` good enough?
6. Should recorder stop with forced termination keep the system in `READY` with warning, or should it always enter `FAULT`?
7. What timing envelope does the downstream learning or evaluation pipeline actually require for image-to-joint alignment?

These questions should be closed only after collecting and inspecting real bags under the intended synchronization mode.
