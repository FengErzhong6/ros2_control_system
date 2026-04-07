# Data Collection Stack Implementation Plan

## 1. Goal

Build a performance-first data collection application on top of the existing device packages:

- `camera_system`
- `htc_system`
- `manus_system`
- `marvin_system`
- `wujihand_system`
- future `picker_system`

The new system must not be implemented as an extension of any single device package. It should be a top-level stack that:

- orchestrates multiple devices with strict startup ordering
- handles success, timeout, retry, rollback, and fault reporting
- records session data in a unified way
- provides a desktop UI with buttons and keyboard shortcuts
- keeps UI off the main data path
- remains extensible when new devices are added


## 2. Design Principle

The stack will follow three clearly separated planes:

- Control Plane
  `supervisor`, state machine, device orchestration, fault handling, startup/shutdown ordering
- Data Plane
  data reception, health probing, rosbag recording, metadata persistence, performance statistics
- Presentation Plane
  desktop UI, camera preview, operator commands, event display

This separation is mandatory. UI must never directly own the core device lifecycle or recording lifecycle.


## 3. Repository Layout

`data_collection_stack` is a domain-level directory, not a ROS package itself.

Recommended structure:

```text
data_collection_stack/
  IMPLEMENTATION_PLAN.md
  README.md
  data_collection_interfaces/
  data_collection_orchestrator/
  data_collection_ui/
  data_collection_bringup/
```

Role of each package:

- `data_collection_interfaces`
  Shared `msg/srv/action` definitions
- `data_collection_orchestrator`
  Supervisor, state machine, adapters, recorder, health monitor, session manager
- `data_collection_ui`
  Qt UI, hotkeys, camera preview panel, event panel
- `data_collection_bringup`
  top-level launch files, recipes, system-level YAML configuration


## 4. Package Responsibilities

### 4.1 `data_collection_interfaces`

This package defines the control contract between UI and orchestrator.

Recommended messages:

- `DeviceState.msg`
- `SystemState.msg`
- `FaultEvent.msg`
- `SessionInfo.msg`

Recommended services:

- `GetSystemState.srv`
- `ListRecipes.srv`
- `AcknowledgeFault.srv`

Recommended actions:

- `StartSystem.action`
- `ShutdownSystem.action`
- `StartSession.action`
- `StopSession.action`
- `GoHome.action`

`SystemState.msg` should contain:

- current system state enum
- current recipe id
- active session id
- per-device state array
- current fault summary
- buttons allowed by current state

`DeviceState.msg` should contain:

- device id
- device class
- lifecycle state
- health state
- is_required
- last_ready_timestamp
- summary text


### 4.2 `data_collection_orchestrator`

This is the core package.

Suggested internal layout:

```text
data_collection_orchestrator/
  package.xml
  setup.py
  resource/
  launch/
    supervisor.launch.py
  config/
    devices.yaml
    policies.yaml
  data_collection_orchestrator/
    __init__.py
    supervisor.py
    state_machine.py
    command_server.py
    health_monitor.py
    record_manager.py
    session_manager.py
    recipe_loader.py
    event_log.py
    models.py
    adapters/
      __init__.py
      base.py
      camera.py
      htc.py
      manus.py
      marvin.py
      picker.py
      wujihand.py
    probes/
      __init__.py
      topic_probe.py
      tf_probe.py
      service_probe.py
      process_probe.py
      rosbag_probe.py
```

Core responsibilities:

- load recipe and device config
- build dependency graph
- run ordered bringup/shutdown
- expose actions/services to UI
- aggregate health and state
- manage session lifecycle
- start and stop rosbag
- write metadata and event logs
- isolate device-specific handling behind adapters


### 4.3 `data_collection_ui`

This package is operator-facing only.

Suggested internal layout:

```text
data_collection_ui/
  package.xml
  setup.py
  resource/
  data_collection_ui/
    __init__.py
    main.py
    ros_client.py
    hotkeys.py
    view_model.py
    widgets/
      main_window.py
      device_panel.py
      command_panel.py
      camera_panel.py
      session_panel.py
      fault_panel.py
      event_panel.py
```

UI rules:

- UI sends commands only through `interfaces`
- UI never calls device packages directly
- UI never manages rosbag directly
- UI can drop preview frames
- UI must remain responsive when preview lags


### 4.4 `data_collection_bringup`

This package provides the top-level entry point.

Suggested layout:

```text
data_collection_bringup/
  package.xml
  setup.py
  launch/
    collection_app.launch.py
    headless_supervisor.launch.py
  config/
    site/
      default_lab/
        cameras.yaml
        trackers.yaml
        manus.yaml
    operators/
      default.yaml
    recipes/
      marvin_tracker_collection.yaml
      marvin_manus_collection.yaml
      marvin_manus_wujihand_collection.yaml
    session/
      recording.yaml
      ui.yaml
    policies/
      startup.yaml
      fault.yaml
```

Responsibilities:

- start orchestrator
- optionally start UI
- inject selected recipe, site config, operator config, and global policies
- define operator-facing launch arguments


## 5. Dependency Direction

Dependency flow must remain one-way:

```text
device packages
  -> no dependency on collection packages

data_collection_interfaces

data_collection_orchestrator
  -> depends on data_collection_interfaces
  -> depends on existing device packages only through launch/topics/services/processes

data_collection_ui
  -> depends on data_collection_interfaces

data_collection_bringup
  -> depends on orchestrator + ui + interfaces
```

The existing device packages must remain independent. The new stack consumes them; it does not become their parent package.


## 6. Device Model

Not all devices are equally managed. The orchestrator must normalize them with adapters, not force all of them into `ros2_control`.

### 6.1 Device Categories

- `native_managed`
  devices with strong lifecycle/control semantics, such as `marvin`, later possibly `wujihand`
- `process_managed`
  devices started as ROS nodes/processes and validated through probes, such as `camera`, `htc`, `manus`
- `passive_monitor`
  devices or pipelines that are only observed, not directly controlled


### 6.1.1 Current Boundary Rule

For the current system boundary:

- the gripper remains part of `marvin`
- the stack should not model the gripper as a separate top-level device
- `manus` is treated as an external input device, not as the owner of the gripper hardware path

This keeps the first version aligned with the current codebase, where the gripper control path is already inside `marvin_system`.


### 6.2 Adapter Contract

All adapters return normalized results. They do not need identical internals.

Mandatory methods:

- `precheck()`
- `bringup()`
- `wait_ready(timeout_sec)`
- `shutdown()`
- `diagnose()`
- `dump_metadata()`
- `record_topics()`
- `preview_topics()`

Optional methods:

- `arm()`
- `disarm()`
- `home()`
- `before_session()`
- `after_session()`

Each method should return a structured result:

- `OK`
- `FAILED`
- `UNSUPPORTED`
- `DEGRADED`

This allows `marvin.arm()` to be fully implemented while `camera.arm()` safely reports `UNSUPPORTED`.


### 6.3 Adapter Expectations Per Device

#### `marvin`

- bringup
  launch `ros2_control_node`, `robot_state_publisher`, controller spawners, and the built-in gripper path when enabled by recipe or config
- ready criteria
  controller manager alive, `joint_state_broadcaster` active, teleop controller active, core services reachable, and gripper command path healthy when gripper is enabled
- optional commands
  `arm`, `disarm`, `home`

#### `camera`

- bringup
  launch driver node(s)
- ready criteria
  image topic and camera info topic arrive at minimum frequency for a stability window
- optional commands
  normally no `arm`, no `home`

#### `htc`

- bringup
  launch tracker publisher
- ready criteria
  required TF frames exist and refresh within latency threshold

#### `manus`

- bringup
  launch raw publisher and related helper nodes
- ready criteria
  glove topics active, calibration loaded if required, topic frequency stable
- boundary
  `manus` provides glove input and calibration state; it does not become a separate gripper ownership boundary in the stack model

#### `wujihand`

- bringup
  launch its control stack
- ready criteria
  controller active, command path healthy


## 7. Recipes and Device Graph

Startup order must not be hardcoded in a large launch file. It must be declared by recipe.

Example recipe:

```yaml
recipe_id: marvin_manus_camera
devices:
  - id: cam_high
    adapter: camera
    required: true
    depends_on: []
  - id: cam_left_wrist
    adapter: camera
    required: true
    depends_on: []
  - id: cam_right_wrist
    adapter: camera
    required: true
    depends_on: []
  - id: htc_tracker
    adapter: htc
    required: true
    depends_on: []
  - id: manus_glove
    adapter: manus
    required: true
    depends_on: []
  - id: marvin_dual
    adapter: marvin
    required: true
    depends_on:
      - htc_tracker
```

While the gripper remains part of `marvin`, no separate gripper device entry is needed in recipes.

Recipe fields should include:

- device id
- adapter type
- required or optional
- dependency list
- bringup config
- health thresholds
- record topics
- preview topics


## 8. System State Machine

The stack must use one authoritative state machine.

Recommended system states:

- `IDLE`
- `PREFLIGHT`
- `STARTING`
- `READY`
- `ARMED`
- `RECORDING`
- `PAUSED`
- `STOPPING`
- `FAULT`

Transition rules:

- `IDLE -> PREFLIGHT`
  operator requests system start
- `PREFLIGHT -> STARTING`
  config, disk, dependencies, calibration checks passed
- `STARTING -> READY`
  all required devices report ready
- `READY -> ARMED`
  optional arm/home preparation completed
- `ARMED -> RECORDING`
  recorder active and session open
- `RECORDING -> PAUSED`
  session paused by operator or policy
- `PAUSED -> RECORDING`
  resume accepted
- `READY/ARMED/RECORDING/PAUSED -> FAULT`
  fault policy triggered
- `FAULT -> IDLE`
  only after fault acknowledge and full rollback or shutdown

Important rule:

UI state must always be derived from `SystemState`, never from local UI assumptions.


## 9. Fault Policy

Faults must be classified and handled consistently.

Recommended fault classes:

- `blocking_fault`
  stop state transition immediately
- `recoverable_fault`
  retry with backoff
- `degraded_fault`
  allow continued operation with visible warning
- `fatal_fault`
  immediate transition to `FAULT`

Example policy:

- camera warmup timeout
  retry 2 times with backoff
- tracker TF stale
  degrade if not recording, fatal if recording
- rosbag write failure
  fatal
- marvin controller not active
  fatal
- preview topic missing
  degraded only

All faults must be logged to:

- in-memory event stream
- session event log
- fault log file


## 10. Performance-First Camera Strategy

The current dashboard mixes ROS spinning and Qt rendering in one process. That structure must not be reused for the new system as the primary camera architecture.

### 10.1 Rules

- UI must not sit on the main recording path
- preview stream must be separate from record stream
- only the latest preview frame is kept
- recording quality must not be reduced to satisfy UI smoothness


### 10.2 Data Path Split

For each camera:

- Record Path
  original topic, full quality, consumed by rosbag
- Preview Path
  lower rate or lower resolution topic for UI

Preview can be produced by:

- a dedicated preview bridge node
- existing low-rate display topic if already available


### 10.3 Camera UI Implementation Rule

UI preview component should use:

- background ROS executor thread
- one latest-frame buffer per stream
- fixed display refresh rate, e.g. 20 to 30 Hz
- separate metrics for RX FPS and Display FPS

If Python preview is still insufficient later, the preview module can be migrated to C++/Qt without changing orchestrator semantics.


### 10.4 Control-State Recording Rule

The same split used for camera preview vs record must also be applied to high-rate control state.

For control-related topics such as `/joint_states`:

- Control Path
  original topic at controller-native rate, consumed by controllers, monitors, and fast diagnostics
- Archive Path
  stack-owned reduced-rate topic such as `/joint_states_record`, consumed by rosbag and replay/export tools

Important rule:

- rosbag must not be expected to downsample a hot control topic on its own
- if archival rate must be reduced, it must be reduced before rosbag by a dedicated bridge or downsampler node

Practical consequence:

- a 1 kHz controller feedback topic should not be recorded directly by default on the control host
- a lower-rate archive topic, for example 50 to 100 Hz, should be the default recording source for trajectory-style datasets


## 11. Recorder Design

Recording must be centralized in `record_manager`.

Responsibilities:

- create `session_id`
- allocate session directory
- snapshot recipe and active config
- start rosbag
- stop rosbag cleanly
- write result summary
- record performance metrics and fault history


### 11.1 Boundary Rule

The recorder must be owned by `data_collection_stack`, not by a device package, UI widget, joystick node, or keyboard gate helper.

Rules:

- device packages may keep standalone recording helpers for local debugging
- stack-driven collection sessions must disable package-owned recording paths
- recipe, topic selection, sample segmentation, and session persistence must be decided by the supervisor and `record_manager`


### 11.2 Bag Lifecycle Rule

The production recorder must use one bag per collection session by default.

Rules:

- `StartSession` starts one rosbag process for the session
- `StopSession` stops that rosbag process
- sample boundaries inside one session should be represented by markers and event logs, not by stopping and recreating rosbag for every sample
- pause or resume may be used within one active session, but stop and flush must not sit in the operator hot path for per-sample interaction

This rule exists because repeated `stop -> flush -> start new bag` on the control host can introduce large latency spikes and unstable teleoperation behavior.


### 11.3 Archived Topic Policy

`record_manager` should record archive-suitable topics, not blindly record every control-path topic.

Rules:

- high-rate control topics should have stack-owned archive variants when needed
- example:
  - control topic `/joint_states`
  - archive topic `/joint_states_record`
- replay and export tools should accept configurable bag topics so they can consume archive topics without requiring the raw control-rate source

For the first trajectory-focused implementation:

- use reduced-rate archived joint state for session recording by default
- do not make raw high-rate `/joint_states` the default recording source on the control host


### 11.4 Performance Rule

Recorder behavior must be chosen to protect teleoperation and control responsiveness first.

Rules:

- synchronous bag stop and flush must not happen on every sample boundary
- compression should not be the default first-line optimization on the control host because online compression can increase CPU load and worsen latency
- control loop frequency must not be reduced only to make recording easier unless there is a separate control justification
- performance validation must first be done with recording disabled, then repeated with the centralized recorder enabled


Recommended session layout:

```text
data/
  2026-04-02/
    session_000001/
      metadata/
        session.yaml
        recipe.yaml
        system_snapshot.yaml
        devices/
          marvin.yaml
          camera_left.yaml
      logs/
        events.jsonl
        faults.jsonl
        performance.jsonl
        samples.jsonl
      rosbag/
      exports/
```

`session.yaml` should include:

- session id
- recipe id
- operator id
- start and stop time
- host info
- git commit if available
- selected devices
- recording mode and bag topic groups
- sample segmentation strategy


## 12. UI Design

The UI is an operator console, not a device owner.

Recommended panels:

- system status header
- device state panel
- camera preview panel
- session control panel
- fault panel
- event log panel

Recommended main commands:

- `Start System`
- `Shutdown System`
- `Arm`
- `Start Session`
- `Pause Session`
- `Resume Session`
- `Stop Session`
- `Go Home`
- `Acknowledge Fault`

Recommended hotkeys:

- `F5`
  Start System
- `Shift+F5`
  Shutdown System
- `Ctrl+Space`
  Arm or Start Session depending on state
- `P`
  Pause or Resume Session
- `S`
  Stop Session
- `H`
  Go Home
- `Esc`
  Acknowledge fault or close current dialog

UI rules:

- all buttons and hotkeys funnel into the same command dispatcher
- command availability is driven only by `SystemState`
- dangerous actions may require confirmation
- every issued command is logged


## 13. Config Files

Configuration must be organized by scope, not by the package that happens to contain the current YAML file today.

### 13.1 Layering Rule

Use four configuration layers:

- `package defaults`
  default values shipped with each device package and tightly coupled to code or controller implementation
- `site inventory`
  hardware serial numbers, namespaces, mounting corrections, deployment-specific paths
- `recipe`
  which devices participate in a collection scenario and how they are combined
- `runtime/operator override`
  operator identity, session root, temporary overrides, one-off launch arguments

Recommended priority:

- `runtime/operator override > recipe > site inventory > package defaults`

Rule:

- device packages keep their own defaults for standalone use
- `data_collection_stack` owns site, recipe, session, and operator configuration
- every device launch should support stack-provided config paths while still having package-local defaults


### 13.2 Package Defaults

Package defaults stay in each device package and should be moved under a clearer `defaults/` subdirectory over time.

Typical contents:

- controller definitions
- algorithm tuning defaults
- driver profile defaults
- package-local topic and service defaults

Examples that should stay package-local:

- `marvin_system/bringup/config/marvin_tracker_teleop_controllers.yaml`
- `marvin_system/bringup/config/marvin_dual_controllers.yaml`
- `marvin_system/bringup/config/marvin_ik_controllers.yaml`
- `camera_system/bringup/config/orbbec_defaults.yaml`
- `camera_system/bringup/config/realsense_defaults.yaml`
- most non-deployment-specific defaults in `manus_system` and `htc_system`


### 13.3 Site Inventory

Site inventory belongs in `data_collection_bringup/config/site/<site_name>/`.

It should contain:

- camera serial numbers
- tracker serial numbers
- namespaces and frame ids
- mounting corrections
- deployment-specific calibration directories
- machine-specific paths and hardware inventory

Recommended layout:

```text
data_collection_bringup/config/site/default_lab/
  cameras.yaml
  trackers.yaml
  manus.yaml
```


### 13.4 Operators

Operator-specific settings belong in `data_collection_bringup/config/operators/`.

Typical contents:

- operator id
- default save root
- preferred session tags
- user-specific manus calibration selection

Recommended layout:

```text
data_collection_bringup/config/operators/
  default.yaml
  hsz.yaml
```


### 13.5 `data_collection_orchestrator/config/devices.yaml`

Defines the stack-owned device registry used by the orchestrator.

This file is not the same as site inventory. It describes how the orchestrator knows about device classes and launchable units.

Should include:

- device id
- adapter type
- package and launch entry
- topic names
- service names
- preview topics
- record topics
- ready thresholds
- retry policy


### 13.6 `recipes/*.yaml`

Defines collection scenarios.

Should include:

- recipe id
- involved devices
- startup dependencies
- required or optional flag
- bag topic selection
- UI layout hints if needed


### 13.7 `session/*.yaml`

Session-level defaults belong in `data_collection_bringup/config/session/`.

Typical contents:

- recording root
- bag naming policy
- default bag topic groups
- reduced-rate archive topic policy
- sample marker policy
- storage backend and writer options
- whether compression is allowed by default
- preview rate limits
- session metadata defaults

Recommended layout:

```text
data_collection_bringup/config/session/
  recording.yaml
  ui.yaml
```


### 13.8 `policies/*.yaml`

Policy files define orchestrator behavior.

Typical contents:

- startup timeout
- retry count and backoff
- fault severity mapping
- ready stability windows
- degraded-mode rules


### 13.9 Existing File Mapping

The current codebase already contains several important YAML files. They should be reorganized as follows.

- `marvin_system/bringup/config/marvin_tracker_teleop_controllers.yaml`
  keep in `marvin_system` as a package default because it is tightly bound to the `TrackerTeleopController` implementation
- `marvin_system/bringup/config/marvin_gripper_teleop.yaml`
  split by concern
  keep `gripper_velocity` and `gripper_acceleration` with `marvin_system`
  move `manus_user_name` to operator config
  remove `start_manus` from `marvin` config and let recipe decide whether `manus` participates
- `marvin_system/bringup/config/marvin_tracker_teleop.yaml`
  move its recording-related contents to stack-owned session or recipe config because recording policy is not a `marvin` intrinsic
  stack-launched collection flow should disable package-owned keyboard-gated recording and let `record_manager` own rosbag lifecycle
- `htc_system/bringup/config/trackers.yaml`
  split publish defaults from deployment inventory
  keep generic publisher defaults in `htc_system`
  move serial numbers, frame ids, and corrections to site inventory
- `manus_system/bringup/config/manus_raw_publisher.yaml`
  keep generic publisher behavior in `manus_system`
  move calibration directory and user-selected calibration files to site or operator config
- `camera_system/bringup/config/cameras.yaml`
  move camera inventory to site config because it depends on the deployed hardware set


### 13.10 `ui.yaml`

Defines:

- refresh rate
- preview grid layout
- hotkey enable flags
- theme or panel defaults


### 13.11 Launch Override Rule

Every device launch file should follow this rule:

- when launched standalone, it uses package-local default config
- when launched from `data_collection_stack`, it accepts externally supplied config paths

This avoids breaking standalone package bringup while still letting the stack own deployment-specific configuration.


## 14. Implementation Phases

### Phase 0: Stack Skeleton

Goal:

- create the 4 packages
- define package boundaries
- establish dependency direction

Deliverables:

- all four ROS packages build successfully
- empty launch entry runs
- interfaces package exports placeholder messages/services/actions
- device launches still work standalone with package-local defaults


### Phase 1: Interfaces and Supervisor Core

Goal:

- implement the baseline state machine
- implement command server and system state publisher

Deliverables:

- `SystemState` and `DeviceState`
- `StartSystem`, `ShutdownSystem`, `StartSession`, `StopSession`, `GoHome`
- supervisor node with no-op adapter hooks


### Phase 2: Marvin Adapter

Goal:

- integrate the existing `marvin` stack first because it has the richest control semantics

Deliverables:

- `marvin` adapter
- bringup through recipe
- controller readiness probing
- `arm`, `disarm`, `home` mapped to supervisor commands
- `marvin` remains the ownership boundary for the integrated gripper path

Acceptance:

- system can bring `marvin` from `IDLE` to `READY`
- home and arm flow is controllable through supervisor


### Phase 3: Camera Adapter and Preview Split

Goal:

- integrate camera inventory
- separate preview from recording path

Deliverables:

- camera adapter
- topic rate probe
- preview bridge or preview policy
- camera readiness based on stability window
- camera inventory loaded from stack-owned site config rather than package-local deployment YAML

Acceptance:

- three cameras can be brought up in recipe order
- preview remains responsive without affecting recording topics


### Phase 4: HTC and Manus Integration

Goal:

- integrate tracker and glove input

Deliverables:

- `htc` adapter with TF freshness probe
- `manus` adapter with topic and calibration readiness probe
- full recipe `marvin + htc + manus + cameras`
- tracker and manus deployment-specific config loaded from stack-owned site and operator config

Acceptance:

- supervisor can validate complete teleop data collection chain


### Phase 5: Recorder and Session Metadata

Goal:

- implement robust session lifecycle

Deliverables:

- `record_manager`
- session directory generation
- one bag per session by default, not one bag per sample
- rosbag lifecycle
- reduced-rate archive topics for high-rate control data such as `/joint_states_record`
- sample marker and event-log based segmentation
- config snapshot and event log persistence

Acceptance:

- start/stop session works through supervisor only
- recording path does not introduce unacceptable teleop latency
- sample boundaries are recoverable from markers and logs without restarting rosbag
- metadata is complete and replayable


### Phase 6: UI

Goal:

- build first production UI

Deliverables:

- main window
- device panels
- preview area
- command buttons
- hotkeys
- fault and event display

Acceptance:

- UI crash must not kill recording if supervisor is separate
- all commands pass through interfaces only


### Phase 7: Wujihand and Additional Devices

Goal:

- prove extensibility

Deliverables:

- `wujihand` adapter
- updated recipes
- no major changes to existing supervisor core


## 15. Testing Strategy

Testing should be layered.

### Unit Tests

- state transitions
- retry policy evaluation
- recipe graph validation
- adapter result normalization

### Integration Tests

- supervisor with mocked adapters
- session start and stop
- fault injection and rollback

### Hardware-In-The-Loop Tests

- full device chain startup with recording disabled
- camera bringup stability
- tracker TF freshness
- marvin controller activation
- recorder throughput
- recorder-on vs recorder-off latency comparison

### Operator Acceptance Tests

- end-to-end system startup
- session workflow via UI
- hotkey workflow
- fault acknowledgement and recovery


## 16. First Concrete Deliverable

The first implementation target should be:

- one headless supervisor
- one minimal UI
- one recipe using:
  - `marvin`
  - `htc`
  - `cam_high`
  - `cam_left_wrist`
  - `cam_right_wrist`

Why this target:

- `marvin` already has strong control semantics
- cameras and tracker expose the main orchestration and performance problems
- it is enough to validate state machine, preview split, and session flow before adding `manus` and `wujihand`


## 17. Similarity to Game Architecture

This design does resemble patterns seen in Unity and game runtime architecture, especially:

- one authoritative application state machine
- manager or service layer for subsystems
- strict separation between simulation/control and presentation
- event-driven UI
- config-driven scene or scenario composition

The important difference is that here the subsystems are ROS devices and processes rather than game entities. The architectural idea is similar, but the implementation contract is centered on ROS topics, services, actions, TF, launch bringup, and hardware fault handling.


## 18. Immediate Next Steps

Recommended immediate actions:

1. Create the 4 packages under `data_collection_stack/`
2. Implement `data_collection_interfaces` first
3. Implement a headless `supervisor` skeleton with a minimal state machine
4. Integrate `marvin` adapter first
5. Integrate cameras with preview and recording split
6. Validate the full hardware chain without centralized recording first
7. Implement centralized recording only after the no-record path is stable
8. Add a minimal Qt UI after the headless flow works

This ordering keeps the core control plane stable before UI complexity is introduced.
