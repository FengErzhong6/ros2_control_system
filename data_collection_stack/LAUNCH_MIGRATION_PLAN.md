# In-Process Launch Migration Plan

## Goal

Migrate data collection device bringup away from external `ros2 launch` / `ros2 run`
subprocesses so the supervisor owns each device runtime directly via Python
`launch` / `LaunchService`.

## Why

- Avoid residual `python3 ros2 launch ...` wrapper processes.
- Make shutdown deterministic by keeping launch handles in supervisor memory.
- Reduce startup ambiguity: track launch start, child process exit, and launch
  shutdown without inferring state from shell process trees.
- Replace command-line orphan matching with authoritative session ownership.

## Scope

The migration covers the device adapters under
`data_collection_orchestrator/data_collection_orchestrator/adapters/`:

- `camera.py`
- `htc.py`
- `manus.py`
- `marvin.py`

It also requires changes in:

- `supervisor.py`
- `runtime_manifest.py`
- `package.xml`
- `setup.py`

## Target Architecture

### Managed Launch Session

Add a new orchestrator-local module, tentatively
`data_collection_orchestrator/managed_launch.py`, with a `ManagedLaunchSession`
class that:

- Creates a private `LaunchService`
- Includes one `LaunchDescription`
- Runs the launch service in a dedicated thread
- Tracks:
  - session id
  - device id
  - launch state
  - child pids
  - child exit codes
  - launch start / stop timestamps
  - captured log file paths
- Exposes:
  - `start()`
  - `shutdown(timeout_sec)`
  - `is_running()`
  - `process_snapshot()`
  - `wait_for_idle(timeout_sec)`

### Supervisor Ownership

The supervisor remains the single parent controller:

- adapters create managed launch sessions
- supervisor keeps adapter objects alive
- shutdown and retries are performed by adapter-owned launch handles, not shell
  process groups

### Runtime Manifest Evolution

Runtime manifest becomes a crash-recovery registry instead of the primary process
control mechanism.

It should store:

- supervisor pid
- recipe id
- system state
- device id
- logical launch session id
- child pids collected from launch events
- timestamps

Stale cleanup should operate on registry entries first. Command-line orphan
matching remains only as a temporary compatibility fallback.

## Migration Stages

### Stage 1: Infrastructure

Deliverables:

- `managed_launch.py`
- launch event helpers for child pid / exit tracking
- unit-testable launch session lifecycle abstraction

Success criteria:

- can start and stop a trivial launch description without leaving wrapper
  processes
- child pids are observable from launch event callbacks

### Stage 2: Camera Adapter

Replace camera adapter subprocess launch with `ManagedLaunchSession`.

Why first:

- smallest blast radius
- easiest to validate through image topics
- directly improves `cam_high` cleanup behavior

Success criteria:

- no `ros2 launch camera_system ...` wrapper process remains after shutdown
- `cam_high` first-launch restart behavior still works
- logs remain accessible in adapter diagnostics

### Stage 3: Manus and HTC Adapters

Migrate:

- `manus.py`
- `htc.py`

Notes:

- `htc` should stop relying on `ros2 run ... tracker_publisher`
- create or reuse a stable launch description path where needed

Success criteria:

- no wrapper process remains for tracker / manuscript bringup
- ready probes still work

### Stage 4: Marvin Adapter

Migrate `marvin.py` last because it has the deepest launch graph:

- `ros2_control_node`
- `robot_state_publisher`
- controller spawners
- `move_group_wrapper.py`
- `motion_server`
- optional gripper bridge

Success criteria:

- Marvin retries no longer depend on parent shell process exit
- launch shutdown drains all child nodes deterministically
- ready diagnostics combine launch state and ROS service state

### Stage 5: Manifest / Cleanup Simplification

Once all adapters use managed launch sessions:

- remove most shell-command orphan matching
- keep only targeted fallback cleanup for legacy or externally started nodes
- simplify adapter `shutdown()` implementations

## Design Constraints

- Keep adapter public behavior stable where possible:
  - `bringup()`
  - `wait_ready()`
  - `shutdown()`
  - `diagnose()`
- Avoid spawning nested sub-supervisors.
- Preserve existing recipe semantics and retry policy.
- Keep device-specific logs on disk for postmortem inspection.
- Cleanup must stay idempotent under repeated signal delivery.

## Risks

- `launch` threading and `rclpy` executor interaction can deadlock if not kept
  isolated.
- Some launch descriptions may rely on behavior specific to the `ros2 launch`
  CLI wrapper.
- Marvin launch startup and shutdown are already timing-sensitive.
- Child process event coverage must be validated against actual launch actions.

## Validation Plan

For each migrated adapter:

1. Start device from supervisor
2. Confirm ready condition
3. Stop device
4. Verify `ros2 node list` and host process table do not retain wrapper
   processes
5. Re-run immediate restart to confirm no stale state leakage

For Marvin:

1. Cold start from supervisor
2. Retry within same supervisor session
3. Shutdown via command
4. Shutdown via `SIGINT`
5. Verify no residual `ros2_control_node`, `motion_server`, `move_group`, or
   spawner processes remain

## Immediate Implementation Order

1. Add `ManagedLaunchSession`
2. Migrate `camera.py`
3. Validate camera cleanup and restart behavior
4. Then proceed to `htc.py` / `manus.py`
5. Migrate `marvin.py` last
