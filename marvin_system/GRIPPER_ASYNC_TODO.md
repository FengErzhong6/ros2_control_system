# Gripper Async Refactor TODO

## Context

Current behavior:

- Arm joint commands are sent through the Marvin SDK batch path inside `MarvinHardware::write()`.
- Gripper commands are sent through the OmniPicker SDK via `set_position_percent()`.
- The gripper SDK call currently happens on the `ros2_control` hot path.
- A temporary mitigation is already in place: gripper command de-duplication and rate limiting in `write()`.

Observed issue:

- When gripper control is enabled, teleoperation responsiveness can degrade.
- The likely cause is that gripper SDK I/O is synchronous and shares the same control-loop thread as arm writes.

Desired end state:

- `MarvinHardware::write()` only updates the latest desired gripper target.
- Actual OmniPicker SDK I/O runs on a dedicated async worker path.
- `MarvinHardware::read()` remains non-blocking and only consumes cached state.

## Goals

- Remove blocking gripper SDK calls from the `ros2_control` write loop.
- Keep existing ROS interfaces unchanged.
- Preserve the current rate-limit and de-dup semantics or improve them.
- Make gripper failures visible without dragging down arm control.

## Non-Goals

- Do not redesign MANUS-to-gripper mapping.
- Do not change gripper controller topic names or message types.
- Do not mix this refactor with unrelated teleop, UI, or supervisor changes.

## Files Likely To Touch

- `marvin_system/hardware/include/marvin_system/marvin.hpp`
- `marvin_system/hardware/src/marvin.cpp`
- `marvin_system/description/ros2_control/marvin_dual_system.ros2_control.xacro`

Possible follow-up files if diagnostics are expanded:

- `marvin_system/bringup/config/marvin_gripper_teleop.yaml`
- `marvin_system/bringup/launch/marvin_tracker_teleop.launch.py`

## TODO

- [ ] Add dedicated per-gripper async runtime state to `marvin.hpp`.
  - Suggested fields: desired command, last sent command, cached position, worker thread, stop flag, mutex, condition variable, failure counters, last error, last send timestamp.

- [ ] Start and stop gripper worker threads in the hardware lifecycle.
  - Start after gripper devices are connected and activated.
  - Stop and join cleanly in `on_deactivate()`, `on_cleanup()`, and destructor cleanup paths.

- [ ] Refactor `MarvinHardware::write()` so it no longer calls `set_position_percent()` directly.
  - Keep bounds checking and command extraction in `write()`.
  - Only publish the latest desired target into shared state.
  - Notify the worker when the target changes enough to matter.

- [ ] Move OmniPicker command dispatch into the worker loop.
  - Enforce `gripper_command_rate_hz` inside the worker.
  - Apply `gripper_command_epsilon` before sending.
  - Always send outside the `ros2_control` hot path.
  - Add backoff or throttled retries if repeated send failures occur.

- [ ] Keep `read()` non-blocking.
  - Do not introduce `query_states()` or any other blocking SDK request into `read()`.
  - Continue using cached state only.
  - If needed, let the worker refresh cached gripper position after successful sends.

- [ ] Decide the ownership model for cached gripper feedback.
  - Option A: continue reading `get_position_percent()` in `read()` if it is guaranteed non-blocking.
  - Option B: let the worker own all gripper feedback updates and expose cached values to `read()`.
  - Prefer the option with the strictest guarantee that `read()` never blocks.

- [ ] Add diagnostics for the async path.
  - Track per-gripper consecutive send failures.
  - Track last successful send time.
  - Track last requested command versus last sent command.
  - Log throttled warnings when lag or failures exceed a threshold.

- [ ] Review thread-safety around shutdown and hardware disconnect.
  - Worker must not call the SDK after device disconnect begins.
  - Shutdown ordering must prevent use-after-free on OmniPicker devices.

- [ ] Keep the current temporary mitigation until the async worker is proven stable.
  - Do not remove rate limiting too early.
  - After the worker lands, simplify or remove duplicate gating only if measurements show it is safe.

## Suggested Implementation Shape

### Phase 1

- Introduce worker state and lifecycle management.
- Keep existing command semantics.
- Move SDK sends off the hot path with minimal architecture change.

### Phase 2

- Improve diagnostics and observability.
- Add latency counters or loop-jitter counters if needed.

### Phase 3

- Revisit whether command de-dup and rate limiting should live in `write()` or entirely inside the worker.

## Acceptance Criteria

- With gripper enabled, arm teleop responsiveness should remain comparable to the no-gripper case.
- `MarvinHardware::write()` should contain no direct OmniPicker send call.
- The gripper path should tolerate transient send failures without destabilizing arm control.
- Shutdown should complete cleanly with no hanging worker threads.
- Behavior should remain correct for:
  - both grippers enabled
  - only left gripper enabled
  - only right gripper enabled
  - no grippers enabled

## Validation Checklist

- [ ] Build `marvin_system` successfully.
- [ ] Verify bringup and shutdown with grippers disabled.
- [ ] Verify bringup and shutdown with one gripper enabled.
- [ ] Verify bringup and shutdown with two grippers enabled.
- [ ] Verify teleop smoothness while repeatedly opening and closing grippers.
- [ ] Check logs for repeated send failures, shutdown races, or stuck worker threads.
- [ ] Compare arm control-loop stability before and after the refactor.

## Notes

- The current rate-limit/de-dup fix is useful as a guardrail but should be treated as an interim mitigation, not the final architecture.
- The main architectural rule for the future change is simple: no blocking gripper SDK I/O inside `read()` or `write()`.
