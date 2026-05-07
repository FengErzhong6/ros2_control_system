# Marvin 正确启动指南

本文记录当前仓库中 Marvin 的正确启动方式，作为硬件插件、motion 层和数据采集编排的共同参考。

## 适用范围

以下入口都必须遵守同一套启动契约：

- `data_collection_bringup/launch/collection_app.launch.py`
- `data_collection_bringup/launch/headless_supervisor.launch.py`
- `marvin_system/bringup/launch/marvin_tracker_teleop.launch.py`

## 典型启动命令

推荐把 Marvin 作为数据采集栈的一部分启动：

```bash
source install/setup.bash
ros2 launch data_collection_bringup collection_app.launch.py use_ui:=true
```

无 UI 的 headless 场景：

```bash
source install/setup.bash
ros2 launch data_collection_bringup headless_supervisor.launch.py
```

## 正确启动顺序

推荐的人工操作顺序是：

1. 启动 `data_collection_bringup` 的顶层入口。
2. 先执行 `StartSystem`，让 Marvin 栈完成 bringup 并进入 READY。
3. 再执行 `StartTeleoperation`。
4. 需要录制时再执行 `StartSession`。
5. 结束录制时先执行 `StopSession`。
6. 结束遥操作时再执行 `StopTeleoperation`。
7. 最后执行 `ShutdownSystem`。

历史兼容行为仍然保留：如果直接执行 `StartSession`，而 teleop 尚未手动启动，supervisor 会先补上旧的“arm + before_session”流程，再进入录制。

## Marvin 侧必须满足的约束

### 1. 硬件 activation 先进入 POSITION

`marvin_dual` 的 `on_activate()` 必须先完成位置模式 bootstrap，并让 `ros2_control_node` 顺利启动控制器管理器。

### 2. JOINT_IMPEDANCE 的切换必须延后到 write loop

从启动安全性看，`startup_control_profile == JOINT_IMPEDANCE` 时，`JOINT_IMPEDANCE` 不应在 activation 阶段强切完成，而应由 `write()` 在控制循环里按状态机完成。

### 3. `read()` 在 deferred startup 期间不能误判

在 deferred startup 期间，硬件短暂停留在 `POSITION` 是正常状态。
`read()` 不能把这段过渡期当成 watchdog 失败，否则 `controller_manager` 会提前 deactivate `marvin_dual`。

### 4. 进入 READY 之后才允许上层控制器接管

只有当硬件、`joint_state_broadcaster`、`tracker_teleop_controller`、`dual_arm_trajectory_controller` 都准备完成后，才算真正 READY。

## 关键验收点

成功启动时，日志里应能看到：

- `Hardware activation completed in POSITION; deferred JOINT_IMPEDANCE startup switch to the write loop.`
- `joint_state_broadcaster` 成功激活
- `tracker_teleop_controller` 成功激活
- `marvin_motion_server` 就绪

同时，`marvin_dual` 的启动日志里不应再出现：

- `Joint-impedance watchdog tripped on arm 0`
- `Unable to activate controller 'tracker_teleop_controller' since the command interface 'Joint1_L/position' is not available`

## 不要再改回去的点

1. 不要把初始阻抗切换塞回 `on_activate()`。
2. 不要让 `read()` 在 deferred startup 时要求 `TORQ + m_ImpType=1`。
3. 不要把 `StartSession` 的兼容性回滚掉，否则旧采集流程会再次断。
4. 不要让残留 launch 树和这次启动共用同一套 owner。
