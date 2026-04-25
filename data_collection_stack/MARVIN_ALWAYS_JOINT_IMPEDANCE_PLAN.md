# Marvin Always Joint Impedance Plan

## 1. 目标

在 `data_collection_stack` 启动的数据采集流程中，保留 Marvin 运动层的上层语义：

- `SAFE_HOLD`
- `TELEOP`
- `MOTION`

但不再让这些上层模式触发底层硬件控制 profile 在 `POSITION_FOLLOW` 和
`JOINT_IMPEDANCE` 之间来回切换。目标行为是：

- 启动后底层硬件 profile 尽快进入 `JOINT_IMPEDANCE`
- 运行过程中 `SAFE_HOLD / TELEOP / MOTION` 都保持底层 `JOINT_IMPEDANCE`
- 上层模式仍然表达不同的系统语义和 controller 状态
- 规避当前 profile 切换逻辑反复报错的问题

## 2. 当前链路

`collection_app.launch.py` 本身不直接控制 Marvin 的底层 profile，它只启动
`data_collection_supervisor` 和可选 UI。

实际链路如下：

1. `data_collection_bringup/launch/collection_app.launch.py`
   - 启动 `data_collection_orchestrator/supervisor`
   - 加载 recipe
2. `data_collection_bringup/config/recipes/*.yaml`
   - 配置 Marvin device
   - 通过 `launch_arguments` 传参给 Marvin launch
3. `data_collection_orchestrator/data_collection_orchestrator/adapters/marvin.py`
   - 启动 `marvin_system/bringup/launch/marvin_tracker_teleop.launch.py`
   - 通过 `/marvin_motion/set_mode` 调用 `TELEOP`、`SAFE_HOLD`
   - 通过 `/marvin_motion/go_home` 调用 home 行为
4. `marvin_system/bringup/launch/marvin_tracker_teleop.launch.py`
   - 启动 `motion_server`
   - 给 `motion_server` 注入 profile、controller、go_home 等参数
5. `marvin_system/motion/src/motion_server.cpp`
   - 接收 `/marvin_motion/set_mode`
   - 将上层 mode 映射到底层硬件 `ControlProfile`
   - 调用 `/marvin_dual/set_control_profile`

当前 `motion_server` 中的映射逻辑是：

```text
TELEOP              -> JOINT_IMPEDANCE
SAFE_HOLD / MOTION  -> POSITION_FOLLOW
```

这意味着正常数据采集流程中，`TELEOP <-> SAFE_HOLD`、`go_home`、`MOTION` 都可能触发：

```text
JOINT_IMPEDANCE <-> POSITION_FOLLOW
```

如果硬件 profile 切换不稳定，就会导致当前观察到的报错。

## 3. 目标行为

新增一个明确的 profile 策略参数：

```yaml
control_profile_policy: always_joint_impedance
```

保留默认兼容行为：

```yaml
control_profile_policy: mode_mapped
```

两种策略定义如下。

### 3.1 `mode_mapped`

保持当前行为不变：

```text
SAFE_HOLD -> POSITION_FOLLOW
TELEOP    -> JOINT_IMPEDANCE
MOTION    -> POSITION_FOLLOW
```

### 3.2 `always_joint_impedance`

新增行为：

```text
SAFE_HOLD -> JOINT_IMPEDANCE
TELEOP    -> JOINT_IMPEDANCE
MOTION    -> JOINT_IMPEDANCE
```

注意：这里只固定底层硬件 profile，不改变上层 mode 语义。

上层 mode 仍然保持区别：

- `SAFE_HOLD`
  - primary controller active
  - teleop disarmed
  - teleop disabled
- `TELEOP`
  - primary controller active
  - teleop armed
  - session enabled 时 teleop enabled
- `MOTION`
  - trajectory controller active
  - MoveIt/go_home 等轨迹行为可用

## 4. 设计原则

1. 不把所有上层状态都改成 `TELEOP`
   - 否则 UI、session、go_home、fault handling 会失去明确语义。
2. 只修改 mode 到 hardware profile 的映射层
   - 保留 controller 切换、teleop enable/disable、go_home 流程。
3. 新策略必须可配置
   - 默认保留当前行为，避免影响其他使用场景。
4. READY 阶段要显式同步底层 profile
   - 硬件启动后默认可能是 `POSITION_FOLLOW`，不能只依赖后续 `TELEOP` 才切换。
5. legacy go_home 不能绕过策略
   - 当前 legacy go_home 准备逻辑中存在硬编码 `POSITION_FOLLOW`，需要一并收敛到统一策略。

## 5. 具体改动方案

### 5.1 `marvin_tracker_teleop.launch.py` 增加 launch 参数

文件：

```text
marvin_system/bringup/launch/marvin_tracker_teleop.launch.py
```

新增：

```python
DeclareLaunchArgument(
    "control_profile_policy",
    default_value="mode_mapped",
    description="Control profile policy: mode_mapped or always_joint_impedance.",
)
```

在 `launch_setup(context)` 中读取：

```python
control_profile_policy_value = (
    LaunchConfiguration("control_profile_policy").perform(context).strip()
)
```

并传给 `motion_server`：

```python
"control_profile_policy": control_profile_policy_value or "mode_mapped",
```

MoveIt backend 和 legacy backend 两个 `motion_server_node` 参数块都需要传入。

### 5.2 `motion_server.cpp` 增加策略参数

文件：

```text
marvin_system/motion/src/motion_server.cpp
```

新增成员变量：

```cpp
std::string control_profile_policy_;
```

构造函数中读取：

```cpp
control_profile_policy_ = get_param_or_declare<std::string>(
    this, "control_profile_policy", "mode_mapped");
```

建议做规范化：

```cpp
control_profile_policy_ = normalize_token(control_profile_policy_);
```

支持：

```text
MODE_MAPPED
ALWAYS_JOINT_IMPEDANCE
```

未知值建议启动时报错，避免错误配置静默退化。

### 5.3 修改 mode 到 profile 的映射

当前核心函数：

```cpp
std::optional<ControlProfile> desired_control_profile_for_mode(
    const std::string &normalized_mode) const
```

目标逻辑：

```cpp
if (control_profile_policy_ == "ALWAYS_JOINT_IMPEDANCE") {
    if (
        normalized_mode == kModeSafeHold ||
        normalized_mode == kModeTeleop ||
        normalized_mode == kModeMotion
    ) {
        return ControlProfile::kJointImpedance;
    }
}

// Existing mode_mapped behavior.
if (normalized_mode == kModeTeleop) {
    return teleop_use_joint_impedance_ ?
        std::optional<ControlProfile>(ControlProfile::kJointImpedance) :
        std::optional<ControlProfile>(ControlProfile::kPositionFollow);
}
if (normalized_mode == kModeSafeHold || normalized_mode == kModeMotion) {
    return ControlProfile::kPositionFollow;
}
return std::nullopt;
```

### 5.4 修正 legacy go_home 的硬编码 profile

当前 `prepare_legacy_go_home()` 中存在：

```cpp
return call_control_profile(ControlProfile::kPositionFollow, error_message);
```

这会绕过统一策略。应改为：

```cpp
return ensure_mode_control_profile(kModeMotion, error_message);
```

或者新增专门函数：

```cpp
bool ensure_go_home_control_profile(std::string &error_message)
```

其内部也走 `control_profile_policy`。推荐使用专门函数，语义更清楚。

在 `always_joint_impedance` 下，legacy go_home 也保持 `JOINT_IMPEDANCE`。

### 5.5 READY 阶段强制同步一次 profile

文件：

```text
data_collection_orchestrator/data_collection_orchestrator/adapters/marvin.py
```

当前 `wait_ready()` 只检查：

- controller_manager ready
- required controllers ready
- `/marvin_motion/*` 服务 ready
- `get_status` healthy

新增逻辑：

在 controllers、services、status 都 ready 后，如果配置了：

```yaml
control_profile_policy: always_joint_impedance
```

则调用：

```text
/marvin_motion/set_mode SAFE_HOLD
```

原因：

- 启动后硬件默认可能仍是 `POSITION_FOLLOW`
- `SAFE_HOLD` 是安全的上层语义
- 在 `always_joint_impedance` 策略下，`SAFE_HOLD` 会把底层同步到 `JOINT_IMPEDANCE`
- READY 返回前即可保证系统已经处于目标 profile

实现上可新增一个 helper：

```python
def _should_force_joint_impedance_on_ready(self) -> bool:
    launch_arguments = self._launch_arguments()
    return (
        str(launch_arguments.get("control_profile_policy", "mode_mapped"))
        .strip()
        .lower()
        == "always_joint_impedance"
    )
```

然后在 READY 成功前调用：

```python
if self._should_force_joint_impedance_on_ready():
    sync_result = self._call_set_mode_service("SAFE_HOLD", timeout_sec=10.0)
    if sync_result.is_failure():
        return sync_result
```

### 5.6 recipe 启用策略

默认数据采集 recipe 中 Marvin device 增加：

```yaml
launch_arguments:
  use_gripper_L: true
  use_gripper_R: true
  start_manus_gripper_bridge: true
  control_profile_policy: always_joint_impedance
```

至少需要覆盖当前默认 recipe：

```text
data_collection_bringup/config/recipes/marvin_tracker_manus_camera_collection.yaml
```

如果其他 recipe 也用于真实 Marvin 数据采集，则一并加上该参数。

## 6. 验证计划

### 6.1 静态验证

编译：

```bash
colcon build --packages-select marvin_system data_collection_orchestrator data_collection_bringup
```

### 6.2 启动验证

启动 collection app 后，检查 Marvin 状态：

```bash
ros2 service call /marvin_motion/get_status marvin_system/srv/GetMotionStatus {}
```

期望：

```text
success: true
mode: SAFE_HOLD
controller_interlock_ok: true
```

同时检查 `/dynamic_joint_states` 中 Marvin anchor joint 的 state interfaces：

```text
active_control_profile == JOINT_IMPEDANCE
requested_control_profile == JOINT_IMPEDANCE
```

如果显示为数值，则根据当前定义：

```text
POSITION_FOLLOW  = 1
JOINT_IMPEDANCE  = 2
```

期望：

```text
active_control_profile == 2
requested_control_profile == 2
```

### 6.3 模式切换验证

执行：

```bash
ros2 service call /marvin_motion/set_mode marvin_system/srv/SetMotionMode "{mode: TELEOP}"
ros2 service call /marvin_motion/set_mode marvin_system/srv/SetMotionMode "{mode: SAFE_HOLD}"
```

期望：

- 上层 `mode` 能在 `TELEOP` 和 `SAFE_HOLD` 之间变化
- 底层 `active_control_profile` 始终为 `JOINT_IMPEDANCE`
- 日志中不再出现 `JOINT_IMPEDANCE -> POSITION_FOLLOW`

### 6.4 go_home / MOTION 验证

执行：

```bash
ros2 service call /marvin_motion/go_home std_srvs/srv/Trigger {}
```

期望：

- `go_home` 能进入 `MOTION` 流程
- trajectory controller 正常 active
- 底层 profile 保持 `JOINT_IMPEDANCE`
- `go_home` 完成后回到配置的 `go_home_return_mode`
- 不触发 `POSITION_FOLLOW`

### 6.5 collection session 验证

通过 data collection stack 执行一次完整 session：

- start system
- start session
- teleop enabled
- stop session
- shutdown system

期望：

- session 前后 `before_session()` / `after_session()` 只改变 teleop enabled 状态
- 不改变底层 hardware profile
- 数据记录流程不受影响

## 7. 风险和边界

### 7.1 轨迹精度风险

`MOTION/go_home` 在 `JOINT_IMPEDANCE` 下执行时，轨迹跟踪会比
`POSITION_FOLLOW` 更软，可能导致：

- 目标误差变大
- 收敛时间变长
- MoveIt trajectory controller 判断到达目标更慢
- go_home verification 需要更宽容的 settle timeout 或 tolerance

### 7.2 安全语义仍需保留

`SAFE_HOLD` 不能简单等价于 `TELEOP`。即使底层 profile 相同，`SAFE_HOLD` 仍应保持：

- teleop disabled
- teleop disarmed
- primary controller active
- 不接受 tracker 目标驱动

### 7.3 启动初始 profile 仍可能先短暂为 POSITION_FOLLOW

硬件 `on_activate()` 当前默认可能进入 `POSITION_FOLLOW`。本方案不强行修改硬件激活流程，而是在 Marvin READY 前通过 `set_mode SAFE_HOLD` 同步到
`JOINT_IMPEDANCE`。

如果未来需要彻底避免启动阶段短暂 `POSITION_FOLLOW`，可以再增加硬件层参数：

```yaml
initial_control_profile: JOINT_IMPEDANCE
```

但这属于第二阶段改动，风险更高。

## 8. 推荐实施顺序

1. 给 `marvin_tracker_teleop.launch.py` 增加 `control_profile_policy` 参数。
2. 给 `motion_server.cpp` 增加策略读取和校验。
3. 修改 `desired_control_profile_for_mode()`。
4. 修正 `prepare_legacy_go_home()` 中的硬编码 `POSITION_FOLLOW`。
5. 在 Marvin adapter READY 阶段增加 `SAFE_HOLD` profile sync。
6. 在数据采集 recipe 中启用 `control_profile_policy: always_joint_impedance`。
7. 编译并做静态检查。
8. 先 mock 或空载验证 service 行为。
9. 再实机验证启动、TELEOP、SAFE_HOLD、go_home、完整 session。

## 9. 成功标准

该方案完成后，应满足：

- `SAFE_HOLD / TELEOP / MOTION` 上层 mode 仍正常报告和切换
- `active_control_profile` 在 READY 后保持 `JOINT_IMPEDANCE`
- `requested_control_profile` 在 READY 后保持 `JOINT_IMPEDANCE`
- `set_mode SAFE_HOLD` 不再触发 `JOINT_IMPEDANCE -> POSITION_FOLLOW`
- `set_mode MOTION` 不再触发 `JOINT_IMPEDANCE -> POSITION_FOLLOW`
- `go_home` 不再触发 `JOINT_IMPEDANCE -> POSITION_FOLLOW`
- collection app 启动后 Marvin READY 条件包含底层阻抗 profile 同步
