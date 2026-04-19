# Marvin Teleop Joint-Impedance Switch Plan

## 1. 目标

本计划的目标是让 Marvin 在不同业务场景下使用不同的底层控制 profile：

1. `teleop` 进入后，底层使用 `joint impedance`。
2. `go_home`、`MoveIt` 轨迹执行、`SAFE_HOLD` 等非遥操作场景，底层使用 `position follow`。
3. 上层 `tracker_teleop_controller` 继续输出关节位置目标，不引入新的高频控制器接口。
4. 模式切换过程中不允许出现明显跳变、失控、长时间阻塞或破坏现有 1 kHz 控制循环。

## 2. 当前系统现状

当前实现中，Marvin 硬件插件始终工作在位置模式：

1. `MarvinHardware::on_activate()` 在激活时请求 `OnSetTargetState_A/B(1)`，即位置跟随模式。
2. `MarvinHardware::write()` 在每个控制周期内持续下发 `OnSetJointCmdPos_A/B(...)`。
3. 当前代码中没有任何 `OnSetImpType_*`、`OnSetJointKD_*`、`OnSetTool_*` 的调用，因此没有真正进入关节阻抗模式。
4. `motion_server` 当前切换的是 ROS 控制器状态和 teleop armed/enabled 状态，不是 Marvin SDK 的底层控制模式。
5. `legacy go_home` 路径当前会先转入 `TELEOP` 再调旧的 `tracker_teleop_controller/go_home`，这会和“TELEOP 一律阻抗模式”的新需求发生冲突。

## 3. 已确认的 SDK 事实

根据 `/home/mmlab/codes/huangshzh/asset/Robot/marvin/sdk/contrlSDK` 中的头文件和实现，已经确认以下事实：

1. `OnSetTargetState_*` 中：
   - `1 = POSITION`
   - `2 = PVT`
   - `3 = TORQ`
2. `OnSetImpType_*` 中：
   - `1 = 关节阻抗`
   - `2 = 笛卡尔阻抗`
   - `3 = 力控`
3. `OnSetJointKD_*` 只在 `OnSetTargetState(3)` 且 `OnSetImpType(1)` 时有意义。
4. `OnSetJointCmdPos_*` 的注释明确说明其适用于“位置模式、扭矩模式下的关节指令”。
5. `OnSetTool_*` 的注释明确说明工具动力学参数在扭矩模式下有意义，因此进入关节阻抗前需要明确考虑工具参数。
6. SDK 要求 API 之间至少间隔 `1 ms`；清错等少数接口建议更长延时。

这意味着从 SDK 能力上，“teleop 使用关节阻抗，但仍然发送关节位置目标作为平衡点”是可行的。

## 4. 核心设计原则

### 4.1 分离 Motion Mode 和 Hardware Control Profile

必须把下面两个概念分开：

1. `MotionMode`
   - `SAFE_HOLD`
   - `TELEOP`
   - `MOTION`
   - `FAULT`
2. `HardwareControlProfile`
   - `POSITION_FOLLOW`
   - `JOINT_IMPEDANCE`

原因：

1. `TELEOP` 是运动层语义，不应自动等价于某一种 go-home 行为。
2. `legacy go_home` 当前依赖 `tracker_teleop_controller`，它需要主控制器仍然在线，但底层控制 profile 应该临时切回位置模式。
3. 后续如果引入更多 teleop 变体，不应把 profile 决策硬编码到 `TELEOP` 模式名上。

### 4.2 不允许从 service callback 直接并发操作 SDK

这是实现里的第一风险点。

当前 SDK 调用都集中在硬件层 `read()/write()` 路径里。如果在 ROS service callback 或 motion_server 线程里直接调用 `OnClearSet()`、`OnSetTargetState()`、`OnSetSend()`，会和 `write()` 并发访问 SDK 内部发送缓存，带来以下风险：

1. 指令包交叉写入。
2. 发送序列错乱。
3. 模式切换过程中被正常位置写循环覆盖。
4. 难以复现的瞬时故障。

因此，模式切换必须通过“请求 + 状态机”方式在硬件主控制路径内串行完成，而不是在 service 线程里直接调 SDK。

### 4.3 模式切换必须无跳变

从 `POSITION_FOLLOW -> JOINT_IMPEDANCE` 或反向切换时，必须先读取当前反馈位置，把当前关节位置作为切换时的 hold target，然后再发目标模式和目标位姿。

否则会出现：

1. 切换瞬间把旧 command 当成新 profile 的平衡点，导致机械臂跳动。
2. teleop 切换时直接追逐一个过时目标，产生明显冲击。

### 4.4 不在 1 kHz fast path 中引入阻塞 sleep

当前系统控制循环为 `1 kHz`。模式切换不允许在 `write()` 中直接执行几百毫秒的 sleep 等待。正确方式应是：

1. 在控制循环里发送一次切换请求包。
2. 随后在多个周期中轮询 `m_CurState` 是否进入目标状态。
3. 通过超时和状态机推进完成切换。

## 5. 目标行为定义

### 5.1 业务场景到 profile 的映射

建议的默认映射如下：

1. `SAFE_HOLD`
   - 主控制器 active
   - teleop disarmed
   - `POSITION_FOLLOW`
2. `TELEOP`
   - 主控制器 active
   - teleop armed/enabled
   - `JOINT_IMPEDANCE`
3. `MOTION`
   - trajectory controller active
   - teleop disarmed
   - `POSITION_FOLLOW`
4. `legacy go_home`
   - 主控制器 active
   - tracker controller 提供 home 逻辑
   - 底层必须临时切回 `POSITION_FOLLOW`

### 5.2 profile 切换的时机

建议切换时机如下：

1. `set_mode(TELEOP)` 成功前，先切到底层 `JOINT_IMPEDANCE`。
2. `set_mode(SAFE_HOLD)` 时，先切到底层 `POSITION_FOLLOW`，再 disarm teleop。
3. `set_mode(MOTION)` 时，先切到底层 `POSITION_FOLLOW`，再切 ROS trajectory controller。
4. `go_home` 启动前，无论当前是否在 teleop，都先切到底层 `POSITION_FOLLOW`。
5. `go_home` 完成并返回 `TELEOP` 时，再切回 `JOINT_IMPEDANCE`。

## 6. 建议的实现架构

### 6.1 在 MarvinHardware 中引入显式 profile 状态机

新增以下状态：

1. `current_profile`
2. `requested_profile`
3. `transition_active`
4. `transition_phase`
5. `transition_start_time`
6. `transition_hold_deg[2][7]`
7. `transition_error_message`

建议的 phase：

1. `Idle`
2. `CaptureHold`
3. `SendSwitchPacket`
4. `WaitArmState`
5. `Stabilize`
6. `Completed`
7. `Failed`

### 6.2 新增硬件层 service，但 service 只提交请求

建议新增一个硬件层 service，例如：

`marvin_system/srv/SetControlProfile.srv`

建议格式：

```srv
string profile
---
bool success
string message
string active_profile
```

其职责：

1. 校验 profile 名称是否合法。
2. 提交 `requested_profile`。
3. 等待状态机完成或超时。
4. 不直接调用任何 SDK 切换接口。

### 6.3 继续保持位置命令接口不变

`tracker_teleop_controller` 不必改成扭矩控制器。当前 teleop controller 继续输出 joint position command 即可，因为 SDK 已明确支持在扭矩模式下使用 `OnSetJointCmdPos_*` 作为关节目标。

这可以显著降低改造范围：

1. 不需要重写 teleop controller 的控制逻辑。
2. 不需要改 `ros2_control` 接口类型。
3. collision/workspace guard 仍然可以继续基于关节目标位置工作。

## 7. 模式切换时序设计

### 7.1 Position Follow -> Joint Impedance

推荐时序：

1. 从最近一次 `read()` 的反馈中读取 `m_FB_Joint_PosE`，保存为 hold target。
2. 冻结正常命令发送，进入 profile transition state。
3. 在一次 `write()` 中发送一包切换命令：
   - `OnClearSet()`
   - `OnSetTool_A/B(...)`
   - `OnSetJointKD_A/B(K, D)`
   - `OnSetImpType_A/B(1)`
   - `OnSetTargetState_A/B(3)`
   - `OnSetJointCmdPos_A/B(hold_target)`
   - `OnSetSend()`
4. 之后在每个控制周期轮询 `m_CurState`，直到双臂都进入 `ARM_STATE_TORQ`。
5. 连续若干周期继续发送 hold target，确认没有异常错误码和明显跳变。
6. 标记 `current_profile = JOINT_IMPEDANCE`，恢复正常 write()。

### 7.2 Joint Impedance -> Position Follow

推荐时序：

1. 从最新反馈中读取当前位置作为 hold target。
2. 冻结正常命令发送。
3. 在一次 `write()` 中发送：
   - `OnClearSet()`
   - `OnSetJointLmt_A/B(vel_ratio, acc_ratio)`
   - `OnSetTargetState_A/B(1)`
   - `OnSetJointCmdPos_A/B(hold_target)`
   - `OnSetSend()`
4. 在后续周期轮询 `m_CurState`，直到双臂进入 `ARM_STATE_POSITION`。
5. 再保持若干周期 hold，确认切换稳定。
6. 标记 `current_profile = POSITION_FOLLOW`，恢复正常 write()。

### 7.3 失败处理

如果出现以下任一情况，应进入失败路径：

1. `OnSetSend()` 失败。
2. 双臂任一进入 `ERROR`。
3. 超过 profile switch timeout。
4. 状态长期停留在 `TRANS_TO_TORQ` 或 `TRANS_TO_POSITION`。

失败策略建议：

1. 记录错误日志。
2. 将 motion layer 模式标记为 `FAULT` 或至少拒绝继续 teleop。
3. 尝试回到 `POSITION_FOLLOW + hold current position`，如果回退失败则升级为 fault。

## 8. 时序和性能约束

### 8.1 控制循环内的约束

1. `read()/write()` 不得分配大对象，不得执行磁盘 I/O。
2. `profile transition` 的执行必须是常量复杂度、无动态内存分配。
3. 每个控制周期至多做一次状态机推进，不在 fast path 中显式 sleep。

### 8.2 SDK 访问串行化

所有 SDK 操作必须只在硬件主路径里发生。推荐约束：

1. `read()` 负责 `OnGetBuf(...)`。
2. `write()` 负责 `OnClearSet/OnSet.../OnSetSend()`。
3. service callback 和 motion_server 线程只设置请求，不直接碰 SDK。

### 8.3 切换窗口内冻结指令

在 `transition_active` 为真时：

1. 忽略 teleop controller 的实时新 command。
2. 统一发送 hold target。
3. 禁止 trajectory controller 或其他路径抢写。

### 8.4 对现有 1 kHz 循环的影响控制

切换过程预计只增加：

1. 若干个额外的状态判断分支。
2. 少量原子变量读写。
3. 一次额外的 batched SDK 配置包发送。

正常 steady state 下不应增加明显 CPU 开销。

## 9. 参数设计

为避免硬编码，建议把以下参数显式化。

### 9.1 profile 开关

1. `teleop_use_joint_impedance`
   - 默认建议先为 `false`
   - 完成验证后再切到 `true`

### 9.2 阻抗参数

1. `joint_impedance_k_left`
2. `joint_impedance_d_left`
3. `joint_impedance_k_right`
4. `joint_impedance_d_right`

建议先放在 `ros2_control` hardware parameters 中，和 `joint_vel_ratio/joint_acc_ratio` 同层管理。

### 9.3 工具参数

1. `tool_kine_left`
2. `tool_dyn_left`
3. `tool_kine_right`
4. `tool_dyn_right`

如果当前末端工具参数未知，必须先确认 vendor 推荐值，不能在扭矩模式下默认留空。

### 9.4 时序参数

1. `profile_switch_timeout_ms`
2. `profile_switch_stabilize_cycles`
3. `profile_switch_reject_when_busy`

## 10. Motion Layer 改造点

### 10.1 motion_server 需要新增 hardware profile client

`motion_server` 需要调用硬件层 `set_control_profile`，但不应直接碰 SDK。

### 10.2 transition_to_mode() 只负责运动层状态，不直接等价到底层 profile

当前 `transition_to_mode()` 的语义过于粗糙。需要改成：

1. `set_mode(TELEOP)`：
   - primary controller active
   - teleop armed
   - hardware profile = `JOINT_IMPEDANCE`
2. `set_mode(SAFE_HOLD)`：
   - primary controller active
   - teleop disarmed
   - hardware profile = `POSITION_FOLLOW`
3. `set_mode(MOTION)`：
   - trajectory controller active
   - teleop disarmed
   - hardware profile = `POSITION_FOLLOW`

### 10.3 legacy go_home 需要特殊路径

`legacy go_home` 不能再简单复用“进入 TELEOP”这一套。

建议改成：

1. 保证 primary controller active。
2. 切到底层 `POSITION_FOLLOW`。
3. 仅把 tracker teleop controller 置于 home 所需的 armed 状态。
4. 调用 legacy go_home。
5. 完成后根据 `go_home_return_mode` 决定是否恢复 `TELEOP + JOINT_IMPEDANCE`。

## 11. 分阶段实施计划

### Phase 0: 观测增强，不改行为

目标：

1. 增加 profile 和 SDK 状态的观测字段。
2. 不改变现有控制行为。

建议输出到日志和分析记录中的字段：

1. `active_control_profile`
2. `requested_control_profile`
3. `arm_cur_state`
4. `arm_err_code`
5. `m_FB_Joint_Cmd`
6. `m_OutFrameSerial`
7. `m_InFrameSerial`（如果结构中可直接访问）

验收标准：

1. 不影响现有 teleop。
2. 能清楚区分“上位 command 已变”与“机器人内部执行未变”。

### Phase 1: 硬件层 profile 状态机骨架

目标：

1. 新增 `HardwareControlProfile` 状态。
2. 新增 service 接口。
3. 切换逻辑先只支持 `POSITION_FOLLOW -> POSITION_FOLLOW` 占位，不改变真实模式。

验收标准：

1. service 可发起请求、返回完成状态。
2. 控制循环无明显性能退化。

### Phase 2: 实现 Position <-> Position 的无跳变切换框架

目标：

1. 验证 hold target 捕获、冻结写入、状态机推进、超时处理框架。
2. 在不引入阻抗的情况下证明“切换基础设施”正确。

验收标准：

1. 连续切换多次不报错。
2. 切换窗口内无明显位置跳变。

### Phase 3: 接入 Joint Impedance 模式

目标：

1. 实现 `POSITION_FOLLOW -> JOINT_IMPEDANCE`
2. 实现 `JOINT_IMPEDANCE -> POSITION_FOLLOW`
3. 接入 `OnSetTool_*`、`OnSetJointKD_*`、`OnSetImpType_*`

验收标准：

1. 双臂稳定进入 `ARM_STATE_TORQ`。
2. 切换后不出现明显突跳。
3. 切回位置模式稳定。

### Phase 4: motion_server 编排改造

目标：

1. `TELEOP` 对应 `JOINT_IMPEDANCE`
2. `SAFE_HOLD/MOTION/go_home` 对应 `POSITION_FOLLOW`
3. 处理 `legacy go_home` 特殊路径

验收标准：

1. `set_mode TELEOP` 后底层 profile 正确。
2. `go_home` 前后 profile 切换正确。
3. `go_home_return_mode=TELEOP` 时能恢复到阻抗模式。

### Phase 5: 实机整定和长时间验证

目标：

1. 调 `K/D` 参数。
2. 验证 teleop 延迟、抖动、稳定性。
3. 做异常场景测试。

验收标准：

1. teleop 滞后明显优于纯位置模式。
2. 无持续高频振荡。
3. 连续切换多轮不出现 fault。

## 12. 验证计划

### 12.0 每个 Phase 的固定测试节奏

从 `Phase 0` 开始，后续每个 phase 完成后都应至少执行以下 4 类测试，避免“代码已提交但功能是否真实生效不明确”。

#### A. 编译测试

执行：

```bash
colcon build --packages-select marvin_system --event-handlers console_direct+
```

通过标准：

1. `marvin_system` 编译通过。
2. 无新增编译错误或链接错误。

#### B. 接口/状态可见性测试

验证本 phase 新增的接口、状态、服务、CSV 字段、参数或 controller 切换路径是否真实可见。

通过标准：

1. 新增接口或字段确实出现。
2. 字段非空，且值域合理。
3. 如果是 ROS service/controller 逻辑，相关节点能正确响应请求。

#### C. 零运动测试

机械臂保持在 home 或静止状态，不做大动作，只验证：

1. 模式切换或请求是否成功。
2. 日志和观测值是否正确。
3. 是否出现自发运动、跳变、fault 或异常报警。

通过标准：

1. 无异常 self motion。
2. 无新增 SDK 错误码。
3. 无明显跳变。

#### D. 小幅运动测试

只做单臂、小幅、低速、短时动作，在确认安全后再扩展到双臂和完整 teleop。

通过标准：

1. 行为符合预期。
2. 无新的高频振荡。
3. 延迟、抖动和状态切换结果与该 phase 目标一致。

#### 每个 Phase 结束时必须保留的测试产物

建议至少保存以下内容：

1. 一条 build 记录。
2. 一份短测试记录，说明做了哪些动作、是否成功。
3. 一份 CSV、日志摘录或截图，证明该 phase 的新增观测或新增逻辑已真实生效。

### 12.1 基础功能验证

1. 上电后保持现有位置模式启动成功。
2. 手动请求 `POSITION_FOLLOW -> JOINT_IMPEDANCE`。
3. 手动请求 `JOINT_IMPEDANCE -> POSITION_FOLLOW`。
4. 验证切换前后 command 和 state 没有阶跃。

### 12.2 Teleop 验证

1. 进入 `TELEOP` 前切换到阻抗模式。
2. 使能 teleop。
3. 重做一段 tracker 录制并重新分析 `command -> state` 延迟。
4. 观察是否改善“明显滞后”和“抖动”。

### 12.3 Go Home 验证

1. 在 teleop 中发起 `go_home`。
2. 验证 go_home 前先回到位置模式。
3. 验证回 home 完成后按配置恢复到目标模式。

### 12.4 压力和异常验证

1. 连续切换 `TELEOP <-> SAFE_HOLD` 20 次以上。
2. 连续执行 `teleop -> go_home -> teleop` 10 次以上。
3. 在 profile switch 过程中人为中断 teleop 输入。
4. 验证超时、错误码和回退逻辑。

### 12.5 各 Phase 的推荐测试重点

#### Phase 0

目标是“只加观测，不改行为”，因此重点不是运动效果，而是观测链是否完整。

建议测试：

1. 编译通过。
2. 新增 state interface 在系统中可见。
3. 新 CSV 表头包含新增字段。
4. 新日志中 `command / sdk_command / state` 三层数据可同时读到。

Phase 0 预期重点字段：

1. `has_sdk_command_joint`
2. `sdk_command_joint*_deg`
3. `has_control_profile`
4. `active_control_profile_code/name`
5. `requested_control_profile_code/name`
6. `has_sdk_diag`
7. `sdk_cur_state_code/name`
8. `sdk_cmd_state_code`
9. `sdk_err_code`
10. `sdk_in_frame_serial`
11. `sdk_out_frame_serial`

#### Phase 1

目标是加入 profile 状态机骨架，但暂不真正切控制模式。

建议测试：

1. 新增 profile 请求入口存在。
2. `requested_profile` 和 `active_profile` 的状态推进逻辑正确。
3. 静止时反复请求多次不报错。
4. 不引入任何动作跳变。

#### Phase 2

目标是验证无跳变切换框架本身正确，但仍不接入真实阻抗。

建议测试：

1. 切换前后 `state_joint` 连续。
2. 切换窗口内 `sdk_err_code = 0`。
3. `sdk_cur_state` 不长期卡在 transition 状态。
4. 反复切换多次结果一致。

#### Phase 3

目标是接入真实 `JOINT_IMPEDANCE`。

建议测试顺序：

1. 空载或最简单末端配置。
2. 单臂。
3. 小幅关节动作。
4. 小范围 teleop。
5. 最后再做双臂。

#### Phase 4

目标是把 motion layer 编排和底层 profile 正确绑定。

建议测试：

1. `set_mode TELEOP` 时底层 profile 正确。
2. `set_mode SAFE_HOLD` 时底层 profile 正确。
3. `go_home` 前后 profile 切换顺序正确。
4. `go_home_return_mode=TELEOP` 时能恢复到阻抗模式。

#### Phase 5

目标是整定和长时间稳定性验证。

建议测试：

1. 连续 10 轮 `teleop -> go_home -> teleop`。
2. 连续 20 轮 `SAFE_HOLD <-> TELEOP`。
3. 15 到 30 分钟长时间 teleop。
4. 新旧 CSV 的延迟和抖动对比。

## 13. 风险列表

### 风险 1: 阻抗参数不合适导致振荡

对策：

1. 初期加 feature flag。
2. 从保守的 K/D 起步。
3. 先单臂、低幅度 teleop 验证。

### 风险 2: 工具动力学参数不完整导致扭矩模式表现异常

对策：

1. 在切到阻抗前明确设置 `OnSetTool_*`。
2. 未确认参数前，禁止默认启用该功能。

### 风险 3: SDK 并发访问导致偶发错误

对策：

1. 所有 SDK 调用只留在硬件主路径。
2. service callback 仅做请求投递。

### 风险 4: legacy go_home 语义与 profile 映射冲突

对策：

1. 单独改造 legacy go_home 路径。
2. 不把 `TELEOP` 和 `JOINT_IMPEDANCE` 做死绑定。

## 14. 推荐的落地顺序

建议严格按以下顺序实施：

1. 先做观测增强。
2. 再做硬件 profile 状态机骨架。
3. 再做真正的模式切换。
4. 最后再把 motion_server 和 go_home 路径接入。

不建议直接一步到位改成：

1. motion_server 直接调 SDK。
2. teleop 一启用就直接切扭矩模式。
3. 没有观测、没有 feature flag 就上实机。

## 15. 本计划对应的主要改动文件

1. `marvin_system/hardware/include/marvin_system/marvin.hpp`
2. `marvin_system/hardware/src/marvin.cpp`
3. `marvin_system/hardware/include/marvin_system/MarvinSDK.h`
4. `marvin_system/motion/src/motion_server.cpp`
5. `marvin_system/description/ros2_control/marvin_dual_system.ros2_control.xacro`
6. `marvin_system/bringup/launch/marvin_tracker_teleop.launch.py`
7. `marvin_system/bringup/config/marvin_tracker_teleop_controllers.yaml`
8. 新增 `marvin_system/srv/SetControlProfile.srv`

## 16. 下一步建议

下一步不建议直接改控制逻辑，建议先做 `Phase 0`：

1. 给分析链路补充 `m_FB_Joint_Cmd`、SDK 当前状态、active/requested profile。
2. 给硬件层加一个 feature flag 和 profile 状态字段，但先不启用阻抗切换。

这样可以先把“未来切换过程中到底发生了什么”看清楚，再进入真正的模式切换实现。
