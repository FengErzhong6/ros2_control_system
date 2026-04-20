# Marvin Runtime Stability Plan

## 1. 文档目的

本文档给出 `marvin_system` 的稳定启动改造方案，目标是解决以下长期问题：

1. `marvin_dual` 启动后偶发残留 `/controller_manager/*`、`/marvin_motion/*` 服务，导致下一次启动前即失败。
2. `go_home`、模式切换、控制器切换对底层 SDK 和 ROS graph 的依赖过强，任何一环抖动都会放大成整轮 bringup 失败。
3. 现有 supervisor 对 `marvin` 栈缺少确定的 ownership 边界，导致“谁启动的、谁负责清理、谁负责重试”不清晰。

本文档的定位是：

1. 作为后续 `marvin_system` 稳定启动改造的唯一计划基线。
2. 明确哪些方案已被验证为不可靠，后续不得再次作为默认推荐方案。
3. 给出可以落地的最小阶段方案、最终阶段方案、验收标准和回滚边界。


## 2. 背景与近期故障事实

### 2.1 2026-04-19 已观察到的故障现象

根据 2026-04-19 的多轮日志，已经确认以下事实：

1. `marvin_dual` 并不是总在“新启动后崩溃”，更常见的是“新启动前 ROS graph 已经脏了”。
2. 有多次失败发生在 prelaunch 阶段，supervisor 在真正启动 `marvin_dual` 之前，就检测到旧的 `/controller_manager/list_controllers`、`/marvin_motion/*` 服务仍然存在。
3. 17:45 附近的失败中，`htc_tracker` 曾先超时一次，但后续真正阻塞整轮启动的仍然是 `marvin_dual` prelaunch stale resources。
4. 同期还观察到 `cam_high` 出现 `Device or resource busy`、`device has been disconnected`，说明系统里不止 `marvin` 一个栈存在“进程退出了，但设备/graph 没有完全释放”的问题。

### 2.2 当前根因不是单个 bug

当前不稳定并不是某一个 `if` 条件写错，而是以下几类问题叠加：

1. 编排层没有 per-device session ownership。
2. 清理层没有 per-session 可验证的回收语义。
3. 运行层没有唯一 owner，允许外部手工 `ros2 launch`、旧 supervisor、异常退出的子进程共同污染环境。
4. 硬件层和 SDK 层的“进程退出”并不等于“设备和 graph 已恢复干净”。


## 3. 已验证的硬约束

本节只记录已经确认过的约束，后续设计不得违反。

### 3.1 `LaunchService` 不能作为同进程多实例并发方案

已确认 ROS Jazzy 的 `launch/launch_service.py` 中有以下约束：

1. `LaunchService.run_async()` 只允许在主线程执行。
2. `LaunchService` 不允许并发运行多个实例。
3. `LaunchService` 自身接管 `SIGINT`/`SIGTERM` 处理。

因此，以下思路被正式判定为**无效方案**：

1. 在同一个 supervisor 进程中，为 `marvin_dual` 单独再起一个新的 `LaunchService` 线程。
2. 通过多线程/多实例 `LaunchService` 为每个设备做独立 session 隔离。

### 3.2 `include_launch_description()` 只有“加入”，没有“摘除”

已确认共享 `LaunchService` 只有 include 能力，没有“移除某个已 include launch 图”的 API。

这意味着：

1. 旧图里的 `OnProcessExit`、`OnProcessStart`、future、event queue 一旦加入共享 `LaunchService`，很难只针对某个设备图彻底摘除。
2. 这与 `marvin_tracker_teleop.launch.py` 中大量 `OnProcessExit -> 启动下一个 spawner` 的链式 bringup 机制天然冲突。
3. 在共享图里做 `marvin_dual` 的强制 restart/cleanup，结构上就不可靠。

### 3.3 “猜旧进程再杀掉”只能作为过渡补丁

`ps` + `cmdline` + marker 的 scavenger 可以提升恢复概率，但它不是稳定 architecture。

原因：

1. 它依赖命令行字符串匹配，理论上总会有漏匹配。
2. 它无法证明“这个进程一定属于当前这次 supervisor 管理的那套 `marvin` 栈”。
3. 它本质上是在 ownership 缺失的前提下做事后补救。

因此，后续阶段不得把 scavenger 作为最终方案，只能保留为过渡期保护。


## 4. 设计目标与非目标

### 4.1 目标

本改造必须达到以下目标：

1. `marvin_dual` 启动、停止、重试必须具备单一 owner。
2. owner 必须同时拥有：
   - 启动权
   - 停止权
   - pid/pgid 或 cgroup 所有权
   - readiness 判定
   - cleanup 判定
3. 下一次启动前，必须能明确证明上一次 `marvin` session 已经被 owner 回收。
4. 出现未知外部污染时，系统应 fail closed 并给出明确诊断，而不是盲目启动或继续猜测。
5. 后续 `go_home`、`set_mode`、`controller switch` 仍然沿用 `marvin_motion` 统一入口，不重新把上层调用散回 `tracker_teleop_controller`。

### 4.2 非目标

以下内容不属于本次计划的主目标：

1. 不在本阶段重写 `tracker_teleop_controller` 的运动学或控制逻辑。
2. 不在本阶段替换 `marvin_motion` 的运动语义。
3. 不在本阶段把相机、HTC、MANUS 全部一次性重构完。
4. 不在本阶段尝试“共享 `LaunchService` 下的可靠 per-device detach”。


## 5. 明确否决的方案

### 5.1 共享 `LaunchManager` 继续管理 `marvin_dual`

正式否决，原因如下：

1. 共享图缺少 per-device detach。
2. `marvin_tracker_teleop.launch.py` 依赖 `OnProcessExit` 链式 bringup，旧图退出事件会污染新图。
3. `marvin_dual` 是最需要强制 restart/cleanup 的复杂栈，不适合继续放在共享图中。

### 5.2 同进程多 `LaunchService`

正式否决，原因如下：

1. 违反 `LaunchService` 主线程约束。
2. 违反 `LaunchService` 不能并发运行的约束。
3. 会再次制造“方案在纸面上看起来合理，实际运行时受框架限制”的错误。

### 5.3 把自动 scavenger 作为长期依赖

正式否决，原因如下：

1. 猜测不是 ownership。
2. 命令行匹配不具备完备性。
3. 该方法对手工启动、命令行变化、launch 参数变化非常脆弱。


## 6. 推荐架构

### 6.1 总体思路

推荐架构是：

1. 为 `marvin` 引入独立的 `runtime agent` 进程。
2. `marvin_dual` 的整个 bringup stack 只允许由该 agent 启动。
3. agent 成为 `marvin` 栈的唯一 owner。
4. supervisor 不再直接启动 `marvin_tracker_teleop.launch.py`，只与 agent 交互。
5. agent 的生命周期由 `systemd --user` 管理，使用 cgroup 提供最强的进程回收能力。

### 6.2 关键设计决策

本计划**不推荐**在 agent 内部再直接嵌入新的 `LaunchService` 作为第一阶段实现。

原因：

1. 第一阶段的目标是建立 ownership 边界，而不是引入新的框架复杂度。
2. 直接由 agent 启动 `ros2 launch marvin_system marvin_tracker_teleop.launch.py` 子进程，更简单、更容易验证、更容易回滚。
3. `ros2 launch` 子进程本身可以作为一个明确的 process group/cgroup 管理对象。

因此，第一阶段推荐路径是：

1. `systemd --user` 启动 `marvin_runtime_agent`
2. `marvin_runtime_agent` 再启动单个 `ros2 launch marvin_system ...` 子进程
3. 所有 cleanup 都按 agent 记录的 pid/pgid/cgroup 精确回收

### 6.3 为什么要有 agent，而不是继续让 supervisor 直接 `subprocess ros2 launch`

因为单纯的 `subprocess ros2 launch` 还缺下面几样：

1. 没有稳定的单一 owner 进程长期存在。
2. supervisor 崩了以后，谁来认领旧 session 不清楚。
3. 缺少稳定的状态文件和生命周期记录。
4. 缺少 “同一设备只允许一个活动 session” 的强约束。

agent 补上的就是这些边界。


## 7. 目标组件划分

### 7.1 `marvin_runtime_agent`

建议放在 `marvin_system/runtime/` 下，职责如下：

1. 读取启动参数和固定配置。
2. 创建唯一 `session_id`。
3. 启动 `ros2 launch marvin_system marvin_tracker_teleop.launch.py ...` 子进程。
4. 记录：
   - agent pid
   - child pid
   - child pgid
   - session_id
   - launch args
   - log path
   - 当前 phase
   - 最近错误
5. 接收 stop/status 请求。
6. 停止时回收整个子进程组。
7. 停止后验证 owned ROS resources 归零。

### 7.2 `marvin_runtime_ctl`

建议提供一个轻量控制 CLI，职责如下：

1. `start`
2. `stop`
3. `status`
4. `ensure-clean`
5. `logs`

注意：

1. `runtime_ctl` 只与 agent 或 `systemd` 交互。
2. 不允许它直接去猜旧进程并杀。

### 7.3 `systemd --user` unit

建议新增 user unit 模板，例如：

1. `marvin-runtime-agent@.service`

职责如下：

1. 管理 agent 的 cgroup
2. 确保 stop 后 cgroup 被完全回收
3. 崩溃时提供统一状态
4. 提供 `systemctl --user start/stop/status` 的标准管理入口


## 8. ownership 与生命周期语义

### 8.1 单一 owner 规则

必须建立以下规则：

1. 任意时刻，一个 `marvin` 实例只允许一个 agent owner。
2. 任何 `marvin` 相关进程都必须归属到该 agent 的记录 session。
3. 若发现 `marvin` graph 存在但当前没有 active owner 记录，则判定环境污染，fail closed。

### 8.2 禁止默认自动回收未知外部进程

最终稳态下不应该默认自动杀掉“未知来源”的 `marvin` 进程。

理由：

1. 稳定系统的关键不是更会猜，而是不再允许未知 owner 存在。
2. 如果未知 owner 存在，说明环境已经违反运行纪律，应当明确报错，而不是偷偷继续补救。

过渡期可以保留 admin-only 的 `force-clean`，但不得作为默认启动路径。

### 8.3 会话状态文件

建议在如下目录记录状态：

1. `/tmp/data_collection_runtime/marvin_dual/session.json`

内容至少包括：

1. `session_id`
2. `agent_pid`
3. `launch_pid`
4. `launch_pgid`
5. `started_at`
6. `phase`
7. `log_path`
8. `launch_arguments`
9. `last_error`
10. `last_exit_code`


## 9. readiness 与 cleanup 定义

### 9.1 readiness 判定必须收敛到 agent 内部

agent 必须自己判定 `marvin` 是否 ready，supervisor 只接受结果。

推荐 readiness 条件：

1. `ros2_control_node` 已存在且 `/controller_manager/list_controllers` 可访问。
2. `joint_state_broadcaster` active。
3. `tracker_teleop_controller` active。
4. 若启用 MoveIt go_home，则 `dual_arm_trajectory_controller` 至少 loaded。
5. `/marvin_motion/get_status` 返回 success 且不在 `FAULT`。

### 9.2 cleanup 判定必须可验证

stop 或异常退出后，agent 必须确认以下资源归零：

1. `/controller_manager/*`
2. `/marvin_motion/*`
3. `/marvin_dual/*` 中属于该栈的服务
4. `controller_manager`
5. `robot_state_publisher`
6. `move_group`
7. `marvin_motion_server`

如果这些资源未归零：

1. agent 不得报告 stop 成功。
2. supervisor 不得允许下一轮启动。


## 10. 与现有 `marvin_system` 的关系

### 10.1 保留的东西

以下内容保留：

1. `marvin_tracker_teleop.launch.py`
2. `marvin_motion` 统一入口
3. `go_home` 相关的 `motion_server` 能力
4. 现有 `tracker_teleop_controller`

### 10.2 需要改造的东西

需要改造的不是 `marvin` 控制语义，而是它的启动 ownership：

1. 现有 supervisor 直连 `ros2 launch marvin_system ...` 的路径要废弃。
2. `data_collection_orchestrator/adapters/marvin.py` 后续应改为 agent client。
3. 现有 graph cleanup/scavenger 逻辑只作为过渡期保护，最终应删除。


## 11. 实施阶段

### 阶段 A：冻结错误方向

目标：

1. 正式停止继续扩展 shared `LaunchManager` 管理 `marvin_dual` 的路径。
2. 把本文档作为后续唯一基线。

交付：

1. 本计划书。

### 阶段 B：最小可用 agent

目标：

1. 新增 `marvin_runtime_agent`。
2. agent 通过单个 `ros2 launch` 子进程拥有完整 `marvin` 栈。
3. 记录 session file。
4. 提供 `start/stop/status`。

建议新增文件：

1. `marvin_system/runtime/agent.py`
2. `marvin_system/runtime/state.py`
3. `marvin_system/runtime/ctl.py`
4. `marvin_system/runtime/__init__.py`

### 阶段 C：接入 `systemd --user`

目标：

1. 由 `systemd --user` 托管 agent。
2. 让 cgroup 成为最终进程回收边界。

建议新增文件：

1. `marvin_system/systemd/marvin-runtime-agent@.service`
2. `marvin_system/systemd/README.md`

### 阶段 D：supervisor 接入 agent

目标：

1. `data_collection_orchestrator` 不再直接拥有 `marvin` 栈。
2. `adapters/marvin.py` 改成 agent client。

外部联动文件：

1. `data_collection_stack/data_collection_orchestrator/data_collection_orchestrator/adapters/marvin.py`
2. `data_collection_stack/data_collection_orchestrator/data_collection_orchestrator/supervisor.py`

### 阶段 E：删除过渡期 scavenger

目标：

1. 当唯一 owner 路径稳定后，删除对未知外部进程的自动猜测清理。
2. 保留 admin-only `force-clean` 作为运维工具，而不是默认启动路径。


## 12. 测试与验收标准

### 12.1 启停稳定性

验收项：

1. 连续 20 轮 `StartSystem -> ShutdownSystem`，无 stale `/marvin_motion/*`。
2. 任一轮 stop 后，owned ROS services/nodes 必须归零。
3. 任一轮 next start 前不允许检测到旧 session graph 残留。

### 12.2 崩溃恢复

验收项：

1. 在 `marvin` 运行中 kill 掉 supervisor，重新启动 supervisor 后，agent state 仍可被识别。
2. kill 掉 agent 后，由 `systemd --user` 或运维流程可确定地收尾。
3. 下一轮启动不会依赖 `ps` 猜测旧进程。

### 12.3 功能连续性

验收项：

1. `marvin_motion/go_home`
2. `marvin_motion/set_mode`
3. `marvin_motion/get_status`
4. `tracker teleop`

以上功能在新 ownership 架构下仍然可用。

### 12.4 明确失败

验收项：

1. 如果存在未知外部 `marvin` 进程，系统必须显式报环境污染。
2. 不允许“猜不到但继续启动”。
3. 不允许“先启动再等更大问题出现”。


## 13. 风险与缓解

### 风险 1：`systemd --user` 在现场环境不可用

缓解：

1. 第一阶段 agent 允许直接由 supervisor 启动。
2. 但最终推荐仍然是 `systemd --user` 托管。

### 风险 2：旧的手工启动习惯继续污染环境

缓解：

1. 必须建立运行纪律：禁止手工 `ros2 launch marvin_system ...`。
2. 对未知 owner 直接 fail closed。

### 风险 3：相机/HTC 仍会单独污染环境

缓解：

1. 本计划首先解决 `marvin`。
2. 但 architecture 已经为 `camera_agent`、`htc_agent` 预留相同路径。


## 14. 最终建议

最终建议如下：

1. 立即停止把 `marvin_dual` 视为“可以继续放在共享 `LaunchManager` 下修补”的对象。
2. 第一阶段即引入 `marvin_runtime_agent`，并且**不要**在 agent 第一版内部再嵌入新的 `LaunchService` 复杂度。
3. 第一阶段坚持使用：
   - 单个 owner agent
   - 单个 `ros2 launch` 子进程
   - 明确 session file
   - 明确 pid/pgid
   - 明确 readiness
   - 明确 cleanup
4. 第二阶段再把 agent 放进 `systemd --user`，用 cgroup 解决 orphan process 问题。

这条路径的核心优点是：

1. 不违反 `LaunchService` 主线程约束。
2. 不依赖 shared `LaunchManager` 的 per-session detach 幻觉。
3. 不把“猜旧进程”当成长期架构。
4. 可以在现有 `marvin_system` 代码基础上渐进落地。
