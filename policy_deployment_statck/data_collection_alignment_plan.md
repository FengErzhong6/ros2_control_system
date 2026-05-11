# Policy Deployment Alignment Plan

目标：让 `policy_deployment_statck` 在启动链路、Marvin 进程模型、清理逻辑、重试语义和 UI 行为上尽量贴近 `data_collection_stack`，同时保留策略部署所需的 forward-control 语义。

## 1. 当前状态

- 入口已经具备 `deployment_app.launch.py`。
- 相机、右手、Marvin 的 recipe 已拆分。
- Marvin 已切到外部 `ros2 launch` 进程组模型。
- UI 已支持状态、连接、断开、执行、回家和处理后图像预览。

## 2. 仍需对齐的关键点

### 2.1 Marvin 进程模型

- 把 Marvin 从 supervisor 内部 managed launch 彻底迁出。
- 保持外部进程组、独立日志、独立 shutdown。
- 补齐 `precheck -> bringup -> wait_ready -> diagnose -> shutdown` 的 adapter 生命周期。

### 2.2 启动前清理

- 在 supervisor 启动时清理 stale runtime。
- 清理已知 orphan 进程组。
- Marvin 启动前再做一次 graph cleanup 和外部进程组 scavenging。

### 2.3 关闭时序

- Marvin shutdown 前先尝试进入 `SAFE_HOLD`。
- 先 `set_enabled false`，再退出进程组。
- 保证 disconnect、启动失败回滚、SIGINT 退出三条路径行为一致。

### 2.4 启动编排

- 逐步向 `devices + depends_on` 的 DAG 编排靠拢。
- 保留 camera -> wujihand -> marvin 的顺序，但让依赖表达显式化。
- 补统一的 startup policy，至少包括稳定窗口和重试策略。

### 2.5 WujiHand 启动校验

- 对齐 identity / controller / namespace 检查。
- 允许和 data_collection 一样自动确认 forward controller 可用。

### 2.6 UI 和预览

- 保持 processed preview。
- 确认 UI 退出不阻塞 shutdown。
- 继续保证连接状态、motion 状态和部署状态的同步刷新。

## 3. 推荐实施顺序

1. 抽出 `PolicyMarvinAdapter`，接管 Marvin 启动与关闭。
2. 加 runtime manifest 和 orphan cleanup。
3. 补 Marvin shutdown 的 `SAFE_HOLD` / `set_enabled false` 顺序。
4. 把 recipe 启动策略补成统一 startup policy。
5. 再对齐 WujiHand 的启动检查与自动激活逻辑。
6. 最后收束 UI 退出和整体回滚语义。

## 4. 验收标准

- 冷启动后，Marvin 不依赖 supervisor 内部 managed launch。
- 旧进程残留时，重新启动能自动清理。
- Marvin 启动失败时，回滚不会留下 controller_manager、motion_server、move_group 残留。
- disconnect 后，Marvin 进入安全态并退出。
- 连接 UI 可以稳定显示状态和 processed preview。

## 5. 风险

- 过激的 orphan cleanup 可能误伤正在调试的外部 Marvin 进程。
- 统一 startup policy 需要和现有 recipe 的超时参数一起调。
- 如果 WujiHand 和 Marvin 的命名空间进一步统一，profile、topic、recipe 也要同步改。

## 6. 当前进展

- 已完成：Marvin 使用外部 `ros2 launch` 进程组启动。
- 已完成：新增 policy deployment runtime manifest，记录 Marvin pid/pgid。
- 已完成：supervisor 启动时清理 stale runtime 和已知 Marvin orphan。
- 已完成：Marvin shutdown 前尝试 `set_enabled false` 和 `SAFE_HOLD`。
- 已完成：抽出 `PolicyMarvinAdapter` 生命周期，覆盖 precheck、bringup、wait_ready、shutdown_session。
- 已完成：将 `PolicyMarvinAdapter` 从 supervisor 内部类迁移到独立模块。
- 已完成：启动 recipe 从顺序列表演进到 `devices + depends_on`，并按依赖层启动。
- 已完成：补齐 per-device READY 稳定窗口，避免瞬时 READY 误判。
- 已完成：WujiHand 新增独立 adapter，覆盖 identity/controllers 预检、controller READY 检查和 forward controller 激活补偿。
- 已完成：右手 launch wrapper 显式暴露 identity_file、controllers_file、activate_forward_controller、gui、use_jsp_gui。
- 已完成：相机启动粒度拆成 `cam_high` / `cam_left_wrist` 两个独立 device。
- 待完成：如果后续还要更像 data_collection，可再给相机加专用 adapter 级早期重启逻辑。
