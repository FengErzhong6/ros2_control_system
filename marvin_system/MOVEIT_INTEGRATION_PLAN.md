# Marvin MoveIt Integration Plan

## 目标

将 Marvin 双臂系统升级为以 MoveIt 为核心的安全运动架构，用于：

- 以碰撞检测和规划场景为约束执行 `go_home`
- 为后续双臂协作预留统一的规划与执行入口
- 保持 `marvin_system` 作为 Marvin 相关内容的唯一主包，避免仓库中出现多个并列的 Marvin 包

本计划不将 MoveIt 逻辑混入实时硬件与 ros2_control controller，而是在 `marvin_system` 包内增加独立的 `motion/` 分层。

## 设计原则

- 单包归口：所有 Marvin 相关内容继续保留在 `marvin_system/`
- 实时与非实时分层：
  - `hardware/` 只负责 SDK 与 ros2_control 硬件接口
  - `controller/` 只负责 ros2_control controller
  - `motion/` 负责 MoveIt 规划、场景、轨迹执行、命名位姿
- 运行期禁止危险自动回零：移除硬件激活阶段的分步回零
- 数据采集侧不承载规划逻辑，只调用 Marvin 运动接口

## 目标目录结构

```text
marvin_system/
├── description/
│   ├── urdf/
│   ├── rviz/
│   └── srdf/
├── hardware/
│   ├── include/marvin_system/
│   └── src/
├── controller/
│   ├── include/marvin_system/
│   └── src/
├── motion/
│   ├── include/marvin_system/motion/
│   ├── src/
│   ├── config/
│   └── README.md
├── bringup/
│   ├── config/
│   └── launch/
└── MOVEIT_INTEGRATION_PLAN.md
```

## 组件职责

### 1. hardware

- 保持现有 `MarvinHardware`
- 删除激活时自动执行的危险分步 home
- 继续提供底层关节位置控制与 workspace z-floor 兜底保护

### 2. controller

- 保留现有 tracker teleop / IK controller
- 新增或启用 `joint_trajectory_controller`，作为 MoveIt 的执行目标
- `tracker_teleop_controller/go_home` 不再作为安全回 home 的主入口

### 3. motion

- 新增 MoveIt 运动层，统一对外提供安全动作入口
- 第一阶段目标仅实现 `go_home`
- 后续扩展为：
  - 命名位姿
  - 单臂/双臂规划
  - 双臂协作预动作
  - 规划场景管理

### 4. description

- 继续使用现有 `marvin_dual.urdf`
- 新增 `srdf/` 保存 MoveIt semantic 配置
- 工作站环境不硬编码进 URDF，通过 planning scene 注入

### 5. bringup

- 新增 MoveIt 相关 launch
- 新增 trajectory controller 配置
- 明确区分：
  - teleop/debug launch
  - moveit/safe-operation launch

## 配置归属

### 保留在 `description/`

- 机器人结构
- 安装位姿
- 本体 collision geometry
- SRDF semantic 配置

### 放在 `motion/config/`

- `home_poses.yaml`
- `cell_scene.yaml`
- MoveIt kinematics / planning pipeline / controller mapping

### 放在 `bringup/config/`

- ros2_control controllers yaml
- trajectory controller 配置

## 实施步骤

### Phase 1: 结构与安全兜底

- 在 `marvin_system` 内创建 `motion/` 目录骨架
- 去除硬件激活阶段自动 home
- 新增 `marvin_dual_trajectory_controllers.yaml`
- 新增 `marvin_moveit.launch.py` 与 `marvin_safe_operation.launch.py` 骨架
- 在 `data_collection` 中预留到新 motion 接口的调用路径

### Phase 2: MoveIt 配置接入

- 新增 `description/srdf/marvin_dual.srdf`
- 新增 MoveIt 配置文件：
  - `kinematics.yaml`
  - `joint_limits.yaml`
  - `planning_pipelines.yaml`
  - `ompl_planning.yaml`
  - `moveit_controllers.yaml`
- 配置双臂 planning groups、自碰撞矩阵、执行控制器映射

### Phase 3: 安全 go_home

- 在 `motion/` 中实现 go-home 服务器
- 通过 MoveIt：
  - 加载机器人模型
  - 加载桌面等 planning scene
  - 规划到命名 home 位姿
  - 执行经碰撞检查的轨迹
- `data_collection` 的 `marvin.home()` 切换到该接口

### Phase 4: 双臂协作扩展

- 增加双臂联合 planning group
- 增加协作命名位姿与中间位姿
- 增加场景对象和 Allowed Collision Matrix 精调

## 第一阶段验收标准

- 仓库内出现清晰的 `motion/` 分层
- 启动阶段不再自动执行危险分步 home
- 轨迹执行控制器配置就位
- 新的 MoveIt / safe-operation launch 骨架存在
- `data_collection` 不再硬编码依赖 `tracker_teleop_controller/go_home`

## 第二阶段验收标准

- MoveIt 能正确加载 Marvin 双臂模型与语义模型
- 规划场景中可注入桌面等环境
- 可通过 trajectory controller 正常执行规划轨迹

## 最终验收标准

- `go_home` 通过 MoveIt 完成规划与执行
- 双臂与环境碰撞检查在规划时生效
- `data_collection` 的 GoHome 调用统一走 `motion` 层
- 危险的分步回零路径不再作为默认安全路径存在

## 本轮修改范围

本轮优先落实 Phase 1：

- 目录分层
- 启动危险路径移除
- trajectory controller 配置
- motion 层骨架
- bringup 骨架
- data_collection 接口改线到新入口占位

MoveIt 的完整语义配置和实际规划执行将在后续 Phase 2 / 3 中继续完成。
