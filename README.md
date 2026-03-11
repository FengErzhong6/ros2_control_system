# ROS2 控制系统 (ros2_control_system)

本仓库基于 **ROS 2** 与 **ros2_control** 框架，为 **舞肌灵巧手 (WujiHand)** 和 **Marvin 双臂机器人** 提供完整的硬件接口与启动配置，便于在仿真或实机上统一使用标准控制器进行关节控制与状态发布。

---

## 一、ros2_control 简介

**ros2_control** 是 ROS 2 中用于机器人底层控制的统一框架，主要作用包括：

- **硬件抽象**：通过实现 `SystemInterface` 等接口，将不同厂商、不同协议的机器人硬件（电机、机械臂、灵巧手等）抽象成统一的「关节命令 / 关节状态」接口。
- **控制器管理**：由 `controller_manager` 加载、切换多种控制器（如位置控制、速度控制、关节状态广播等），同一套上层逻辑可复用于不同机器人。
- **配置驱动**：机器人的关节限位、控制器参数等通过 URDF/XACRO 与 YAML 描述，便于维护和复用。

因此，为某款机器人编写一次 ros2_control 的 **硬件接口 (Hardware Interface)** 后，即可使用 ROS 2 生态中的 `forward_command_controller`、`joint_state_broadcaster`、MoveIt2 等标准组件进行控制与规划。本仓库中的 **ros2_control_wujihand** 和 **ros2_control_marvin** 即分别为舞肌手和 Marvin 双臂提供的这类实现。

---

## 二、仓库中的两个库

### 1. ros2_control_wujihand（舞肌灵巧手）

**ros2_control_wujihand** 为 **舞肌灵巧手 (WujiHand)** 提供 ros2_control 硬件接口与启动、配置，支持单右手模型（可扩展为左手或双手）。

| 项目 | 说明 |
|------|------|
| **机器人** | 舞肌灵巧手，5 指 × 每指 4 关节，共 **20 个关节** |
| **硬件插件** | `WujiHandHardware`（依赖 **wujihandcpp** 与设备通信） |
| **控制接口** | 位置命令 + 位置/速度状态；可选低通滤波参数 `low_pass_cutoff_frequency` |
| **控制器** | `joint_state_broadcaster`、`forward_position_controller`；另提供自定义 **RL 控制器** (`RLController`) 用于强化学习等应用 |
| **描述与可视化** | URDF 在包内；RViz 配置使用 `asset_description` 中的 `wuji/rviz/wujihand_right.rviz` |

**主要目录与文件：**

- `description/`：URDF、ros2_control 的 xacro（关节限位与硬件插件声明）
- `hardware/`：`WujiHandHardware` 的 C++ 实现（读/写关节、生命周期管理）
- `controller/`：自定义 RL 相关控制器（如 `RLController`、策略接口）
- `bringup/launch/`：启动文件，如 `wujihand_right_control.launch.py`、`wujihand_right_custom_controller.launch.py`
- `bringup/config/wujihand_controllers.yaml`：控制器管理器与各控制器参数

**典型用法：**

- 启动右手控制（含 RViz、joint_state_publisher_gui、GUI 到 forward 的桥接）：
  ```bash
  ros2 launch ros2_control_wujihand wujihand_right_control.launch.py
  ```
- 仅查看模型：
  ```bash
  ros2 launch ros2_control_wujihand view_wujihand_right.launch.py
  ```

---

### 2. ros2_control_marvin（Marvin 双臂机器人）

**ros2_control_marvin** 为 **Marvin 双臂机器人** 提供 ros2_control 硬件接口与启动、配置，当前为 **双臂一体** 描述：左臂 + 右臂共 14 个关节，通过同一硬件插件与 Marvin SDK 通信。

| 项目 | 说明 |
|------|------|
| **机器人** | Marvin 双臂，每臂 7 关节，共 **14 个关节**（`Joint1_L`～`Joint7_L`、`Joint1_R`～`Joint7_R`） |
| **硬件插件** | `MarvinHardware`（通过 **Marvin SDK** 经网络与机械臂控制器通信，需配置 IP） |
| **配置参数** | `ip`（默认如 `192.168.1.190`）、`joint_vel_ratio`、`joint_acc_ratio`、超时等，在 ros2_control xacro 中配置 |
| **控制接口** | 位置命令 + 位置/速度状态 |
| **控制器** | `joint_state_broadcaster`、`forward_position_controller` |
| **描述与可视化** | 复合 URDF 在包内（`marvin_dual.urdf` 等），引用 `asset_description` 中的左右臂模型与 mesh；RViz 使用包内 `description/rviz/marvin_dual.rviz` |

**主要目录与文件：**

- `description/`：双臂复合 URDF/xacro、ros2_control 的 xacro（`marvin_dual_system.ros2_control.xacro`）、RViz 配置与 view/display 启动文件
- `hardware/`：`MarvinHardware` 的 C++ 实现（解析 IP、调用 Marvin SDK、读/写关节、超时与重连逻辑）
- `bringup/launch/`：如 `marvin_dual_joint_gui_control.launch.py`（带 GUI 滑条控制）
- `bringup/config/marvin_dual_controllers.yaml`：控制器管理器与 forward 控制器关节列表
- `bringup/src/`：如 `gui_joint_state_to_forward_command` 等 GUI 到 forward 命令的桥接工具

**典型用法：**

- 启动双臂控制（含 RViz、joint_state_publisher_gui、滑条控制）：
  ```bash
  ros2 launch ros2_control_marvin marvin_dual_joint_gui_control.launch.py
  ```
- 仅显示模型：
  ```bash
  ros2 launch ros2_control_marvin view_marvin_dual.launch.py
  ```
- 使用前请根据实际设备修改 ros2_control 中的 `ip` 及超时等参数（在 `description/ros2_control/marvin_dual_system.ros2_control.xacro` 或对应 xacro 中）。

---

## 三、依赖与构建

- **ROS 2**（建议 Humble 或更高）
- **ros2_control** 相关包：`controller_manager`、`hardware_interface`、`forward_command_controller`、`joint_state_broadcaster` 等
- **舞肌手**：`wujihandcpp`（灵巧手底层通信库）
- **Marvin**：Marvin SDK（头文件/库在包内或通过依赖引入，用于网络控制）
- **asset_description**：本 workspace 内的描述包，提供 WujiHand 与 Marvin 的 URDF/mesh 等资源

在 workspace 根目录下：

```bash
colcon build --packages-select ros2_control_wujihand ros2_control_marvin asset_description
source install/setup.bash
```

再按上文示例启动对应 launch 文件即可。

---

## 四、仓库结构概览

```
ros2_control_system/
├── README.md
├── asset_description/          # 舞肌手、Marvin 的 URDF/mesh/RViz 等描述资源
├── ros2_control_wujihand/      # 舞肌灵巧手 ros2_control 接口与启动
└── ros2_control_marvin/       # Marvin 双臂 ros2_control 接口与启动
```

两个库均包含：**硬件接口实现**、**ros2_control 的 xacro 描述**、**控制器配置** 以及 **launch 与可选 GUI 桥接**，便于在仿真或实机上统一使用 ros2_control 进行控制与开发。
