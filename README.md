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

### 2. ros2_control_marvin（Marvin 双臂机器人 + OmniPicker 夹爪）

**ros2_control_marvin** 为 **Marvin 双臂机器人** 提供 ros2_control 硬件接口与启动、配置，当前为 **双臂一体** 描述：左臂 + 右臂共 14 个关节，通过同一硬件插件与 Marvin SDK 通信。可选集成 **OmniPicker 夹爪**（每臂最多 1 个，共最多 2 个）。

| 项目 | 说明 |
|------|------|
| **机器人** | Marvin 双臂，每臂 7 关节，共 **14 个关节**（`Joint1_L`～`Joint7_L`、`Joint1_R`～`Joint7_R`） |
| **夹爪（可选）** | OmniPicker 夹爪，通过 Marvin 末端通信模组（ChData）经 CAN-FD 控制，输入 0～1 开合百分比 |
| **硬件插件** | `MarvinHardware`（通过 **Marvin SDK** 经网络与机械臂控制器通信，需配置 IP；夹爪通过 **OmniPicker SDK** 经 ChData 通道独立通信） |
| **配置参数** | `ip`（默认如 `192.168.1.190`）、`joint_vel_ratio`、`joint_acc_ratio`、超时等，在 ros2_control xacro 中配置 |
| **控制接口** | 臂关节：位置命令 + 位置/速度状态；夹爪关节：位置命令（0～1）+ 位置状态 |
| **控制器** | `joint_state_broadcaster`、`forward_position_controller`；另提供自定义 **IK 控制器** (`IKController`) 用于末端位姿指令的逆运动学跟随 |
| **描述与可视化** | 复合 URDF 在包内（`marvin_dual.urdf` 等），引用 `asset_description` 中的左右臂模型与 mesh；RViz 使用包内 `description/rviz/marvin_dual.rviz` |

**主要目录与文件：**

- `description/`：双臂复合 URDF/xacro、ros2_control 的 xacro（`marvin_dual_system.ros2_control.xacro`）、RViz 配置与 view/display 启动文件
- `hardware/`：`MarvinHardware` 的 C++ 实现（解析 IP、调用 Marvin SDK、读/写关节、超时与重连逻辑），以及 OmniPicker SDK 头文件
- `bringup/launch/`：如 `marvin_dual_joint_gui_control.launch.py`（带 GUI 滑条控制，支持可选夹爪）
- `bringup/config/`：`marvin_dual_controllers.yaml`（纯臂关节）和 `marvin_dual_gripper_controllers.yaml`（含夹爪）
- `bringup/src/`：如 `gui_joint_state_to_forward_command` 等 GUI 到 forward 命令的桥接工具（关节列表可通过 `joint_names` 参数配置）
- `controller/`：自定义 IK 控制器（`IKController`，基于 Marvin 运动学 SDK 的逆运动学求解）
- `third_party/`：Marvin SDK (`marvinSDK/`)、运动学 SDK (`marvinKine/`)、运动学配置 (`marvinCfg/`) 与 OmniPicker SDK (`omnipickerSDK/`)

**OmniPicker 夹爪集成原理：**

夹爪通信路径与臂关节独立——臂关节走 `OnClearSet → OnSetJointCmdPos → OnSetSend` 通道，夹爪走 `OnSetChData` 末端通信模组通道（CAN-FD），二者互不干扰。在 ros2_control xacro 中，夹爪作为额外关节声明（`type=omnipicker`），`MarvinHardware` 的 `on_init` 自动识别并在生命周期各阶段管理连接与控制。

**典型用法：**

- 启动双臂控制（不含夹爪，和原来完全一致）：
  ```bash
  ros2 launch ros2_control_marvin marvin_dual_joint_gui_control.launch.py
  ```
- 启动双臂 + 双夹爪控制（GUI 滑条可控制 0～1 夹爪开合）：
  ```bash
  ros2 launch ros2_control_marvin marvin_dual_joint_gui_control.launch.py \
      use_gripper_L:=true use_gripper_R:=true
  ```
- 仅显示模型：
  ```bash
  ros2 launch ros2_control_marvin view_marvin_dual.launch.py
  ```
- 启动双臂 IK 控制（通过发布末端位姿，控制器自动进行逆运动学求解并驱动关节）：
  ```bash
  ros2 launch ros2_control_marvin marvin_ik_control.launch.py
  ```
- 使用前请根据实际设备修改 ros2_control 中的 `ip` 及超时等参数（在 `description/ros2_control/marvin_dual_system.ros2_control.xacro` 或对应 xacro 中）。

**IK 控制器详细说明：**

`IKController` 是一个基于 Marvin 运动学 SDK 的自定义 ros2_control 控制器，接收末端位姿（`geometry_msgs/msg/PoseStamped`）并实时进行逆运动学求解，将结果关节角度下发给硬件接口。

*订阅的 Topic：*

| Topic | 类型 | 说明 |
|-------|------|------|
| `~/target_pose_left` | `geometry_msgs/msg/PoseStamped` | 左臂末端目标位姿 |
| `~/target_pose_right` | `geometry_msgs/msg/PoseStamped` | 右臂末端目标位姿 |

*发布的 Topic（IK 状态反馈）：*

| Topic | 类型 | 说明 |
|-------|------|------|
| `~/ik_status_left` | `std_msgs/msg/String` | 左臂 IK 求解状态（仅状态变化时发布） |
| `~/ik_status_right` | `std_msgs/msg/String` | 右臂 IK 求解状态（仅状态变化时发布） |

状态值包括：`SUCCESS`、`INVALID_QUATERNION`、`OUT_OF_RANGE`、`JOINT_LIMIT_EXCEEDED`、`SINGULARITY`、`SDK_ERROR`、`NO_TARGET`。

*发送末端位姿示例：*

```bash
# 左臂示例（单位：米，四元数表示姿态）
ros2 topic pub --once /ik_controller/target_pose_left geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: 0.1, z: 0.4}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"

# 右臂示例
ros2 topic pub --once /ik_controller/target_pose_right geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.3, y: -0.1, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

*查看 IK 状态反馈：*

```bash
ros2 topic echo /ik_controller/ik_status_left
ros2 topic echo /ik_controller/ik_status_right
```

*注意事项：*

- **坐标系**：`frame_id` 目前仅作标注，SDK 内部以各臂基座为参考坐标系。位置的 x/y/z 单位为**米**（控制器内部自动转换为 SDK 所需的毫米）。
- **姿态**：使用标准四元数 (x, y, z, w) 表示末端朝向。控制器会自动检查四元数的有效性（分量有限、模长为 1）。`w=1.0, x=y=z=0.0` 表示无旋转（末端保持初始朝向）。
- **关节限位**：如果 IK 求解结果超出关节限位，控制器会拒绝该指令并保持当前位置，同时在 `~/ik_status_*` 话题上发布 `JOINT_LIMIT_EXCEEDED`。
- **实时安全**：控制器在 `update()` 循环中避免了堆内存分配，使用枚举和状态变化检测来最小化开销。
- **单次求解**：每个新的目标位姿仅触发一次 IK 求解，避免重复计算。

**夹爪相关 xacro 参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_gripper_L` | `false` | 启用左臂 OmniPicker 夹爪 |
| `use_gripper_R` | `false` | 启用右臂 OmniPicker 夹爪 |
| `gripper_L_arm` | `A` | 左夹爪连接的 Marvin 臂侧（A/B） |
| `gripper_R_arm` | `B` | 右夹爪连接的 Marvin 臂侧（A/B） |
| `gripper_L_can_id` | `1` | 左夹爪 CAN 节点 ID |
| `gripper_R_can_id` | `2` | 右夹爪 CAN 节点 ID |

---

## 三、依赖与构建

- **ROS 2**（建议 Humble 或更高）
- **ros2_control** 相关包：`controller_manager`、`hardware_interface`、`controller_interface`、`forward_command_controller`、`joint_state_broadcaster` 等
- **舞肌手**：`wujihandcpp`（灵巧手底层通信库）
- **Marvin**：Marvin SDK（头文件/库在包内，用于网络控制）；运动学 SDK（`libKine.so`，用于 IK 控制器的逆运动学求解）
- **OmniPicker**（可选）：OmniPicker SDK（`libomnipicker.so`，用于夹爪 CAN-FD 通信）
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
