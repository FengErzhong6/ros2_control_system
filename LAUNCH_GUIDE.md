# ROS2 Launch 文件功能与使用说明

本文档按工作区顶层文件夹分组，整理当前源码树中的所有 `launch.py` 文件，说明它们的功能、主要参数和推荐使用方法。

## 使用约定

### 推荐启动方式

先在工作区根目录执行：

```bash
source install/setup.bash
```

然后用下面的格式启动：

```bash
ros2 launch <package_name> <launch_file.launch.py> [arg_name:=value ...]
```

### 何时需要重新构建

本仓库里有少数 launch 文件的源码名和当前 `install/` 目录暴露出来的名字不完全一致，尤其是 `manus_system`。如果你刚修改过源码，或者发现 `ros2 launch` 找不到源码里存在的 launch 文件，请先重新构建：

```bash
colcon build --packages-select camera_system htc_system manus_system marvin_system wujihand_system
source install/setup.bash
```

### 当前源码扫描结果

- `camera_system`: 2 个
- `htc_system`: 1 个
- `manus_system`: 3 个
- `marvin_system`: 6 个
- `wujihand_system`: 3 个
- 合计: 15 个

## `camera_system`

### `camera_system/bringup/launch/dual_wrist_orbbec.launch.py`

功能：
- 同时启动左右手腕两个 Orbbec 相机节点。
- 从 `cameras.yaml` 和 `orbbec_defaults.yaml` 动态读取左右相机配置。
- 可选启动两个 `image_tools/showimage` 窗口显示左右腕部图像。

启动节点：
- `camera_system/orbbec_camera_node` x 2
- `image_tools/showimage` x 2，可选

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `cameras_config` | `camera_system/bringup/config/cameras.yaml` | 相机清单配置 |
| `orbbec_defaults_config` | `camera_system/bringup/config/orbbec_defaults.yaml` | Orbbec 默认 profile 配置 |
| `use_showimage` | `false` | 是否弹出左右图像显示窗口 |

推荐命令：

```bash
ros2 launch camera_system dual_wrist_orbbec.launch.py
```

示例：

```bash
ros2 launch camera_system dual_wrist_orbbec.launch.py use_showimage:=true
ros2 launch camera_system dual_wrist_orbbec.launch.py cameras_config:=/tmp/cameras.yaml
```

使用说明：
- 要求 `cameras.yaml` 中存在 `cam_left_wrist` 和 `cam_right_wrist`。
- 这两个相机的 `driver` 必须是 `orbbec`。
- 对应 profile 必须启用彩色图像，否则 launch 会直接报错退出。

### `camera_system/bringup/launch/single_realsense.launch.py`

功能：
- 启动单个 RealSense 相机。
- 从配置文件里按 `camera_name` 查找目标相机并拼装 driver 参数。
- 可选启动 `showimage` 观察彩色图像。

启动节点：
- `realsense2_camera/realsense2_camera_node`
- `image_tools/showimage`，可选

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `camera_name` | `cam_high` | `cameras.yaml` 中的相机键名 |
| `cameras_config` | `camera_system/bringup/config/cameras.yaml` | 相机清单配置 |
| `realsense_defaults_config` | `camera_system/bringup/config/realsense_defaults.yaml` | RealSense 默认 profile 配置 |
| `use_showimage` | `true` | 是否显示彩色图像窗口 |

推荐命令：

```bash
ros2 launch camera_system single_realsense.launch.py
```

示例：

```bash
ros2 launch camera_system single_realsense.launch.py camera_name:=cam_high
ros2 launch camera_system single_realsense.launch.py camera_name:=cam_front use_showimage:=false
```

使用说明：
- `camera_name` 必须能在 `cameras.yaml` 的 `cameras` 字段中找到。
- 目标相机的 `driver` 必须是 `realsense`。
- 实际图像话题会落在 `/<namespace>/<namespace>/color/image_raw`。

## `htc_system`

### `htc_system/bringup/launch/tracker_publisher.launch.py`

功能：
- 启动 HTC Tracker 发布节点。
- 从 `trackers.yaml` 读取追踪器配置。

启动节点：
- `htc_system/tracker_publisher`

主要参数：
- 无 CLI 参数。

推荐命令：

```bash
ros2 launch htc_system tracker_publisher.launch.py
```

使用说明：
- 默认读取 `htc_system/bringup/config/trackers.yaml`。
- 适合作为 `marvin_tracker_teleop.launch.py` 的独立调试入口。

## `manus_system`

### `manus_system/bringup/launch/manus_raw_publisher.launch.py`

功能：
- 启动 Manus 原始数据发布节点。
- 可按用户目录加载校准文件。

启动节点：
- `manus_system/manus_raw_publisher_node`

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `user_name` | 空字符串 | 从 `bringup/calibrations/<user_name>` 读取该用户校准数据 |

推荐命令：

```bash
ros2 launch manus_system manus_raw_publisher.launch.py
```

示例：

```bash
ros2 launch manus_system manus_raw_publisher.launch.py user_name:=alice
```

### `manus_system/bringup/launch/manus_calibration.launch.py`

功能：
- 启动 Manus 校准节点。
- 同时启动校准工作流脚本，负责引导用户完成校准过程。
- 当工作流脚本退出时，整个 launch 会自动关闭。

启动节点：
- `manus_system/manus_calibration_node`
- `manus_system/manus_calibration_workflow.py`

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `user_name` | 空字符串 | 校准结果写入 `bringup/calibrations/<user_name>` |

推荐命令：

```bash
ros2 launch manus_system manus_calibration.launch.py user_name:=alice
```

使用说明：
- 这个 launch 主要用于正式标定，不建议无 `user_name` 直接运行。
- 工作流结束后会触发 `Shutdown`，属于预期行为。

### `manus_system/bringup/launch/manus_gripper.launch.py`

功能：
- 同时启动 Manus 原始数据发布和模拟夹爪节点。
- 常用于把手套输入转成仿真夹爪控制数据。

启动节点：
- `manus_system/manus_raw_publisher_node`
- `manus_system/manus_simulated_gripper_node`

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `user_name` | 空字符串 | 为原始数据发布节点加载对应用户校准目录 |

推荐命令：

```bash
ros2 launch manus_system manus_gripper.launch.py user_name:=alice
```

使用说明：
- 源码文件名是 `manus_gripper.launch.py`。
- 当前工作区的 `install/manus_system` 里仍然暴露的是旧名字 `manus_simulated_gripper.launch.py`，说明安装结果和源码未完全同步。
- 如果 `ros2 launch manus_system manus_gripper.launch.py` 找不到文件，请先重新执行 `colcon build` 再 `source install/setup.bash`。

## `marvin_system`

### `marvin_system/description/launch/view_marvin_dual.launch.py`

功能：
- 仅做模型可视化。
- 通过 xacro/URDF 生成双臂机器人描述。
- 可选启动 `joint_state_publisher_gui` 和 RViz。

启动节点：
- `joint_state_publisher_gui/joint_state_publisher_gui`，可选
- `robot_state_publisher/robot_state_publisher`
- `rviz2/rviz2`，可选

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `description_package` | `marvin_system` | 机器人描述所在包 |
| `description_file` | `description/urdf/marvin_dual.urdf` | URDF/XACRO 路径 |
| `gui` | `true` | 是否启动 RViz 和 joint GUI |
| `left_xyz` | `0 0.037 0.3618964` | 左臂底座平移 |
| `left_rpy` | `-1.5707963 0 0` | 左臂底座姿态 |
| `right_xyz` | `0 -0.037 0.3618964` | 右臂底座平移 |
| `right_rpy` | `1.5707963 0 0` | 右臂底座姿态 |

推荐命令：

```bash
ros2 launch marvin_system view_marvin_dual.launch.py
```

示例：

```bash
ros2 launch marvin_system view_marvin_dual.launch.py gui:=false
ros2 launch marvin_system view_marvin_dual.launch.py left_xyz:="0 0.05 0.36" right_xyz:="0 -0.05 0.36"
```

### `marvin_system/description/launch/display_marvin_dual.launch.py`

功能：
- 启动双臂模型显示和 RViz。
- 启动自定义 `joint_gui_publisher.py` 发布关节状态。
- 可通过命令行附加目标位姿 TF、方向向量和目标平面可视化标记。

启动节点：
- `robot_state_publisher/robot_state_publisher`
- `marvin_system/joint_gui_publisher.py`
- `marvin_system/vector_marker_publisher.py`，按需
- `tf2_ros/static_transform_publisher`，按需
- `rviz2/rviz2`

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `use_sim_time` | `false` | 是否使用 `/clock` |
| `left_xyz` | `0 0.037 0.3618964` | 左臂底座平移 |
| `left_rpy` | `-1.5707963 0 0` | 左臂底座姿态 |
| `right_xyz` | `0 -0.037 0.3618964` | 右臂底座平移 |
| `right_rpy` | `1.5707963 0 0` | 右臂底座姿态 |
| `use_gripper_L` | `true` | 左夹爪是否参与模型显示 |
| `use_gripper_R` | `true` | 右夹爪是否参与模型显示 |
| `model` | `marvin_system/description/urdf/marvin_dual.urdf` | 模型文件 |
| `target_pose_L` | 空字符串 | 左臂目标位姿，格式 `x y z qx qy qz qw` |
| `target_pose_R` | 空字符串 | 右臂目标位姿，格式 `x y z qx qy qz qw` |
| `target_vector_L` | 空字符串 | 左臂目标向量，支持 `;` 分隔多个 |
| `target_vector_R` | 空字符串 | 右臂目标向量，支持 `;` 分隔多个 |
| `target_plane_L` | 空字符串 | 左臂目标平面 |
| `target_plane_R` | 空字符串 | 右臂目标平面 |

推荐命令：

```bash
ros2 launch marvin_system display_marvin_dual.launch.py
```

示例：

```bash
ros2 launch marvin_system display_marvin_dual.launch.py target_pose_L:="0.2 0.0 0.1 0 0 0 1"
ros2 launch marvin_system display_marvin_dual.launch.py target_vector_R:="0 0 1"
```

使用说明：
- `target_pose_*` 会生成静态 TF：`Base_L/R -> target_L/R`。
- `target_vector_*` 和 `target_plane_*` 非空时才会启动 `vector_marker_publisher.py`。

### `marvin_system/bringup/launch/marvin_dual_joint_gui_control.launch.py`

功能：
- 启动双臂 `ros2_control`。
- 启动 `forward_position_controller`。
- 使用自定义 GUI 发布器和桥接节点，把滑块关节命令转成控制器输入。

启动节点：
- `controller_manager/ros2_control_node`
- `robot_state_publisher/robot_state_publisher`
- `rviz2/rviz2`，可选
- `marvin_system/joint_gui_publisher.py`，可选
- `marvin_system/gui_joint_state_to_forward_command`
- `controller_manager/spawner` 若干

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 是否启动 RViz |
| `use_mock_hardware` | `false` | 是否使用 mock hardware |
| `description_package` | `marvin_system` | 描述包 |
| `description_file` | `description/urdf/marvin_dual.urdf` | 模型文件 |
| `controllers_file` | 空字符串 | 手动指定控制器 YAML；为空时自动选 |
| `use_jsp_gui` | `true` | 是否启动 GUI 滑块 |
| `use_gripper_L` | `true` | 是否启用左夹爪 |
| `use_gripper_R` | `true` | 是否启用右夹爪 |
| `left_xyz` | `0 0.037 0.3618964` | 左臂底座平移 |
| `left_rpy` | `-1.5707963 0 0` | 左臂底座姿态 |
| `right_xyz` | `0 -0.037 0.3618964` | 右臂底座平移 |
| `right_rpy` | `1.5707963 0 0` | 右臂底座姿态 |

推荐命令：

```bash
ros2 launch marvin_system marvin_dual_joint_gui_control.launch.py
```

示例：

```bash
ros2 launch marvin_system marvin_dual_joint_gui_control.launch.py use_mock_hardware:=true
ros2 launch marvin_system marvin_dual_joint_gui_control.launch.py use_gripper_L:=false use_gripper_R:=false
```

使用说明：
- `controllers_file` 为空时，会根据是否启用夹爪在两个默认控制器 YAML 之间自动切换。
- `joint_state_broadcaster` 启动后才会继续拉起 `forward_position_controller` 和 GUI 桥接节点。

### `marvin_system/bringup/launch/marvin_ik_control.launch.py`

功能：
- 启动双臂 `ros2_control` 和 `ik_controller`。
- 自动注入运动学配置文件 `ccs_m6_40.MvKDCfg`。
- 可按需为左右夹爪动态生成独立 controller 参数文件。

启动节点：
- `controller_manager/ros2_control_node`
- `robot_state_publisher/robot_state_publisher`
- `rviz2/rviz2`，可选
- `controller_manager/spawner` 若干

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 是否启动 RViz |
| `description_package` | `marvin_system` | 描述包 |
| `description_file` | `description/urdf/marvin_dual.urdf` | 模型文件 |
| `use_gripper_L` | `false` | 是否启用左夹爪 controller |
| `use_gripper_R` | `false` | 是否启用右夹爪 controller |
| `left_xyz` | `0 0.037 0.3618964` | 左臂底座平移 |
| `left_rpy` | `-1.5707963 0 0` | 左臂底座姿态 |
| `right_xyz` | `0 -0.037 0.3618964` | 右臂底座平移 |
| `right_rpy` | `1.5707963 0 0` | 右臂底座姿态 |

推荐命令：

```bash
ros2 launch marvin_system marvin_ik_control.launch.py
```

示例：

```bash
ros2 launch marvin_system marvin_ik_control.launch.py use_gripper_L:=true use_gripper_R:=true
ros2 launch marvin_system marvin_ik_control.launch.py gui:=false
```

使用说明：
- 启动时会临时生成 IK 和夹爪控制器参数文件。
- 适合给上层位姿输入节点或 teleop 节点提供底层控制环境。

### `marvin_system/bringup/launch/marvin_joy_control.launch.py`

功能：
- 先包含 `marvin_ik_control.launch.py`。
- 再启动 `joy_node` 和 `joy_to_pose`，把手柄输入映射成末端位姿控制。

启动节点：
- `marvin_ik_control.launch.py` 中的全部节点
- `joy/joy_node`
- `marvin_system/joy_to_pose`

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 是否启动 RViz |
| `use_gripper_L` | `false` | 是否启用左夹爪 |
| `use_gripper_R` | `false` | 是否启用右夹爪 |
| `left_xyz` | `0 0.037 0.3618964` | 左臂底座平移 |
| `left_rpy` | `-1.5707963 0 0` | 左臂底座姿态 |
| `right_xyz` | `0 -0.037 0.3618964` | 右臂底座平移 |
| `right_rpy` | `1.5707963 0 0` | 右臂底座姿态 |
| `linear_speed` | `0.1` | 末端平移速度 |
| `angular_speed` | `0.5` | 模拟轴角速度 |
| `button_angular_speed` | `0.3` | 按键角速度 |
| `deadzone` | `0.15` | 手柄死区 |

推荐命令：

```bash
ros2 launch marvin_system marvin_joy_control.launch.py
```

示例：

```bash
ros2 launch marvin_system marvin_joy_control.launch.py linear_speed:=0.05 angular_speed:=0.3
ros2 launch marvin_system marvin_joy_control.launch.py use_gripper_L:=true
```

使用说明：
- 这是在 IK 控制基础上的上层 teleop 入口。
- `joy_node` 默认使用 `device_id=0`。

### `marvin_system/bringup/launch/marvin_tracker_teleop.launch.py`

功能：
- 启动 HTC Tracker 驱动和双臂 `ros2_control`。
- 拉起 `tracker_teleop_controller`，把 Tracker 位姿映射成双臂动作。
- 可选启用键盘门控逻辑：空格开始/暂停，`n` 回 home 进入下一次采样。

启动节点：
- `controller_manager/ros2_control_node`
- `robot_state_publisher/robot_state_publisher`
- `htc_system/tracker_publisher`
- `controller_manager/spawner` 若干
- `rviz2/rviz2`，可选
- 终端键盘门控线程，按需

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 是否启动 RViz |
| `use_mock_hardware` | `false` | 是否启用 mock hardware |
| `use_keyboard_gate` | `true` | 是否启用终端按键门控 |
| `description_package` | `marvin_system` | 描述包 |
| `description_file` | `description/urdf/marvin_dual.urdf` | 模型文件 |
| `use_gripper_L` | `false` | 是否启用左夹爪 |
| `use_gripper_R` | `false` | 是否启用右夹爪 |
| `left_xyz` | `0 0.037 0.3618964` | 左臂底座平移 |
| `left_rpy` | `-1.5707963 0 0` | 左臂底座姿态 |
| `right_xyz` | `0 -0.037 0.3618964` | 右臂底座平移 |
| `right_rpy` | `1.5707963 0 0` | 右臂底座姿态 |

推荐命令：

```bash
ros2 launch marvin_system marvin_tracker_teleop.launch.py
```

示例：

```bash
ros2 launch marvin_system marvin_tracker_teleop.launch.py use_mock_hardware:=true
ros2 launch marvin_system marvin_tracker_teleop.launch.py use_keyboard_gate:=false
```

使用说明：
- 依赖 `htc_system` 与 `marvin_system` 在同一工作区可用。
- 启用 `use_keyboard_gate:=true` 时，需要当前终端可读取按键输入。
- 空格键会调用 `/tracker_teleop_controller/set_armed` 和 `/set_enabled`，`n` 会调用 `/tracker_teleop_controller/go_home`。

## `wujihand_system`

### `wujihand_system/description/launch/view_wujihand_right.launch.py`

功能：
- 单纯查看右手模型。
- 加载 URDF/XACRO 后启动 TF 和 RViz。
- 可选启动 `joint_state_publisher_gui` 调关节。

启动节点：
- `joint_state_publisher_gui/joint_state_publisher_gui`，可选
- `robot_state_publisher/robot_state_publisher`
- `rviz2/rviz2`，可选

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `description_package` | `wujihand_system` | 描述包 |
| `description_file` | `urdf/wujihand-right.urdf` | 模型文件 |
| `gui` | `true` | 是否启动 RViz 和 joint GUI |

推荐命令：

```bash
ros2 launch wujihand_system view_wujihand_right.launch.py
```

### `wujihand_system/bringup/launch/wujihand_right_control.launch.py`

功能：
- 启动右手 `ros2_control`。
- 启动 `joint_state_broadcaster` 与 `forward_position_controller`。
- 可选拉起 `joint_state_publisher_gui`，再通过桥接脚本把 GUI 的 `JointState` 转成控制器命令。

启动节点/进程：
- `controller_manager/ros2_control_node`
- `robot_state_publisher/robot_state_publisher`
- `rviz2/rviz2`，可选
- `joint_state_publisher_gui/joint_state_publisher_gui`，可选
- `controller_manager/spawner` x 2
- `gui_joint_state_to_forward_command.py`，可选
- `ros2 control switch_controllers ...` 延时激活动作

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 是否启动 RViz |
| `description_package` | `wujihand_system` | 描述包 |
| `description_file` | `urdf/wujihand-right.urdf` | 模型文件 |
| `controllers_file` | `config/wujihand_controllers.yaml` | 控制器配置 |
| `use_jsp_gui` | `true` | 是否启动 GUI 滑块 |

推荐命令：

```bash
ros2 launch wujihand_system wujihand_right_control.launch.py
```

示例：

```bash
ros2 launch wujihand_system wujihand_right_control.launch.py use_jsp_gui:=false
ros2 launch wujihand_system wujihand_right_control.launch.py controllers_file:=config/custom_wujihand_controllers.yaml
```

使用说明：
- GUI 模式下，`joint_state_publisher_gui` 的输出会被重映射到 `gui_joint_states`，避免和真实反馈 `/joint_states` 冲突。
- `forward_position_controller` 先以 `inactive` 启动，1 秒后再通过 `ros2 control switch_controllers` 激活。

### `wujihand_system/bringup/launch/wujihand_right_custom_controller.launch.py`

功能：
- 启动右手 `ros2_control`。
- 额外加载策略参数文件并拉起 `rl_controller`。
- 适合测试自定义控制器或强化学习策略控制链路。

启动节点：
- `controller_manager/ros2_control_node`
- `robot_state_publisher/robot_state_publisher`
- `rviz2/rviz2`，可选
- `controller_manager/spawner` x 2

主要参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `gui` | `true` | 是否启动 RViz |
| `description_package` | `wujihand_system` | 描述包 |
| `description_file` | `urdf/wujihand-right.urdf` | 模型文件 |
| `controllers_file` | `config/wujihand_controllers.yaml` | 控制器配置 |
| `policy_file` | `config/rl_controller.yaml` | RL 策略参数 |

推荐命令：

```bash
ros2 launch wujihand_system wujihand_right_custom_controller.launch.py
```

示例：

```bash
ros2 launch wujihand_system wujihand_right_custom_controller.launch.py policy_file:=config/rl_controller.yaml
ros2 launch wujihand_system wujihand_right_custom_controller.launch.py gui:=false
```

使用说明：
- `policy_file` 会和 `controllers_file` 一起传给 `ros2_control_node`。
- `rl_controller` 通过 controller manager 的 `spawner` 拉起，控制器名固定为 `rl_controller`。

## 差异与注意事项

### `manus_system` 源码名和安装名不一致

当前源码目录里有：

- `manus_raw_publisher.launch.py`
- `manus_calibration.launch.py`
- `manus_gripper.launch.py`

但当前 `install/manus_system/share/manus_system/launch` 里看到的是：

- `manus_raw_publisher.launch.py`
- `manus_calibration.launch.py`
- `manus_calibration_workflow.launch.py`
- `manus_simulated_gripper.launch.py`

这通常表示当前 `install/` 来自较旧的一次构建。对 `manus_system` 使用 `ros2 launch` 前，建议优先重新构建并重新 `source`。

### 适合纯显示的 launch

下面这些 launch 更偏向模型显示和调参，不负责完整控制链：

- `view_marvin_dual.launch.py`
- `display_marvin_dual.launch.py`
- `view_wujihand_right.launch.py`

### 适合控制与联调的 launch

下面这些 launch 会拉起 controller、硬件接口或 teleop 链路：

- `marvin_dual_joint_gui_control.launch.py`
- `marvin_ik_control.launch.py`
- `marvin_joy_control.launch.py`
- `marvin_tracker_teleop.launch.py`
- `wujihand_right_control.launch.py`
- `wujihand_right_custom_controller.launch.py`
- `dual_wrist_orbbec.launch.py`
- `single_realsense.launch.py`
- `tracker_publisher.launch.py`
- `manus_raw_publisher.launch.py`
- `manus_calibration.launch.py`
- `manus_gripper.launch.py`
