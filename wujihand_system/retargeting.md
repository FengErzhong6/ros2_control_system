# Wuji 手部 Retargeting 接入计划

## 1. 背景

当前工作空间已经具备两条基础链路：

- `manus_system` 负责从 Manus SDK 采集原始手部骨架数据，并发布 `/manus_raw_publisher_node/gloves_raw`
- `wujihand_system` 已经具备左右手硬件、控制器和双手 bringup 能力

这次要补齐的是 Manus 手势到 Wuji 灵巧手 20 维关节命令之间的 teleop 与 retargeting 链路。

在分析参考工作空间 `wuji-teleop-ros2-private/src/input_devices/manus_input` 及其下游消费方式后，确认有两个关键事实：

1. `manus_input` 的职责不是做 retarget，而是把 Manus 原始数据转换为统一的 `/hand_input`
2. 真正的 `Retargeter` 调用发生在输出设备侧，消费 `/hand_input`，再分别生成左右手关节命令

因此，本阶段的设计不应再写成“直接从 Manus raw 进入 retarget”。更合理的边界是：

- 原始输入层：`/manus_raw_publisher_node/gloves_raw`
- 标准手部接口层：`/hand_input`
- 输出控制层：左右手各自的控制命令

## 2. 参考方案提炼

### 2.1 参考工作空间的真实职责划分

参考工作空间的 Manus 链路可以概括为：

1. `manus_ros2` 从 SDK 取数据，发布 `/manus_glove_0`、`/manus_glove_1`
2. `manus_input_py` 订阅 Manus 原始话题，转换为 `/hand_input`
3. 下游 `wujihand_controller` 订阅 `/hand_input`
4. `wujihand_controller` 内部调用左右手 `Retargeter`
5. 最终分别控制左右灵巧手

其中真正稳定的工作空间边界不是 `ManusGlove`，而是 `/hand_input`。

### 2.2 参考方案对我们的启示

对当前工作空间，参考方案的可迁移结论如下：

1. `manus_system` 继续只负责原始采集，不承担 retarget 逻辑。
2. `wujihand_system` 内部应显式保留“输入适配层”和“输出 retarget 层”的职责边界。
3. 标准接口应当是 `/hand_input`，格式使用 `Float32MultiArray`：
   - 单手：63 个值，即 `(21, 3)`
   - 双手：126 个值，即右手在前、左手在后
4. 左右手 Manus 重定向配置应分开管理，至少保留 `mediapipe_rotation.z` 的左右手差异。

## 3. 本次方案边界

本阶段决定采用以下包边界：

- `wuji-retargeting` 作为 `wujihand_system/third_party` 中的 vendored 第三方代码
- `manus_system` 继续只负责原始 Manus 数据采集
- `wujihand_system` 负责：
  - Manus 原始输入适配到 `/hand_input`
  - `/hand_input` 到 Wuji 手命令的 retarget 输出

即使最终实现为了性能采用单进程，也必须保留上述逻辑边界，而不是重新把输入转换和 retarget 职责混在概念上。

## 4. 改造目标

本次改造目标如下：

1. 将 `wuji-retargeting` 集成到 `wujihand_system`
2. 将 `manus_system` 的 `raw_nodes` 稳定转换为标准 `/hand_input`
3. 使用 `/hand_input` 作为 `wujihand_system` 内部统一手部输入接口
4. 基于 `/hand_input` 分别完成左右手 retarget，并输出到现有控制器入口
5. 保留原始 Manus 话题和标准 `/hand_input` 接口，便于录包、调试、回放和后续扩展

## 5. 非目标

以下内容不作为本阶段目标：

- 修改 `manus_system` 消息定义，使其直接输出 `/hand_input`
- 将 Python retargeting 逻辑嵌入 `ros2_control` controller 的 `update()` 实时循环
- 在本阶段引入新的抓取策略、协同控制或学习策略
- 在本阶段重构 `wujihand_system` 的现有硬件插件结构

## 6. 总体设计

### 6.1 标准话题边界

本阶段建议采用以下话题设计：

- 原始输入层：
  - `/manus_raw_publisher_node/gloves_raw`
- 标准手部接口层：
  - `/hand_input`
- 最终命令层：
  - `/left/forward_position_controller/commands`
  - `/right/forward_position_controller/commands`

其中 `/hand_input` 的格式明确约定为：

- 类型：`std_msgs/Float32MultiArray`
- 单手：63 值，即 `(21, 3)`
- 双手：126 值，即右手在前、左手在后

### 6.2 逻辑分层

应当将链路划分为两个逻辑层：

1. 输入适配层
   - 输入：`/manus_raw_publisher_node/gloves_raw`
   - 输出：`/hand_input`
   - 责任：从 `raw_nodes` 映射到固定 21 点、处理左右手拆分、镜像和轴变换

2. 输出 retarget 层
   - 输入：`/hand_input`
   - 输出：左右手控制命令
   - 责任：分别调用左右手 `Retargeter`，并映射到 Wuji 手 20 维关节顺序

### 6.3 物理实现方式

为了兼顾结构清晰与性能，本阶段建议区分“逻辑分层”和“进程分层”：

#### 方案 A：兼容参考工作空间的两节点实现

- `manus_input_adapter.py`
- `wujihand_retarget_bridge.py`

优点：

- 与参考工作空间职责边界一致
- `/hand_input` 的生产者和消费者分离清晰
- 调试、录包和替换输入源更直接

缺点：

- 多一次 ROS topic hop

#### 方案 B：性能优先的一进程实现

单个节点内部包含两个模块：

- `ManusInputAdapter`
- `WujihandRetargetBridge`

内部直接传递 `(21, 3)` 数组，可选地对外发布 `/hand_input` 供调试和录包。

优点：

- 少一次进程间消息转发
- 结构仍然清晰，但延迟更低

缺点：

- `/hand_input` 作为在线主链路接口的独立性会弱一些

本阶段建议采用：

- 逻辑上严格按两层设计
- 实现上优先落地方案 B
- `/hand_input` 作为可配置输出保留

这样既对齐参考工作空间的接口边界，也尽量避免不必要延迟。

## 7. 包内组织

### 7.1 保持单 ROS 包边界

本阶段不新建顶层 ROS 包，仍然在 `wujihand_system` 内完成接入。  
原因：

1. 与当前工作空间的“系统包”组织方式一致
2. 便于与现有 hand bringup、controller 配置、launch 和测试放在一起管理
3. 与用户已确认的“放进 `wujihand_system`”决策一致

### 7.2 推荐文件树

建议在 `wujihand_system` 内新增以下结构：

- `third_party/wuji_retargeting/`
- `bringup/python/wujihand_teleop/__init__.py`
- `bringup/python/wujihand_teleop/manus_mapping.py`
- `bringup/python/wujihand_teleop/manus_input_adapter.py`
- `bringup/python/wujihand_teleop/wujihand_retarget_bridge.py`
- `bringup/python/wujihand_teleop/wujihand_manus_pipeline.py`
- `bringup/python/wujihand_teleop/config_types.py`
- `bringup/python/wujihand_teleop/profiling.py`
- `bringup/scripts/manus_input_adapter.py`
- `bringup/scripts/wujihand_retarget_bridge.py`
- `bringup/scripts/wujihand_manus_pipeline.py`
- `bringup/config/manus_input.yaml`
- `bringup/config/retarget_manus_left.yaml`
- `bringup/config/retarget_manus_right.yaml`
- `bringup/launch/wujihand_manus_teleop.launch.py`
- `test/manus_mapping_test.py`
- `test/manus_input_format_test.py`
- `test/wujihand_manus_launch_test.py`

设计意图：

- `bringup/python/wujihand_teleop/` 放共享 Python 逻辑，避免多个脚本重复粘贴
- `bringup/scripts/*.py` 只做薄入口，负责解析参数并启动节点
- `third_party/wuji_retargeting/` 仅保存第三方库，不承载工作空间特有逻辑

### 7.3 各文件职责

#### `third_party/wuji_retargeting/`

- 保存上游源码
- 记录上游版本和本地补丁
- 不放 ROS2 逻辑

#### `bringup/python/wujihand_teleop/manus_mapping.py`

- 固化 Manus `node_id -> MediaPipe 21 点` 映射
- 实现左右手坐标变换
- 实现：
  - `raw glove -> (21, 3)`
  - `dual gloves -> hand_input flat array`

#### `bringup/python/wujihand_teleop/manus_input_adapter.py`

- 纯输入适配逻辑
- 订阅 `/manus_raw_publisher_node/gloves_raw`
- 输出标准 `/hand_input`

#### `bringup/python/wujihand_teleop/wujihand_retarget_bridge.py`

- 纯输出 retarget 逻辑
- 订阅 `/hand_input`
- 调用左右手 `Retargeter`
- 发布左右手控制命令

#### `bringup/python/wujihand_teleop/wujihand_manus_pipeline.py`

- 高性能主实现
- 内部持有输入适配器和 retarget bridge 两个模块
- 默认内部直连，不做一次额外 ROS hop
- 可选发布 `/hand_input`

#### `bringup/python/wujihand_teleop/config_types.py`

- YAML 参数加载
- 参数校验和默认值
- 统一管理节点配置结构

#### `bringup/python/wujihand_teleop/profiling.py`

- 记录 raw 输入、`/hand_input` 构造、retarget 求解、命令发布耗时
- 提供固定窗口统计和调试日志

#### `bringup/scripts/*.py`

- 薄入口
- 避免把核心逻辑直接堆在脚本文件里

### 7.4 构建与依赖变更

为了支撑共享 Python 模块，建议对 `wujihand_system` 做如下构建调整：

1. 增加 `ament_cmake_python`
2. 安装 `bringup/python/wujihand_teleop/` 作为 Python package
3. 继续用 `install(PROGRAMS ...)` 安装入口脚本

`package.xml` 预计新增或确认以下依赖：

- `ament_cmake_python`
- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `manus_system`
- `python3-numpy`

`wuji_retargeting` 本身不建议作为系统 pip 依赖名写死在 `package.xml`。  
本阶段仍按 vendored third-party 或本地 editable 安装方式管理。

## 8. 节点与接口设计

### 8.1 `/hand_input` 标准接口

`/hand_input` 作为工作空间内统一手部输入接口，约定如下：

- 类型：`std_msgs/Float32MultiArray`
- 单手：63 值，形状 `(21, 3)`
- 双手：126 值，顺序为：
  - `0:63` 右手
  - `63:126` 左手

这条约定一旦确定，后续所有输入源都应遵守，不能再针对 Manus 特判新的格式。

### 8.2 `manus_input_adapter` 节点接口

订阅：

- `/manus_raw_publisher_node/gloves_raw`

发布：

- `/hand_input`
- 可选 `/hand_input_debug/left`
- 可选 `/hand_input_debug/right`

核心参数建议：

- `input_topic`
- `output_topic`
- `publish_hand_input`
- `include_left_hand`
- `include_right_hand`
- `drop_if_missing_hand`
- `right_first`
- `flip_x`
- `flip_y`
- `flip_z`
- `expected_left_side_name`
- `expected_right_side_name`

节点责任：

1. 按 `msg.side` 区分左右手
2. 将 `raw_nodes` 转成 `(21, 3)`
3. 拼接成 63 或 126 值 flat array
4. 处理单手缺失和数据异常

### 8.3 `wujihand_retarget_bridge` 节点接口

订阅：

- `/hand_input`

发布：

- `/left/forward_position_controller/commands`
- `/right/forward_position_controller/commands`
- 可选 `/left/wujihand_retarget/qpos_debug`
- 可选 `/right/wujihand_retarget/qpos_debug`

核心参数建议：

- `hand_input_topic`
- `left_command_topic`
- `right_command_topic`
- `left_retarget_config`
- `right_retarget_config`
- `require_both_hands`
- `hold_last_command_on_missing_input`
- `command_timeout_sec`
- `publish_debug_qpos`

节点责任：

1. 解析 `/hand_input`
2. 拆出左右手 `(21, 3)`
3. 分别调用左右手 `Retargeter`
4. 输出现有控制器可接受的命令格式

### 8.4 `wujihand_manus_pipeline` 节点接口

订阅：

- `/manus_raw_publisher_node/gloves_raw`

发布：

- `/left/forward_position_controller/commands`
- `/right/forward_position_controller/commands`
- 可选 `/hand_input`

核心参数建议：

- `mode`: `pipeline` 或 `bridge_only`
- `publish_hand_input`
- `profiling_enabled`
- `profiling_log_period_sec`
- `left_only`
- `right_only`

节点责任：

1. 作为最终推荐运行入口
2. 内部调用输入适配模块与 retarget 输出模块
3. 在单进程内完成主链路

## 9. 性能与延迟设计

### 9.1 为什么不会带来巨大延迟

按这条链路，新增的数据处理主要是：

1. `raw_nodes -> (21, 3)` 的索引映射
2. `Float32MultiArray` 的 63/126 值组织
3. `Retargeter` 求解

其中：

- 第 1 步和第 2 步都是轻量数组操作
- 真正可能占时的是第 3 步 retarget 求解

所以设计重点不是“避免 `/hand_input` 这个概念层”，而是：

- 减少额外 ROS hop
- 避免积压旧帧
- 把 profiling 做到位

### 9.2 延迟预算

本阶段建议设定如下软件链路目标值：

- `raw callback -> 21点转换`：
  - 平均 `< 0.5 ms`
- `21点转换 -> retarget 输出`：
  - 平均 `< 3 ms`
- `raw callback -> 最终 command publish` 总软件开销：
  - 平均 `< 5 ms`
  - `p95 < 10 ms`

说明：

- 这里不包含 Manus SDK 内部采样延迟
- 也不包含硬件执行层机械响应延迟

### 9.3 运行模式与性能选择

建议明确以下结论：

1. 开发调试期：
   - 可使用两节点模式
   - 强制发布 `/hand_input`
2. 实机 teleop 期：
   - 默认使用单进程 pipeline
   - `/hand_input` 作为可选调试输出

也就是说，`/hand_input` 作为逻辑标准接口必须保留，但并不要求它一定是高性能模式下的“额外在线 hop”。

### 9.4 必须加入的性能观测点

至少记录以下时间点：

1. 收到 `/manus_raw_publisher_node/gloves_raw`
2. 完成左手 `(21, 3)` 转换
3. 完成右手 `(21, 3)` 转换
4. 完成左手 retarget
5. 完成右手 retarget
6. 发布左手命令
7. 发布右手命令

建议输出：

- 平均耗时
- 最大耗时
- p95
- 当前输入帧率
- 当前输出命令帧率

## 10. 分阶段实施计划

### 阶段一：第三方依赖与构建接入

目标：让 `wujihand_system` 能稳定导入并调用 `wuji-retargeting`。

需要完成：

1. 增加 `third_party/wuji_retargeting/`
2. 增加 Python 共享模块目录
3. 更新 `CMakeLists.txt`
4. 更新 `package.xml`
5. 验证脚本与 Python package 安装链路

阶段产物：

- 第三方源码目录
- Python 共享模块骨架
- 构建与安装改动

### 阶段二：输入适配层落地

目标：稳定产出标准 `/hand_input`。

需要完成：

1. 实现 `manus_mapping.py`
2. 对照参考生产实现，确认 21 点映射表
3. 固化右前左后的双手拼接顺序
4. 写 `manus_input_adapter.py`
5. 写映射与格式测试

阶段产物：

- `manus_mapping.py`
- `manus_input_adapter.py`
- `manus_input.yaml`
- 输入格式测试

### 阶段三：输出 retarget 层落地

目标：基于 `/hand_input` 产出左右手命令。

需要完成：

1. 写 `wujihand_retarget_bridge.py`
2. 引入左右手 Manus retarget 配置
3. 校验 20 维命令顺序与现有 controller 对齐
4. 增加缺帧保护和持位逻辑

阶段产物：

- `wujihand_retarget_bridge.py`
- `retarget_manus_left.yaml`
- `retarget_manus_right.yaml`

### 阶段四：高性能 pipeline 落地

目标：在单进程内打通主链路。

需要完成：

1. 写 `wujihand_manus_pipeline.py`
2. 集成 profiling
3. 增加是否发布 `/hand_input` 开关
4. 验证与两节点模式结果一致

阶段产物：

- `wujihand_manus_pipeline.py`
- `profiling.py`
- 主链路延迟观测结果

### 阶段五：launch、联调与验收

目标：让整条链路可直接使用。

需要完成：

1. 写 `wujihand_manus_teleop.launch.py`
2. 提供：
   - 两节点调试模式
   - 单进程高性能模式
3. 与 `manus_system` 联调
4. 与现有 `forward_position_controller` 联调
5. 做录包与回放验证

阶段产物：

- launch 文件
- 联调记录
- 录包与回放样例

## 11. 需要修改的文件范围

本阶段预计涉及以下区域：

- `wujihand_system/CMakeLists.txt`
- `wujihand_system/package.xml`
- `wujihand_system/third_party/wuji_retargeting/`
- `wujihand_system/bringup/python/wujihand_teleop/`
- `wujihand_system/bringup/scripts/`
- `wujihand_system/bringup/config/`
- `wujihand_system/bringup/launch/`
- `wujihand_system/test/`

原则上不修改：

- `manus_system` 原始消息定义
- `manus_system` SDK 采集逻辑
- `wujihand_system` 现有硬件插件接口

## 12. 测试与验收矩阵

### 12.1 单元测试

- 映射表完整性测试
- 单手 63 值格式测试
- 双手 126 值格式测试
- 右前左后顺序测试
- 丢失节点容错测试

### 12.2 集成测试

- `gloves_raw -> /hand_input`
- `/hand_input -> left/right commands`
- 单进程 pipeline 全链路
- 两节点模式与单进程模式输出一致性

### 12.3 实机测试

- 左手独立
- 右手独立
- 双手同时
- 手套重连
- 短时丢帧
- 无新输入超时

### 12.4 性能验收

- 平均端到端软件开销满足目标
- 不出现明显旧帧排队
- 输出命令频率稳定

## 13. 风险与注意事项

### 13.1 Manus 映射应以生产链路为准

参考工作空间里既有生产转换逻辑，也有调试脚本。  
本阶段应以 `manus_input_node.py` 生产实现为准，不应直接照抄可视化脚本中的映射或坐标处理。

### 13.2 左右手坐标系不能假设完全一致

必须在实机数据下明确：

- 是否需要翻转 X 轴
- 是否需要翻转 Y 轴
- 左右手是否共用同一套转换函数
- 左右手 `Retargeter` 是否只差 `mediapipe_rotation.z`

### 13.3 控制命令顺序必须严格对齐现有控制器

retarget 输出的 20 维关节顺序必须严格匹配：

- 左手 `forward_position_controller`
- 右手 `forward_position_controller`

否则会出现动作错位且不易排查。

### 13.4 不应将 Python retargeting 逻辑塞入实时 controller 循环

本阶段应避免把 Python 调用放进 `ros2_control` controller `update()` 路径，防止影响控制器周期稳定性和故障隔离。

### 13.5 性能优化应以 profiling 结果为依据

在没有 profiling 结果前，不应主观判断 `/hand_input` 一层一定是瓶颈。  
如果后续需要优化，应优先：

1. 先做消息堆积与回调耗时分析
2. 再决定是否关闭 `/hand_input` 在线发布
3. 最后再考虑是否进一步压缩或合并节点

## 14. 建议的首批实现顺序

建议按下面顺序推进：

1. 接入 `third_party/wuji_retargeting`
2. 落地 Python 共享模块骨架
3. 固化 `raw_nodes -> /hand_input` 映射
4. 打通 `/hand_input -> retarget -> 命令`
5. 落地单进程高性能 pipeline
6. 补 launch、测试、录包和 profiling

## 15. 本阶段完成定义

当以下条件同时满足时，可认为本阶段完成：

1. `wujihand_system` 内可直接运行 Manus teleop
2. Manus 原始 `raw_nodes` 可稳定转换为标准 `/hand_input`
3. 左右手都能基于 `/hand_input` 分别完成 retarget
4. 左右手命令可以分别驱动现有 Wuji 手控制器
5. 支持两节点调试模式和单进程高性能模式
6. launch、参数、调试、录包和 profiling 入口齐备
7. 有至少一条实机验证记录和一份可回放数据
