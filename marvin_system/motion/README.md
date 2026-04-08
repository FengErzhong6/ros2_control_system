# Marvin Motion Layer

`motion/` 保存 Marvin 的非实时运动层内容。

当前阶段目标：

- 固定 `marvin_system` 内部的 MoveIt 归属位置
- 提供统一的 `/marvin_motion/go_home` 接口
- 让 `data_collection` 不再直接依赖 `tracker_teleop_controller/go_home`

当前实现仍是迁移骨架：

- `motion_server.py` 提供统一入口
- 支持 `backend=legacy` 与 `backend=moveit`
- `backend=moveit` 时会检测 MoveIt 运行依赖是否已安装
- 在 tracker teleop 启动链中，可通过参数允许临时回落到旧的 `tracker_teleop_controller/go_home`
- 在 safe-operation 启动链中，默认禁止 legacy fallback，为后续 MoveIt 规划实现保留安全边界

后续阶段将在此目录继续加入：

- MoveIt 规划配置
- 命名位姿管理
- 规划场景管理
- 双臂协作前置动作
