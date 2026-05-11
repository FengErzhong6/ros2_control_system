# Policy Deployment Stack

这个目录实现的是一个参考 `data_collection_stack` 组织方式的策略部署栈。

## 职责

1. 通过 recipe 启动部署所需硬件
2. 启动 supervisor 和 UI
3. 提供 `Connect` / `Disconnect` / `Execute Policy` / `Stop Policy` / `Go Home`
4. 通过 websocket 连接外部 policy server，持续拉取 action chunk
5. 解释 action 维度并下发到右臂、右手和 home 逻辑
6. 启动摄像头

## 当前结构

- `policy_deployment_interfaces`
- `policy_deployment_orchestrator`
- `policy_deployment_ui`
- `policy_deployment_bringup`

## 默认部署配置

默认 profile:

- `pi05_tianji_wuji_pick_place`

默认 policy server:

- `127.0.0.1:8000`

默认策略启动命令由用户在外部手工运行，例如：

```bash
CUDA_VISIBLE_DEVICES=0 uv run scripts/serve_policy.py \
  --port 8000 \
  --default_prompt="pick and place." \
  policy:checkpoint \
  --policy.config=pi05_tianji_wuji_pick_place \
  --policy.dir=/home/mmlab/codes/huangshzh/openpi_dexhand/checkpoints/pi05_tianji_wuji_pick_place/hsz_right_pick_place/29999
```

## 27 维动作解释

针对 `29999` 这个策略，当前按下面方式解释：

- `0..6`：右机械臂 7 维
- `7..26`：右手 20 维
- 左机械臂保持当前位姿
- 没有左手

## 图像处理

当前 profile 会对两路相机做 topic-aware square crop，再缩放到 224x224。
policy server 侧的 `ResizeImages` 已加了同尺寸 fast path，所以不会重复做一次 resize。

## 启动方式

```bash
ros2 launch policy_deployment_bringup deployment_app.launch.py
```

如果只想启动 supervisor，不带 UI：

```bash
ros2 launch policy_deployment_bringup headless_supervisor.launch.py
```

## Recipe

默认 recipe 会启动：

- `cam_high`
- `cam_left_wrist`
- Marvin 右臂链
- Wuji 右手链

其中 policy profile 只把 `cam_high` 和 `cam_left_wrist` 送入 policy server。

## 参考点

这个栈的启动层次、supervisor 结构和 managed launch 模式，都尽量贴近 `data_collection_stack`，这样更容易复用已经验证过的启动和关闭行为。
