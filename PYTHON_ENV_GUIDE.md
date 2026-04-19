# Python 环境配置与使用说明

本文档说明本工作空间推荐的 Python 环境管理方式，以及如何使用 `uv` 为 ROS2 工作空间创建一个不污染系统环境、又能正常使用 `rclpy` 的虚拟环境。

适用工作空间：

- `/home/mmlab/codes/huangshzh/ros2_control_system`

适用前提：

- ROS2 发行版：`jazzy`
- 系统 Python：`/usr/bin/python3`
- 环境管理工具：`uv`

## 1. 设计目标

本工作空间的 Python 环境方案有四个目标：

1. 不直接往系统 Python 里安装第三方依赖
2. 不使用 conda 作为本工作空间的默认运行环境
3. 仍然能够使用 ROS2 自带的 Python 包，例如 `rclpy`
4. 能为 `wuji_retargeting` 这类额外 Python 依赖提供隔离安装位置

## 2. 为什么不用 conda

本工作空间不推荐使用 conda 作为默认开发和运行环境，原因如下：

- ROS2 的 Python 扩展和系统 Python ABI 绑定更紧
- `rclpy` 在 conda 环境中更容易出现兼容问题
- `ros2 run`、`ros2 launch`、`colcon build` 与 conda 混用时更容易出现解释器不一致
- 本工作空间已经按系统 Python 3.12 和 ROS2 Jazzy 的组合在运行

如果确实存在独立研究项目需要 conda，请和本工作空间分开使用，不要混到 `ros2_control_system` 的默认终端里。

## 3. 推荐方案

推荐使用：

- `uv` 创建虚拟环境
- 选择系统 Python：`/usr/bin/python3`
- 使用 `--system-site-packages`

这样做的意义是：

- ROS2 自带的 Python 包仍然可见
- 额外安装的第三方包只进入虚拟环境
- 不污染系统 Python

本工作空间当前采用的虚拟环境目录是：

- `./.venv_ros2`

## 4. 首次创建环境

在工作空间根目录执行：

```bash
cd /home/mmlab/codes/huangshzh/ros2_control_system
UV_CACHE_DIR=/tmp/uv-cache uv venv --clear --python /usr/bin/python3 --no-managed-python --system-site-packages .venv_ros2
```

参数说明：

- `--python /usr/bin/python3`
  使用系统 Python，而不是 `uv` 自己下载的 Python
- `--no-managed-python`
  禁止 `uv` 自动切换到它管理的解释器
- `--system-site-packages`
  让虚拟环境可以看到系统 Python 的 site-packages
- `.venv_ros2`
  虚拟环境放在当前工作空间根目录

说明：

- 如果 `uv` 默认缓存目录不可写，可以像上面那样显式设置 `UV_CACHE_DIR=/tmp/uv-cache`

## 5. 安装本工作空间需要的 Python 依赖

先激活虚拟环境：

```bash
cd /home/mmlab/codes/huangshzh/ros2_control_system
source .venv_ros2/bin/activate
```

然后安装工作空间额外需要的 Python 依赖：

```bash
UV_CACHE_DIR=/tmp/uv-cache uv pip install numpy scipy pyyaml nlopt pin
```

说明：

- `numpy`、`scipy`、`pyyaml` 是通用依赖
- `nlopt`、`pin` 是 `wuji_retargeting` 需要的运行时依赖
- `pin` 安装后通常通过 `import pinocchio` 导入，而不是 `import pin`

## 6. 正确进入本工作空间环境

每次打开一个新终端，推荐按下面顺序执行：

```bash
cd /home/mmlab/codes/huangshzh/ros2_control_system
source .venv_ros2/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

说明：

1. 先激活 `uv` 虚拟环境
2. 再加载 ROS2
3. 最后加载当前工作空间的安装结果

如果还没构建过工作空间，可以先执行：

```bash
colcon build --packages-select manus_system wujihand_system
source install/setup.bash
```

## 7. 快速验证环境是否正确

进入环境后执行：

```bash
python3 - <<'PY'
import rclpy
import nlopt
import pinocchio
import wuji_retargeting
print("all good")
PY
```

如果输出 `all good`，说明：

- ROS2 Python 包可见
- `uv` venv 生效
- `wuji_retargeting` 运行依赖齐全

还可以额外检查当前解释器路径：

```bash
which python3
echo $VIRTUAL_ENV
echo $ROS_DISTRO
```

正常预期：

- `which python3` 指向 `.../.venv_ros2/bin/python3`
- `VIRTUAL_ENV` 指向 `.../.venv_ros2`
- `ROS_DISTRO=jazzy`

## 8. 使用 `uv` 的常用命令

### 8.1 查看 `uv` 版本

```bash
uv --version
```

### 8.2 创建虚拟环境

```bash
UV_CACHE_DIR=/tmp/uv-cache uv venv --python /usr/bin/python3 --no-managed-python --system-site-packages .venv_ros2
```

### 8.3 安装包

```bash
source .venv_ros2/bin/activate
UV_CACHE_DIR=/tmp/uv-cache uv pip install <package>
```

### 8.4 升级包

```bash
source .venv_ros2/bin/activate
UV_CACHE_DIR=/tmp/uv-cache uv pip install -U <package>
```

### 8.5 查看已安装包

```bash
source .venv_ros2/bin/activate
uv pip list
```

### 8.6 删除并重建虚拟环境

```bash
rm -rf /home/mmlab/codes/huangshzh/ros2_control_system/.venv_ros2
UV_CACHE_DIR=/tmp/uv-cache uv venv --python /usr/bin/python3 --no-managed-python --system-site-packages /home/mmlab/codes/huangshzh/ros2_control_system/.venv_ros2
```

## 9. VS Code 中的使用方式

本工作空间已经配置了工作空间级 `.vscode/settings.json`：

- 默认 Python 解释器指向 `${workspaceFolder}/.venv_ros2/bin/python3`
- 集成终端默认 profile 为 `ROS2 UV`
- 新终端会自动执行：
  - `source .venv_ros2/bin/activate`
  - `source /opt/ros/jazzy/setup.bash`
  - `source install/setup.bash`
- Python 扩展的终端自动激活已关闭，避免它偷偷激活旧的 conda 环境

如果配置没有立刻生效，请执行：

1. 关闭当前所有集成终端
2. 在 VS Code 中执行 `Developer: Reload Window`
3. 再新建一个终端

## 10. 和 `colcon` 的关系

这里要明确分工：

- `uv`
  负责 Python 虚拟环境和第三方 Python 依赖
- `colcon`
  负责 ROS2 包构建和安装

不要把它们混为一谈。

推荐开发流程：

```bash
cd /home/mmlab/codes/huangshzh/ros2_control_system
source .venv_ros2/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select manus_system wujihand_system
source install/setup.bash
```

推荐运行流程：

```bash
cd /home/mmlab/codes/huangshzh/ros2_control_system
source .venv_ros2/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch wujihand_system wujihand_manus_teleop.launch.py
```

## 11. 不推荐的做法

以下做法不推荐用于本工作空间：

- 直接往系统 Python 里安装 `nlopt`、`pinocchio` 等依赖
- 在这个工作空间默认使用 conda 环境
- 让 `uv` 下载另一套独立 Python 作为本工作空间解释器
- 在没有激活 `.venv_ros2` 的情况下直接跑 `wujihand_system` 的 Python teleop 节点

## 12. 常见问题

### 12.1 `uv` 提示缓存目录不可写

表现：

```bash
error: Could not acquire lock
Caused by: Read-only file system ...
```

解决：

```bash
UV_CACHE_DIR=/tmp/uv-cache uv venv ...
```

或：

```bash
UV_CACHE_DIR=/tmp/uv-cache uv pip install ...
```

### 12.2 `ModuleNotFoundError: No module named 'rclpy'`

原因：

- 没有 `source /opt/ros/jazzy/setup.bash`
- 没有使用 `--system-site-packages`
- 没有进入正确的 `.venv_ros2`

检查：

```bash
which python3
echo $VIRTUAL_ENV
echo $ROS_DISTRO
```

### 12.3 `ModuleNotFoundError: No module named 'wuji_retargeting'`

原因可能有两类：

1. 没有 `source install/setup.bash`
2. `wujihand_system` 还没重新构建安装

解决：

```bash
cd /home/mmlab/codes/huangshzh/ros2_control_system
source .venv_ros2/bin/activate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select wujihand_system
source install/setup.bash
```

### 12.4 `ModuleNotFoundError: No module named 'nlopt'`

说明：

- 这表示 `uv` 虚拟环境里还没装第三方求解依赖

解决：

```bash
source .venv_ros2/bin/activate
UV_CACHE_DIR=/tmp/uv-cache uv pip install nlopt pin
```

### 12.5 新开 VS Code 终端总是跑到旧 conda 环境

说明：

- 这是编辑器或 Python 扩展自动激活旧环境的典型表现

本工作空间已经通过 `.vscode/settings.json` 禁用了这类自动激活。  
如果还出现，请：

1. 关闭所有终端
2. `Developer: Reload Window`
3. 手动确认当前工作空间解释器是 `.venv_ros2/bin/python3`

## 13. 当前工作空间的最终建议

一句话总结：

- 本工作空间统一使用 `uv + system Python + system-site-packages + ROS2 setup + install/setup.bash`

推荐固定流程：

```bash
cd /home/mmlab/codes/huangshzh/ros2_control_system
source .venv_ros2/bin/activate
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果你只是开发和运行这个工作空间，不需要再切到 conda。  
只要维护好 `.venv_ros2` 就可以了。
