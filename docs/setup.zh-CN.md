# 安装脚本

[English](setup.md)

`scripts/setup.sh` 一次完成 Lookie 工作空间的部署：系统包、依赖仓库、Python SDK、编译和机型选择。适用于新机器和产线测试工位。脚本不处理 CAN，CAN 见 [CAN 配置](../lookie_bringup/docs/can.zh-CN.md)。

## 环境要求

- 已在 Ubuntu 22.04 + ROS 2 Humble 上测试通过。更早的版本脚本会拒绝，更新的版本未经测试，脚本会给出警告。
- ROS 2 安装在 `/opt/ros` 下。装了多个发行版且未设置 `ROS_DISTRO` 时，脚本会让你选择。
- 能访问 GitHub 和 PyPI。

## 快速开始

工作空间可以是任意目录，下面的 `~/lookie_ws` 只是示例。脚本根据 lookie_ros 的克隆位置（`<工作空间>/src/lookie_ros`）确定工作空间。

```bash
mkdir -p ~/lookie_ws/src && cd ~/lookie_ws/src
git clone https://github.com/AgilexOpensource/lookie_ros.git
./lookie_ros/scripts/setup.sh
```

Python SDK 默认装到系统 Python。生产部署和开发可以预先指定：

```bash
LOOKIE_LOCKED=1 LOOKIE_VENV=0 ./lookie_ros/scripts/setup.sh   # 生产：系统 Python + 固定版本
LOOKIE_VENV=1 ./lookie_ros/scripts/setup.sh                   # 开发：工作空间虚拟环境
```

选 `1`，按提示操作，从列表中选择机型。然后打开新终端，加载工作空间环境，启用 CAN 并启动机器人：

```bash
source ~/lookie_ws/lookie_env.bash   # <工作空间>/lookie_env.bash
ros2 run lookie_bringup can_bind.sh
ros2 launch lookie_bringup lookie.launch.py
```

使用 VR 前，完成 [HTTPS 准备](install.zh-CN.md#vr-的-https-准备)。脚本会安装 XR SDK，但不会生成或覆盖 TLS 证书。

## 菜单

| 选项 | 操作 |
| --- | --- |
| `1` | 完整安装：依次执行下列全部步骤 |
| `2` | 拉取所有仓库、导入新增仓库并重新编译 |
| `3` | 仅重新编译 |
| `4` | 选择机型 |
| `5` 或 `q` | 退出 |

完整安装的步骤：

1. 安装缺少的 apt 包（编译工具、vcstool、colcon、rosdep、venv、pip），使用 `sudo` 前先询问。从未执行过 `rosdep init` 时执行它。
2. 导入 [lookie.repos](../lookie.repos) 中的仓库，已存在的仓库保持不变。
3. 按 [requirements.txt](../requirements.txt) 安装 Python SDK；设置 `LOOKIE_LOCKED=1` 时改装 [requirements.lock.txt](../requirements.lock.txt) 中固定的版本。默认装到系统 Python（生产）；脚本会询问是否改用工作空间虚拟环境 `<工作空间>/.venv`（开发，回答 `y`）。`LOOKIE_VENV=1` 或 `LOOKIE_VENV=0` 可预先回答。已有 `.venv` 时直接沿用。只有系统 Python 拒绝 pip 安装时（Ubuntu 24.04 及以上）才自动创建虚拟环境。
4. 用 rosdep 安装 ROS 依赖，跳过已由 pip 安装的 `xr_media` 和 Humble 没有提供的 `warehouse_ros_mongo`。
5. 用 `python3 -m colcon build` 编译。
6. 生成 `<工作空间>/lookie_env.bash`：激活虚拟环境（如有）、加载工作空间并导出所选的 `LOOKIE_MODEL`。

## 改动范围

- `<工作空间>/src`、`build`、`install` 和 `.venv`。
- `<工作空间>/lookie_env.bash`。加载时按文件自身位置找到工作空间，所以工作空间可以克隆到任意位置或整体移动。脚本不修改 `~/.bashrc`，机器上其他模块和开发者的 Python 环境不受影响。在运行 Lookie 的终端中执行 `source <工作空间>/lookie_env.bash`；只有希望所有终端都使用它时，才自行把这一行加入 `~/.bashrc`。
- 旧版脚本会在 `~/.bashrc` 中写入 `# >>> lookie >>>` 一段，使所有终端都激活虚拟环境。脚本发现它时会给出警告，如有影响请手动删除。
- 系统 apt 包，仅在你确认后安装。

脚本可重复运行。用 `4` 切换机型，用 `2` 更新。

某一步失败时，见手动安装指南的[常见问题](install.zh-CN.md#常见问题)。
