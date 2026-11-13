# 安装脚本

[English](setup.md)

`scripts/setup.sh` 一次完成 Lookie 工作空间的部署：系统包、依赖仓库、Python SDK、编译和机型选择。适用于新机器和产线测试工位。脚本不处理 CAN，CAN 见 [CAN 配置](../lookie_bringup/docs/can.zh-CN.md)。

## 环境要求

- 已在 Ubuntu 22.04 + ROS 2 Humble 上测试通过。更早的版本脚本会拒绝，更新的版本未经测试，脚本会给出警告。
- ROS 2 安装在 `/opt/ros` 下。装了多个发行版且未设置 `ROS_DISTRO` 时，脚本会让你选择。
- 能访问 GitHub 和 PyPI。

## 快速开始

```bash
mkdir -p ~/lookie_ws/src && cd ~/lookie_ws/src
git clone https://github.com/AgilexOpensource/lookie_ros.git
./lookie_ros/scripts/setup.sh
```

选 `1`，按提示操作，从列表中选择机型。然后打开新终端，启用 CAN 并启动机器人：

```bash
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
3. 把 Python SDK 装到 `~/lookie_ws/.venv`。系统 Python 允许 pip 安装时，脚本会询问用虚拟环境还是系统 Python。
4. 用 rosdep 安装 ROS 依赖，跳过已由 pip 安装的 `xr_media` 和 Humble 没有提供的 `warehouse_ros_mongo`。
5. 用 `python3 -m colcon build` 编译。
6. 把所选机型写入 `~/.bashrc`。

## 改动范围

- `~/lookie_ws/src`、`build`、`install` 和 `.venv`。
- `~/.bashrc` 中 `# >>> lookie >>>` 与 `# <<< lookie <<<` 之间的一段，用于激活虚拟环境、加载工作空间并导出 `LOOKIE_MODEL`。重复运行会替换这一段，删除它即可撤销。
- 系统 apt 包，仅在你确认后安装。

脚本可重复运行。用 `4` 切换机型，用 `2` 更新。

某一步失败时，见手动安装指南的[常见问题](install.zh-CN.md#常见问题)。
