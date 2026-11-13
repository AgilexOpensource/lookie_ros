# 安装

[English](install.md)

手动安装 Lookie 工作空间。[安装脚本](setup.zh-CN.md)会为整机执行同样的步骤。

已在 Ubuntu 22.04 + ROS 2 Humble 上测试通过。不支持更早的版本，更新的版本未经测试。下文命令以 Humble 为例。

## 系统软件包

```bash
sudo apt update
sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep \
  python3-venv python3-pip
```

每台机器初始化一次 rosdep。若 `/etc/ros/rosdep/sources.list.d` 已存在，跳过 `sudo rosdep init`：

```bash
sudo rosdep init
rosdep update
```

## 克隆

```bash
mkdir -p ~/lookie_ws/src && cd ~/lookie_ws/src
git clone https://github.com/AgilexOpensource/lookie_ros.git
vcs import --recursive < lookie_ros/lookie.repos
```

只用其中一个功能包时，改为导入它自己的文件：

| 功能包 | 仓库列表 | Python SDK |
| --- | --- | --- |
| lookie_bringup | [lookie_bringup.repos](../lookie_bringup/lookie_bringup.repos)（全部） | 下面全部 |
| lookie_driver | [lookie_driver.repos](../lookie_driver/lookie_driver.repos) | pyAgxArm（机械臂）、agx_motor_sdk（头部、升降、折叠）、bc-stark-sdk（Revo2 灵巧手） |
| lookie_control | [lookie_control.repos](../lookie_control/lookie_control.repos) | 无 |
| lookie_teleop | [lookie_teleop.repos](../lookie_teleop/lookie_teleop.repos) | xr_media |

## Python SDK

虚拟环境不改动系统 Python，同时仍能使用 ROS 的 Python 包：

```bash
cd ~/lookie_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install git+https://github.com/agilexrobotics/pyAgxArm.git \
  git+https://github.com/kehuanjack/agx_motor_sdk.git bc-stark-sdk \
  git+https://github.com/AgilexOpensource/xr_media.git 'numpy<2' 'setuptools<80'
```

不用虚拟环境时，用 `pip3` 执行同样的安装，下面改用 `colcon`。

## 编译

```bash
cd ~/lookie_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m colcon build
source install/setup.bash
```

`python3 -m colcon` 让节点使用虚拟环境的 Python。加 `--packages-up-to <功能包>` 只编译该包及其依赖。rosdep 会提示 `xr_media` 无法解析，它已由 pip 安装。

每个新终端都要先执行 `source ~/lookie_ws/.venv/bin/activate`，再 `source ~/lookie_ws/install/setup.bash`，也可以把这两行写进 `~/.bashrc`。

## 更新

```bash
vcs pull ~/lookie_ws/src
vcs import --recursive --skip-existing ~/lookie_ws/src < ~/lookie_ws/src/lookie_ros/lookie.repos
```

然后重新编译。第二条命令导入新增的依赖仓库。

## 常见问题

- 安装 `agx_motor_sdk` 时报 `GitHub release metadata request failed`：SDK 安装时会从 GitHub 下载协议库，匿名访问每个 IP 每小时只能请求 60 次 API。等限额重置，或把 `GITHUB_TOKEN` 设为 GitHub token 后重装。
- 启动时驱动报 `ModuleNotFoundError`：在该终端先激活 venv 再 `source install/setup.bash`，并用 `python3 -m colcon build` 重新编译。
