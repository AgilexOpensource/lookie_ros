# 安装

[English](install.md)

手动安装 Lookie 工作空间。[安装脚本](setup.zh-CN.md)会为整机执行同样的步骤。

已在 Ubuntu 22.04 + ROS 2 Humble 上测试通过。不支持更早的版本，更新的版本未经测试。下文命令以 Humble 为例。

## 系统软件包

`agx_motor_sdk` 在 `pip install` 时会编译 C++ 扩展，所以要先装好编译工具：

```bash
sudo apt update
sudo apt install build-essential cmake libasio-dev libyaml-cpp-dev python3-dev \
  python3-vcstool python3-colcon-common-extensions python3-rosdep python3-venv python3-pip
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

默认装到系统 Python，与生产机器一致。生产部署安装 [requirements.lock.txt](../requirements.lock.txt) 中已验证的版本：

```bash
cd ~/lookie_ws
pip3 install -r src/lookie_ros/requirements.lock.txt   # 生产：固定版本
pip3 install -r src/lookie_ros/requirements.txt        # 或：各默认分支的最新版本
```

不加 `sudo` 时，pip 装到当前用户的 `~/.local`；机器人要以该用户运行。

**可选，开发用：** 工作空间虚拟环境把调试中的 SDK 版本与系统 Python 隔离，同时仍能使用 ROS 的 Python 包：

```bash
cd ~/lookie_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r src/lookie_ros/requirements.txt
```

使用虚拟环境时，编译和运行前都要先激活它（见下文）。[requirements.txt](../requirements.txt) 列出全部 SDK，跟踪各自的默认分支。

[requirements.lock.txt](../requirements.lock.txt) 把 git 来源的 SDK 固定到 commit，PyPI 来源的固定到版本号。一组版本在真机上验证通过后，在该环境中重新生成并提交：

```bash
python3 src/lookie_ros/scripts/lock_python.py                # 当前 Python 中已安装的版本
python3 src/lookie_ros/scripts/lock_python.py --from remote  # 远端最新 HEAD / 最新发布版
```

默认模式会拒绝从本地目录安装的 SDK，因为无法复现其版本；先改为从 git commit 安装。

## VR 的 HTTPS 准备

首次使用 VR 前生成开发证书（使用虚拟环境时先激活它）：

```bash
xr_media --init-certs
```

此命令会覆盖 `${XDG_CONFIG_HOME:-$HOME/.config}/xr_media/tls` 下的 `cert.pem` 和 `key.pem`，已有有效证书时跳过。默认配置中的证书路径为空时，XR 服务会自动查找这些文件。在 Quest 浏览器打开节点打印的 **HTTPS** 地址，接受自签名证书后进入 VR。远程 WebXR 需要安全上下文；HTTP 页面能打开不代表 VR 可用。自定义证书路径见 [xr_media_ros](https://github.com/AgilexOpensource/xr_media_ros/blob/main/docs/GUIDE.zh-CN.md)。

## 编译

```bash
cd ~/lookie_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "xr_media warehouse_ros_mongo"
python3 -m colcon build
source install/setup.bash
```

`python3 -m colcon` 让节点使用运行 colcon 的 Python：已激活虚拟环境时用虚拟环境的，否则用系统的。同一工作空间应始终在同一环境中编译。加 `--packages-up-to <功能包>` 只编译该包及其依赖。`--skip-keys` 跳过已由 pip 安装的 `xr_media`，以及 agx_arm_ros 的 MoveIt 包声明、但 Humble 没有提供的 `warehouse_ros_mongo`。

在运行 Lookie 的终端中执行 `source ~/lookie_ws/install/setup.bash`；使用虚拟环境时先执行 `source ~/lookie_ws/.venv/bin/activate`。多人共用的机器上不要在 `~/.bashrc` 中激活虚拟环境：这会改变所有终端的 Python，包括不使用它的其他模块。然后按 [lookie_bringup](../lookie_bringup/README.zh-CN.md#使用) 设置机型。

## 更新

```bash
vcs pull ~/lookie_ws/src
vcs import --recursive --skip-existing ~/lookie_ws/src < ~/lookie_ws/src/lookie_ros/lookie.repos
```

第二条命令导入新增的依赖仓库。

> **重要：** 更新后必须重新编译全部功能包（`python3 -m colcon build`）。只更新或只编译部分仓库时，配置与程序版本可能不一致，例如 kstone 启动即报 `unknown key`。编译后用 `python3 src/lookie_ros/scripts/check_install.py` 确认 install 与源码一致，见[回归检查](regression.zh-CN.md)。

## 常见问题

- 安装 `agx_motor_sdk` 时报 `GitHub release metadata request failed`：SDK 安装时会从 GitHub 下载协议库，匿名访问每个 IP 每小时只能请求 60 次 API。等限额重置，或把 `GITHUB_TOKEN` 设为 GitHub token 后重装。
- 启动时驱动报 `ModuleNotFoundError`：在该终端先激活 venv 再 `source install/setup.bash`，并用 `python3 -m colcon build` 重新编译。
