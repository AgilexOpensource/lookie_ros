# lookie_ros

[English](README.md)

Lookie 机器人（Trio 和 Neo）的 ROS 2 功能包：驱动、基于 kstone 的全身控制、RViz 和 VR 遥操作。

整体流程框架图见 [docs/architecture.zh-CN.md](docs/architecture.zh-CN.md)；改动前后的回归检查见 [docs/regression.zh-CN.md](docs/regression.zh-CN.md)。

## 功能包

| 功能包 | 说明 |
| --- | --- |
| [lookie_bringup](lookie_bringup/README.zh-CN.md) | 整机启动文件，以及驱动、kstone_ros 和遥操作的各机型配置 |
| [lookie_driver](lookie_driver/README.zh-CN.md) | 头部、躯干、机械臂和底盘驱动，以及关节桥 |
| [lookie_control](lookie_control/README.zh-CN.md) | 按给定配置、URDF 和话题运行 kstone_ros 的启动文件，不含配置 |
| [lookie_teleop](lookie_teleop/README.zh-CN.md) | VR 全身遥操作：把 xr_media_ros 输入转成 kstone_ros 目标 |

依赖仓库由 [vcstool](https://github.com/dirk-thomas/vcstool) 克隆到本仓库旁边：

| 仓库 | 说明 | 使用方 |
| --- | --- | --- |
| [kstone](https://github.com/AgilexOpensource/kstone) | C++ 全身 IK 引擎 | lookie_control |
| [kstone_ros](https://github.com/AgilexOpensource/kstone_ros) | kstone 的 ROS 2 节点 | lookie_control |
| [lookie_description](https://github.com/AgilexOpensource/lookie_description) | URDF、网格和 RViz 显示 | lookie_control、lookie_bringup |
| [xr_media_ros](https://github.com/AgilexOpensource/xr_media_ros) | XR 头显追踪和媒体流 | lookie_teleop |
| [head_ros](https://github.com/AgilexOpensource/head_ros)、[lift_ros](https://github.com/AgilexOpensource/lift_ros)、[fold_ros](https://github.com/AgilexOpensource/fold_ros)、[agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2)、[scout_ros2](https://github.com/AgilexOpensource/scout_ros2/tree/humble)、[ugv_sdk](https://github.com/westonrobot/ugv_sdk) | 头部、躯干、机械臂和底盘驱动 | lookie_driver |

## 环境要求

已在 Ubuntu 22.04 + ROS 2 Humble 上测试通过。不支持更早的版本，更新的版本未经测试。

机器人需要 [CAN 配置](lookie_bringup/docs/can.zh-CN.md#接口)中列出的 CAN 接口。

## 安装

- [安装脚本](docs/setup.zh-CN.md)：交互式，适合快速部署，例如产线测试工位。
- [手动安装](docs/install.zh-CN.md)：手动执行同样的步骤，也可只装单个功能包。

## 快速开始

使用安装脚本时第 1 步已完成，改为在每个终端执行 `source <工作空间>/lookie_env.bash`（其中已设置 `LOOKIE_MODEL`）。

1. 选定机型，只需一次。[lookie_bringup/config](lookie_bringup/config) 下每个文件夹对应一个机型：

   ```bash
   sed -i '/^export LOOKIE_MODEL=/d' ~/.bashrc
   echo 'export LOOKIE_MODEL=lookie__pf-lookie_trio_r01__ee-revo2std_r01' >> ~/.bashrc
   source ~/.bashrc
   ```

2. 启用 CAN 接口。在其他电脑上使用时，先按 [CAN 配置](lookie_bringup/docs/can.zh-CN.md)设置端口映射：

   ```bash
   ros2 run lookie_bringup can_bind.sh
   ```

> **警告：第 3 步启动驱动时全部电机即被使能；第 4 步 kstone 收到反馈后立即开始控制机器人，双臂会先在零空间调整几秒。** 启动前清空机器人周围，并确保急停在手边。

3. 终端 1，驱动：

   ```bash
   ros2 launch lookie_bringup lookie.launch.py
   ```

   需要 RViz 时加 `rviz:=true`。没有真机时加 `sim:=true`：用关节滑块代替驱动，不会向硬件发送任何指令。仿真的限制见[没有真机时](#没有真机时)。

4. 终端 2，全身控制。收到反馈后 kstone 保持当前位姿：

   ```bash
   ros2 launch lookie_bringup kstone.launch.py
   ```

5. 终端 3，VR 遥操作：

   ```bash
   ros2 launch lookie_bringup teleop.launch.py
   ```

### 没有真机时

```bash
ros2 launch lookie_bringup lookie.launch.py sim:=true rviz:=true fixed_frame:=base_footprint
```

- 关节滑块代替驱动发布关节反馈，kstone 和 RViz 把它当作真机。
- 仿真没有底盘里程计：RViz 必须用 `fixed_frame:=base_footprint`，否则看不到机器人；kstone 的 `base` 反馈组一直不新鲜，state 中 `fresh` 为 `false`，底盘不可控。这是预期现象。
- kstone 的关节命令不会回到滑块：RViz 中的机器人只跟随滑块。要查看 kstone 输出的命令，用下面的 [kstone_viewer](#可视化工具)。

## VR 遥操作

1. 首次使用时执行 `xr_media --init-certs` 生成 HTTPS 证书。每次执行都会覆盖已有证书。
2. 启动上面第 5 步，日志会打印 `Open in headset browser: https://<机器人IP>:9443/...`。
3. 在头显浏览器中打开该地址，信任证书并进入 VR。

按键、校准和安全事项见 [VR 遥操作快速上手](lookie_teleop/docs/vr_teleop_quickstart.zh-CN.md)。

## 可视化工具

[kstone_viewer](https://github.com/AgilexOpensource/kstone_viewer) 在浏览器中显示 kstone 的状态、目标和求解质量：

```bash
python3 -m venv ~/.venvs/kstone-viewer
~/.venvs/kstone-viewer/bin/pip install "git+https://github.com/AgilexOpensource/kstone_viewer.git"
```

viewer 不依赖 ROS，所以单独用一个 venv；工作区 venv 固定了 `numpy<2`，装 viewer 会把它升级。

第 4 步加上 `debug_port:=7311` 启动，再在另一个终端运行 viewer：

```bash
ros2 launch lookie_bringup kstone.launch.py debug_port:=7311
~/.venvs/kstone-viewer/bin/kstone-viewer --connect 127.0.0.1:7311
```

打开 http://127.0.0.1:8080。要在另一台电脑上查看，在那台电脑上执行 `ssh -N -L 8080:127.0.0.1:8080 <用户>@<机器人IP>`，再打开同一地址。

## 常见问题

- **驱动启动后立即退出**：对应的 CAN 接口未启用或名称不对，运行 `ros2 run lookie_bringup can_bind.sh --list` 检查。
- **RViz 中看不到机器人**：没有底盘里程计（例如 `sim:=true`）。第 3 步加上 `fixed_frame:=base_footprint`。
- **机器人不跟随 kstone**：用 `ros2 topic echo /trio/kstone/state --qos-durability transient_local` 查看状态（Neo 为 `/neo/...`）；kstone 收到关节反馈后才会保持位姿。
- **头显页面打不开**：头显和机器人需在同一网络，并且已信任证书。

## 许可证

Apache-2.0，见各功能包中的 `LICENSE` 文件。
