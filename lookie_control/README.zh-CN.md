# lookie_control

按给定的引擎配置、URDF、命名空间和话题运行 [kstone_ros](https://github.com/AgilexOpensource/kstone_ros) 全身控制。本包不含机型列表和配置文件。[English](README.md)

## 安装

按[安装指南](../docs/install.zh-CN.md)操作，使用 [lookie_control.repos](lookie_control.repos)，无需 Python SDK。

## 启动

通常通过 [lookie_bringup](../lookie_bringup/README.zh-CN.md) 启动，它读取 `LOOKIE_MODEL`，传入该机型的 `kstone.yaml`、collision URDF 和 [lookie_driver](../lookie_driver/README.zh-CN.md) 的话题：

```bash
ros2 launch lookie_bringup kstone.launch.py
```

直接用 `$LOOKIE_MODEL` 的文件启动：

```bash
ros2 launch lookie_control kstone.launch.py namespace:=trio \
  config_file:=$(ros2 pkg prefix lookie_bringup)/share/lookie_bringup/config/$LOOKIE_MODEL/kstone.yaml \
  urdf:=$(ros2 pkg prefix lookie_description)/share/lookie_description/urdf/${LOOKIE_MODEL}_collision.urdf
```

节点名为 `/<namespace>/kstone`，话题均相对机器人命名空间。

参数：

- `namespace`：机器人命名空间，必填。
- `config_file`：kstone 引擎配置绝对路径，必填。
- `urdf`：URDF 绝对路径，必填，用于填充引擎配置中的 `${urdf}`。
- `package_dirs`：填充引擎配置中的 `${package_dirs}`；留空则取包含 URDF 所在包的 share 目录。
- `feedback_topic`、`control_topic`：关节反馈与命令话题，默认 `driver/feedback/joint_states` 和 `driver/control/joint_states`。
- `base_command_topic`、`odom_topic`：底盘速度命令与里程计，默认 `driver/chassis/cmd_vel` 和 `driver/chassis/odom`。
- `realtime`：`inherit`（默认）沿用配置里的线程设置；`true` 或 `false` 覆盖它。
- `debug_port`：kstone 调试服务端口；`inherit` 沿用配置，`0` 关闭。
- `use_sim_time`：默认 `false`。

TF 帧带 `<namespace>/` 前缀，世界坐标系为 `odom`，根连杆为 `base_footprint`。

## 配置

各机型的引擎配置是 [lookie_bringup](../lookie_bringup/README.zh-CN.md#配置) `config/<model>/` 中的 `kstone.yaml`：控制频率、反馈超时、实时线程、求解器、机器人描述和模块。格式以及节点的话题和服务见 [kstone_ros README](https://github.com/AgilexOpensource/kstone_ros)。
