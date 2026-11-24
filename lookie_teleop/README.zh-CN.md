# lookie_teleop

[English](README.md)

Lookie 的 VR 全身遥操作：把 [xr_media_ros](https://github.com/AgilexOpensource/xr_media_ros) 的头显和手柄输入转换成 [kstone_ros](https://github.com/AgilexOpensource/kstone_ros) 的手臂、腰部、底盘和末端执行器目标。

## 安装

按[安装指南](../docs/install.zh-CN.md)操作，使用 [lookie_teleop.repos](lookie_teleop.repos)，并安装 Python SDK `xr_media`。

## 启动

在机器人上使用整机启动文件，它会同时启动 `xr_media_ros` 并加载 `LOOKIE_MODEL` 所选机型的配置（见 [lookie_bringup](../lookie_bringup/README.zh-CN.md#使用)）：

```bash
ros2 launch lookie_bringup teleop.launch.py
```

参数和配置见 [lookie_bringup](../lookie_bringup/README.zh-CN.md#vr-遥操作)。只启动节点：

```bash
ros2 launch lookie_teleop teleop.launch.py namespace:=trio params_file:=/abs/a.yaml,/abs/b.yaml
```

- `params_file`：逗号分隔的参数 YAML，按顺序加载；留空使用节点默认值。
- `namespace`：节点命名空间。
- `chassis_cmd_topic`：底盘速度发布话题，留空使用 discover 中的 kstone 速度端口；接入速度仲裁器时设为其输入话题，例如 `lookie_teleop/cmd_vel`。

## 接口

- 输入：XR 位姿 `<xr_namespace>/*/transform` 和手柄按键 `<xr_namespace>/{left,right}_controller/buttons`；`xr_namespace` 默认 `/xr_media`。
- kstone：读取 `<kstone_namespace>/discover` 和 `<kstone_namespace>/state`，发布其中列出的位姿、关节目标，并调用 `<kstone_namespace>/<module>/home` 与 `<kstone_namespace>/home_cancel`；`kstone_namespace` 默认 `kstone`。速度目标默认直接发送到发现的速度端口；整机启动启用 `twist_mux` 时，先发到 `lookie_teleop/cmd_vel` 再由 mux 转发。
- 震动：向 `<xr_namespace>/{left,right}_controller/haptic` 发布手柄震动，用于确认长按和侧扳机接管。
- 状态：`~/status`。

## 文档

- [VR 遥操作快速上手](docs/vr_teleop_quickstart.zh-CN.md)
- [VR 遥操作设计](docs/vr_teleop_dev_design.zh-CN.md)
