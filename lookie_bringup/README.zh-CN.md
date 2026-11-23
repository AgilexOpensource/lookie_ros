# lookie_bringup

[English](README.md)

Lookie 整机启动包：驱动（可选 RViz，无真机时用关节滑块）、kstone_ros 全身控制和 VR 遥操作，均按所选机型加载配置。

## 安装

按[安装指南](../docs/install.zh-CN.md)操作，使用 [lookie_bringup.repos](lookie_bringup.repos)（列出整机需要的全部仓库），并安装全部 Python SDK。

## 使用

各 launch 文件从 `LOOKIE_MODEL` 读取机型，取值为 [`config/`](config/) 下的目录名。在 `~/.bashrc` 中设置一次：

```bash
sed -i '/^export LOOKIE_MODEL=/d' ~/.bashrc
echo 'export LOOKIE_MODEL=lookie__pf-lookie_trio_r01__ee-revo2std_r01' >> ~/.bashrc
```

| launch 文件 | 启动内容 |
| --- | --- |
| `lookie.launch.py` | 驱动、`robot_state_publisher`，可选 RViz |
| `kstone.launch.py` | kstone_ros 全身控制；可用 `ros2 topic echo /trio/kstone/state --qos-durability transient_local` 查看状态 |
| `teleop.launch.py` | VR 遥操作，见 [VR 遥操作](#vr-遥操作) |

先启用 [lookie_driver](../lookie_driver/README.zh-CN.md#can-接口) 中列出的 CAN 接口，再按上表顺序各开一个终端启动。

> **警告：** 默认的 `driver_ros.yaml`（`auto_enable: true`、`control_enabled: true`）下，`lookie.launch.py` 启动驱动时就会使能全部电机。`kstone.launch.py` 收到新鲜反馈后即开始控制机器人：先从实测位姿开始，随后双臂会在零空间调整几秒后停稳。启动前请清空机器人周围。

### lookie.launch.py

```bash
ros2 launch lookie_bringup lookie.launch.py
```

启动 `lookie_driver` 和 `lookie_description` `display.launch.py` 中的 `robot_state_publisher`，它跟随 `driver/feedback/joint_states`，不发布任何指令。`rviz:=true` 时同时打开 RViz。

没有真机时，`sim:=true` 不启动驱动，改为启动关节滑块，初值为 `initial` 位姿，Center 按钮回到该位姿。滑块发布到 `driver/feedback/joint_states`，kstone 和 RViz 会把它当作真机。仿真没有底盘里程计，需加 `fixed_frame:=base_footprint`：

```bash
ros2 launch lookie_bringup lookie.launch.py sim:=true rviz:=true fixed_frame:=base_footprint
```

> **注意：** 仿真中 kstone 的 `base` 反馈组一直不新鲜（state 中 `fresh: false`），底盘不可控；kstone 的关节命令也不会回到滑块，RViz 中的机器人只跟随滑块。查看 kstone 的命令用 [kstone_viewer](../README.zh-CN.md#可视化工具)。

启动参数：

- `model`：`config/` 下的目录名，默认取 `$LOOKIE_MODEL`；`model:=<model>` 可在单次启动时覆盖。未设置或未知机型时报错并列出可用机型。
- `config`：bringup YAML 绝对路径；为空时使用 `config/<model>/bringup.yaml`。
- `namespace`：见[命名空间](#命名空间)。
- `sim`：默认 `false`；`true` 时不启动驱动，滑块发布到反馈话题。
- `driver`：默认 `true`；`false` 时连接已运行的驱动。
- `gui`：默认 `true`；`sim:=true` 时设为 `false` 则不显示滑块，保持 `initial` 位姿。
- `rviz`：默认 `false`；`true` 时打开 RViz。`robot_state_publisher` 始终发布 TF。
- `fixed_frame`：RViz 的 Fixed Frame，不含命名空间，默认 `odom`。没有底盘里程计时用 `base_footprint`。

## 命名空间

所有 launch 都有 `namespace` 参数，为空时取 `bringup.yaml` 中的 `namespace`（`trio` 或 `neo`）。全部节点和话题都在它下面，改一个值整套系统跟着变。以 `trio` 为例：

| 部分 | 节点与话题 |
| --- | --- |
| 驱动 | `/trio/driver/{head,torso,left_arm,right_arm}/...` |
| 底盘 | `/trio/driver/chassis/{cmd_vel,odom,battery_state,rc/joy}` |
| joint bridge | `/trio/driver/feedback/joint_states`、`/trio/driver/control/joint_states` |
| kstone_ros | `/trio/kstone/...` |
| 遥操作 | `/trio/lookie_teleop` |
| XR | `/xr_media`（共用，不加命名空间） |

TF 坐标系使用前缀 `<namespace>/`。kstone 的世界坐标系是 `trio/odom`。底盘驱动广播 `trio/odom` → `trio/base_footprint`。URDF 的根是 `base_footprint`（底盘正下方的地面点），`robot_state_publisher` 从这里接续 TF 树，kstone 也直接把这份里程计当作根位姿读取。RViz 的 Fixed Frame 为 `trio/odom`，可以看到底盘运动。没有底盘驱动或其他 `odom` 来源时，`trio/odom` 不存在，RViz 无法显示机器人，此时用 `fixed_frame:=base_footprint`。

## 配置

每个机型在 `config/` 下有独立目录，目录名即机型名：

```text
config/
├── <model>/
│   ├── bringup.yaml              # 指定下列全部文件
│   ├── driver_motors.yaml        # lookie_driver：头部和躯干电机
│   ├── driver_ros.yaml           # lookie_driver：节点参数
│   ├── driver_joint_bridge.yaml  # lookie_driver：joint bridge 映射
│   ├── kstone.yaml               # lookie_control：kstone 引擎配置
│   └── teleop.yaml               # lookie_teleop：机型相关的值
├── lookie_teleop/common.yaml     # 所有机型共用
└── xr_media_ros/xr_media.yaml    # 所有机型共用
```

`bringup.yaml` 中的路径为 `package://<pkg>/<relative>`、绝对路径，或相对 `bringup.yaml` 的路径：

```yaml
namespace: trio
description:
  urdf: package://lookie_description/urdf/lookie__pf-lookie_trio_r01__ee-revo2std_r01.urdf
  collision_urdf: package://lookie_description/urdf/lookie__pf-lookie_trio_r01__ee-revo2std_r01_collision.urdf
  initial: package://lookie_description/config/lookie__pf-lookie_trio_r01__ee-revo2std_r01_initial.yaml
driver:
  robot_yaml: driver_motors.yaml
  params_file: driver_ros.yaml
  bridge_config: driver_joint_bridge.yaml
teleop:  # 见 VR 遥操作
  ...
kstone:
  config: kstone.yaml
```

启动任何节点前会对照 URDF 校验驱动配置。

新增机型或 URDF 修订时，把 URDF 加入 `lookie_description`，再新建 `config/<model>/` 目录并重新编译即可，代码中没有机型列表。

## VR 遥操作

```bash
ros2 launch lookie_bringup teleop.launch.py
```

启动 `xr_media_ros`（只有 XR 卡片，不配置 media）与 `lookie_teleop`，不启动 `kstone_ros`，需先用 `kstone.launch.py` 启动。

参数：`model`、`config`、`namespace`（同上），`xr`、`teleop`（默认 `true`，`false` 时不启动对应节点）。

`bringup.yaml` 的 `teleop` 段先指定 `config/xr_media_ros/` 与 `config/lookie_teleop/` 下的共用文件，再指定本机型的 `teleop.yaml`：

```yaml
teleop:
  xr_media: package://lookie_bringup/config/xr_media_ros/xr_media.yaml
  lookie_teleop:  # 按顺序加载，后者覆盖前者
    - package://lookie_bringup/config/lookie_teleop/common.yaml
    - teleop.yaml
```

`teleop.yaml` 存放机型相关的值：`body_axes`（头显可驱动的轴）与末端执行器（`ee_modules`、`ee_open`、`ee_close`）。
