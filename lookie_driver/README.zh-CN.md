# lookie_driver

[English](README.md)

按给定的配置文件启动 Lookie 的头部、躯干、双臂、底盘和关节桥。

## 安装

按[安装指南](../docs/install.zh-CN.md)操作，使用 [lookie_driver.repos](lookie_driver.repos)，并安装 Python SDK：pyAgxArm（机械臂）、agx_motor_sdk（头部、升降、折叠）和 bc-stark-sdk（仅 Revo2 灵巧手）。本包会启动以下驱动包，安全注意事项见各自的 README：

| 功能包 | 仓库 |
| --- | --- |
| head_ros | [head_ros](https://github.com/AgilexOpensource/head_ros)：2 自由度头部 |
| lift_ros | [lift_ros](https://github.com/AgilexOpensource/lift_ros)：升降柱和腰部俯仰（Neo） |
| fold_ros | [fold_ros](https://github.com/AgilexOpensource/fold_ros)：4 自由度折叠模组（Trio） |
| agx_arm_ctrl | [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2)：松灵机械臂 |
| lookie_neo_base | [lookie_neo_base](https://github.com/AgilexOpensource/lookie_neo_base) 和 [ugv_sdk](https://github.com/westonrobot/ugv_sdk)：差速底盘（Neo） |
| lookie_trio_base | [lookie_trio_base](https://github.com/AgilexOpensource/lookie_trio_base) 和 [ugv_sdk](https://github.com/westonrobot/ugv_sdk)：全向底盘（Trio） |

## 机型

| 平台 | 躯干 | 底盘 | 机型（`ee-agx_gripper_r01` / `ee-revo2std_r01`） |
| --- | --- | --- | --- |
| Neo | lift_ros | lookie_neo_base，差速 | `lookie__pf-lookie_neo_r01__ee-<ee>` |
| Trio | fold_ros | lookie_trio_base，全向轮 | `lookie__pf-lookie_trio_r01__ee-<ee>` |

本包不含机型列表和配置文件。各机型的驱动配置位于 [lookie_bringup](../lookie_bringup/README.zh-CN.md#配置) 的 `config/<model>/`。躯干驱动由电机 YAML 中除 `head` 外的顶层键决定：`lift` 启动 `lift_ros`，`fold` 启动 `fold_ros`。

## CAN 接口

启动前先激活 CAN 接口：

```bash
ros2 run lookie_bringup can_bind.sh
```

接口名、波特率以及在其他电脑上的用法见 [CAN 指南](../lookie_bringup/docs/can.zh-CN.md)。

## 启动

通常通过 [lookie_bringup](../lookie_bringup/README.zh-CN.md) 启动，它读取 `LOOKIE_MODEL` 并传入该机型的文件：

```bash
ros2 launch lookie_bringup lookie.launch.py rviz:=false
```

只启动驱动时，直接传入 `$LOOKIE_MODEL` 的文件：

```bash
CFG=$(ros2 pkg prefix lookie_bringup)/share/lookie_bringup/config/$LOOKIE_MODEL
URDF=$(ros2 pkg prefix lookie_description)/share/lookie_description/urdf/$LOOKIE_MODEL.urdf
ros2 launch lookie_driver drivers.launch.py namespace:=trio \
  robot_yaml:=$CFG/driver_motors.yaml params_file:=$CFG/driver_ros.yaml \
  bridge_config:=$CFG/driver_joint_bridge.yaml urdf:=$URDF

# 只启动关节桥
ros2 launch lookie_driver joint_bridge.launch.py bridge_config:=$CFG/driver_joint_bridge.yaml
```

`drivers.launch.py` 参数均为必填，路径为绝对路径：

| 参数 | 内容 |
| --- | --- |
| `namespace` | 机器人命名空间，驱动运行在 `<namespace>/driver` 下 |
| `robot_yaml` | 头部和躯干电机 |
| `params_file` | 头部、躯干、机械臂和底盘节点参数 |
| `bridge_config` | 关节桥映射 |
| `urdf` | 用于校验关节限位和覆盖范围的 URDF |

任何节点启动前都会检查这些文件彼此之间以及与 URDF 是否一致。

## 话题

以 `trio` 为例，关节桥把各模组的关节状态合并发布到 `/trio/driver/feedback/joint_states`，并在 `/trio/driver/control/joint_states` 接收命令。关节名与 Lookie URDF 一致；Revo2 的 mimic 关节（`*_distal`）不经过关节桥。

底盘节点运行在 `/trio/driver/chassis` 下，所用的包由 `driver_ros.yaml` 中的 `/**/chassis/<package>` 段指定：Neo 为 [lookie_neo_base](https://github.com/AgilexOpensource/lookie_neo_base)，Trio 为 [lookie_trio_base](https://github.com/AgilexOpensource/lookie_trio_base)。它订阅 `cmd_vel`，发布 `odom`、`battery_state` 和 `rc/joy`（以及 `/diagnostics`），并广播 TF `trio/odom` → `trio/base_footprint`。速度符号等底盘参数见各包 README。

## 配置

| `lookie_bringup/config/<model>/` 中的文件 | 内容 |
| --- | --- |
| `driver_motors.yaml` | 头部和躯干电机 |
| `driver_ros.yaml` | 头部、躯干、机械臂和底盘节点参数（关节反馈：50 Hz） |
| `driver_joint_bridge.yaml` | 命名空间、话题、频率（50 Hz）和各模组关节映射 |

头部、折叠躯干和机械臂诊断默认按 30 Hz 发布，与 50 Hz 关节反馈配置独立。这里只重复发布缓存状态，不产生新的硬件采样；底盘发布频率保持不变。
