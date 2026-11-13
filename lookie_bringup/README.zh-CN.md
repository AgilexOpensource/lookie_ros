# lookie_bringup

Lookie 机器人组合启动包。目前提供驱动与 RViz 的组合启动，可选关节滑块控制。

## 快速使用

将 [lookie_ros](../README.zh-CN.md#安装) 总仓克隆到 `~/lookie_ws/src`；它包含本包以及 `lookie_control`（含 `kstone`）、`lookie_description` 和 `lookie_driver`。

1. 编译：

   ```bash
   cd ~/lookie_ws
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-up-to lookie_bringup kstone_ros --cmake-args \
     -DCMAKE_BUILD_TYPE=Release -DKSTONE_PINOCCHIO_SOURCE=SYSTEM \
     -DKSTONE_BUILD_PYTHON=OFF -DKSTONE_BUILD_TESTS=OFF \
     -DKSTONE_BUILD_TOOLS=OFF -DKSTONE_BUILD_EXAMPLES=OFF
   ```

2. 按 [driver_bringup](../lookie_driver/driver_bringup/README.md#can-interfaces) 中的列表启用 CAN 接口，包括 `can_chassis`。

3. 选定机型，只需一次；写入 `~/.bashrc` 后每个新终端自动生效：

   ```bash
   echo 'source ~/lookie_ws/install/setup.bash' >> ~/.bashrc
   echo 'export MODEL=lookie__pf-lookie_trio_r01__ee-revo2std_r01' >> ~/.bashrc
   source ~/.bashrc
   ```

4. 终端 1，驱动与 RViz：

   ```bash
   ros2 launch lookie_bringup lookie.launch.py model:=$MODEL
   ```

5. 终端 2，kstone_ros：

   ```bash
   ros2 launch lookie_bringup kstone.launch.py model:=$MODEL
   ```

   收到反馈后 kstone 保持当前位姿。可用 `ros2 topic echo /trio/kstone/state --qos-durability transient_local` 查看状态。

6. 终端 3，VR 遥操：

   ```bash
   ros2 launch lookie_bringup teleop.launch.py model:=$MODEL
   ```

第 5 步不要与第 4 步的 `control:=true` 同时使用，两者都会发布到 `control/joint_states`。请确保急停在手边。

## 使用

```bash
ros2 launch lookie_bringup lookie.launch.py \
  model:=lookie__pf-lookie_trio_r01__ee-revo2std_r01
```

启动 `driver_bringup` 与 `lookie_description` 的 `display.launch.py`（`follow:=true`）。RViz 跟随 `driver/feedback/joint_states`。`control:=true` 时立即启动关节滑块，初值为 `initial` 位姿，发布到 `driver/control/joint_states`。

启动参数：

- `model`：四个驱动型号之一，默认 `lookie__pf-lookie_trio_r01__ee-revo2std_r01`。
- `config`：bringup YAML 绝对路径；为空时使用 `config/<model>.yaml`。
- `namespace`：见[命名空间](#命名空间)。
- `driver`：默认 `true`；`false` 时连接已运行的驱动。
- `control`：默认 `false`；`true` 时启动关节滑块。
- `gui`：默认 `true`；`false` 时使用无滑块的 `joint_state_publisher`。
- `rviz`：默认 `true`；`false` 时不启动 RViz，`robot_state_publisher` 仍发布 TF。
- `fixed_frame`：RViz 的 Fixed Frame，不含命名空间，默认 `odom`。没有底盘里程计时用 `base_footprint`。

## 命名空间

所有 launch 都有 `namespace` 参数，为空时取平台名（`trio` 或 `neo`）。全部节点和话题都在它下面，改一个值整套系统跟着变。以 `trio` 为例：

| 部分 | 节点与话题 |
| --- | --- |
| 驱动 | `/trio/driver/{head,torso,left_arm,right_arm}/...` |
| 底盘 | `/trio/driver/chassis/{odom,cmd_vel,scout_status,rc_status,light_control}` |
| joint bridge | `/trio/driver/feedback/joint_states`、`/trio/driver/control/joint_states` |
| kstone_ros | `/trio/kstone/...` |
| 遥操 | `/trio/lookie_wbc_teleop` |
| XR | `/xr_media`（共用，不加命名空间） |

TF 坐标系使用前缀 `<namespace>/`。kstone 的世界坐标系是 `trio/odom`。底盘驱动广播 `trio/odom` → `trio/base_footprint`。URDF 的根是 `base_footprint`（底盘正下方的地面点），`robot_state_publisher` 从这里接续 TF 树，kstone 也直接把这份里程计当作根位姿读取。RViz 的 Fixed Frame 为 `trio/odom`，可以看到底盘运动。没有底盘驱动或其他 `odom` 来源时，`trio/odom` 不存在，RViz 无法显示机器人，此时用 `fixed_frame:=base_footprint`。

## 配置

`config/<model>.yaml` 指定各包的配置文件，路径为 `package://<pkg>/<relative>` 或绝对路径：

```yaml
model: lookie__pf-lookie_trio_r01__ee-revo2std_r01
description:
  initial: package://lookie_description/config/lookie__pf-lookie_trio_r01__ee-revo2std_r01_initial.yaml
driver:
  robot_yaml: package://driver_bringup/config/lookie__pf-lookie_trio_r01_motors.yaml
  params_file: package://driver_bringup/config/lookie__pf-lookie_trio_r01__ee-revo2std_r01_ros.yaml
  bridge_config: package://driver_bringup/config/lookie__pf-lookie_trio_r01__ee-revo2std_r01_joint_bridge.yaml
teleop:  # 见 VR 遥操
  ...
kstone:
  config: package://lookie_bringup/config/kstone_ros/lookie__pf-lookie_trio_r01__ee-revo2std_r01.yaml
```

启动任何节点前会对照 URDF 校验驱动配置。

## VR 遥操

```bash
ros2 launch lookie_bringup teleop.launch.py \
  model:=lookie__pf-lookie_trio_r01__ee-revo2std_r01
```

启动 `xr_media_ros`（只有 XR 卡片，不配置 media）与 `lookie_wbc_teleop`，不启动 `kstone_ros`，需按[快速使用](#快速使用)第 5 步另行启动。

参数：`model`、`config`、`namespace`（同上），`xr`、`wbc`（默认 `true`，`false` 时不启动对应节点）。

`config/<model>.yaml` 的 `teleop` 段指定 `config/xr_media_ros/` 与 `config/lookie_wbc_teleop/` 下的文件：

```yaml
teleop:
  xr_media: package://lookie_bringup/config/xr_media_ros/xr_media.yaml
  wbc:  # 按顺序加载，后者覆盖前者
    - package://lookie_bringup/config/lookie_wbc_teleop/common.yaml
    - package://lookie_bringup/config/lookie_wbc_teleop/<model>.yaml
```

`lookie_wbc_teleop/<model>.yaml` 存放机型相关的值：`body_axes`（头盔可驱动的轴）与末端执行器（`ee_modules`、`ee_open`、`ee_close`）。

`control:=true` 时，驱动一接受指令，机器人就会从当前位姿运动到 `initial` 位姿，Center 按钮也回到该位姿。滑块直接下发位置指令，请确保急停在手边。
