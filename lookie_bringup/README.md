# lookie_bringup

Combined bringup for the Lookie robot. Currently starts the drivers together with RViz and optional joint slider control.

## Quick start

Clone the [lookie_ros](../README.md#install) umbrella into `~/lookie_ws/src`; it contains this package plus `lookie_control` (with `kstone`), `lookie_description` and `lookie_driver`.

1. Build:

   ```bash
   cd ~/lookie_ws
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-up-to lookie_bringup kstone_ros --cmake-args \
     -DCMAKE_BUILD_TYPE=Release -DKSTONE_PINOCCHIO_SOURCE=SYSTEM \
     -DKSTONE_BUILD_PYTHON=OFF -DKSTONE_BUILD_TESTS=OFF \
     -DKSTONE_BUILD_TOOLS=OFF -DKSTONE_BUILD_EXAMPLES=OFF
   ```

2. Bring up the CAN interfaces listed in [driver_bringup](../lookie_driver/driver_bringup/README.md#can-interfaces), including `can_chassis`.

3. Select the model once; the line in `~/.bashrc` applies to every new terminal:

   ```bash
   echo 'source ~/lookie_ws/install/setup.bash' >> ~/.bashrc
   echo 'export MODEL=lookie__pf-lookie_trio_r01__ee-revo2std_r01' >> ~/.bashrc
   source ~/.bashrc
   ```

4. Terminal 1, drivers and RViz:

   ```bash
   ros2 launch lookie_bringup lookie.launch.py model:=$MODEL
   ```

5. Terminal 2, kstone_ros:

   ```bash
   ros2 launch lookie_bringup kstone.launch.py model:=$MODEL
   ```

   kstone holds the current pose once feedback arrives. Check it with `ros2 topic echo /trio/kstone/state --qos-durability transient_local`.

6. Terminal 3, VR teleop:

   ```bash
   ros2 launch lookie_bringup teleop.launch.py model:=$MODEL
   ```

Do not run step 5 together with `control:=true` in step 4; both publish to `control/joint_states`. Keep an emergency stop within reach.

## Usage

```bash
ros2 launch lookie_bringup lookie.launch.py \
  model:=lookie__pf-lookie_trio_r01__ee-revo2std_r01
```

This starts `driver_bringup` and `lookie_description` `display.launch.py` with `follow:=true`. RViz follows `driver/feedback/joint_states`. With `control:=true`, joint sliders start at the `initial` pose and publish to `driver/control/joint_states` immediately.

Launch arguments:

- `model`: one of the four driver models; defaults to `lookie__pf-lookie_trio_r01__ee-revo2std_r01`.
- `config`: absolute bringup YAML path; empty selects `config/<model>.yaml`.
- `namespace`: see [Namespaces](#namespaces).
- `driver`: defaults to `true`; `false` attaches to drivers that are already running.
- `control`: defaults to `false`. `true` starts joint sliders.
- `gui`: defaults to `true`; `false` uses `joint_state_publisher` without sliders.
- `rviz`: defaults to `true`; `false` skips RViz, and `robot_state_publisher` still publishes TF.
- `fixed_frame`: RViz Fixed Frame without the namespace; defaults to `odom`. Use `base_footprint` when no chassis odometry is running.

## Namespaces

Every launch takes `namespace`; empty selects the platform (`trio` or `neo`). All nodes and topics live under it, so one value moves the whole system. With `trio`:

| Part | Nodes and topics |
| --- | --- |
| Drivers | `/trio/driver/{head,torso,left_arm,right_arm}/...` |
| Chassis | `/trio/driver/chassis/{odom,cmd_vel,scout_status,rc_status,light_control}` |
| Joint bridge | `/trio/driver/feedback/joint_states`, `/trio/driver/control/joint_states` |
| kstone_ros | `/trio/kstone/...` |
| Teleop | `/trio/lookie_wbc_teleop` |
| XR | `/xr_media` (shared, not namespaced) |

TF frames use the prefix `<namespace>/`. kstone uses `trio/odom` as its world frame. The chassis driver broadcasts `trio/odom` → `trio/base_footprint`. The URDF root is `base_footprint` (the ground point under the chassis), so `robot_state_publisher` continues the tree from there, and kstone reads that odometry directly as its root pose. RViz uses Fixed Frame `trio/odom` to show chassis motion. Without a chassis driver or another `odom` source, `trio/odom` does not exist and RViz cannot place the robot; use `fixed_frame:=base_footprint` in that case.

## Configuration

`config/<model>.yaml` selects the config files for every package. Paths are `package://<pkg>/<relative>` or absolute:

```yaml
model: lookie__pf-lookie_trio_r01__ee-revo2std_r01
description:
  initial: package://lookie_description/config/lookie__pf-lookie_trio_r01__ee-revo2std_r01_initial.yaml
driver:
  robot_yaml: package://driver_bringup/config/lookie__pf-lookie_trio_r01_motors.yaml
  params_file: package://driver_bringup/config/lookie__pf-lookie_trio_r01__ee-revo2std_r01_ros.yaml
  bridge_config: package://driver_bringup/config/lookie__pf-lookie_trio_r01__ee-revo2std_r01_joint_bridge.yaml
teleop:  # see VR teleop
  ...
kstone:
  config: package://lookie_bringup/config/kstone_ros/lookie__pf-lookie_trio_r01__ee-revo2std_r01.yaml
```

Driver configs are checked against the URDF before any node starts.

## VR teleop

```bash
ros2 launch lookie_bringup teleop.launch.py \
  model:=lookie__pf-lookie_trio_r01__ee-revo2std_r01
```

This starts `xr_media_ros` (XR card only, no media) and `lookie_wbc_teleop`. It does not start `kstone_ros`; start it as in [Quick start](#quick-start) step 5.

Arguments: `model`, `config`, `namespace` (same as above), `xr` and `wbc` (default `true`; `false` skips that node).

The `teleop` section of `config/<model>.yaml` selects the files under `config/xr_media_ros/` and `config/lookie_wbc_teleop/`:

```yaml
teleop:
  xr_media: package://lookie_bringup/config/xr_media_ros/xr_media.yaml
  wbc:  # applied in order, later files override earlier ones
    - package://lookie_bringup/config/lookie_wbc_teleop/common.yaml
    - package://lookie_bringup/config/lookie_wbc_teleop/<model>.yaml
```

`lookie_wbc_teleop/<model>.yaml` holds the model-specific values: `body_axes` (axes the headset may drive) and the end effector (`ee_modules`, `ee_open`, `ee_close`).

With `control:=true`, the robot moves from its current pose to the `initial` pose as soon as the drivers accept commands, and the Center button returns to that pose. Sliders send position commands directly. Keep an emergency stop within reach.
