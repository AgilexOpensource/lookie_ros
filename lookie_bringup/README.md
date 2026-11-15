# lookie_bringup

Whole-robot launch files for Lookie: drivers with optional RViz, joint sliders without a robot, kstone_ros whole-body control and VR teleop, each using the config of the selected model. [中文](README.zh-CN.md)

## Install

Follow the [install guide](../docs/install.md) with [lookie_bringup.repos](lookie_bringup.repos), which lists every repository the robot needs, and all Python SDKs.

## Usage

Each launch file reads the model from `LOOKIE_MODEL`, a directory under [`config/`](config/). Set it once in `~/.bashrc`:

```bash
sed -i '/^export LOOKIE_MODEL=/d' ~/.bashrc
echo 'export LOOKIE_MODEL=lookie__pf-lookie_trio_r01__ee-revo2std_r01' >> ~/.bashrc
```

| Launch file | Starts |
| --- | --- |
| `lookie.launch.py` | Drivers, `robot_state_publisher` and optionally RViz |
| `kstone.launch.py` | kstone_ros whole-body control; check it with `ros2 topic echo /trio/kstone/state --qos-durability transient_local` |
| `teleop.launch.py` | VR teleop, see [VR teleop](#vr-teleop) |

Start them in this order, one per terminal, after the CAN interfaces listed in [lookie_driver](../lookie_driver/README.md#can-interfaces) are up.

> [!WARNING]
> With the default `driver_ros.yaml` (`auto_enable: true`, `control_enabled: true`), `lookie.launch.py` enables every motor as soon as the drivers start. `kstone.launch.py` drives the robot as soon as it receives fresh feedback: it starts from the measured pose, then the arms settle for a few seconds (null space). Keep the area around the robot clear before starting either.

### lookie.launch.py

```bash
ros2 launch lookie_bringup lookie.launch.py
```

This starts `lookie_driver` and `robot_state_publisher` from `lookie_description` `display.launch.py`, which follows `driver/feedback/joint_states` and never publishes commands. `rviz:=true` also opens RViz.

Without a robot, `sim:=true` skips the drivers and starts joint sliders at the `initial` pose. The sliders publish `driver/feedback/joint_states`, so kstone and RViz see them as the robot:

```bash
ros2 launch lookie_bringup lookie.launch.py sim:=true rviz:=true
```

Launch arguments:

- `model`: a directory under `config/`; defaults to `$LOOKIE_MODEL`. `model:=<model>` overrides it for one launch. An unset or unknown model fails with the list of available models.
- `config`: absolute bringup YAML path; empty selects `config/<model>/bringup.yaml`.
- `namespace`: see [Namespaces](#namespaces).
- `sim`: defaults to `false`. `true` skips the drivers and starts joint sliders on the feedback topic.
- `driver`: defaults to `true`; `false` attaches to drivers that are already running.
- `gui`: defaults to `true`. With `sim:=true`, `false` holds the `initial` pose without sliders.
- `rviz`: defaults to `false`; `true` opens RViz. `robot_state_publisher` publishes TF either way.
- `fixed_frame`: RViz Fixed Frame without the namespace; defaults to `odom`. Use `base_footprint` when no chassis odometry is running.

## Namespaces

Every launch takes `namespace`; empty uses `namespace` from `bringup.yaml` (`trio` or `neo`). All nodes and topics live under it, so one value moves the whole system. With `trio`:

| Part | Nodes and topics |
| --- | --- |
| Drivers | `/trio/driver/{head,torso,left_arm,right_arm}/...` |
| Chassis | `/trio/driver/chassis/{odom,cmd_vel,scout_status,rc_status,light_control}` |
| Joint bridge | `/trio/driver/feedback/joint_states`, `/trio/driver/control/joint_states` |
| kstone_ros | `/trio/kstone/...` |
| Teleop | `/trio/lookie_teleop` |
| XR | `/xr_media` (shared, not namespaced) |

TF frames use the prefix `<namespace>/`. kstone uses `trio/odom` as its world frame. The chassis driver broadcasts `trio/odom` → `trio/base_footprint`. The URDF root is `base_footprint` (the ground point under the chassis), so `robot_state_publisher` continues the tree from there, and kstone reads that odometry directly as its root pose. RViz uses Fixed Frame `trio/odom` to show chassis motion. Without a chassis driver or another `odom` source, `trio/odom` does not exist and RViz cannot place the robot; use `fixed_frame:=base_footprint` in that case.

## Configuration

Each model has its own directory under `config/`. The directory name is the model name:

```text
config/
├── <model>/
│   ├── bringup.yaml              # selects every file below
│   ├── driver_motors.yaml        # lookie_driver: head and torso motors
│   ├── driver_ros.yaml           # lookie_driver: node parameters
│   ├── driver_joint_bridge.yaml  # lookie_driver: joint bridge mapping
│   ├── kstone.yaml               # lookie_control: kstone engine config
│   └── teleop.yaml               # lookie_teleop: model-specific values
├── lookie_teleop/common.yaml     # shared by all models
└── xr_media_ros/xr_media.yaml    # shared by all models
```

`bringup.yaml` paths are `package://<pkg>/<relative>`, absolute, or relative to `bringup.yaml`:

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
teleop:  # see VR teleop
  ...
kstone:
  config: kstone.yaml
```

Driver configs are checked against the URDF before any node starts.

To add a model or a URDF revision, add the URDF to `lookie_description` and a new `config/<model>/` directory, then rebuild. No code lists the models.

## VR teleop

```bash
ros2 launch lookie_bringup teleop.launch.py
```

This starts `xr_media_ros` (XR card only, no media) and `lookie_teleop`. It does not start `kstone_ros`; start it with `kstone.launch.py` first.

Arguments: `model`, `config`, `namespace` (same as above), `xr` and `teleop` (default `true`; `false` skips that node).

The `teleop` section of `bringup.yaml` selects the shared files under `config/xr_media_ros/` and `config/lookie_teleop/`, then `teleop.yaml` of the model:

```yaml
teleop:
  xr_media: package://lookie_bringup/config/xr_media_ros/xr_media.yaml
  lookie_teleop:  # applied in order, later files override earlier ones
    - package://lookie_bringup/config/lookie_teleop/common.yaml
    - teleop.yaml
```

`teleop.yaml` holds the model-specific values: `body_axes` (axes the headset may drive) and the end effector (`ee_modules`, `ee_open`, `ee_close`).

With `sim:=true`, the sliders start at the `initial` pose and the Center button returns to it.
