# lookie_teleop

[中文](README.zh-CN.md)

VR whole-body teleoperation for Lookie: turns [xr_media_ros](https://github.com/AgilexOpensource/xr_media_ros) headset and controller input into [kstone_ros](https://github.com/AgilexOpensource/kstone_ros) arm, waist, chassis and end-effector targets.

## Install

Follow the [install guide](../docs/install.md) with [lookie_teleop.repos](lookie_teleop.repos) and the `xr_media` Python SDK.

## Launch

On the robot, use the whole-robot launch, which also starts `xr_media_ros` and applies the config of the model in `LOOKIE_MODEL` (see [lookie_bringup](../lookie_bringup/README.md#usage)):

```bash
ros2 launch lookie_bringup teleop.launch.py
```

See [lookie_bringup](../lookie_bringup/README.md#vr-teleop) for its arguments and configs. To start only the node:

```bash
ros2 launch lookie_teleop teleop.launch.py namespace:=trio params_file:=/abs/a.yaml,/abs/b.yaml
```

- `params_file`: comma-separated parameter YAMLs, applied in order; empty uses the node defaults.
- `namespace`: node namespace.
- `chassis_cmd_topic`: chassis velocity output topic; empty uses the kstone velocity port from discover. With a velocity arbiter, set its input topic, for example `lookie_teleop/cmd_vel`.

## Interfaces

- Input: XR poses `<xr_namespace>/*/transform` and controller buttons `<xr_namespace>/{left,right}_controller/buttons`; `xr_namespace` defaults to `/xr_media`.
- kstone: reads `<kstone_namespace>/discover` and `<kstone_namespace>/state`, publishes the pose and joint targets listed there, and calls `<kstone_namespace>/<module>/home` and `<kstone_namespace>/home_cancel`; `kstone_namespace` defaults to `kstone`. Velocity targets go directly to the discovered velocity port by default; the whole-robot launch with `twist_mux` sends them to `lookie_teleop/cmd_vel` for forwarding instead.
- Haptics: publishes controller pulses on `<xr_namespace>/{left,right}_controller/haptic` to confirm long presses and grip engagement.
- Status: `~/status`.

## Documentation

- [VR teleop quick start](docs/vr_teleop_quickstart.md)
- [VR teleop design](docs/vr_teleop_dev_design.md)
