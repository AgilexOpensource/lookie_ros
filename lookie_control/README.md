# lookie_control

Runs [kstone_ros](https://github.com/AgilexOpensource/kstone_ros) whole-body control from an explicit engine config, URDF, namespace and topics. It holds no model list or config files. [中文](README.zh-CN.md)

## Install

Follow the [install guide](../docs/install.md) with [lookie_control.repos](lookie_control.repos). No Python SDK is needed.

## Launch

The usual way is [lookie_bringup](../lookie_bringup/README.md), which reads `LOOKIE_MODEL` and passes this model's `kstone.yaml`, collision URDF and the [lookie_driver](../lookie_driver/README.md) topics:

```bash
ros2 launch lookie_bringup kstone.launch.py
```

To start it directly with the files of `$LOOKIE_MODEL`:

```bash
ros2 launch lookie_control kstone.launch.py namespace:=trio \
  config_file:=$(ros2 pkg prefix lookie_bringup)/share/lookie_bringup/config/$LOOKIE_MODEL/kstone.yaml \
  urdf:=$(ros2 pkg prefix lookie_description)/share/lookie_description/urdf/${LOOKIE_MODEL}_collision.urdf
```

The node runs as `/<namespace>/kstone`. Topics are relative to the robot namespace.

Arguments:

- `namespace`: robot namespace, required.
- `config_file`: absolute kstone engine config path, required.
- `urdf`: absolute URDF path, required. It fills `${urdf}` in the engine config.
- `package_dirs`: fills `${package_dirs}` in the engine config; empty uses the share directory that contains the URDF package.
- `feedback_topic`, `control_topic`: joint feedback and command topics; default `driver/feedback/joint_states` and `driver/control/joint_states`.
- `base_command_topic`, `odom_topic`: chassis velocity command and odometry; default `driver/chassis/cmd_vel` and `driver/chassis/odom`.
- `realtime`: `inherit` (default) keeps the thread settings of the config; `true` or `false` overrides it.
- `debug_port`: kstone debug server port; `inherit` keeps the config setting, `0` turns it off.
- `use_sim_time`: defaults to `false`.

TF frames use the prefix `<namespace>/`. The world frame is `odom` and the root link is `base_footprint`.

## Configuration

The engine config of each model is `kstone.yaml` in [lookie_bringup](../lookie_bringup/README.md#configuration) `config/<model>/`: control rate, feedback timeout, realtime thread, solver, robot description and modules. See the [kstone_ros README](https://github.com/AgilexOpensource/kstone_ros) for the format and the topics and services of the node.
