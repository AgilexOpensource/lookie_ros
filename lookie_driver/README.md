# lookie_driver

[中文](README.zh-CN.md)

Starts the Lookie head, torso, dual arms, chassis and joint bridge from explicit config files.

## Install

Follow the [install guide](../docs/install.md) with [lookie_driver.repos](lookie_driver.repos) and the Python SDKs pyAgxArm (arms), agx_motor_sdk (head, lift, fold) and bc-stark-sdk (Revo2 hands only). It runs these driver packages; see their READMEs for safety notes:

| Package | Repository |
| --- | --- |
| head_ros | [head_ros](https://github.com/AgilexOpensource/head_ros): 2-DOF head |
| lift_ros | [lift_ros](https://github.com/AgilexOpensource/lift_ros): lift column and waist pitch (Neo) |
| fold_ros | [fold_ros](https://github.com/AgilexOpensource/fold_ros): 4-DOF fold module (Trio) |
| agx_arm_ctrl | [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2): AgileX arms |
| lookie_neo_base | [lookie_neo_base](https://github.com/AgilexOpensource/lookie_neo_base) and [ugv_sdk](https://github.com/westonrobot/ugv_sdk): differential chassis (Neo) |
| lookie_trio_base | [lookie_trio_base](https://github.com/AgilexOpensource/lookie_trio_base) and ugv_sdk: omnidirectional chassis (Trio) |

## Models

| Platform | Torso | Chassis | Models (`ee-agx_gripper_r01` / `ee-revo2std_r01`) |
| --- | --- | --- | --- |
| Neo | lift_ros | lookie_neo_base, differential | `lookie__pf-lookie_neo_r01__ee-<ee>` |
| Trio | fold_ros | lookie_trio_base, omni wheel | `lookie__pf-lookie_trio_r01__ee-<ee>` |

This package holds no model list or config files. Each model's driver configs live in [lookie_bringup](../lookie_bringup/README.md#configuration) `config/<model>/`. The torso driver comes from the top-level key of the motors YAML other than `head`: `lift` starts `lift_ros`, `fold` starts `fold_ros`.

## CAN interfaces

Bring up the CAN interfaces before launching:

```bash
ros2 run lookie_bringup can_bind.sh
```

Interface names, bitrates and use on other computers are in the [CAN guide](../lookie_bringup/docs/can.md).

## Launch

The usual way is [lookie_bringup](../lookie_bringup/README.md), which reads `LOOKIE_MODEL` and passes this model's files:

```bash
ros2 launch lookie_bringup lookie.launch.py rviz:=false
```

To start only the drivers, pass the files of `$LOOKIE_MODEL` directly. The example uses Trio; for Neo change `namespace:=trio` to `namespace:=neo`. The joint-bridge-only launch defaults to the driver namespace in its YAML:

```bash
CFG=$(ros2 pkg prefix lookie_bringup)/share/lookie_bringup/config/$LOOKIE_MODEL
URDF=$(ros2 pkg prefix lookie_description)/share/lookie_description/urdf/$LOOKIE_MODEL.urdf
ros2 launch lookie_driver drivers.launch.py namespace:=trio \
  robot_yaml:=$CFG/driver_motors.yaml params_file:=$CFG/driver_ros.yaml \
  bridge_config:=$CFG/driver_joint_bridge.yaml urdf:=$URDF

# Joint bridge only
ros2 launch lookie_driver joint_bridge.launch.py bridge_config:=$CFG/driver_joint_bridge.yaml
```

`drivers.launch.py` arguments, all required, paths absolute:

| Argument | Content |
| --- | --- |
| `namespace` | Robot namespace; drivers run under `<namespace>/driver` |
| `robot_yaml` | Head and torso motors |
| `params_file` | Head, torso, arm and chassis node parameters |
| `bridge_config` | Joint bridge mapping |
| `urdf` | URDF used to check joint limits and coverage |

The files are checked against each other and the URDF before any node starts.

## Topics

With `trio`, the joint bridge merges module joint states into `/trio/driver/feedback/joint_states` and accepts commands on `/trio/driver/control/joint_states`. Joint names follow the Lookie URDF; Revo2 mimic joints (`*_distal`) are not bridged.

The chassis node runs in `/trio/driver/chassis`. Its package is the one named by the `/**/chassis/<package>` section of `driver_ros.yaml`: `lookie_neo_base` on Neo, `lookie_trio_base` on Trio. It subscribes to `cmd_vel`, publishes `odom`, `battery_state` and `rc/joy` (plus `/diagnostics`), and broadcasts TF `trio/odom` → `trio/base_footprint`. The package READMEs linked in the install table describe velocity signs and other chassis parameters.

## Configuration

| File in `lookie_bringup/config/<model>/` | Content |
| --- | --- |
| `driver_motors.yaml` | Head and torso motors |
| `driver_ros.yaml` | Head, torso, arm and chassis node parameters (joint feedback: 50 Hz) |
| `driver_joint_bridge.yaml` | Namespace, topics, rate (50 Hz) and per-module joint mapping |

Head, fold and arm diagnostics default to 30 Hz independently of the 50 Hz joint feedback configuration. This republishes cached status, not new hardware samples; chassis publication rates are unchanged.
