# lookie_ros

[中文](README.zh-CN.md)

ROS 2 packages for the Lookie robots (Trio and Neo): drivers, whole-body control with kstone, RViz and VR teleoperation.

## Packages

| Package | Description |
| --- | --- |
| [lookie_bringup](lookie_bringup/README.md) | Whole-robot launch files and per-model configs for drivers, kstone_ros and teleop |
| [lookie_driver](lookie_driver/README.md) | Head, torso, arm and chassis drivers, and the joint bridge |
| [lookie_control](lookie_control/README.md) | Launch file that runs kstone_ros from a given config, URDF and topics; no configs |
| [lookie_teleop](lookie_teleop/README.md) | VR whole-body teleop: turns xr_media_ros input into kstone_ros targets |

Dependency repositories are cloned next to this one with [vcstool](https://github.com/dirk-thomas/vcstool):

| Repository | Description | Used by |
| --- | --- | --- |
| [kstone](https://github.com/AgilexOpensource/kstone) | C++ whole-body IK engine | lookie_control |
| [kstone_ros](https://github.com/AgilexOpensource/kstone_ros) | ROS 2 node for kstone | lookie_control |
| [lookie_description](https://github.com/AgilexOpensource/lookie_description) | URDF, meshes and RViz display | lookie_control, lookie_bringup |
| [xr_media_ros](https://github.com/AgilexOpensource/xr_media_ros) | XR headset tracking and media streaming | lookie_teleop |
| [head_ros](https://github.com/AgilexOpensource/head_ros), [lift_ros](https://github.com/AgilexOpensource/lift_ros), [fold_ros](https://github.com/AgilexOpensource/fold_ros), [agx_arm_ros](https://github.com/agilexrobotics/agx_arm_ros/tree/ros2), [scout_ros2](https://github.com/AgilexOpensource/scout_ros2/tree/humble), [ugv_sdk](https://github.com/westonrobot/ugv_sdk) | Head, torso, arm and chassis drivers | lookie_driver |

## Requirements

Tested on Ubuntu 22.04 with ROS 2 Humble. Older releases are not supported; newer ones are untested.

The robot needs the CAN interfaces listed in the [CAN guide](lookie_bringup/docs/can.md#interfaces).

## Install

- [Setup script](docs/setup.md): interactive, for quick deployment such as production test stations.
- [Manual install](docs/install.md): the same steps by hand, also for a single package.

## Quick start

The setup script already covers step 1.

1. Choose the model once. Each folder in [lookie_bringup/config](lookie_bringup/config) is one model:

   ```bash
   sed -i '/^export LOOKIE_MODEL=/d' ~/.bashrc
   echo 'export LOOKIE_MODEL=lookie__pf-lookie_trio_r01__ee-revo2std_r01' >> ~/.bashrc
   source ~/.bashrc
   ```

2. Bring up the CAN interfaces. On another computer, set the port mapping first as described in the [CAN guide](lookie_bringup/docs/can.md):

   ```bash
   ros2 run lookie_bringup can_bind.sh
   ```

3. Terminal 1, drivers:

   ```bash
   ros2 launch lookie_bringup lookie.launch.py
   ```

   Add `rviz:=true` to open RViz. Without a robot, add `sim:=true`: joint sliders replace the drivers and nothing is sent to hardware.

4. Terminal 2, whole-body control. kstone holds the current pose once feedback arrives:

   ```bash
   ros2 launch lookie_bringup kstone.launch.py
   ```

5. Terminal 3, VR teleop:

   ```bash
   ros2 launch lookie_bringup teleop.launch.py
   ```

Keep an emergency stop within reach.

## VR teleop

1. Create the HTTPS certificate once with `xr_media --init-certs`. Each run overwrites the existing certificate.
2. Start step 5 above. The log prints `Open in headset browser: https://<robot-ip>:9443/...`.
3. Open that URL in the headset browser, trust the certificate and enter VR.

The [VR teleop quickstart](lookie_teleop/docs/vr_teleop_quickstart.md) covers the controller buttons, calibration and safety.

## Viewer

[kstone_viewer](https://github.com/AgilexOpensource/kstone_viewer) shows the kstone state, targets and solve quality in a browser:

```bash
python3 -m venv ~/.venvs/kstone-viewer
~/.venvs/kstone-viewer/bin/pip install "git+https://github.com/AgilexOpensource/kstone_viewer.git"
```

It needs no ROS, so it has its own venv; the workspace venv pins `numpy<2`, which the viewer would upgrade.

Start step 4 with `debug_port:=7311`, then run the viewer in another terminal:

```bash
ros2 launch lookie_bringup kstone.launch.py debug_port:=7311
~/.venvs/kstone-viewer/bin/kstone-viewer --connect 127.0.0.1:7311
```

Open http://127.0.0.1:8080. To view it from another computer, run `ssh -N -L 8080:127.0.0.1:8080 <user>@<robot-ip>` there and open the same address.

## Troubleshooting

- **A driver exits at startup**: its CAN interface is down or misnamed. Run `ros2 run lookie_bringup can_bind.sh --list`.
- **RViz shows no robot**: no chassis odometry is running. Add `fixed_frame:=base_footprint` to step 3.
- **The robot does not follow kstone**: check `ros2 topic echo /trio/kstone/state --qos-durability transient_local` (`/neo/...` on Neo); it waits for joint feedback before it holds.
- **The headset page does not load**: the headset and the robot must be on the same network, and the certificate must be trusted.

## License

Apache-2.0, see the `LICENSE` file in each package.
