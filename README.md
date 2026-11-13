# lookie_ros

ROS 2 Humble workspace for the Lookie robot (Trio and Neo). [中文](README.zh-CN.md)

## Packages

| Path | Description |
| --- | --- |
| [lookie_bringup](lookie_bringup/README.md) | Whole-robot launches: drivers, kstone_ros and VR teleop |
| [lookie_control](https://github.com/AgilexOpensource/lookie_control) | kstone, kstone_ros, lookie_wbc_teleop and xr_media_ros |
| [lookie_description](https://github.com/AgilexOpensource/lookie_description) | URDFs, meshes and RViz display |
| [lookie_driver](https://github.com/AgilexOpensource/lookie_driver) | Head, torso, arm and chassis drivers |

## Install

```bash
mkdir -p ~/lookie_ws/src && cd ~/lookie_ws/src
git clone --recurse-submodules https://github.com/AgilexOpensource/lookie_ros.git
# Already cloned without submodules: git -C lookie_ros submodule update --init --recursive
```

Install the driver Python SDKs listed in [lookie_driver](lookie_driver/README.md#install), then build and run as described in the [lookie_bringup quick start](lookie_bringup/README.md#quick-start).
