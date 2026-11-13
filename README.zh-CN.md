# lookie_ros

Lookie 机器人（Trio 和 Neo）的 ROS 2 Humble 工作空间。[English](README.md)

## 包

| 路径 | 说明 |
| --- | --- |
| [lookie_bringup](lookie_bringup/README.zh-CN.md) | 整机启动：驱动、kstone_ros 和 VR 遥操作 |
| [lookie_control](https://github.com/AgilexOpensource/lookie_control) | kstone、kstone_ros、lookie_wbc_teleop 和 xr_media_ros |
| [lookie_description](https://github.com/AgilexOpensource/lookie_description) | URDF、网格和 RViz 显示 |
| [lookie_driver](https://github.com/AgilexOpensource/lookie_driver) | 头部、躯干、机械臂和底盘驱动 |

## 安装

```bash
mkdir -p ~/lookie_ws/src && cd ~/lookie_ws/src
git clone --recurse-submodules https://github.com/AgilexOpensource/lookie_ros.git
# 已克隆但没有子模块：git -C lookie_ros submodule update --init --recursive
```

先按 [lookie_driver](lookie_driver/README.md#install) 安装驱动的 Python SDK，再按 [lookie_bringup 快速使用](lookie_bringup/README.zh-CN.md#快速使用) 编译和运行。
