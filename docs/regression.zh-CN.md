# 回归检查

[English](regression.md)

改动驱动、配置、launch 文件或依赖仓库之前和之后，用下面两步确认没有引入回归：部署的就是源码、测试全部通过。

## 1. install 是否与源码一致

> **重要：** `colcon build` 默认复制文件。只改 `src/` 不重新编译时，机器人运行的仍是旧代码，测试结果也不能代表机器人上的行为。部分包更新、部分没有时，新旧版本混用可能直接启动失败。

```bash
python3 src/lookie_ros/scripts/check_install.py            # 工作空间内全部功能包
python3 src/lookie_ros/scripts/check_install.py lookie_bringup kstone_ros -v
```

- `OK`：install 与源码一致。
- `STALE`：Python 模块、launch、config 或 URDF 内容不同，或有文件未安装。
- `CHECK`：只有编译型源码（C++、CMake、消息）比上次编译新。git 拉取或拷贝也会更新时间，需要人工确认。

有 `STALE` 或 `CHECK` 时退出码为 1。

## 2. 编译与测试

```bash
python3 -m colcon build --packages-up-to lookie_bringup
source install/setup.bash
python3 -m colcon test --packages-select lookie_bringup lookie_driver lookie_teleop
colcon test-result --verbose
```

各包测试通过 `test/conftest.py` 导入源码而不是 install，所以在 `src/lookie_ros` 下直接运行 `python3 -m pytest lookie_bringup/test` 测的也是当前源码。

## 启动时的跨文件检查

`kstone.launch.py` 启动时检查关节桥 `feedback_timeout_s` 是否不大于 `kstone.yaml` 的 `feedback_timeout_s`。不满足时打印警告但仍启动。关节桥会给缓存的反馈重新打时间戳，驱动掉线后，kstone 要经过两个超时之和才让该组脱离控制，期间按冻结的关节位置继续计算；关节桥超时越大，这段时间越长。

## 改动验收清单

1. 改动前：`check_install.py` 全部 `OK`，测试通过。
2. 改动后：测试通过。
3. 重新编译部署，`check_install.py` 全部 `OK`。
4. 真机：按固定流程验证启动、保持位姿、home、遥操作和急停。
