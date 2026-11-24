# Regression checks

[中文](regression.zh-CN.md)

Before and after changing drivers, configs, launch files or dependency repositories, use these two steps to confirm nothing regressed: the install is the source and the tests pass.

## 1. Does the install match the source?

> **Important:** `colcon build` copies files by default. After editing `src/` without rebuilding, the robot still runs the old code, and test results no longer describe it. With some packages rebuilt and others not, mixed versions can fail to start.

```bash
python3 src/lookie_ros/scripts/check_install.py            # every package in the workspace
python3 src/lookie_ros/scripts/check_install.py lookie_bringup kstone_ros -v
```

- `OK`: no difference found in the checked runtime files.
- `STALE`: a Python module, launch, config or URDF file differs, or a file is not installed.
- `CHECK`: only compiled sources (C++, CMake, messages) are newer than the last build. A git pull or copy also updates times, so confirm by hand.

The exit code is 1 when any package is `STALE` or `CHECK`.

The check covers runtime file contents and compiled-source modification times, not docs, dependency versions or git state.

## 2. Build and test

```bash
python3 -m colcon build --packages-up-to lookie_bringup
source install/setup.bash
python3 -m colcon test --packages-select lookie_bringup lookie_driver lookie_teleop
colcon test-result --verbose
```

Each package's `test/conftest.py` imports the source, not the install, so `python3 -m pytest lookie_bringup/test` run in `src/lookie_ros` also tests the current source.

## Cross-file check at launch

`kstone.launch.py` checks that the joint bridge `feedback_timeout_s` is at most the `kstone.yaml` `feedback_timeout_s`. If not, it logs a warning and still starts. The bridge restamps cached feedback, so after a driver goes silent kstone detaches its group only after the sum of both timeouts, solving on the frozen joint positions until then; a larger bridge timeout makes this window longer. For the full chain and latencies see [Feedback freshness and timeout chain](architecture.md#6-feedback-freshness-and-timeout-chain).

## Change checklist

Launch also checks that head/lift/fold driver `feedback_timeout` exceeds kstone `feedback_timeout_s` by at least 0.05 s; it warns but still starts if not. This gap lets kstone detect feedback interruption after e-stop recovery or power restoration and re-attach from fresh measured positions.

1. Before: `check_install.py` all `OK`, tests pass.
2. After: tests pass.
3. Rebuild and deploy; `check_install.py` all `OK`.
4. On the robot: run the fixed checks for startup, holding pose, home, teleop and e-stop.
