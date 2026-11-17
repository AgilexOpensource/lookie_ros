# Setup script

[中文](setup.zh-CN.md)

`scripts/setup.sh` deploys a Lookie workspace in one run: system packages, dependency repositories, Python SDKs, the build and the model. It is meant for new machines and production test stations. CAN is not part of it; see the [CAN guide](../lookie_bringup/docs/can.md).

## Requirements

- Tested on Ubuntu 22.04 with ROS 2 Humble. The script refuses older releases and warns on newer ones, which are untested.
- ROS 2 installed under `/opt/ros`. If several distributions are installed and `ROS_DISTRO` is not set, the script asks which one to use.
- Network access to GitHub and PyPI.

## Quick start

The workspace can be any directory; `~/lookie_ws` below is an example. The script finds it from where lookie_ros is cloned (`<workspace>/src/lookie_ros`).

```bash
mkdir -p ~/lookie_ws/src && cd ~/lookie_ws/src
git clone https://github.com/AgilexOpensource/lookie_ros.git
./lookie_ros/scripts/setup.sh
```

The Python SDKs go into the system Python by default. Production and development can be chosen in advance:

```bash
LOOKIE_LOCKED=1 LOOKIE_VENV=0 ./lookie_ros/scripts/setup.sh   # production: system Python, pinned versions
LOOKIE_VENV=1 ./lookie_ros/scripts/setup.sh                   # development: workspace venv
```

Choose `1`, answer the prompts and pick the model from the list. Then open a new terminal, load the workspace environment, bring up CAN and start the robot:

```bash
source ~/lookie_ws/lookie_env.bash   # <workspace>/lookie_env.bash
ros2 run lookie_bringup can_bind.sh
ros2 launch lookie_bringup lookie.launch.py
```

Before using VR, complete the [HTTPS preparation](install.md#https-for-vr). The script installs the XR SDK but does not generate or overwrite TLS certificates.

## Menu

| Option | Action |
| --- | --- |
| `1` | Full install: everything below, in order |
| `2` | Pull all repositories, import new ones and rebuild |
| `3` | Rebuild only |
| `4` | Choose the model |
| `5` or `q` | Quit |

A full install runs these steps:

1. Install missing apt packages (build tools, vcstool, colcon, rosdep, venv, pip), asking before `sudo`. Run `rosdep init` if it was never run.
2. Import the repositories in [lookie.repos](../lookie.repos). Existing ones are left as they are.
3. Install the Python SDKs from [requirements.txt](../requirements.txt), or with `LOOKIE_LOCKED=1` the pinned versions from [requirements.lock.txt](../requirements.lock.txt). They go into the system Python by default (production); the script asks whether to use a workspace venv `<workspace>/.venv` instead (development, answer `y`). `LOOKIE_VENV=1` or `LOOKIE_VENV=0` answers the question in advance. An existing `.venv` is reused. Only when the system Python refuses pip (Ubuntu 24.04 and later) is a venv created automatically.
4. Install ROS dependencies with rosdep, skipping `xr_media` (installed by pip) and `warehouse_ros_mongo` (not shipped for Humble).
5. Build with `python3 -m colcon build`.
6. Write `<workspace>/lookie_env.bash`, which activates the venv (if used), sources the workspace and exports the chosen `LOOKIE_MODEL`.

## What it changes

- `<workspace>/src`, `build`, `install` and `.venv`.
- `<workspace>/lookie_env.bash`. It finds the workspace from its own location when sourced, so the workspace can be cloned anywhere or moved. The script never edits `~/.bashrc`, so other modules and developers on the machine keep their own Python environment. Run `source <workspace>/lookie_env.bash` in each terminal that runs Lookie; add that line to your own `~/.bashrc` only if every terminal should use it.
- Older versions of the script wrote a `# >>> lookie >>>` block to `~/.bashrc` that activated the venv in every terminal. The script warns when it finds one; delete it by hand if it gets in the way.
- System apt packages, only after you confirm.

The script is safe to rerun. Use `4` to switch the model and `2` to update.

If a step fails, see [Troubleshooting](install.md#troubleshooting) in the manual install guide.
