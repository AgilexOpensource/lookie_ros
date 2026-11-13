# Setup script

[中文](setup.zh-CN.md)

`scripts/setup.sh` deploys a Lookie workspace in one run: system packages, dependency repositories, Python SDKs, the build and the model. It is meant for new machines and production test stations. CAN is not part of it; see the [CAN guide](../lookie_bringup/docs/can.md).

## Requirements

- Tested on Ubuntu 22.04 with ROS 2 Humble. The script refuses older releases and warns on newer ones, which are untested.
- ROS 2 installed under `/opt/ros`. If several distributions are installed and `ROS_DISTRO` is not set, the script asks which one to use.
- Network access to GitHub and PyPI.

## Quick start

```bash
mkdir -p ~/lookie_ws/src && cd ~/lookie_ws/src
git clone https://github.com/AgilexOpensource/lookie_ros.git
./lookie_ros/scripts/setup.sh
```

Choose `1`, answer the prompts and pick the model from the list. Then open a new terminal, bring up CAN and start the robot:

```bash
ros2 run lookie_bringup can_bind.sh
ros2 launch lookie_bringup lookie.launch.py
```

## Menu

| Option | Action |
| --- | --- |
| `1` | Full install: everything below, in order |
| `2` | Pull all repositories, import new ones and rebuild |
| `3` | Rebuild only |
| `4` | Choose the model |
| `5` or `q` | Quit |

A full install runs these steps:

1. Install missing apt packages (vcstool, colcon, rosdep, venv, pip), asking before `sudo`. Run `rosdep init` if it was never run.
2. Import the repositories in [lookie.repos](../lookie.repos). Existing ones are left as they are.
3. Install the Python SDKs into `~/lookie_ws/.venv`. Where the system Python allows pip, the script asks whether to use a venv or the system Python.
4. Install ROS dependencies with rosdep. `xr_media` is reported unresolved; pip already installed it.
5. Build with `python3 -m colcon build`.
6. Write the chosen model to `~/.bashrc`.

## What it changes

- `~/lookie_ws/src`, `build`, `install` and `.venv`.
- A block in `~/.bashrc` between `# >>> lookie >>>` and `# <<< lookie <<<`. It activates the venv, sources the workspace and exports `LOOKIE_MODEL`. Rerunning replaces the block; deleting it undoes the change.
- System apt packages, only after you confirm.

The script is safe to rerun. Use `4` to switch the model and `2` to update.

If a step fails, see [Troubleshooting](install.md#troubleshooting) in the manual install guide.
