# Install

[中文](install.zh-CN.md)

Manual install of the Lookie workspace. The [setup script](setup.md) runs the same steps for the whole robot.

Tested on Ubuntu 22.04 with ROS 2 Humble. Older releases are not supported; newer ones are untested. The commands use Humble.

## System packages

`agx_motor_sdk` compiles a C++ extension during `pip install`, so the build tools come first:

```bash
sudo apt update
sudo apt install build-essential cmake libasio-dev libyaml-cpp-dev python3-dev \
  python3-vcstool python3-colcon-common-extensions python3-rosdep python3-venv python3-pip
```

Initialize rosdep once per machine. Skip `sudo rosdep init` if `/etc/ros/rosdep/sources.list.d` already exists:

```bash
sudo rosdep init
rosdep update
```

## Clone

```bash
mkdir -p ~/lookie_ws/src && cd ~/lookie_ws/src
git clone https://github.com/AgilexOpensource/lookie_ros.git
vcs import --recursive < lookie_ros/lookie.repos
```

To use one package only, import its own file instead of `lookie.repos`:

| Package | Repositories | Python SDKs |
| --- | --- | --- |
| lookie_bringup | [lookie_bringup.repos](../lookie_bringup/lookie_bringup.repos) (all) | all below |
| lookie_driver | [lookie_driver.repos](../lookie_driver/lookie_driver.repos) | pyAgxArm (arms), agx_motor_sdk (head, lift, fold), bc-stark-sdk (Revo2 hands) |
| lookie_control | [lookie_control.repos](../lookie_control/lookie_control.repos) | none |
| lookie_teleop | [lookie_teleop.repos](../lookie_teleop/lookie_teleop.repos) | xr_media |

## Python SDKs

A virtual environment keeps the system Python clean and still sees the ROS Python packages:

```bash
cd ~/lookie_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install git+https://github.com/agilexrobotics/pyAgxArm.git \
  git+https://github.com/kehuanjack/agx_motor_sdk.git bc-stark-sdk \
  git+https://github.com/AgilexOpensource/xr_media.git 'numpy<2' 'setuptools<80'
```

Without a venv, run the same `pip install` with `pip3` and use plain `colcon` below.

## Build

```bash
cd ~/lookie_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
python3 -m colcon build
source install/setup.bash
```

`python3 -m colcon` makes the nodes use the venv Python. Add `--packages-up-to <package>` to build one package and its dependencies. rosdep reports `xr_media` as unresolved; pip already installed it.

In every new terminal, run `source ~/lookie_ws/.venv/bin/activate` before `source ~/lookie_ws/install/setup.bash`, or add both lines to `~/.bashrc`.

## Update

```bash
vcs pull ~/lookie_ws/src
vcs import --recursive --skip-existing ~/lookie_ws/src < ~/lookie_ws/src/lookie_ros/lookie.repos
```

Then build again. The second command imports dependency repositories added since the last install.

## Troubleshooting

- `GitHub release metadata request failed` while installing `agx_motor_sdk`: the SDK downloads protocol libraries from GitHub, and anonymous access allows 60 API requests per hour per IP. Wait for the limit to reset, or set `GITHUB_TOKEN` to a GitHub token and install again.
- `ModuleNotFoundError` from a driver at launch: activate the venv in that terminal before `source install/setup.bash`, then rebuild with `python3 -m colcon build`.
