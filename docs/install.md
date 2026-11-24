# Install

[中文](install.zh-CN.md)

Install the Lookie workspace, for the whole robot or a single package.

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

`lookie.repos` follows the development branches. For the tested commits (release, production), import `lookie.lock.repos` instead, see [Pinned versions](#pinned-versions).

To use one package only, import its own file instead of `lookie.repos`:

| Package | Repositories | Python SDKs |
| --- | --- | --- |
| lookie_bringup | [lookie_bringup.repos](../lookie_bringup/lookie_bringup.repos) (all) | all below |
| lookie_driver | [lookie_driver.repos](../lookie_driver/lookie_driver.repos) | pyAgxArm (arms), agx_motor_sdk (head, lift, fold), bc-stark-sdk (Revo2 hands) |
| lookie_control | [lookie_control.repos](../lookie_control/lookie_control.repos) | none |
| lookie_teleop | [lookie_teleop.repos](../lookie_teleop/lookie_teleop.repos) | xr_media |

## Python SDKs

By default the SDKs go into the system Python, as on production robots. Production installs the tested versions from [requirements.lock.txt](../requirements.lock.txt):

```bash
cd ~/lookie_ws
pip3 install -r src/lookie_ros/requirements.lock.txt   # production: pinned versions
pip3 install -r src/lookie_ros/requirements.txt        # or: latest from each default branch
```

Without `sudo`, pip installs into `~/.local` of the current user; run the robot as that user.

**Optional, for development:** a workspace venv keeps SDK versions under test out of the system Python and still sees the ROS Python packages:

```bash
cd ~/lookie_ws
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r src/lookie_ros/requirements.txt
```

With a venv, activate it before building and before running (below). [requirements.txt](../requirements.txt) lists all SDKs and follows their default branches.

That lock file pins each git SDK to a commit and each PyPI SDK to a version. After verifying a set of versions on the robot, regenerate it from that environment and commit it:

```bash
python3 src/lookie_ros/scripts/lock_python.py                # versions installed in this Python
python3 src/lookie_ros/scripts/lock_python.py --from remote  # current HEAD / latest release
```

The default mode refuses SDKs installed from a local directory, since their version cannot be reproduced; reinstall them from a git commit first.

## Pinned versions

[lookie.lock.repos](../lookie.lock.repos) pins every repository in `lookie.repos` to a commit, as `requirements.lock.txt` does for the SDKs. Use both together:

```bash
cd ~/lookie_ws/src
vcs import --recursive < lookie_ros/lookie.lock.repos     # clone at the pinned commits
python3 lookie_ros/scripts/lock_repos.py --check          # existing repositories: list differences
python3 lookie_ros/scripts/lock_repos.py --checkout       # check out the pinned commits (detached HEAD)
```

`--checkout` also checks out submodules at the commits each pinned commit records, and leaves repositories with uncommitted changes untouched. To return to development, check out the branch again (`git checkout main`, or the branch in `lookie.repos`).

After verifying a workspace on the robot, push every repository, then regenerate the lock from that workspace and commit it:

```bash
python3 src/lookie_ros/scripts/lock_repos.py                # commits checked out in this workspace
python3 src/lookie_ros/scripts/lock_repos.py --from remote  # current head of each branch
```

The default mode refuses repositories with uncommitted changes or commits that are not on the remote branch.

## HTTPS for VR

Before the first VR session, generate a development certificate (activate the workspace venv first if you use one):

```bash
xr_media --init-certs
```

This overwrites `cert.pem` and `key.pem` under `${XDG_CONFIG_HOME:-$HOME/.config}/xr_media/tls`; skip it if valid certificates already exist. With the default empty certificate paths, the XR server discovers these files automatically. Open the printed **HTTPS** address in the Quest browser, accept the self-signed certificate and enter VR. Remote WebXR requires a secure context; an HTTP page loading successfully does not verify VR support. For custom certificate paths, see [xr_media_ros](https://github.com/AgilexOpensource/xr_media_ros/blob/main/docs/GUIDE.md).

## Build

```bash
cd ~/lookie_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys "xr_media warehouse_ros_mongo"
python3 -m colcon build
source install/setup.bash
```

`python3 -m colcon` makes the nodes use the Python that runs colcon: the venv Python when it is active, otherwise the system Python. Build the workspace always in the same environment. Add `--packages-up-to <package>` to build one package and its dependencies. `--skip-keys` skips `xr_media`, which pip already installed, and `warehouse_ros_mongo`, which the agx_arm_ros MoveIt package declares but Humble does not ship.

In every terminal that runs Lookie, run `source ~/lookie_ws/install/setup.bash`; with a venv, run `source ~/lookie_ws/.venv/bin/activate` first. Avoid activating the venv in `~/.bashrc` on shared machines: it changes the Python of every terminal, including other modules that do not use it. Then set the model as described in [lookie_bringup](../lookie_bringup/README.md#usage).

## Update

```bash
vcs pull ~/lookie_ws/src
vcs import --recursive --skip-existing ~/lookie_ws/src < ~/lookie_ws/src/lookie_ros/lookie.repos
vcs custom ~/lookie_ws/src --git --args submodule update --init --recursive
```

The second command imports dependency repositories added since the last install. The third checks out each repository's submodules (for example kstone's `third_party`) at the commits that repository records; `vcs pull` does not update them. With pinned versions, pull only `lookie_ros`, then run the commands under Pinned versions above.

> **Important:** Rebuild every package after updating (`python3 -m colcon build`). Updating or building only some repositories can leave configs and programs at different versions, for example kstone exiting at startup with `unknown key`. After building, confirm the install matches the source with `python3 src/lookie_ros/scripts/check_install.py`, see [Regression checks](regression.md).

## Troubleshooting

- `GitHub release metadata request failed` while installing `agx_motor_sdk`: the SDK downloads protocol libraries from GitHub, and anonymous access allows 60 API requests per hour per IP. Wait for the limit to reset, or set `GITHUB_TOKEN` to a GitHub token and install again.
- `ModuleNotFoundError` from a driver at launch: activate the venv in that terminal before `source install/setup.bash`, then rebuild with `python3 -m colcon build`.
