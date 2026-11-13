#!/usr/bin/env bash
# Interactive installer for the Lookie workspace: dependencies, build and model.
# Run from the cloned repo: ~/lookie_ws/src/lookie_ros/scripts/setup.sh
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="$(cd "$REPO_DIR/../.." && pwd)"
VENV="$WS/.venv"
CONFIG_DIR="$REPO_DIR/lookie_bringup/config"
MIN_UBUNTU=22.04
TESTED="Ubuntu 22.04 with ROS 2 Humble"
# ROS 2 releases older than Humble, the oldest supported one.
OLD_DISTROS=(ardent bouncy crystal dashing eloquent foxy galactic)
APT_PKGS=(build-essential cmake libasio-dev libyaml-cpp-dev python3-dev
  python3-vcstool python3-colcon-common-extensions python3-rosdep python3-venv python3-pip)
PIP_PKGS=(
  "git+https://github.com/agilexrobotics/pyAgxArm.git"
  "git+https://github.com/kehuanjack/agx_motor_sdk.git"
  bc-stark-sdk
  "git+https://github.com/AgilexOpensource/xr_media.git"
  "numpy<2" "setuptools<80"
)
BASHRC="$HOME/.bashrc"
MARK_BEGIN="# >>> lookie >>>"
MARK_END="# <<< lookie <<<"

info() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33mWarning:\033[0m %s\n' "$*"; }
die() { printf '\033[1;31mError:\033[0m %s\n' "$*" >&2; exit 1; }
confirm() {
  local reply
  read -rp "$1 [Y/n] " reply
  [[ -z "$reply" || "$reply" =~ ^[Yy] ]]
}

find_ros() {
  local setups=() distro
  if [[ -n "$ROS_DISTRO" && -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
    echo "$ROS_DISTRO"; return
  fi
  mapfile -t setups < <(ls -d /opt/ros/*/setup.bash 2>/dev/null)
  (( ${#setups[@]} )) || die "ROS 2 not found in /opt/ros. Install Humble or later: https://docs.ros.org"
  if (( ${#setups[@]} == 1 )); then
    basename "$(dirname "${setups[0]}")"; return
  fi
  echo "Several ROS 2 distributions are installed:" >&2
  PS3="ROS 2 distribution number: "
  select distro in "${setups[@]%/setup.bash}"; do
    [[ -n "$distro" ]] && { basename "$distro"; return; }
  done
  die "No ROS 2 distribution selected."
}

check_system() {
  local release distro
  release="$(lsb_release -rs 2>/dev/null || true)"
  if [[ -z "$release" ]] || ! dpkg --compare-versions "$release" ge "$MIN_UBUNTU"; then
    die "Ubuntu $MIN_UBUNTU or later is required, found ${release:-unknown}."
  fi
  distro="$(find_ros)" || exit 1
  [[ " ${OLD_DISTROS[*]} " == *" $distro "* ]] && die "ROS 2 Humble or later is required, found $distro."
  [[ "$(basename "$(dirname "$REPO_DIR")")" == "src" ]] \
    || die "lookie_ros must be cloned into <workspace>/src, found $REPO_DIR"
  info "Ubuntu $release, ROS 2 $distro."
  [[ "$release" == "22.04" && "$distro" == "humble" ]] || warn "Untested environment; tested on $TESTED."
  # shellcheck disable=SC1090
  source "/opt/ros/$distro/setup.bash"
}

use_venv() { [[ -f "$VENV/bin/activate" ]]; }
activate_python() {
  # shellcheck disable=SC1091
  if use_venv; then source "$VENV/bin/activate"; fi
}

install_apt() {
  local missing=() pkg
  for pkg in "${APT_PKGS[@]}"; do
    dpkg -s "$pkg" >/dev/null 2>&1 || missing+=("$pkg")
  done
  if (( ${#missing[@]} )); then
    info "Missing system packages: ${missing[*]}"
    confirm "Install them with sudo apt?" || die "Cannot continue without: ${missing[*]}"
    sudo apt-get update
    sudo apt-get install -y "${missing[@]}"
  else
    info "System packages are installed."
  fi
  if [[ ! -d /etc/ros/rosdep/sources.list.d ]]; then
    info "Initializing rosdep."
    sudo rosdep init
  fi
}

import_repos() {
  info "Fetching dependency repositories into $WS/src."
  vcs import --recursive --skip-existing "$WS/src" < "$REPO_DIR/lookie.repos"
}

update_repos() {
  info "Pulling all repositories in $WS/src."
  vcs pull "$WS/src"
}

externally_managed() {
  compgen -G "/usr/lib/python3*/EXTERNALLY-MANAGED" >/dev/null
}

install_python() {
  if ! use_venv; then
    echo "Python SDKs go into a virtual environment (recommended) or the system Python."
    if externally_managed; then
      info "The system Python is externally managed; creating $VENV."
      python3 -m venv --system-site-packages "$VENV"
    elif confirm "Create the virtual environment $VENV?"; then
      python3 -m venv --system-site-packages "$VENV"
    fi
  fi
  activate_python
  if use_venv; then
    info "Installing Python SDKs into $VENV."
    python3 -m pip install "${PIP_PKGS[@]}"
  else
    info "Installing Python SDKs into the system Python."
    pip3 install "${PIP_PKGS[@]}"
  fi
}

install_rosdeps() {
  info "Installing ROS dependencies with rosdep (xr_media is reported unresolved; pip installs it)."
  rosdep update
  rosdep install --from-paths "$WS/src" --ignore-src -r -y
}

build() {
  activate_python
  info "Building $WS."
  (cd "$WS" && python3 -m colcon build)
}

list_models() {
  local dir
  for dir in "$CONFIG_DIR"/*/; do
    if [[ -f "$dir/bringup.yaml" ]]; then basename "$dir"; fi
  done
}

choose_model() {
  local models=() model
  mapfile -t models < <(list_models)
  (( ${#models[@]} )) || die "No model found in $CONFIG_DIR"
  echo "Available models (current: ${LOOKIE_MODEL:-none}):" >&2
  PS3="Model number: "
  select model in "${models[@]}"; do
    [[ -n "$model" ]] && { echo "$model"; return; }
    echo "Invalid choice." >&2
  done
  die "No model selected."
}

write_bashrc() {
  local model="$1" block tmp
  block="$MARK_BEGIN"$'\n'
  use_venv && block+="source $VENV/bin/activate"$'\n'
  block+="source $WS/install/setup.bash"$'\n'
  block+="export LOOKIE_MODEL=$model"$'\n'
  block+="$MARK_END"
  tmp="$(mktemp "$HOME/.bashrc.lookie.XXXXXX")"
  [[ -f "$BASHRC" ]] && sed "/^$MARK_BEGIN\$/,/^$MARK_END\$/d" "$BASHRC" > "$tmp"
  printf '%s\n' "$block" >> "$tmp"
  cat "$tmp" > "$BASHRC"
  rm -f "$tmp"
  info "Updated $BASHRC: LOOKIE_MODEL=$model. Open a new terminal or run: source ~/.bashrc"
}

set_model() { write_bashrc "$(choose_model)"; }

full_install() {
  install_apt
  import_repos
  install_python
  install_rosdeps
  build
  set_model
  info "Done. Bring up CAN, then in a new terminal: ros2 launch lookie_bringup lookie.launch.py"
}

main() {
  check_system
  echo "Lookie workspace: $WS"
  local actions=(
    "Full install (system packages, repositories, Python SDKs, build, model)"
    "Update repositories and rebuild"
    "Rebuild only"
    "Choose the model"
    "Quit"
  )
  local i reply
  while true; do
    echo
    for i in "${!actions[@]}"; do printf '%d) %s\n' $((i + 1)) "${actions[i]}"; done
    read -rp "Select an action: " reply || break
    case "$reply" in
      1) full_install ;;
      2) update_repos; import_repos; build ;;
      3) build ;;
      4) set_model ;;
      5|q) break ;;
      *) echo "Invalid choice: $reply" ;;
    esac
  done
}

main "$@"
