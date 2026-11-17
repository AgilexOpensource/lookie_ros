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
# Repositories and Python SDKs follow their branches; LOOKIE_LOCKED=1 uses the pinned commits and
# versions instead (release, production).
# The SDKs go into the system Python unless a venv is chosen: LOOKIE_VENV=1 (yes) / 0 (no) skips the prompt.
LOCKED="${LOOKIE_LOCKED:-0}"
REPOS_FILE="$REPO_DIR/lookie.repos"
PIP_REQUIREMENTS="$REPO_DIR/requirements.txt"
if [[ "$LOCKED" == 1 ]]; then
  REPOS_FILE="$REPO_DIR/lookie.lock.repos"
  PIP_REQUIREMENTS="$REPO_DIR/requirements.lock.txt"
fi
# Per-terminal environment: venv (if used), workspace and model. Never written to ~/.bashrc,
# so other users and modules on this machine keep their own Python setup.
ENV_FILE="$WS/lookie_env.bash"
# Block written to ~/.bashrc by older versions of this script.
OLD_MARK_BEGIN="# >>> lookie >>>"

info() { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33mWarning:\033[0m %s\n' "$*"; }
die() { printf '\033[1;31mError:\033[0m %s\n' "$*" >&2; exit 1; }
confirm() {
  local reply
  read -rp "$1 [Y/n] " reply
  [[ -z "$reply" || "$reply" =~ ^[Yy] ]]
}
confirm_no() {
  local reply
  read -rp "$1 [y/N] " reply
  [[ "$reply" =~ ^[Yy] ]]
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
  vcs import --recursive --skip-existing "$WS/src" < "$REPOS_FILE"
  if [[ "$LOCKED" == 1 ]] && ! python3 "$REPO_DIR/scripts/lock_repos.py" --check; then
    warn "Existing repositories above are not at the pinned commits. Choose 'Update repositories"
    warn "and rebuild' with LOOKIE_LOCKED=1 to check them out."
  fi
}

update_repos() {
  if [[ "$LOCKED" == 1 ]]; then
    info "Pulling lookie_ros, then checking out the pinned commits in $WS/src."
    git -C "$REPO_DIR" pull --ff-only
    vcs import --recursive --skip-existing "$WS/src" < "$REPOS_FILE"
    python3 "$REPO_DIR/scripts/lock_repos.py" --checkout ||
      warn "Some repositories were left unchanged; see above."
    return
  fi
  info "Pulling all repositories in $WS/src."
  vcs pull "$WS/src"
}

externally_managed() {
  compgen -G "/usr/lib/python3*/EXTERNALLY-MANAGED" >/dev/null
}

install_python() {
  # Default: the system Python (production). A workspace venv is optional, for development.
  if use_venv && [[ "${LOOKIE_VENV:-}" == 0 ]]; then
    die "LOOKIE_VENV=0 but $VENV exists and would be used; remove it for a system Python install."
  fi
  if ! use_venv; then
    echo "Python SDKs go into the system Python (default, production) or a workspace venv (development)."
    if externally_managed; then
      info "The system Python is externally managed and refuses pip; creating $VENV."
      python3 -m venv --system-site-packages "$VENV"
    elif [[ "${LOOKIE_VENV:-}" == 1 ]] || { [[ -z "${LOOKIE_VENV:-}" ]] && confirm_no "Use a virtual environment $VENV (development)?"; }; then
      python3 -m venv --system-site-packages "$VENV"
    fi
  fi
  activate_python
  if use_venv; then
    info "Installing Python SDKs from $(basename "$PIP_REQUIREMENTS") into $VENV."
    python3 -m pip install -r "$PIP_REQUIREMENTS"
  else
    info "Installing Python SDKs from $(basename "$PIP_REQUIREMENTS") into the system Python."
    pip3 install -r "$PIP_REQUIREMENTS"
  fi
}

install_rosdeps() {
  info "Installing ROS dependencies with rosdep."
  rosdep update
  # xr_media comes from pip; warehouse_ros_mongo is not released for Humble.
  rosdep install --from-paths "$WS/src" --ignore-src -r -y --skip-keys "xr_media warehouse_ros_mongo"
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

write_env() {
  local model="$1" tmp
  tmp="$(mktemp "$WS/.lookie_env.XXXXXX")"
  # Paths resolve from the file's own location when sourced, so the workspace can live anywhere.
  cat > "$tmp" <<ENV
# Lookie workspace environment, written by lookie_ros/scripts/setup.sh (option 4 changes the model).
# Load it in each terminal that runs Lookie: source <workspace>/lookie_env.bash
if [ -z "\${BASH_VERSION:-}" ]; then
  echo "lookie_env.bash: source it from bash" >&2
  return 1 2>/dev/null || exit 1
fi
_lookie_ws="\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)"
if [ -f "\$_lookie_ws/.venv/bin/activate" ]; then
  source "\$_lookie_ws/.venv/bin/activate"
fi
if [ -f "\$_lookie_ws/install/setup.bash" ]; then
  source "\$_lookie_ws/install/setup.bash"
else
  echo "lookie_env.bash: \$_lookie_ws/install/setup.bash not found; build the workspace first" >&2
fi
export LOOKIE_MODEL=$model
unset _lookie_ws
ENV
  mv "$tmp" "$ENV_FILE"
  info "Wrote $ENV_FILE: LOOKIE_MODEL=$model."
  info "In each terminal that runs Lookie: source $ENV_FILE"
  if [[ -f "$HOME/.bashrc" ]] && grep -qxF "$OLD_MARK_BEGIN" "$HOME/.bashrc"; then
    warn "~/.bashrc still has a '$OLD_MARK_BEGIN' block from an older setup.sh that activates the venv"
    warn "in every terminal. Delete that block unless every module on this machine should use the venv."
  fi
}

set_model() { write_env "$(choose_model)"; }

full_install() {
  install_apt
  import_repos
  install_python
  install_rosdeps
  build
  set_model
  info "Done. Bring up CAN, then in a new terminal: source $ENV_FILE && ros2 launch lookie_bringup lookie.launch.py"
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
      2) update_repos; [[ "$LOCKED" == 1 ]] || import_repos; build ;;
      3) build ;;
      4) set_model ;;
      5|q) break ;;
      *) echo "Invalid choice: $reply" ;;
    esac
  done
}

main "$@"
