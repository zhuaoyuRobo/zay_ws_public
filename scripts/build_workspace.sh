#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"

if [[ ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "ROS 2 ${ROS_DISTRO} was not found under /opt/ros." >&2
  exit 1
fi

source "/opt/ros/${ROS_DISTRO}/setup.bash"
cd "${ROOT_DIR}"
rosdep install --from-paths src --ignore-src -r -y
colcon build --base-paths src --symlink-install "$@"
