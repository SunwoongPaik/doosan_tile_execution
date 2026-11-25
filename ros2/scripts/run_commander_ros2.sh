#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
ROS2_WS="${DOOSAN_ROS2_WS:-/home/sun/ros2_ws/doosan_ros2_ws}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
if [ -f "${ROS2_WS}/install/setup.bash" ]; then
    source "${ROS2_WS}/install/setup.bash"
fi

python3 "${REPO_ROOT}/ros2/commanders/tile_commander_ros2.py" "$@"
