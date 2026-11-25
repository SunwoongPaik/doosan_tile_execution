#!/bin/bash
set -e

HOST="${1:-192.168.7.3}"
GUI="${2:-true}"
ROS2_WS="${DOOSAN_ROS2_WS:-/home/sun/ros2_ws/doosan_ros2_ws}"

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/jazzy/setup.bash
source "${ROS2_WS}/install/setup.bash"

echo "======================================================================"
echo "  Doosan H2017 ROS 2 Jazzy Native Driver + MoveIt2"
echo "  Host: ${HOST} | Port: 12345 | Mode: real | RMW: CycloneDDS"
echo "  SAFETY: Keep physical E-Stop button ready at all times!"
echo "======================================================================"

ros2 launch dsr_moveit_config_h2017 start.launch.py \
    mode:=real \
    model:=h2017 \
    host:="${HOST}" \
    port:=12345 \
    gui:="${GUI}"
