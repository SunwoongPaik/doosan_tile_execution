#!/bin/bash
# ==============================================================================
# Terminal 1: Launch Doosan Hardware Driver in Native Docker
# ==============================================================================
set -e

CONTAINER_CMD="source /opt/ros/noetic/setup.bash && roscore & sleep 2 && source ~/ws/ros1/devel/setup.bash && roslaunch dootion_bringup dootion_bringup.launch mode:=real host:=192.168.7.3"

echo "================================================================="
echo " Starting Doosan H2017 Real Robot Driver Container (Host Network)"
echo " Controller IP: 192.168.7.3"
echo " Workstation IP: 192.168.7.100"
echo "================================================================="

docker run -it --rm \
  --net=host \
  --privileged \
  --entrypoint bash \
  -v ~/ros2_ws:/root/ros2_ws \
  -v ~/ros2_ws/doosan_docker/ws/ros1/src/dootion_common/dootion_bringup/launch/dootion_bringup.launch:/root/ws/ros1/src/dootion_common/dootion_bringup/launch/dootion_bringup.launch \
  doosan_docker-deploy:latest \
  -c "$CONTAINER_CMD"
