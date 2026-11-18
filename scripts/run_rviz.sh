#!/bin/bash
# ==============================================================================
# Terminal 3: Launch 3D Real-Time Digital Twin RViz Viewer
# ==============================================================================
set -e

xhost +local: > /dev/null 2>&1 || true

echo "================================================================="
echo " Launching Doosan H2017 Real-Time Digital Twin 3D Viewer (RViz)"
echo " Subscribing to: /joint_states (50Hz live robot encoder stream)"
echo "================================================================="

docker run -it --rm \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  doosan_docker-deploy:latest \
  bash -c "source /opt/ros/noetic/setup.bash && source ~/ws/ros1/devel/setup.bash && rosrun rviz rviz -d /root/ws/ros1/src/dootion_common/dootion_moveit_config/launch/moveit.rviz"
