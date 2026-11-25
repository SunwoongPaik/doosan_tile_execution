#!/bin/bash
# ==============================================================================
# Terminal 2: Run Master Execution Commander
# ==============================================================================
set -e

CONTAINER_ID=$(docker ps -q | head -n 1)
if [ -z "$CONTAINER_ID" ]; then
    echo "[ERROR] No active Doosan Docker container found! Please run run_driver.sh in Terminal 1 first."
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Copy latest commander script and trajectory directly into container
docker cp "$SCRIPT_DIR/tile_commander.py" "$CONTAINER_ID:/root/tile_commander.py"
docker cp "$SCRIPT_DIR/trajectories/optimal_tile_removal_30.69deg.csv" "$CONTAINER_ID:/root/joints.csv"

# Run Commander in container
docker exec -it "$CONTAINER_ID" bash -c "source /opt/ros/noetic/setup.bash && source ~/ws/ros1/devel/setup.bash && python3 /root/tile_commander.py"
