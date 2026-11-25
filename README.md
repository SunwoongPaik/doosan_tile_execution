# Doosan H2017 Ceiling Tile Removal — ROS 2 Jazzy Execution Suite

Native **ROS 2 Jazzy** real-robot execution framework for automated ceiling tile removal on the **Doosan H2017** manipulator (no ROS 1, no Docker, no `ros1_bridge`).

Designed as an extensible repository to host multiple execution modes (saved-trajectory replay, live MoveIt 2 planning integration, and future RealSense RGB-D perception pipelines).

---

## Repository Structure

```text
doosan_tile_execution/
├── ros2/                                      # [PRIMARY] Native ROS 2 Jazzy Execution Suite
│   ├── commanders/
│   │   └── tile_commander_ros2.py             # Mode 1: Interactive Saved-Trajectory Action Commander
│   ├── scripts/
│   │   ├── run_driver_ros2.sh                 # Terminal 1: Native H2017 Driver + MoveIt2 + RViz2
│   │   └── run_commander_ros2.sh              # Terminal 2: Launch Interactive Action Commander
│   └── patches/
│       └── dsr_hw_interface2_idle_fix.patch   # Upstream 0x8611 idle-debounce fix for doosan-robot2
├── trajectories/
│   └── optimal_tile_removal_30.69deg.csv      # Verified 659-waypoint tile extraction path (30.69 deg)
└── legacy_ros1/                               # [BACKUP] Legacy ROS 1 + Docker service-based scripts
    ├── tile_commander.py
    └── scripts/
```

---

## Key Architecture Highlights

1. **Pure ROS 2 Jazzy Single Environment (No Bridge)**
   - Uses official [`DoosanRobotics/doosan-robot2`](https://github.com/DoosanRobotics/doosan-robot2) (`jazzy` branch) directly on Ubuntu 24.04 (WSL2 mirrored networking or native Linux).
   - Connects directly to the H2017 controller (`192.168.7.3:12345` + 100 Hz real-time stream on port `12347`).
   - Uses **CycloneDDS (`rmw_cyclonedds_cpp`)** to eliminate WSL2 FastDDS discovery/large-topic stalls.
   - Verified on H2017 controller firmware `GV02100300` (v2.10.3).

2. **Standard `FollowJointTrajectory` Action (Positions + Velocities + Timing)**
   - Sends all **659 waypoints** (joint positions, smooth velocities, and strictly monotonic `time_from_start`) in **one single `FollowJointTrajectory` Action Goal** to `/dsr_moveit_controller/follow_joint_trajectory`.
   - Automatically measures and reports **Planned Duration vs. Actual Execution Duration** and **Max Joint Tracking Error** via Action Feedback.

3. **Driver Patch (`dsr_hw_interface2_idle_fix.patch`)**
   - Fixes a 1-tick idle transition bug in `dsr_hardware2/src/dsr_hw_interface2.cpp` where slowing down near the end of a smooth trajectory (`sum(|dq|) < 0.0001 rad`) flipped `idle_ = true` and re-invoked blocking `set_safety_mode()` (~22 ms stall), triggering EtherCAT Following Error `0x8611` → `SAFE_STOP (7073)`.
   - Debounces `idle_` by 50 ticks (0.5 s of complete stillness) and keeps streaming `servoj_rt` cleanly until settled.

---

## Prerequisites & Driver Setup

```bash
# 1. Install dependencies (Ubuntu 24.04 + ROS 2 Jazzy)
sudo apt-get update
sudo apt-get install -y libpoco-dev ros-jazzy-rmw-cyclonedds-cpp

# 2. Clone official Doosan ROS 2 Jazzy driver into an isolated workspace
mkdir -p ~/ros2_ws/doosan_ros2_ws/src
cd ~/ros2_ws/doosan_ros2_ws/src
git clone -b jazzy --depth 1 https://github.com/DoosanRobotics/doosan-robot2.git

# 3. Apply the 0x8611 idle-debounce patch
cd doosan-robot2
git apply /home/sun/ros2_ws/doosan_tile_execution/ros2/patches/dsr_hw_interface2_idle_fix.patch

# 4. Build required H2017 packages (2.x controller default build)
cd ~/ros2_ws/doosan_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to \
    dsr_hardware2 dsr_controller2 dsr_bringup2 dsr_description2 dsr_moveit_config_h2017
```

---

## Quick Start (Real H2017 Robot)

> **SAFETY**: Always operate with a teammate present and the physical red **E-Stop** button on the Teach Pendant in hand. Never rely on `Ctrl+C` as an emergency stop.

### Terminal 1 — Launch Native ROS 2 Driver + MoveIt2 + RViz2
```bash
./ros2/scripts/run_driver_ros2.sh 192.168.7.3
```

### Terminal 2 — Launch Interactive Action Commander (Mode 1)
```bash
./ros2/scripts/run_commander_ros2.sh
```
*(Optional: specify `--duration 30` or `--csv /path/to/joints.csv`, or `--dry-run` to validate offline without a robot.)*

#### Commander Menu
- `0`: **Safety Micro-Jog** — Move a single joint (default `J1 +5.0°`) via Action for initial hardware verification
- `1`: **Approach Start Pose** — Smooth minimum-jerk S-curve Action trajectory to WP 0
- `2`: **Step-by-Step Replay** — Execute the 4 extraction stages with dense Action trajectories and pause confirmation
- `3`: **Continuous Replay** — Execute all **659 waypoints** in **one single `FollowJointTrajectory` Action Goal**
- `4`: **Return to Home** — Move safely to Standard Safe Park (`-90, 0, 90, 0, 90, 0`) or Lab Standby (`-90, 20, -140, 0, 115, -25`)
- `5`: **Inspect Pose** — Live `/joint_states` readout

---

## Verified Real-Robot Benchmark Results (H2017, `GV02100300`)

| Test Case | Waypoints | Planned Time | Actual Time | Time Error | Max Tracking Error | Result |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| Safety Micro-Jog (`J6 +2.0°`) | 51 | `3.000 s` | `3.001 s` | `+0.001 s (+0.03%)` | `0.250 deg` | `SUCCESS` (0 alarms) |
| Approach Start Pose (WP 0) | 51 | `32.396 s` | `32.410 s` | `+0.014 s (+0.04%)` | `2.991 deg` | `SUCCESS` (0 alarms) |
| **Full Continuous Extraction (1-Shot)** | **659** | **`20.000 s`** | **`20.309 s`** | **`+0.309 s (+1.54%)`** | **`2.946 deg`** | **`SUCCESS` (0 alarms)** |

---

## Roadmap (Upcoming Modes)
- [ ] Custom 4-leg End-Effector (`Assembly_cut.stl`) + Vention lift URDF integration into `dsr_moveit_config_h2017`
- [ ] Direct integration with online tile motion planner (`plan_one_tile.py`)
- [ ] RealSense RGB-D camera perception & tile pose estimation pipeline
