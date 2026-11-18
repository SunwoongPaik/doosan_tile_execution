# Doosan H2017 Ceiling Tile Removal - Real Hardware Execution System

Autonomous ceiling tile extraction trajectory execution and real-time 3D digital twin monitoring system for the **Doosan H2017** collaborative robot.

---

## 1. System Architecture & Verified Setup

- **Hardware**: Doosan Robotics H2017 Cobot (CS-01 Controller), Custom 4-leg End-Effector, 4×4 Ceiling Tile Testbed.
- **Control Workstation**: Windows 11 + WSL2 (Ubuntu 24.04), `networkingMode=mirrored`.
- **Network IPs**:
  - Robot Controller: `192.168.7.3:12345` (Subnet: `255.255.0.0`, Gateway: `192.168.7.100`)
  - Workstation (PC / WSL2): `192.168.7.100`
- **Docker Environment**: Native WSL2 Docker Engine (`--net=host` mode prevents Docker Desktop NAT isolation and resolves `[1.3004] Client T/P authentication failed`).
- **Optimal Trajectory**: 659-waypoint optimal non-contact extraction motion (`trajectories/optimal_tile_removal_30.69deg.csv`, max tilt 30.698°).

```text
 ┌────────────────────────┐         TCP/IP (12345)        ┌────────────────────────┐
 │   Doosan CS-01 Box     │ ◄───────────────────────────► │  Native Docker on WSL2 │
 │     (192.168.7.3)      │   DRFL Auth (192.168.7.100)   │    (Host Network)      │
 └────────────────────────┘                               └───────────┬────────────┘
             ▲                                                        │
             │ Real-time Telemetry (50Hz)                             │
             └────────────────────────────────────────────────────────┘
                                      │
                         ┌────────────▼───────────┐
                         │  RViz 3D Digital Twin  │
                         │    (/joint_states)     │
                         └────────────────────────┘
```

---

## 2. Directory Structure

```text
doosan_tile_execution/
├── README.md                   # Complete documentation & user guide
├── tile_commander.py           # Master Execution Commander (English CLI, 50Hz digital twin)
├── scripts/
│   ├── run_driver.sh           # Terminal 1: Starts native Docker robot driver container
│   ├── run_commander.sh        # Terminal 2: Launches the interactive CLI Commander
│   └── run_rviz.sh             # Terminal 3: Launches 3D live Digital Twin viewer in RViz
└── trajectories/
    └── optimal_tile_removal_30.69deg.csv  # 659 verified joint trajectory waypoints
```

---

## 3. Quick Start Guide (3-Terminal Workflow)

### Terminal 1: Launch Robot Driver
Starts the ROS Noetic driver stack connected to the physical robot (`192.168.7.3`):
```bash
cd ~/ros2_ws/doosan_tile_execution
./scripts/run_driver.sh
```
> **Check**: The Teach Pendant will display `Transfer completed`. This indicates control authority has been transferred to the PC. Leave this terminal open.

---

### Terminal 2: Run Master Commander
Launches the menu-driven master controller:
```bash
cd ~/ros2_ws/doosan_tile_execution
./scripts/run_commander.sh
```

**Commander Menu Overview**:
- **`1: [Approach Start Pose]`**: Slowly (10.0s) moves the robot from home to the ceiling tile extraction starting pose (`WP 0`).
- **`2: [Step-by-Step Replay]`**: Executes the 30.69° tile removal across 4 checkpoints with pauses (`[y]` prompt before each stage):
  - *Stage 1 (0% -> 25%)*: Vertical lift & initial rotation.
  - *Stage 2 (25% -> 50%)*: Optimal 30.69° tilt entry.
  - *Stage 3 (50% -> 75%)*: Grid clearance passage & downward extraction.
  - *Stage 4 (75% -> 100%)*: Complete separation & final clearance.
- **`3: [Continuous Replay]`**: Smoothly executes the full removal motion in ~20.0s (responsive, safe 25 deg/s speed).
- **`4: [Return to Home]`**: Moves the arm to a safe parking position before turning off the robot:
  - *Option 1 (Standard Safe Park)*: `[-90°, 0°, 90°, 0°, 90°, 0°]` (Minimizes torque on motor gearboxes).
  - *Option 2 (Original Lab Standby)*: `[-90°, 20°, -140°, 0°, 115°, -25°]`.
- **`5: [Inspect Pose]`**: Displays live joint angles (deg/rad) and Cartesian Flange/TCP coordinates (X, Y, Z, Roll, Pitch, Yaw).

---

### Terminal 3 (Optional): 3D Digital Twin in RViz
Opens the 3D RViz GUI window:
```bash
cd ~/ros2_ws/doosan_tile_execution
./scripts/run_rviz.sh
```
> **Digital Twin**: `tile_commander.py` broadcasts live encoder states at **50Hz** to `/joint_states`. As the real robot moves in the lab, the 3D robot model in RViz mirrors its motion simultaneously in real time.

---

## 4. Safety & Emergency Stop

1. **Hardware E-Stop (Highest Priority)**:
   - Hold the Teach Pendant with your thumb positioned over the red mushroom **Emergency Stop (E-Stop)** button. Press immediately if any unexpected motion or obstacle collision is imminent.
2. **Software Cancel**:
   - Press **`Ctrl + C`** in Terminal 2 to cancel active trajectory goals.

---

## 5. Connecting with GitHub

To push this package to GitHub as a standalone repository or add it to an existing lab repository:

### Option A: Create a New Standalone GitHub Repository
```bash
cd ~/ros2_ws/doosan_tile_execution
git init
git add .
git commit -m "feat: Add Doosan H2017 real robot tile execution system & digital twin"
git branch -M main
git remote add origin https://github.com/<YOUR_GITHUB_USERNAME>/doosan_tile_execution.git
git push -u origin main
```

### Option B: Add to Existing `doosan_docker` Repository
```bash
cd ~/ros2_ws
cp -r doosan_tile_execution doosan_docker/
cd doosan_docker
git add doosan_tile_execution
git commit -m "feat: integrate ceiling tile execution commander & digital twin scripts"
git push origin main
```
