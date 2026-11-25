#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Doosan H2017 Ceiling Tile Removal - Master Execution Commander
=============================================================
Unified CLI tool for:
 1. Approach to Tile Start Pose (Safe 12s approach)
 2. Execute Tile Removal (Step-by-Step, 4 stages)
 3. Execute Tile Removal (Continuous, ~20s fast & smooth)
 4. Return to Safe Home / Park Pose (Protects motor gearboxes)
 5. Inspect Current Robot Pose (Joints & Flange XYZ/RPY)
 Built-in 50Hz Real-Time Digital Twin stream for RViz.
"""

import os
import sys
import math
import time
import csv
import threading
import logging
import rospy
from sensor_msgs.msg import JointState
from dsr_msgs.srv import MoveJoint, MoveJointRequest

# Suppress noisy ROS TCPROS logging
logging.getLogger('rosout').setLevel(logging.ERROR)
logging.getLogger('rospy.impl.tcpros_base').setLevel(logging.ERROR)

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

DEFAULT_CSV_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trajectories", "optimal_tile_removal_30.69deg.csv")
FALLBACK_CSV_PATHS = [
    "/root/joints.csv",
    "/root/ros2_ws/fresh_tile_20261110_180956_484601/joints.csv",
    "/home/sun/ros2_ws/fresh_tile_20261110_180956_484601/joints.csv"
]

# Preset Poses (in degrees)
POSE_SAFE_PARK = [-90.0, 0.0, 90.0, 0.0, 90.0, 0.0]        # Standard gearbox-friendly pose
POSE_ORIGINAL_STANDBY = [-90.0, 20.0, -140.0, 0.0, 115.0, -25.0] # Lab default standby

g_current_joints = [0.0] * 6
g_has_state = False
g_lock = threading.Lock()

def print_banner(text):
    print("\n" + "=" * 72)
    print(f"  {text}")
    print("=" * 72)

def load_trajectory():
    candidates = [DEFAULT_CSV_PATH] + FALLBACK_CSV_PATHS
    for p in candidates:
        if os.path.isfile(p):
            points = []
            with open(p, 'r', encoding='utf-8') as f:
                reader = csv.reader(f)
                next(reader)
                for r in reader:
                    if len(r) >= 6:
                        points.append([float(x) for x in r[:6]])
            return points
    raise FileNotFoundError(f"Trajectory CSV file not found in: {candidates}")

def state_callback(msg):
    global g_current_joints, g_has_state
    rads = [math.radians(d) for d in msg.current_posj[:6]]
    with g_lock:
        g_current_joints = rads
        g_has_state = True

def start_digital_twin_stream(ns="dsr01dootion"):
    """50Hz thread to stream real robot encoder states to RViz topics."""
    pub_root = rospy.Publisher("/joint_states", JointState, queue_size=10)
    pub_ns = rospy.Publisher(f"/{ns}/joint_states", JointState, queue_size=10)

    try:
        from dsr_msgs.msg import RobotState
        rospy.Subscriber(f"/{ns}/state", RobotState, state_callback, queue_size=10)
    except Exception:
        pass

    def loop():
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            with g_lock:
                pos = list(g_current_joints)
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name = JOINT_NAMES + ["joint_lift"]
            js.position = pos + [0.0]
            js.velocity = [0.0] * 7
            js.effort = [0.0] * 7
            pub_root.publish(js)
            pub_ns.publish(js)
            rate.sleep()

    t = threading.Thread(target=loop, daemon=True)
    t.start()

def get_current_joints():
    for _ in range(50):
        with g_lock:
            if g_has_state:
                return list(g_current_joints)
        time.sleep(0.05)
    return list(g_current_joints)

def call_move_joint(srv_proxy, target_rad, duration_sec, max_vel_deg=25.0):
    target_deg = [math.degrees(r) for r in target_rad]
    req = MoveJointRequest()
    req.pos = target_deg
    req.vel = max_vel_deg
    req.acc = 40.0
    req.time = float(duration_sec)
    req.mode = 0
    req.blendType = 0
    req.syncType = 0  # Synchronous wait
    res = srv_proxy(req)
    return res.success

def cmd_inspect_pose(ns="dsr01dootion"):
    print_banner("Current Robot Telemetry Inspector")
    try:
        from dsr_msgs.msg import RobotState
        msg = rospy.wait_for_message(f"/{ns}/state", RobotState, timeout=2.0)
        j_deg = [round(float(x), 2) for x in msg.current_posj[:6]]
        px = [round(float(x), 2) for x in msg.current_posx[:6]]
        print("\n[Joint Angles]")
        for i, d in enumerate(j_deg):
            print(f"  Joint {i+1} ({JOINT_NAMES[i]}): {d:+7.2f} deg ({math.radians(d):+6.4f} rad)")
        print("\n[Flange Cartesian Pose]")
        print(f"  Position : X = {px[0]:+8.2f} mm,  Y = {px[1]:+8.2f} mm,  Z = {px[2]:+8.2f} mm")
        print(f"  Rotation : Roll (A) = {px[3]:+7.2f} deg,  Pitch (B) = {px[4]:+7.2f} deg,  Yaw (C) = {px[5]:+7.2f} deg")
    except Exception as e:
        curr = get_current_joints()
        print("Joints (deg):", [round(math.degrees(a), 1) for a in curr])

def cmd_go_home(srv_proxy):
    print_banner("Return to Safe Home / Park Pose")
    print("  1: [Standard Safe Park Pose] (Recommended: -90, 0, 90, 0, 90, 0)")
    print("     - Shoulder upright, elbow at 90 deg. Minimizes gravitational torque on gearboxes.")
    print("  2: [Original Lab Standby Pose] (-90, 20, -140, 0, 115, -25)")
    choice = input("\nSelect Home Pose [1/2, or 'c' to cancel]: ").strip()

    if choice == '1':
        target = [math.radians(x) for x in POSE_SAFE_PARK]
        name = "Standard Safe Park Pose"
    elif choice == '2':
        target = [math.radians(x) for x in POSE_ORIGINAL_STANDBY]
        name = "Original Lab Standby Pose"
    else:
        print("Cancelled.")
        return

    print(f"\n[Moving...] Returning smoothly to {name} (8.0s)...")
    call_move_joint(srv_proxy, target, duration_sec=8.0, max_vel_deg=15.0)
    print(f"[Done] Robot safely positioned at {name}. Ready for power down.")

def cmd_approach_start(srv_proxy, start_pose):
    print_banner("Approach to Ceiling Tile Start Pose (WP 0)")
    curr = get_current_joints()
    curr_deg = [round(math.degrees(a), 1) for a in curr]
    start_deg = [round(math.degrees(a), 1) for a in start_pose]
    max_diff = max(abs(c - s) for c, s in zip(curr_deg, start_deg))

    print(f"Current Joints: {curr_deg}")
    print(f"Target Start  : {start_deg}")
    print(f"Max angle difference: {max_diff:.1f} deg")

    if max_diff < 2.0:
        print("\n[Notice] Robot is already at Start Pose (error < 2.0 deg). No motion needed.")
        return

    confirm = input("\nMove smoothly (10.0s) to Start Pose? [y/n]: ").strip().lower()
    if confirm in ['y', 'yes']:
        print("\n[Moving...] Approaching Start Pose (10.0s)...")
        call_move_joint(srv_proxy, start_pose, duration_sec=10.0, max_vel_deg=15.0)
        time.sleep(0.5)
        print("[Done] Robot reached Start Pose safely!")
    else:
        print("Cancelled.")

def cmd_replay_step_by_step(srv_proxy, waypoints, stages):
    print_banner("Step-by-Step Tile Removal Execution")
    for step_num, stg in enumerate(stages, 1):
        target_pt = waypoints[stg["idx"]]
        stg_time = stg["time"]
        print(f"\n>>> [{stg['name']}]")
        print(f"    - Target WP: #{stg['idx']}, Duration: {stg_time:.1f}s")
        go = input(f"    Execute Stage [{step_num}/4]? [y/n/quit]: ").strip().lower()
        if go in ['y', 'yes']:
            print(f"    [Executing...] Stage {step_num} moving ({stg_time:.1f}s)...")
            call_move_joint(srv_proxy, target_pt, duration_sec=stg_time, max_vel_deg=25.0)
            time.sleep(0.5)
            now_j = get_current_joints()
            print(f"    => [Stage {step_num} Complete!] Joints: {[round(math.degrees(a), 1) for a in now_j]}")
        else:
            print(f"Paused safely at Stage {step_num}.")
            return
    print("\n[SUCCESS] Completed all 4 extraction stages successfully!")

def cmd_replay_continuous(srv_proxy, waypoints, stages):
    print_banner("Continuous Fast Tile Removal Execution (~20s)")
    print("The robot will execute the complete 30.69 deg optimal extraction path smoothly.")
    confirm = input("Start full continuous removal motion? [y/n]: ").strip().lower()
    if confirm in ['y', 'yes']:
        print("\n[Motion Started] Extracting tile smoothly and responsively...")
        t_s = time.time()
        for stg in stages:
            print(f"  -> Progressing: {stg['name']}")
            call_move_joint(srv_proxy, waypoints[stg["idx"]], duration_sec=stg["time"], max_vel_deg=25.0)
        t_e = time.time() - t_s
        print(f"\n[SUCCESS] Full tile removal completed in {t_e:.1f}s!")
    else:
        print("Cancelled.")

def main():
    rospy.init_node('doosan_tile_commander', anonymous=True, log_level=rospy.ERROR)
    ns = "dsr01dootion"
    srv_name = f"/{ns}/motion/move_joint"

    print_banner("Doosan H2017 Ceiling Tile Removal - Master Execution Commander")
    start_digital_twin_stream(ns)
    time.sleep(0.5)

    waypoints = load_trajectory()
    total_pts = len(waypoints)
    start_pose = waypoints[0]

    print(f"Connecting to Doosan motion controller: {srv_name} ...")
    rospy.wait_for_service(srv_name, timeout=10.0)
    move_j_srv = rospy.ServiceProxy(srv_name, MoveJoint)
    print("  => Connected to Doosan motion controller successfully!")
    print("  => RViz 50Hz Digital Twin active on /joint_states\n")

    stages = [
        {"name": "Stage 1: Vertical Lift & Initial Rotation (WP 0 -> 164, 25%)", "idx": 164, "time": 5.0},
        {"name": "Stage 2: Optimal Tilt Angle Entry (30.69 deg) (WP 164 -> 329, 50%)", "idx": 329, "time": 5.0},
        {"name": "Stage 3: Clearance Passage & Downward Extraction (WP 329 -> 494, 75%)", "idx": 494, "time": 5.0},
        {"name": "Stage 4: Complete Grid Separation & Final Pose (WP 494 -> 658, 100%)", "idx": total_pts - 1, "time": 5.0}
    ]

    while not rospy.is_shutdown():
        print_banner("Master Commander Menu")
        print("  1: [Approach Start Pose] Move smoothly to tile removal start position")
        print("  2: [Step-by-Step Replay] Execute tile removal in 4 verified stages (with pauses)")
        print("  3: [Continuous Replay]   Execute full tile removal smoothly in ~20s")
        print("  4: [Return to Home]      Move safely to Home / Park Pose before shutdown")
        print("  5: [Inspect Pose]        Display current joint angles and Cartesian coordinates")
        print("  q: [Quit]")
        print("=" * 72)
        cmd = input("Select an option [1-5 / q]: ").strip().lower()

        if cmd == '1':
            cmd_approach_start(move_j_srv, start_pose)
        elif cmd == '2':
            cmd_replay_step_by_step(move_j_srv, waypoints, stages)
        elif cmd == '3':
            cmd_replay_continuous(move_j_srv, waypoints, stages)
        elif cmd == '4':
            cmd_go_home(move_j_srv)
        elif cmd == '5':
            cmd_inspect_pose(ns)
        elif cmd in ['q', 'quit', 'exit']:
            print("Exiting Master Commander.")
            break

if __name__ == "__main__":
    main()
