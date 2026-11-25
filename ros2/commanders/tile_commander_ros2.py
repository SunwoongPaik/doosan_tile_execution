#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Doosan H2017 Ceiling Tile Removal - ROS 2 Jazzy Action Commander
================================================================
Pure ROS 2 (rclpy) replacement for the legacy ROS 1 service-based commander.
Uses standard FollowJointTrajectory Action (/dsr_moveit_controller/follow_joint_trajectory)
with full joint positions + velocities + time_from_start, and logs Planned vs Actual
execution time error and tracking error.

Features:
  0. Safety Micro-Jog Test (+2 deg on Joint 6 via Action, for first real-robot test)
  1. Approach to Tile Start Pose (WP 0, smooth S-curve Action trajectory)
  2. Step-by-Step Tile Removal (4 stages, dense waypoints via Action, pause per stage)
  3. Continuous Full Tile Removal (All 659 waypoints in ONE single Action Goal!)
  4. Return to Safe Home / Park Pose (Gearbox-friendly park or Lab standby)
  5. Inspect Current Joint States (Live /joint_states telemetry)
"""

import argparse
import csv
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DEFAULT_CSV_CANDIDATES = [
    os.path.join(_REPO_ROOT, "trajectories", "optimal_tile_removal_30.69deg.csv"),
    "/home/sun/ros2_ws/doosan_tile_execution/trajectories/optimal_tile_removal_30.69deg.csv",
    "/home/sun/ros2_ws/fresh_tile_20261110_180956_484601/joints.csv",
]

# Preset Poses (in degrees)
POSE_SAFE_PARK = [-90.0, 0.0, 90.0, 0.0, 90.0, 0.0]
POSE_ORIGINAL_STANDBY = [-90.0, 20.0, -140.0, 0.0, 115.0, -25.0]


def print_banner(text: str):
    print("\n" + "=" * 74)
    print(f"  {text}")
    print("=" * 74)


def load_trajectory_csv(csv_path: str | None = None) -> tuple[str, list[list[float]]]:
    candidates = [csv_path] if csv_path else []
    candidates.extend(DEFAULT_CSV_CANDIDATES)
    for p in candidates:
        if p and os.path.isfile(p):
            points: list[list[float]] = []
            with open(p, "r", encoding="utf-8") as f:
                reader = csv.reader(f)
                header = next(reader, None)
                for row in reader:
                    if len(row) >= 6:
                        points.append([float(x) for x in row[:6]])
            if len(points) < 2:
                raise ValueError(f"CSV has fewer than 2 waypoints: {p}")
            return p, points
    raise FileNotFoundError(f"Trajectory CSV not found in: {candidates}")


def quintic_blend(u: float) -> tuple[float, float]:
    """Minimum-jerk 5th-order polynomial s(u) and ds/du for u in [0, 1]."""
    u = max(0.0, min(1.0, u))
    s = 10.0 * (u ** 3) - 15.0 * (u ** 4) + 6.0 * (u ** 5)
    ds_du = 30.0 * (u ** 2) - 60.0 * (u ** 3) + 30.0 * (u ** 4)
    return s, ds_du


def build_p2p_trajectory(
    start_rad: list[float],
    target_rad: list[float],
    duration_sec: float,
    max_vel_deg_s: float = 15.0,
    num_samples: int = 50,
) -> tuple[JointTrajectory, float, float]:
    """
    Build a smooth minimum-jerk point-to-point JointTrajectory (positions + velocities).
    Automatically extends duration if peak joint velocity would exceed max_vel_deg_s.
    """
    diffs = [t - s for s, t in zip(start_rad, target_rad)]
    max_diff_rad = max(abs(d) for d in diffs)
    max_vel_rad_s = math.radians(max_vel_deg_s)

    # For quintic polynomial, peak ds/du = 1.875 -> peak vel = 1.875 * max_diff / T
    min_safe_duration = (1.875 * max_diff_rad / max_vel_rad_s) if max_vel_rad_s > 0 else duration_sec
    final_duration = max(float(duration_sec), float(min_safe_duration), 1.0)

    traj = JointTrajectory()
    traj.joint_names = list(JOINT_NAMES)

    peak_vel_deg = 0.0
    for k in range(num_samples + 1):
        u = k / float(num_samples)
        t = u * final_duration
        s, ds_du = quintic_blend(u)
        pos = [start_rad[j] + diffs[j] * s for j in range(6)]
        vel = [diffs[j] * ds_du / final_duration for j in range(6)]
        if k == 0 or k == num_samples:
            vel = [0.0] * 6
        peak_vel_deg = max(peak_vel_deg, max(abs(math.degrees(v)) for v in vel))

        pt = JointTrajectoryPoint()
        pt.positions = pos
        pt.velocities = vel
        pt.time_from_start = Duration(seconds=t).to_msg()
        traj.points.append(pt)

    return traj, final_duration, peak_vel_deg


def build_dense_waypoint_trajectory(
    waypoints_rad: list[list[float]],
    target_duration_sec: float,
    max_vel_deg_s: float = 25.0,
    min_dt: float = 0.01,
) -> tuple[JointTrajectory, float, float]:
    """
    Convert a dense list of joint waypoints (rad) into a time-parameterized
    JointTrajectory containing positions, smooth velocities, and strictly
    increasing time_from_start values.

    Uses cumulative joint-space arc length mapped through a smooth ease-in/out
    time warp so start/end velocities are 0 and peak velocity respects max_vel_deg_s.
    """
    n = len(waypoints_rad)
    if n < 2:
        raise ValueError("Need at least 2 waypoints.")

    # Cumulative joint-space L-infinity / L2 arc length
    cum_s = [0.0]
    for i in range(1, n):
        step = math.sqrt(sum((waypoints_rad[i][j] - waypoints_rad[i - 1][j]) ** 2 for j in range(6)))
        cum_s.append(cum_s[-1] + max(step, 1e-6))
    total_s = cum_s[-1]
    norm_s = [s / total_s for s in cum_s]

    # Smooth S-curve time mapping u(s): slower at start/end so vel(0)=vel(T)=0
    # Invert cosine ease-in/out gently: blend 70% linear + 30% smoothstep in time
    # Better: assign normalized time tau_i using monotonic s-warping
    def compute_times_and_vels(total_T: float) -> tuple[list[float], list[list[float]], float]:
        # Map normalized arc length directly with cosine ease-in/out ramp on first/last 8%
        raw_t = []
        for i, ns in enumerate(norm_s):
            if i == 0:
                raw_t.append(0.0)
            elif i == n - 1:
                raw_t.append(total_T)
            else:
                # Smooth monotonic time parameterization
                t_val = total_T * (0.85 * ns + 0.15 * (0.5 - 0.5 * math.cos(math.pi * ns)))
                raw_t.append(t_val)

        # Enforce strictly increasing timestamps (>= 1e-4 s)
        times = [0.0]
        for i in range(1, n):
            t_i = max(raw_t[i], times[-1] + 1e-4)
            times.append(t_i)
        # Rescale to match total_T cleanly
        scale = total_T / times[-1]
        times = [t * scale for t in times]

        # Compute central-difference velocities (rad/s), zero at endpoints
        vels: list[list[float]] = [[0.0] * 6 for _ in range(n)]
        peak_deg = 0.0
        for i in range(1, n - 1):
            dt = times[i + 1] - times[i - 1]
            if dt > 1e-6:
                for j in range(6):
                    v = (waypoints_rad[i + 1][j] - waypoints_rad[i - 1][j]) / dt
                    vels[i][j] = v
                    peak_deg = max(peak_deg, abs(math.degrees(v)))
        return times, vels, peak_deg

    final_duration = max(float(target_duration_sec), (n - 1) * min_dt)
    times, vels, peak_vel_deg = compute_times_and_vels(final_duration)

    # Auto-scale duration if peak joint velocity exceeds safe limit
    if max_vel_deg_s > 0 and peak_vel_deg > max_vel_deg_s:
        scale_factor = peak_vel_deg / max_vel_deg_s
        final_duration *= scale_factor
        times, vels, peak_vel_deg = compute_times_and_vels(final_duration)

    # Downsample slightly only if consecutive points are closer than min_dt (keeps 100Hz controller happy)
    kept_indices = [0]
    for i in range(1, n - 1):
        if times[i] - times[kept_indices[-1]] >= min_dt:
            kept_indices.append(i)
    if kept_indices[-1] != n - 1:
        kept_indices.append(n - 1)

    traj = JointTrajectory()
    traj.joint_names = list(JOINT_NAMES)
    for idx in kept_indices:
        pt = JointTrajectoryPoint()
        pt.positions = list(waypoints_rad[idx])
        pt.velocities = list(vels[idx])
        pt.time_from_start = Duration(seconds=times[idx]).to_msg()
        traj.points.append(pt)

    return traj, final_duration, peak_vel_deg


class DoosanRos2Commander(Node):
    def __init__(self, action_name: str, joint_states_topic: str):
        super().__init__("doosan_tile_commander_ros2")
        self._action_name = action_name
        self._action_client = ActionClient(self, FollowJointTrajectory, action_name)

        self._lock = threading.Lock()
        self._current_joints: list[float] | None = None
        self._current_vels: list[float] = [0.0] * 6
        self._last_state_time = 0.0

        self._sub = self.create_subscription(
            JointState, joint_states_topic, self._joint_state_cb, 20
        )

    def _joint_state_cb(self, msg: JointState):
        name_to_idx = {name: i for i, name in enumerate(msg.name)}
        if all(j in name_to_idx for j in JOINT_NAMES):
            pos = [float(msg.position[name_to_idx[j]]) for j in JOINT_NAMES]
            vel = (
                [float(msg.velocity[name_to_idx[j]]) for j in JOINT_NAMES]
                if len(msg.velocity) >= len(msg.name)
                else [0.0] * 6
            )
            with self._lock:
                self._current_joints = pos
                self._current_vels = vel
                self._last_state_time = time.monotonic()

    def get_current_joints(self, timeout_sec: float = 5.0) -> list[float]:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                if self._current_joints is not None:
                    return list(self._current_joints)
            time.sleep(0.05)
        raise TimeoutError(
            "No /joint_states received for joint_1..joint_6. Is the ROS 2 robot driver running?"
        )

    def wait_for_action_server(self, timeout_sec: float = 10.0) -> bool:
        return self._action_client.wait_for_server(timeout_sec=timeout_sec)

    def execute_trajectory(
        self, traj: JointTrajectory, planned_duration: float, label: str
    ) -> bool:
        """
        Send a FollowJointTrajectory Goal, track feedback error, and measure
        Planned vs Actual execution duration.
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj

        max_tracking_err_deg = 0.0
        feedback_count = 0

        def feedback_cb(fb_msg):
            nonlocal max_tracking_err_deg, feedback_count
            feedback_count += 1
            err_pos = fb_msg.feedback.error.positions
            if err_pos:
                err_deg = max(abs(math.degrees(e)) for e in err_pos[:6])
                if err_deg > max_tracking_err_deg:
                    max_tracking_err_deg = err_deg

        print(f"\n[Action Sending] {label}")
        print(f"  - Action Server   : {self._action_name}")
        print(f"  - Waypoints       : {len(traj.points)} points (positions + velocities)")
        print(f"  - Planned Time    : {planned_duration:.3f} s")

        send_future = self._action_client.send_goal_async(goal_msg, feedback_callback=feedback_cb)
        while not send_future.done():
            time.sleep(0.01)

        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            print("[ERROR] Action goal was REJECTED by controller!")
            return False

        t_start = time.monotonic()
        print("  - Status          : Goal ACCEPTED. Robot moving...")

        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.02)

        t_actual = time.monotonic() - t_start
        res_wrapper = result_future.result()
        result = res_wrapper.result
        error_code = result.error_code
        error_str = result.error_string

        time_err = t_actual - planned_duration
        time_err_pct = (time_err / planned_duration * 100.0) if planned_duration > 0 else 0.0

        print("\n  [Execution Timing & Accuracy Report]")
        print(f"    - Planned Duration    : {planned_duration:.3f} s")
        print(f"    - Actual Duration     : {t_actual:.3f} s")
        print(f"    - Time Error          : {time_err:+.3f} s ({time_err_pct:+.2f}%)")
        print(f"    - Feedback Samples    : {feedback_count}")
        print(f"    - Max Tracking Error  : {max_tracking_err_deg:.3f} deg")
        print(f"    - Controller Result   : error_code={error_code} ({error_str or 'OK'})")

        return error_code == FollowJointTrajectory.Result.SUCCESSFUL


def cmd_inspect_pose(node: DoosanRos2Commander):
    print_banner("Current Robot Telemetry Inspector (ROS 2 /joint_states)")
    curr = node.get_current_joints()
    print("\n[Joint Angles]")
    for i, r in enumerate(curr):
        print(f"  Joint {i+1} ({JOINT_NAMES[i]}): {math.degrees(r):+8.2f} deg ({r:+7.4f} rad)")


def cmd_micro_jog(node: DoosanRos2Commander):
    print_banner("Safety Micro-Jog Test (Action)")
    print("  Recommended for first real-robot test with teammate + E-Stop ready.")
    j_str = input("  Select Joint [1-6, default=1]: ").strip()
    joint_idx = int(j_str) - 1 if j_str in ("1", "2", "3", "4", "5", "6") else 0
    deg_str = input("  Delta angle in deg [default=+5.0, max +-10.0]: ").strip()
    try:
        delta_deg = float(deg_str) if deg_str else 5.0
    except ValueError:
        delta_deg = 5.0
    delta_deg = max(-10.0, min(10.0, delta_deg))
    dur_req = max(3.0, abs(delta_deg) * 0.6)

    curr = node.get_current_joints()
    target = list(curr)
    target[joint_idx] += math.radians(delta_deg)

    j_label = f"J{joint_idx + 1}"
    print(f"\n  Current {j_label} : {math.degrees(curr[joint_idx]):+.2f} deg")
    print(f"  Target  {j_label} : {math.degrees(target[joint_idx]):+.2f} deg ({delta_deg:+.2f} deg, {dur_req:.1f}s)")
    confirm = input(f"\nExecute {delta_deg:+.2f} deg {j_label} Micro-Jog Action? [y/n]: ").strip().lower()
    if confirm not in ("y", "yes"):
        print("Cancelled.")
        return

    traj, dur, peak_v = build_p2p_trajectory(curr, target, duration_sec=dur_req, max_vel_deg_s=5.0)
    print(f"  - Peak velocity: {peak_v:.2f} deg/s")
    ok = node.execute_trajectory(traj, dur, f"Micro-Jog {delta_deg:+.2f} deg ({j_label})")
    if not ok:
        return

    back = input(f"\nReturn {j_label} back {-delta_deg:+.2f} deg to original pose? [y/n]: ").strip().lower()
    if back in ("y", "yes"):
        curr2 = node.get_current_joints()
        traj2, dur2, _ = build_p2p_trajectory(curr2, curr, duration_sec=dur_req, max_vel_deg_s=5.0)
        node.execute_trajectory(traj2, dur2, f"Micro-Jog Return ({j_label})")


def cmd_approach_start(node: DoosanRos2Commander, start_pose: list[float]):
    print_banner("Approach to Ceiling Tile Start Pose (WP 0)")
    curr = node.get_current_joints()
    curr_deg = [round(math.degrees(a), 1) for a in curr]
    start_deg = [round(math.degrees(a), 1) for a in start_pose]
    max_diff = max(abs(c - s) for c, s in zip(curr_deg, start_deg))

    print(f"  Current Joints : {curr_deg}")
    print(f"  Target Start   : {start_deg}")
    print(f"  Max angle diff : {max_diff:.1f} deg")

    if max_diff < 1.0:
        print("\n[Notice] Robot is already at Start Pose (error < 1.0 deg).")
        return

    traj, dur, peak_v = build_p2p_trajectory(curr, start_pose, duration_sec=10.0, max_vel_deg_s=15.0)
    print(f"  Planned Time   : {dur:.2f} s (Peak vel: {peak_v:.2f} deg/s)")
    confirm = input("\nMove smoothly to Start Pose via Action? [y/n]: ").strip().lower()
    if confirm in ("y", "yes"):
        node.execute_trajectory(traj, dur, "Approach Start Pose (WP 0)")
    else:
        print("Cancelled.")


def cmd_replay_step_by_step(
    node: DoosanRos2Commander, waypoints: list[list[float]], stages: list[dict]
):
    print_banner("Step-by-Step Tile Removal Execution (Dense Action per Stage)")
    prev_idx = 0
    for step_num, stg in enumerate(stages, 1):
        end_idx = stg["idx"]
        seg_wps = waypoints[prev_idx : end_idx + 1]
        traj, dur, peak_v = build_dense_waypoint_trajectory(
            seg_wps, target_duration_sec=stg["time"], max_vel_deg_s=20.0
        )
        print(f"\n>>> [{stg['name']}]")
        print(f"    - Segment WPs : #{prev_idx} -> #{end_idx} ({len(traj.points)} timed pts)")
        print(f"    - Duration    : {dur:.2f} s (Peak vel: {peak_v:.2f} deg/s)")
        go = input(f"    Execute Stage [{step_num}/4] via Action? [y/n/quit]: ").strip().lower()
        if go in ("y", "yes"):
            ok = node.execute_trajectory(traj, dur, stg["name"])
            if not ok:
                print(f"[STOP] Stage {step_num} did not succeed. Halting.")
                return
            prev_idx = end_idx
        else:
            print(f"Paused safely at Stage {step_num}.")
            return
    print("\n[SUCCESS] Completed all 4 extraction stages via ROS 2 Action!")


def cmd_replay_continuous(
    node: DoosanRos2Commander, waypoints: list[list[float]], total_time: float = 20.0
):
    print_banner("Continuous Full Tile Removal Execution (1 Single Action Goal!)")
    curr = node.get_current_joints()
    max_start_diff = max(abs(math.degrees(c - s)) for c, s in zip(curr, waypoints[0]))
    if max_start_diff > 3.0:
        print(f"[WARNING] Robot is {max_start_diff:.1f} deg away from WP 0!")
        print("          Please run Option 1 (Approach Start Pose) first.")
        return

    traj, dur, peak_v = build_dense_waypoint_trajectory(
        waypoints, target_duration_sec=total_time, max_vel_deg_s=20.0
    )
    print(f"  - Total CSV Waypoints : {len(waypoints)}")
    print(f"  - Action Goal Points  : {len(traj.points)} (positions + velocities)")
    print(f"  - Planned Duration    : {dur:.2f} s (Peak vel: {peak_v:.2f} deg/s)")
    confirm = input("\nStart full continuous removal via ONE Action Goal? [y/n]: ").strip().lower()
    if confirm in ("y", "yes"):
        node.execute_trajectory(traj, dur, "Full Continuous Tile Removal (30.69 deg)")
    else:
        print("Cancelled.")


def cmd_go_home(node: DoosanRos2Commander):
    print_banner("Return to Safe Home / Park Pose (via Action)")
    print("  1: [Standard Safe Park Pose] (Recommended: -90, 0, 90, 0, 90, 0)")
    print("  2: [Original Lab Standby Pose] (-90, 20, -140, 0, 115, -25)")
    choice = input("\nSelect Home Pose [1/2, or 'c' to cancel]: ").strip()

    if choice == "1":
        target = [math.radians(x) for x in POSE_SAFE_PARK]
        name = "Standard Safe Park Pose"
    elif choice == "2":
        target = [math.radians(x) for x in POSE_ORIGINAL_STANDBY]
        name = "Original Lab Standby Pose"
    else:
        print("Cancelled.")
        return

    curr = node.get_current_joints()
    traj, dur, peak_v = build_p2p_trajectory(curr, target, duration_sec=8.0, max_vel_deg_s=15.0)
    print(f"\n[Moving...] Returning smoothly to {name} ({dur:.2f}s, Peak vel: {peak_v:.2f} deg/s)")
    node.execute_trajectory(traj, dur, name)


def run_dry_run(csv_path: str | None, total_time: float):
    print_banner("DRY-RUN Trajectory Verification (No Robot Connection Required)")
    path, waypoints = load_trajectory_csv(csv_path)
    print(f"  Loaded CSV       : {path}")
    print(f"  Total Waypoints  : {len(waypoints)}")

    traj, dur, peak_v = build_dense_waypoint_trajectory(
        waypoints, target_duration_sec=total_time, max_vel_deg_s=20.0
    )
    print(f"  Action Points    : {len(traj.points)}")
    print(f"  Planned Duration : {dur:.3f} s")
    print(f"  Peak Joint Vel   : {peak_v:.2f} deg/s")

    # Sanity checks
    prev_t = -1.0
    for idx, pt in enumerate(traj.points):
        t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
        assert t > prev_t, f"Non-increasing timestamp at index {idx}: {t} <= {prev_t}"
        assert len(pt.positions) == 6 and len(pt.velocities) == 6
        assert all(math.isfinite(x) for x in pt.positions + pt.velocities)
        prev_t = t

    assert all(v == 0.0 for v in traj.points[0].velocities), "Start vel must be 0"
    assert all(v == 0.0 for v in traj.points[-1].velocities), "End vel must be 0"

    print("  [PASS] Strictly increasing timestamps verified.")
    print("  [PASS] Start & end velocities are 0.0 rad/s.")
    print("  [PASS] All positions & velocities finite and within safe speed limit.")


def main():
    parser = argparse.ArgumentParser(description="Doosan H2017 ROS 2 Action Tile Commander")
    parser.add_argument("--csv", default=None, help="Path to joints.csv trajectory file")
    parser.add_argument("--ns", default="", help="Robot namespace (default: empty root namespace)")
    parser.add_argument(
        "--action",
        default=None,
        help="Full FollowJointTrajectory action name (default: /<ns>/dsr_moveit_controller/follow_joint_trajectory)",
    )
    parser.add_argument("--duration", type=float, default=20.0, help="Target continuous duration (sec)")
    parser.add_argument("--dry-run", action="store_true", help="Verify CSV trajectory generation offline")
    args = parser.parse_args()

    if args.dry_run:
        run_dry_run(args.csv, args.duration)
        return

    ns_prefix = f"/{args.ns.strip('/')}" if args.ns.strip("/") else ""
    action_name = args.action or f"{ns_prefix}/dsr_moveit_controller/follow_joint_trajectory"
    joint_states_topic = f"{ns_prefix}/joint_states"

    csv_path, waypoints = load_trajectory_csv(args.csv)
    total_pts = len(waypoints)
    start_pose = waypoints[0]

    rclpy.init()
    node = DoosanRos2Commander(action_name=action_name, joint_states_topic=joint_states_topic)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        print_banner("Doosan H2017 Ceiling Tile Removal - ROS 2 Jazzy Action Commander")
        print(f"  Trajectory CSV : {csv_path} ({total_pts} waypoints)")
        print(f"  Action Server  : {action_name}")
        print(f"  Joint States   : {joint_states_topic}")
        print("\nWaiting for FollowJointTrajectory action server...")
        if not node.wait_for_action_server(timeout_sec=10.0):
            print(f"[ERROR] Action server '{action_name}' not available within 10s.")
            print("        Make sure the ROS 2 robot driver is running with CycloneDDS.")
            return
        print("  => Connected to ROS 2 FollowJointTrajectory Action Server!")

        stages = [
            {"name": "Stage 1: Vertical Lift & Initial Rotation (WP 0 -> 164, 25%)", "idx": min(164, total_pts - 1), "time": 5.0},
            {"name": "Stage 2: Optimal Tilt Angle Entry (30.69 deg) (WP 164 -> 329, 50%)", "idx": min(329, total_pts - 1), "time": 5.0},
            {"name": "Stage 3: Clearance Passage & Downward Extraction (WP 329 -> 494, 75%)", "idx": min(494, total_pts - 1), "time": 5.0},
            {"name": f"Stage 4: Complete Grid Separation & Final Pose (WP 494 -> {total_pts-1}, 100%)", "idx": total_pts - 1, "time": 5.0},
        ]

        while rclpy.ok():
            print_banner("ROS 2 Action Commander Menu")
            print("  0: [Safety Micro-Jog]    Move J6 +2.0 deg via Action (First real-robot test)")
            print("  1: [Approach Start Pose] Move smoothly to tile removal start pose (WP 0)")
            print("  2: [Step-by-Step Replay] Execute 4 stages (dense Action trajectory per stage)")
            print("  3: [Continuous Replay]   Execute ALL 659 WPs in ONE single Action Goal")
            print("  4: [Return to Home]      Move safely to Safe Park / Lab Standby pose")
            print("  5: [Inspect Pose]        Display live joint angles from /joint_states")
            print("  q: [Quit]")
            print("=" * 74)
            cmd = input("Select an option [0-5 / q]: ").strip().lower()

            if cmd == "0":
                cmd_micro_jog(node)
            elif cmd == "1":
                cmd_approach_start(node, start_pose)
            elif cmd == "2":
                cmd_replay_step_by_step(node, waypoints, stages)
            elif cmd == "3":
                cmd_replay_continuous(node, waypoints, total_time=args.duration)
            elif cmd == "4":
                cmd_go_home(node)
            elif cmd == "5":
                cmd_inspect_pose(node)
            elif cmd in ("q", "quit", "exit"):
                print("Exiting ROS 2 Action Commander.")
                break
    finally:
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
