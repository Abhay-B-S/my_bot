#!/usr/bin/env python3
"""Floor self-calibration: measures how far the robot REALLY turns on this floor, with the lidar, and computes the
wheel_separation_multiplier for config/my_controllers.yaml.

Why: turning on the spot slips by a floor-dependent amount (2026-10-26 on marble: 360 deg of wheel odometry was only
~220 deg real), so the multiplier found on one floor is wrong on another. Re-run this on every new floor.

How: the robot turns on the spot in `step_deg` steps by its own odometry and stops after each one. The lidar scan
after each step is matched to the one before (2D point matching, ICP), which gives the real turn of that step; small
steps cannot be mistaken for a different angle. The sum over a full odometry turn gives
    new multiplier = current multiplier * odometry turn / real turn.

  python3 src/my_bot/scripts/calibrate_turn.py              (robot on the floor, ~1 m free around it)
  python3 src/my_bot/scripts/calibrate_turn.py --turns 2    (two full odometry turns, a little more accurate)
It drives /diff_cont/cmd_vel_unstamped directly (keep teleop and Nav2 idle) and only PRINTS the new value; put it into
config/my_controllers.yaml on the Pi and restart launch_robot to use it.
"""
import argparse
import math
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def scan_points(scans, max_range=6.0):
    """Median of a few scans -> Nx2 points in the laser frame."""
    r = np.array([s.ranges for s in scans], dtype=float)
    r[~np.isfinite(r) | (r < 0.15) | (r > max_range)] = np.nan
    r = np.nanmedian(r, axis=0)
    s = scans[-1]
    a = s.angle_min + np.arange(len(r)) * s.angle_increment
    ok = np.isfinite(r)
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def icp(a, b, angle0, iters=30, reject=0.3):
    """Rotation (rad) and translation that map points b onto points a (a ~ R b + t), starting at angle0."""
    th, t = angle0, np.zeros(2)
    for _ in range(iters):
        c, s = math.cos(th), math.sin(th)
        bb = b @ np.array([[c, s], [-s, c]]) + t
        d = ((bb[:, None, :] - a[None, :, :]) ** 2).sum(axis=2)
        j = d.argmin(axis=1)
        keep = np.sqrt(d[np.arange(len(bb)), j]) < reject
        if keep.sum() < 20:
            return None
        p, q = b[keep], a[j[keep]]
        pm, qm = p.mean(axis=0), q.mean(axis=0)
        h = (p - pm).T @ (q - qm)
        th = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
        c, s = math.cos(th), math.sin(th)
        t = qm - pm @ np.array([[c, s], [-s, c]])
        reject = max(0.08, reject * 0.85)
    return th, t, int(keep.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--turns', type=float, default=1.0, help='full odometry turns (default 1)')
    ap.add_argument('--step-deg', type=float, default=30.0)
    ap.add_argument('--speed', type=float, default=0.5, help='rad/s (too slow stalls a heavy robot)')
    ap.add_argument('--multiplier', type=float, default=None, help='current wheel_separation_multiplier')
    args, _ = ap.parse_known_args()

    rclpy.init()
    n = rclpy.create_node('calibrate_turn')
    pub = n.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
    yaws, scans = [], []
    n.create_subscription(Odometry, '/odom', lambda m: yaws.append(
        2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)), 10)
    n.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos_profile_sensor_data)

    def spin_for(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    def grab():
        scans.clear()
        end = time.time() + 5
        while len(scans) < 4 and time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.05)
        return scan_points(scans)

    end = time.time() + 15.0                              # discovery over WiFi can take a few seconds
    while (not yaws or not scans) and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)
    if not yaws or not scans:
        raise SystemExit('No /odom or /scan: is the robot (launch_robot + lidar) running?')
    mult = args.multiplier
    if mult is None:
        try:
            out = subprocess.run(['ros2', 'param', 'get', '/diff_cont', 'wheel_separation_multiplier'],
                                 capture_output=True, text=True, timeout=15).stdout
            mult = float(out.strip().split()[-1])
        except Exception:
            raise SystemExit('Could not read the current multiplier; give it with --multiplier 1.47')

    steps = int(round(args.turns * 360.0 / args.step_deg))
    step = math.radians(args.step_deg)
    odo_total = real_total = 0.0
    prev = grab()
    print('Current wheel_separation_multiplier: %.3f. Turning %d x %.0f deg (odometry)...' % (mult, steps, args.step_deg))
    for i in range(steps):
        last = yaws[-1]
        turned = 0.0
        t0 = time.time()
        while turned < step and time.time() - t0 < 15:
            cmd = Twist(); cmd.angular.z = args.speed
            pub.publish(cmd)
            rclpy.spin_once(n, timeout_sec=0.02)
            y = yaws[-1]; turned += math.atan2(math.sin(y - last), math.cos(y - last)); last = y
        for _ in range(10):
            pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.03)
        spin_for(0.8)                                         # let it settle; count the coast too
        y = yaws[-1]; turned += math.atan2(math.sin(y - last), math.cos(y - last))
        cur = grab()
        res = icp(prev, cur, turned)
        if res is None:
            print('  step %2d: odometry %5.1f deg, lidar match FAILED (too few points)' % (i + 1, math.degrees(turned)))
            prev = cur
            continue
        real, _, used = res
        odo_total += turned; real_total += real
        print('  step %2d: odometry %5.1f deg, real %5.1f deg (%d points matched)'
              % (i + 1, math.degrees(turned), math.degrees(real), used))
        prev = cur
    for _ in range(10):
        pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.03)
    if real_total <= 0.1:
        raise SystemExit('No usable measurement.')
    new = mult * odo_total / real_total
    print('\nOdometry %.0f deg, lidar %.0f deg -> the wheels count %.0f %% of the real turn.'
          % (math.degrees(odo_total), math.degrees(real_total), 100 * odo_total / real_total))
    print('Recommended wheel_separation_multiplier for this floor: %.2f  (now %.2f)' % (new, mult))
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
