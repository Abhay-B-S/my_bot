"""Measure the REAL rotation of an odometry turn with the lidar: scan before, turn DEG by odometry, scan after;
the angle by which the second scan must be rotated to match the first = real turn minus DEG (mod 360)."""
import sys, time, math, subprocess
import numpy as np, rclpy
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
deg = float(sys.argv[1]) if len(sys.argv) > 1 else 360.0
rclpy.init(); n = rclpy.create_node('lidar_turn_check'); scans = []
n.create_subscription(LaserScan, '/scan', lambda m: scans.append(m), qos_profile_sensor_data)
def grab(k=8):
    scans.clear(); t = time.time()
    while len(scans) < k and time.time() - t < 10: rclpy.spin_once(n, timeout_sec=0.1)
    r = np.array([s.ranges for s in scans[-k:]], dtype=float)
    r[~np.isfinite(r) | (r < 0.15)] = np.nan
    return np.nanmedian(r, axis=0), scans[-1]
before, s0 = grab()
subprocess.run([sys.executable, sys.argv[0].replace('lidar_turn_check', 'turn_test'), str(deg)])
time.sleep(1.0)
after, _ = grab()
N = len(before); inc = math.degrees(s0.angle_increment)
b = np.nan_to_num(before, nan=0.0); best = None
for k in range(N):                                         # after rotated by k beams vs before
    a = np.roll(np.nan_to_num(after, nan=0.0), k)
    ok = (b > 0) & (a > 0)
    err = np.mean(np.abs(a[ok] - b[ok])) if ok.sum() > N // 3 else 1e9
    if best is None or err < best[0]: best = (err, k)
shift = best[1] * inc                                      # degrees the room appears rotated
shift = (shift + 180) % 360 - 180
print('lidar: the room looks rotated by %.0f deg after the turn (mean range error %.2f m)' % (shift, best[0]))
print('=> real turn about %.0f deg for %.0f deg of odometry' % (deg + shift, deg))
