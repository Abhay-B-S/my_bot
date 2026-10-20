#!/usr/bin/env python3
"""Quick health check of the REAL robot's data (run after the two Pi programs are up; nothing moves).

Usage:  python3 src/my_bot/scripts/robot_check.py

Prints: what the lidar sees (valid readings, nearest obstacle per direction), whether the wheel counters
and odometry stand still, and whether the odom -> laser_frame transform exists.
"""
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    n = Node('robot_check')
    got = {}

    def keep(prefix):
        def cb(m):
            got.setdefault(prefix + '0', m)
            got[prefix + '1'] = m
        return cb

    n.create_subscription(LaserScan, '/scan', keep('scan'), qos_profile_sensor_data)
    n.create_subscription(Odometry, '/odom', keep('odom'), 10)
    n.create_subscription(JointState, '/joint_states', keep('js'), 10)
    buf = Buffer()
    TransformListener(buf, n)
    end = time.time() + 6
    while time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.1)

    s = got.get('scan1')
    if s:
        ok = [r for r in s.ranges if math.isfinite(r) and s.range_min <= r <= s.range_max]
        print('scan: frame %s, %d beams, %d valid (%.0f%%), nearest %.2f m, farthest %.2f m' % (
            s.header.frame_id, len(s.ranges), len(ok), 100 * len(ok) / len(s.ranges), min(ok), max(ok)))
        sec = {}
        for i, r in enumerate(s.ranges):
            if math.isfinite(r) and s.range_min <= r <= s.range_max:
                a = (s.angle_min + i * s.angle_increment) % (2 * math.pi)
                k = int(((a + math.pi / 8) % (2 * math.pi)) // (math.pi / 4))
                sec[k] = min(sec.get(k, 99), r)
        names = ['ahead', 'ahead-left', 'left', 'behind-left', 'behind', 'behind-right', 'right', 'ahead-right']
        print('  nearest per direction:', ', '.join('%s %.2f' % (names[k], sec[k]) for k in sorted(sec)))
    else:
        print('NO scan message received')
    o0, o1 = got.get('odom0'), got.get('odom1')
    if o0 and o1:
        print('odom %s -> %s: x %.4f -> %.4f, y %.4f -> %.4f (unchanged = standing still)' % (
            o0.header.frame_id, o0.child_frame_id, o0.pose.pose.position.x, o1.pose.pose.position.x,
            o0.pose.pose.position.y, o1.pose.pose.position.y))
    else:
        print('NO odometry received')
    j0, j1 = got.get('js0'), got.get('js1')
    if j0 and j1:
        print('wheel joints', list(j0.name), 'first', [round(v, 3) for v in j0.position],
              'last', [round(v, 3) for v in j1.position])
    try:
        t = buf.lookup_transform('odom', 'laser_frame', rclpy.time.Time())
        print('TF odom -> laser_frame: OK (%.3f, %.3f, %.3f)' % (
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z))
    except Exception:
        print('TF odom -> laser_frame: MISSING')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
