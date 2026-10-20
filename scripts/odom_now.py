#!/usr/bin/env python3
"""Print the robot's odometry position once (x, y in metres, heading in degrees).

Usage: run it before and after a test drive; the difference is how far the robot THINKS it moved.
  python3 src/my_bot/scripts/odom_now.py            # prints the current pose
  python3 src/my_bot/scripts/odom_now.py --since    # also prints the change since the previous run
"""
import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

MEMORY = os.path.expanduser('~/.odom_now_last.json')


def main():
    rclpy.init()
    node = Node('odom_now')
    got = []
    node.create_subscription(Odometry, '/odom', got.append, 10)
    end = time.time() + 5
    while not got and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    if not got:
        sys.exit('No /odom received. Are the Pi programs running?')
    p = got[-1].pose.pose
    q = p.orientation
    yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
    now = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw}
    print('odometry now: x = %.3f m, y = %.3f m, heading = %.1f deg' % (now['x'], now['y'], now['yaw']))
    if '--since' in sys.argv and os.path.exists(MEMORY):
        old = json.load(open(MEMORY))
        dist = math.hypot(now['x'] - old['x'], now['y'] - old['y'])
        turn = (now['yaw'] - old['yaw'] + 180) % 360 - 180
        print('since the previous run: moved %.3f m in a straight line, turned %.1f deg' % (dist, turn))
    json.dump(now, open(MEMORY, 'w'))


if __name__ == '__main__':
    main()
