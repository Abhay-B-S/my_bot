#!/usr/bin/env python3
"""Save the robot's current map-frame pose (from AMCL) under a name.

Usage:  python3 src/my_bot/scripts/record_location.py cabin_A   (a trailing --sim is accepted and ignored)
Drive the robot to the spot first. The pose is written into
src/my_bot/config/locations.yaml as  name: {x, y, yaw}  (map frame, metres/radians).
"""
import math
import os
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

LOCATIONS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              '..', 'config', 'locations.yaml')


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    if len(args) != 1:
        print(__doc__)
        sys.exit(1)
    name = args[0]

    rclpy.init()
    node = Node('record_location')   # wall clock on purpose: the timeout below must not follow /clock
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    got = []
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose',
                             lambda m: got.append(m), qos)
    node.get_logger().info('Waiting for /amcl_pose ...')
    end = time.monotonic() + 10.0
    while not got and rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()   # always shut ROS down cleanly, even when nothing was received
    if not got:
        print('No /amcl_pose received. Is localization running and the pose estimate set?')
        sys.exit(1)

    p = got[-1].pose.pose
    q = p.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    entry = {'x': round(p.position.x, 3), 'y': round(p.position.y, 3), 'yaw': round(yaw, 3)}

    data = {}
    if os.path.exists(LOCATIONS_FILE):
        with open(LOCATIONS_FILE) as f:
            data = yaml.safe_load(f) or {}
    data.setdefault('frame_id', 'map')
    data.setdefault('locations', {})[name] = entry
    with open(LOCATIONS_FILE, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False)
    print('Saved %s = %s  ->  %s' % (name, entry, os.path.normpath(LOCATIONS_FILE)))


if __name__ == '__main__':
    main()
