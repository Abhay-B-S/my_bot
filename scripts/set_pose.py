#!/usr/bin/env python3
"""Tell localisation (AMCL) that the robot stands at a recorded spot: the script version of RViz's 2D Pose Estimate.

  python3 src/my_bot/scripts/set_pose.py home --file src/my_bot/config/locations_friend.yaml
  python3 src/my_bot/scripts/set_pose.py home --file ... --ros-args -p use_sim_time:=true    (simulation)

Put the robot on the spot, facing the recorded direction, before running it.
"""
import argparse
import math
import sys
import time

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('spot')
    ap.add_argument('--file', required=True)
    args, _ = ap.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    spots = yaml.safe_load(open(args.file))['locations']
    if args.spot not in spots:
        sys.exit('No spot "%s" in %s (spots: %s)' % (args.spot, args.file, ', '.join(spots)))
    p = spots[args.spot]
    rclpy.init()
    node = rclpy.create_node('set_pose')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(p['x']), float(p['y'])
    msg.pose.pose.orientation.z = math.sin(float(p['yaw']) / 2)
    msg.pose.pose.orientation.w = math.cos(float(p['yaw']) / 2)
    cov = [0.0] * 36
    cov[0] = cov[7] = cov[35] = 0.05
    msg.pose.covariance = cov
    # Sent ONCE, as soon as AMCL listens, with the stamp left at 0 (= "the latest"). Foxy's AMCL crashed in
    # simulation (pf_update_resample assertion, 2026-10-24) on an old stamp and on several poses in quick succession.
    end = time.time() + 10.0
    while pub.get_subscription_count() == 0 and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        sys.exit('Nobody listens on /initialpose: is localisation (AMCL) running?')
    time.sleep(0.5)                         # let the connection settle
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.5)
    print('Robot position set to %s (x=%.2f y=%.2f)' % (args.spot, p['x'], p['y']))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
