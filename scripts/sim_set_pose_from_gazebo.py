#!/usr/bin/env python3
"""SIMULATION ONLY: tell AMCL where the robot really is, using Gazebo's true pose.

Use it when the red laser arcs in RViz no longer sit on the obstacles (AMCL lost the robot) instead of
guessing a 2D Pose Estimate. Works because the Gazebo world frame equals the map frame in our sim
(the robot spawns at the origin). It cannot be used on the real robot.

Usage:  python3 src/my_bot/scripts/sim_set_pose_from_gazebo.py
"""
import math
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

out = subprocess.run(['gz', 'model', '-m', 'my_bot', '-p'], capture_output=True, text=True).stdout.split()
if len(out) < 6:
    raise SystemExit('Could not read the robot pose from Gazebo (is the simulation running?)')
x, y, yaw = float(out[0]), float(out[1]), float(out[5])

rclpy.init()
node = rclpy.create_node('set_initial_pose')
pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
msg = PoseWithCovarianceStamped()
msg.header.frame_id = 'map'
msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
cov = [0.0] * 36
cov[0] = cov[7] = 0.05
cov[35] = 0.03
msg.pose.covariance = cov
for _ in range(6):                                # a few copies, so AMCL surely receives one
    msg.header.stamp = rclpy.time.Time().to_msg()  # time 0 = "latest"
    pub.publish(msg)
    time.sleep(0.4)
print('Set initial pose to x=%.3f y=%.3f yaw=%.3f' % (x, y, yaw))
node.destroy_node()
rclpy.shutdown()
