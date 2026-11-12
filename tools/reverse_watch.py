#!/usr/bin/env python3
"""Logs which cmd_vel topic asks the robot to REVERSE (linear.x < -0.01) and the final output. For tracing surprise backing-up."""
import time, rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
T = ['/cmd_vel', '/cmd_vel_assist', '/cmd_vel_guard', '/cmd_vel_key', '/cmd_vel_recovery', '/cmd_vel_ultrasonic', '/diff_cont/cmd_vel_unstamped']
class W(Node):
    def __init__(s):
        super().__init__('reverse_watch'); s.last = {}
        for t in T: s.create_subscription(Twist, t, lambda m, t=t: s.cb(t, m), 10)
    def cb(s, t, m):
        if m.linear.x < -0.01 and time.time() - s.last.get(t, 0) > 1.0:
            s.last[t] = time.time()
            print('%s  %s  x=%.2f z=%.2f' % (time.strftime('%H:%M:%S'), t, m.linear.x, m.angular.z), flush=True)
rclpy.init(); rclpy.spin(W())
