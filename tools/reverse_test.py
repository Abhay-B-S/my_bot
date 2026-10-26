#!/usr/bin/env python3
"""Wheels LIFTED: forward / stop / backward / stop cycles at gentle speeds; reports wheel-counter jumps
(a jump = the motor Arduino restarted). Runs ON THE PI, talking to diff_cont directly."""
import sys, time
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
cycles = int(sys.argv[1]) if len(sys.argv) > 1 else 3
fwd, back = 0.10, -0.05
rclpy.init(); n = rclpy.create_node('reverse_test')
pub = n.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
pos = []
n.create_subscription(JointState, '/joint_states', lambda m: pos.append((time.time(), dict(zip(m.name, m.position)))), 10)
def run(v, secs):
    t = time.time(); cmd = Twist(); cmd.linear.x = v
    while time.time() - t < secs:
        pub.publish(cmd); rclpy.spin_once(n, timeout_sec=0.05)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.05)
for c in range(cycles):
    for label, v, secs in (('forward', fwd, 2.0), ('stop', 0.0, 1.5), ('BACKWARD', back, 2.0), ('stop', 0.0, 2.0)):
        p0 = len(pos); run(v, secs)
        seg = pos[p0:]
        jump = 0.0
        for (t1, a), (t2, b) in zip(seg, seg[1:]):
            for j in ('left_wheel_joint', 'right_wheel_joint'):
                if j in a and j in b and t2 > t1:
                    jump = max(jump, abs(b[j] - a[j]) / (t2 - t1))
        l = seg[-1][1].get('left_wheel_joint', 0) - seg[0][1].get('left_wheel_joint', 0) if len(seg) > 1 else 0
        r = seg[-1][1].get('right_wheel_joint', 0) - seg[0][1].get('right_wheel_joint', 0) if len(seg) > 1 else 0
        print('cycle %d %-8s wheels moved L %+5.2f R %+5.2f rad, fastest change %.1f rad/s %s'
              % (c + 1, label, l, r, jump, '<-- JUMP (Arduino restart?)' if jump > 30 else ''), flush=True)
for _ in range(10): pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
