#!/usr/bin/env python3
"""Drive straight at SPEED m/s for SECS s (forward only), then stop; report wheel speeds and odometry distance."""
import sys, time, math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.15
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6.7
rclpy.init()
n = rclpy.create_node('drive_test')
pub = n.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
odom, js = [], []
n.create_subscription(Odometry, '/odom', lambda m: odom.append((m.pose.pose.position.x, m.pose.pose.position.y,
                      2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w))), 10)
n.create_subscription(JointState, '/joint_states', lambda m: js.append(dict(zip(m.name, m.velocity))), 10)
end = time.time() + 2.0
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
x0, y0, th0 = odom[-1]
cmd = Twist(); cmd.linear.x = speed
t0 = time.time(); js.clear()
while time.time() - t0 < secs:
    pub.publish(cmd); rclpy.spin_once(n, timeout_sec=0.05)
for _ in range(20):
    pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
end = time.time() + 1.5
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
x1, y1, th1 = odom[-1]
fwd = (x1 - x0) * math.cos(th0) + (y1 - y0) * math.sin(th0)
side = -(x1 - x0) * math.sin(th0) + (y1 - y0) * math.cos(th0)
l = sum(v.get('left_wheel_joint', 0) for v in js) / max(1, len(js))
r = sum(v.get('right_wheel_joint', 0) for v in js) / max(1, len(js))
print('odometry: %.2f m forward, %.2f m sideways (+ = left), turned %.0f deg' % (fwd, side, math.degrees(th1 - th0)))
print('average wheel speed: left %.2f, right %.2f rad/s (right/left %.2f)' % (l, r, r / l if l else float('nan')))
n.destroy_node(); rclpy.shutdown()
