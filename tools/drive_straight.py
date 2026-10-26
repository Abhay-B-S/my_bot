#!/usr/bin/env python3
"""Drive straight ahead DIST m at 0.15 m/s (keyboard priority), measured by wheel odometry."""
import math, sys, time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
dist = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6
rclpy.init(); n = rclpy.create_node('drive_straight')
pub = n.create_publisher(Twist, '/cmd_vel_key', 10)
st = {}
n.create_subscription(Odometry, '/odom', lambda m: st.update(x=m.pose.pose.position.x, y=m.pose.pose.position.y), 10)
t = time.time()
while 'x' not in st and time.time() - t < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
x0, y0 = st['x'], st['y']; t0 = time.time(); d = 0.0
while d < dist and time.time() - t0 < dist / 0.15 + 4:
    c = Twist(); c.linear.x = 0.15; pub.publish(c)
    rclpy.spin_once(n, timeout_sec=0.05)
    d = math.hypot(st['x'] - x0, st['y'] - y0)
for _ in range(15):
    pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
print('drove %.2f m in %.1f s' % (d, time.time() - t0))
