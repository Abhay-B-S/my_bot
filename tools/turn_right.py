#!/usr/bin/env python3
"""Turn right on the spot at 0.5 rad/s (keyboard priority) until the robot faces ~180 deg (towards cabin A)."""
import math, time, rclpy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node('turn_right')
pub = n.create_publisher(Twist, '/cmd_vel_key', 10)
st = {'odo': None, 'amcl': None}
n.create_subscription(Odometry, '/odom', lambda m: st.update(odo=2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)), 10)
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: st.update(amcl=math.degrees(2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w))), q)
t = time.time()
while (st['odo'] is None or st['amcl'] is None) and time.time() - t < 10:
    rclpy.spin_once(n, timeout_sec=0.1)
print('start: map heading %.0f deg' % st['amcl'])
last = st['odo']; turned = 0.0; t0 = time.time()
while time.time() - t0 < 14:
    c = Twist(); c.angular.z = -0.5; pub.publish(c)
    rclpy.spin_once(n, timeout_sec=0.05)
    d = st['odo'] - last; turned += math.atan2(math.sin(d), math.cos(d)); last = st['odo']
    a = st['amcl']
    if abs(math.degrees(turned)) >= 125 or (a is not None and (a <= -168 or a >= 170)):
        break
for _ in range(15):
    pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
time.sleep(1.5)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.05)
print('stopped after %.1f s: odometry turned %.0f deg, map heading now %.0f deg' % (time.time() - t0, math.degrees(turned), st['amcl']))
