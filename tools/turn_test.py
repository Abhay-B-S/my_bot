"""Turn on the spot by DEG degrees (anticlockwise, by wheel odometry), slowing down near the end; report the total."""
import sys, time, math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
deg = float(sys.argv[1]) if len(sys.argv) > 1 else 360.0
rclpy.init(); n = rclpy.create_node('turn_test')
pub = n.create_publisher(Twist, '/cmd_vel_key', 10)
yaws = []
n.create_subscription(Odometry, '/odom', lambda m: yaws.append(2 * math.atan2(m.pose.pose.orientation.z, m.pose.pose.orientation.w)), 10)
t = time.time()
while len(yaws) < 5 and time.time() - t < 10:
    rclpy.spin_once(n, timeout_sec=0.05)
total, last = 0.0, yaws[-1]
goal = math.radians(deg)
t0 = time.time()
while total < goal and time.time() - t0 < 60:
    rclpy.spin_once(n, timeout_sec=0.02)
    y = yaws[-1]; total += math.atan2(math.sin(y - last), math.cos(y - last)); last = y
    left = goal - total
    cmd = Twist(); cmd.angular.z = max(0.08, min(0.4, 0.8 * left))
    pub.publish(cmd)
for _ in range(30):
    pub.publish(Twist()); rclpy.spin_once(n, timeout_sec=0.05)
end = time.time() + 1.5
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05)
    y = yaws[-1]; total += math.atan2(math.sin(y - last), math.cos(y - last)); last = y
print('odometry says it turned %.1f degrees' % math.degrees(total))
