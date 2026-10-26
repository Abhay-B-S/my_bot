"""Share of lidar points that land on (or within 10 cm of) a wall of the map, at the robot's current position."""
import math, time
import numpy as np, rclpy, tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
rclpy.init(); n = rclpy.create_node('match_now'); b = tf2_ros.Buffer(); tf2_ros.TransformListener(b, n)
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
mp, sc = [], []
n.create_subscription(OccupancyGrid, '/map', lambda m: mp.append(m), latched)
n.create_subscription(LaserScan, '/scan', lambda m: sc.append(m), qos_profile_sensor_data)
t = time.time(); tr = None
while time.time() - t < 10 and (not mp or not sc or tr is None):
    rclpy.spin_once(n, timeout_sec=0.1)
    if sc:
        try: tr = b.lookup_transform('map', sc[-1].header.frame_id, rclpy.time.Time())
        except Exception: pass
m, s = mp[-1], sc[-1]; info = m.info
g = np.array(m.data).reshape(info.height, info.width) >= 65
near = g.copy()
for dx in range(-2, 3):
    for dy in range(-2, 3):
        near |= np.roll(np.roll(g, dx, 1), dy, 0)
r = np.array(s.ranges); a = s.angle_min + np.arange(len(r)) * s.angle_increment
ok = np.isfinite(r) & (r > s.range_min) & (r < 6.0)
q = tr.transform.rotation; yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
x = tr.transform.translation.x + r[ok] * np.cos(a[ok] + yaw); y = tr.transform.translation.y + r[ok] * np.sin(a[ok] + yaw)
cx = ((x - info.origin.position.x) / info.resolution).astype(int); cy = ((y - info.origin.position.y) / info.resolution).astype(int)
inside = (cx >= 0) & (cx < info.width) & (cy >= 0) & (cy < info.height)
hit = np.zeros(len(x), bool); hit[inside] = near[cy[inside], cx[inside]]
print('lidar matches the map: %.0f %%' % (100 * hit.mean()))
