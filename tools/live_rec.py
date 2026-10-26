#!/usr/bin/env python3
"""Records a delivery run: robot pose (AMCL), wheel commands, lidar match, every 0.5 s -> CSV, for later analysis."""
import json, math, sys, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

out = open(sys.argv[1], 'a', buffering=1)
rclpy.init()
n = rclpy.create_node('live_rec')
st = {'x': None, 'y': None, 'yaw': None, 'vx': 0.0, 'wz': 0.0, 'src': '', 'match': '', 'state': ''}
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

def pose(m):
    p = m.pose.pose
    st.update(x=p.position.x, y=p.position.y, yaw=math.degrees(2 * math.atan2(p.orientation.z, p.orientation.w)))

def cmd(src):
    def f(m):
        st.update(vx=m.linear.x, wz=m.angular.z, src=src)
    return f

n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', pose, q)
for topic, src in (('/cmd_vel', 'nav'), ('/cmd_vel_guard', 'guard'), ('/cmd_vel_recovery', 'recov')):
    n.create_subscription(Twist, topic, cmd(src), 10)
n.create_subscription(String, '/localization_match', lambda m: st.update(match=m.data), 10)
n.create_subscription(String, '/delivery_status', lambda m: st.update(state=json.loads(m.data).get('state', '')), q)
out.write('time,x,y,yaw,vx,wz,cmd_from,match,status\n')

def tick():
    if st['x'] is None:
        return
    out.write('%.1f,%.2f,%.2f,%.0f,%.2f,%.2f,%s,%s,"%s"\n' % (time.time(), st['x'], st['y'], st['yaw'], st['vx'], st['wz'],
                                                       st['src'], st['match'], st['state'].replace('"', "'")))
    st.update(vx=0.0, wz=0.0, src='')

n.create_timer(0.5, tick)
rclpy.spin(n)
