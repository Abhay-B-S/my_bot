#!/usr/bin/env python3
"""Health check (start-up self-test that keeps running): is everything the robot needs working right now?

Every `period` s it checks, from the laptop, and gives each item ok / warn / fail with a short reason:
  lidar           /scan arriving at a normal rate (the X2 gives about 7 per second)
  motors          wheel odometry (/odom) arriving: the motor controller on the Pi runs
  position        share of lidar points on the map's walls (/localization_match from self_recovery)
  navigation      Nav2's navigate_to_pose action server is ready
  delivery        the delivery manager's state (/delivery_status), emergency stop
  lid             /cargo_state (or "simulated")
  battery         /battery_status (only when the battery monitor runs)
  wifi            the robot's signal (/wifi_signal from wifi_probe on the Pi)
  programs        every program of the delivery system is running (on the laptop AND on the Pi)
Publishes /health (String JSON, latched): {t, overall, checks: [{name, level, text}]}; firebase_bridge.py forwards it
to the website (Admin: "Robot health"). Also printed in the terminal at start-up, as a self-test before a demo.

  python3 src/my_bot/scripts/health_check.py
"""
import json
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose

# programs (ROS node names) and where they run; missing optional ones only give a warning
PROGRAMS = [
    ('ydlidar_ros2_driver_node', 'Pi', True), ('controller_manager', 'Pi', True), ('safety_watchdog', 'Pi', True),
    ('wifi_probe', 'Pi', False),
    ('amcl', 'laptop', True), ('bt_navigator', 'laptop', True), ('controller_server', 'laptop', True),
    ('planner_server', 'laptop', True), ('twist_mux', 'laptop', True), ('delivery_manager', 'laptop', True),
    ('firebase_bridge', 'laptop', True), ('self_recovery', 'laptop', True), ('path_guard', 'laptop', True),
    ('gap_assist', 'laptop', True), ('event_logger', 'laptop', False), ('eta_estimator', 'laptop', False),
    ('wifi_mapper', 'laptop', False), ('change_detector', 'laptop', False), ('delivery_stats', 'laptop', False),
    ('cargo_controller', 'Pi or laptop', True),
]
ORDER = {'ok': 0, 'warn': 1, 'fail': 2}


class HealthCheck(Node):
    def __init__(self):
        super().__init__('health_check')
        self.declare_parameter('period', 30.0)
        self.declare_parameter('skip', '')    # programs not expected here, comma-separated (simulator: the Pi's)
        self.skip = {n.strip() for n in self.get_parameter('skip').value.split(',') if n.strip()}
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.scans, self.odoms = [], []
        self.last = {}                    # topic -> (time, data)
        self.create_subscription(LaserScan, '/scan', lambda m: self.scans.append(time.monotonic()),
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', lambda m: self.odoms.append(time.monotonic()), 10)
        for topic, qos in (('/localization_match', 10), ('/localization_status', latched),
                           ('/delivery_status', latched), ('/cargo_state', latched), ('/battery_status', latched),
                           ('/wifi_signal', 10)):
            self.create_subscription(String, topic, lambda m, t=topic: self.last.__setitem__(t, (time.monotonic(), m.data)),
                                     qos)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.pub = self.create_publisher(String, '/health', latched)
        self.first = True
        self.create_timer(5.0, self._first_check)           # the first one soon after start (topics need a moment)
        self.create_timer(self.get_parameter('period').value, self._check)

    def _first_check(self):
        if self.first:
            self.first = False
            self._check()

    def _fresh(self, topic, seconds):
        v = self.last.get(topic)
        return v[1] if v and time.monotonic() - v[0] < seconds else None

    @staticmethod
    def _rate(stamps):
        now = time.monotonic()
        stamps[:] = [t for t in stamps if now - t < 10.0]
        return len(stamps) / 10.0

    def _checks(self):
        out = []
        r = self._rate(self.scans)
        out.append(('lidar', 'ok' if r >= 4 else 'warn' if r > 0 else 'fail',
                    '%.1f scans per second' % r if r else 'no lidar data: is the lidar running on the Pi?'))
        r = self._rate(self.odoms)
        out.append(('motors', 'ok' if r >= 5 else 'fail',
                    'motor controller answering (%.0f odometry messages per second)' % r if r >= 5
                    else 'no wheel odometry: start the motor controller on the Pi (motor battery on?)'))
        m = self._fresh('/localization_match', 15)
        loc = self._fresh('/localization_status', 1e9) or ''
        if m is None:
            out.append(('position', 'warn', 'no map match yet (self-recovery not running or no map)'))
        else:
            v = float(m)
            level = 'ok' if v >= 0.6 and loc in ('ok', 'recovered', '') else 'warn' if v >= 0.45 else 'fail'
            out.append(('position', level, '%.0f %% of the lidar matches the map%s' % (
                100 * v, '' if loc in ('ok', 'recovered', '') else ' (%s)' % loc.split(':')[0])))
        ready = self.nav.server_is_ready()
        out.append(('navigation', 'ok' if ready else 'fail', 'Nav2 ready' if ready else 'Nav2 is not ready'))
        st = self._fresh('/delivery_status', 1e9)
        if st is None:
            out.append(('delivery', 'fail', 'the delivery manager is not running'))
        else:
            d = json.loads(st)
            if d.get('estop'):
                out.append(('delivery', 'warn', 'EMERGENCY STOP is on: press Resume on the website'))
            else:
                out.append(('delivery', 'ok', '%s, %d waiting: %s' % (d.get('state'), d.get('queue', 0),
                                                                      (d.get('detail') or '')[:80])))
        lid = self._fresh('/cargo_state', 1e9)
        out.append(('lid', 'ok' if lid in ('locked', 'unlocked', 'moving') else 'warn',
                    'lid %s' % lid if lid else 'no lid controller (lid buttons will wait)'))
        b = self._fresh('/battery_status', 30)
        if b is not None:
            b = json.loads(b)
            sim = ' (simulated)' if b.get('source') == 'sim' else ''
            if b.get('error'):
                out.append(('battery', 'warn', 'battery sensor not answering'))
            elif b.get('state') == 'off':
                out.append(('battery', 'fail', 'motor power switched off'))
            else:
                out.append(('battery', {'ok': 'ok', 'charging': 'ok', 'low': 'warn'}.get(b.get('state'), 'fail'),
                            '%s %%, %.1f V%s' % (b.get('pct'), b.get('v', 0), sim)))
        w = self._fresh('/wifi_signal', 15)
        if w is not None:
            dbm = json.loads(w).get('dbm')
            out.append(('wifi', 'ok' if dbm >= -67 else 'warn' if dbm >= -78 else 'fail',
                        'robot signal %.0f dBm%s' % (dbm, '' if dbm >= -67 else ': move the hotspot closer')))
        names = set(self.get_node_names())
        missing = [(n, where, need) for n, where, need in PROGRAMS if n not in names and n not in self.skip]
        bad = [m for m in missing if m[2]]
        out.append(('programs', 'fail' if bad else 'warn' if missing else 'ok',
                    'all %d running' % (len(PROGRAMS) - len(self.skip)) if not missing else
                    'not running: ' + ', '.join('%s (%s)' % (n, where) for n, where, _ in missing)))
        return out

    def _check(self):
        checks = self._checks()
        overall = max((c[1] for c in checks), key=ORDER.get)
        self.pub.publish(String(data=json.dumps({
            't': round(time.time()), 'overall': overall,
            'checks': [{'name': n, 'level': l, 'text': t} for n, l, t in checks]})))
        mark = {'ok': '[OK]', 'warn': '[!!]', 'fail': '[XX]'}
        self.get_logger().info('health %s: %s' % (overall.upper(), '; '.join(
            '%s %s %s' % (mark[l], n, t) for n, l, t in checks if l != 'ok') or 'everything OK'))


def main():
    rclpy.init()
    node = HealthCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.nav.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
