#!/usr/bin/env python3
"""Event logger: one time-stamped record of everything the robot decides or suffers, for the evaluation, the learned
ETA and the "Why?" explanations.

It only LISTENS to the topics the other programs already publish, so it cannot change how the robot drives:
  /delivery_status      (delivery_manager)  job queued/started, legs, arrivals, results, cancel, emergency stop
  /path_status          (path_guard)        waiting for a person, going round, detour, blocked, clear
  /localization_status  (self_recovery)     position lost / found / needs help
  /gap_assist_status    (gap_assist)        stuck in a narrow spot, way out, done
  /safety_event         (safety_watchdog)   motor stop, restart, gave up
  /battery_status       (battery_monitor)   battery low / critical / charging; energy used per leg
  /plan + TF map->base_link                 planned route length, where the robot is

Every event goes to `log_file` (one JSON object per line, appended across runs) and to /robot_event (String, JSON).
A finished drive to a spot also becomes a "leg" record (planned length, driven distance, duration, events during
it), which eta_estimator.py learns the robot's real travel speed from.

  python3 src/my_bot/scripts/event_logger.py [--ros-args -p log_file:=~/dev_ws/run_logs/events.jsonl]
"""
import json
import math
import os
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

MOVING = ('TO_PICKUP', 'TO_DROPOFF', 'RETURNING', 'RETURN_TO_SENDER')


def path_length(points):
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def start_turn(points, pose, ahead=0.5):
    """How far (deg, 0-180) the robot must turn to face the route: its heading vs the plan ~0.5 m ahead."""
    if pose is None:
        return None
    for x, y in points:
        if math.hypot(x - pose[0], y - pose[1]) >= ahead:
            want = math.degrees(math.atan2(y - pose[1], x - pose[0]))
            return round(abs((want - pose[2] + 180) % 360 - 180))
    return 0


class EventLogger(Node):
    def __init__(self):
        super().__init__('event_logger')
        self.declare_parameter('log_file', '~/dev_ws/run_logs/events.jsonl')
        path = os.path.expanduser(self.get_parameter('log_file').value)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.out = open(path, 'a', buffering=1)
        self.pub = self.create_publisher(String, '/robot_event', 50)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(String, '/delivery_status', self._on_delivery, latched)
        self.create_subscription(String, '/path_status', lambda m: self._simple('path', m.data), latched)
        self.create_subscription(String, '/localization_status', lambda m: self._simple('localisation', m.data), latched)
        self.create_subscription(String, '/gap_assist_status', lambda m: self._simple('gap_assist', m.data), latched)
        self.create_subscription(String, '/safety_event', lambda m: self._simple('motor', m.data), 10)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.create_subscription(String, '/battery_status', self._on_battery, latched)
        self.create_timer(0.5, self._track)

        self.state, self.job, self.last = None, {}, {}
        self.leg = None              # the drive in progress
        self.last_pose = None
        self.plan_len = None
        self.battery = {}            # last /battery_status
        self.state_battery = None
        self.get_logger().info('Event logger writing to %s' % path)

    # ---------- helpers ----------
    def _pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        q = t.transform.rotation
        return (round(t.transform.translation.x, 2), round(t.transform.translation.y, 2),
                round(math.degrees(2 * math.atan2(q.z, q.w))))

    def _emit(self, etype, text, **extra):
        ev = {'t': round(time.time(), 2), 'type': etype, 'text': text, 'pose': self._pose(),
              'job': self.job.get('job'), 'ref': self.job.get('ref')}
        ev.update(extra)
        line = json.dumps(ev)
        self.out.write(line + '\n')
        self.pub.publish(String(data=line))

    def _simple(self, kind, text):
        if text == self.last.get(kind):               # latched topics repeat their last value
            return
        self.last[kind] = text
        self._emit(kind, text)
        if self.leg is not None and text not in ('clear', 'ok', 'watching') and not text.startswith('ok at'):
            self.leg['events'].append(kind + ': ' + text[:80])

    # ---------- inputs ----------
    def _on_plan(self, msg):
        if len(msg.poses) > 1:
            pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.plan_len = path_length(pts)
            if self.leg is not None and self.leg['plan_m'] is None:
                self.leg['plan_m'] = round(self.plan_len, 2)   # the first plan of the leg = its planned length
                self.leg['turn_deg'] = start_turn(pts, self.leg['from'])

    def _on_delivery(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        prev, self.state = self.state, st.get('state')
        self.job = {'job': st.get('job'), 'ref': st.get('ref')}
        if st.get('detail') and st.get('detail') != self.last.get('delivery'):
            self.last['delivery'] = st['detail']
            self._emit('delivery', st['detail'], state=self.state, result=st.get('result'),
                       urgent=st.get('urgent'), estop=st.get('estop'), queue=st.get('queue'))
        if self.state != prev:
            if prev in MOVING:
                self._end_leg(prev, self.state)
            if self.state in MOVING:
                target = {'TO_PICKUP': st.get('sender'), 'TO_DROPOFF': st.get('destination'),
                          'RETURN_TO_SENDER': st.get('sender')}.get(self.state, 'home')
                self.leg = {'start': time.time(), 'kind': self.state, 'target': target, 'from': self._pose(),
                            'plan_m': None, 'driven_m': 0.0, 'events': [], 'wh0': self.battery.get('wh'),
                            'pct0': self.battery.get('pct')}
                self.plan_len = None

    def _end_leg(self, kind, new_state):
        leg, self.leg = self.leg, None
        if leg is None:
            return
        arrived = new_state in ('WAIT_SHIPPED', 'WAIT_RECEIVED', 'WAIT_RETURNED', 'IDLE')
        self._emit('leg', '%s to %s %s in %.0f s' % (kind, leg['target'], 'arrived' if arrived else 'ended',
                                                     time.time() - leg['start']),
                   kind=kind, target=leg['target'], arrived=arrived, seconds=round(time.time() - leg['start'], 1),
                   plan_m=leg['plan_m'], driven_m=round(leg['driven_m'], 2), start_pose=leg['from'],
                   turn_deg=leg.get('turn_deg'), wh=self._used(leg), battery_pct=[leg['pct0'], self.battery.get('pct')],
                   during=leg['events'])

    def _on_battery(self, msg):
        try:
            b = json.loads(msg.data)
        except ValueError:
            return
        if 'error' in b:
            self._simple('battery', 'sensor not answering')
            self.state_battery = None       # report the state again once it answers
            return
        if 'wh' in self.battery and b.get('wh', 0) < self.battery['wh']:
            # the battery monitor restarted (its counter starts at 0 again): no energy for the leg in progress
            if self.leg is not None:
                self.leg['wh0'] = None
        self.battery = b
        if b.get('state') != self.state_battery:            # only changes: ok -> low -> critical, charging
            self.state_battery = b.get('state')
            self._simple('battery', '%s at %s %%' % (b.get('state'), b.get('pct')))

    def _used(self, leg):
        if leg.get('wh0') is None or self.battery.get('wh') is None:
            return None
        return round(self.battery['wh'] - leg['wh0'], 3)

    def _track(self):
        """Add up the distance actually driven during the current leg."""
        p = self._pose()
        if p is None:
            return
        if self.leg is not None and self.last_pose is not None:
            step = math.hypot(p[0] - self.last_pose[0], p[1] - self.last_pose[1])
            if step < 0.5:                                  # bigger jumps are relocalisations, not driving
                self.leg['driven_m'] += step
        self.last_pose = p


def main():
    rclpy.init()
    node = EventLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
