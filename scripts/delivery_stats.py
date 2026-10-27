#!/usr/bin/env python3
"""Delivery statistics from the event log (event_logger.py's events.jsonl): the numbers for the Admin page, the slides
and the evaluation, always up to date and never typed by hand.

Every `period` s it reads the new lines of `log_file` and publishes /delivery_stats (String JSON, latched):
  {t, today: {...}, all: {...}}   each with
    deliveries   finished jobs: delivered / failed / returned to the sender / cancelled, and interrupted = started but
                 never finished (system stopped, e.g. after an emergency stop; no event for `idle_minutes`)
    success_pct  delivered out of delivered + failed + returned + interrupted (cancelled ones are left out)
    on_its_own_pct  delivered without an emergency stop or a "needs help" message
    avg_s, median_s  start of the job -> "delivered"
    legs, metres, avg_speed  drives that arrived, distance driven, metres per second while driving
    narrow       narrow passages (lined up and through / tries)
    lost         times the position was lost, found again, average seconds to find it
    waits, around, detours, blocked   path guard: waited for an obstacle, went round, other route, no way
    estops, motor_restarts
    wh_per_delivery   energy per delivered job (only when the battery monitor ran during all its drives)
"Today" = since local midnight. firebase_bridge.py forwards it to /stats for the website.

  python3 src/my_bot/scripts/delivery_stats.py [--ros-args -p log_file:=~/dev_ws/run_logs/events.jsonl]
"""
import json
import os
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def new_job():
    return {'start': None, 'end': None, 'result': None, 'estop': 0, 'help': 0, 'narrow': 0, 'narrow_ok': 0,
            'lost': 0, 'found': 0, 'lost_s': [], 'lost_at': None, 'waits': 0, 'around': 0, 'detours': 0,
            'blocked': 0, 'motor': 0, 'legs': [], 'wh': 0.0, 'wh_known': True}


def add(job, ev):
    """Count one event into its job."""
    typ, text, t = ev.get('type'), ev.get('text') or '', ev.get('t', 0)
    if typ == 'delivery':
        if text.startswith('Starting') and job['start'] is None:
            job['start'] = t
        if text.startswith('EMERGENCY STOP'):
            job['estop'] += 1
        if text.startswith('Robot needs help'):
            job['help'] += 1
        if ev.get('result') and job['result'] is None:
            job['result'], job['end'] = ev['result'], t
    elif typ == 'gap_assist':
        job['narrow'] += text.startswith('Nav2 is stuck')
        job['narrow_ok'] += text.startswith('done: through the gap')
    elif typ == 'localisation':
        if text == 'lost':
            job['lost'] += 1
            job['lost_at'] = t
        elif text in ('recovered', 'ok') and job['lost_at'] is not None:
            job['found'] += 1
            job['lost_s'].append(t - job['lost_at'])
            job['lost_at'] = None
        elif text.startswith('help'):
            job['help'] += 1
    elif typ == 'path':
        kind = text.split(':', 1)[0]
        key = {'waiting': 'waits', 'around': 'around', 'detour': 'detours', 'blocked': 'blocked'}.get(kind)
        if key:
            job[key] += 1
    elif typ == 'motor' and text.startswith('RESTARTED'):
        job['motor'] += 1
    elif typ == 'leg' and ev.get('arrived'):
        job['legs'].append((ev.get('seconds') or 0, ev.get('driven_m') or 0))
        if ev.get('wh') is None:
            job['wh_known'] = False
        else:
            job['wh'] += ev['wh']


def summary(jobs, now, idle_s):
    done = [j for j in jobs if j['result']]
    res = [j['result'] for j in done]
    delivered = [j for j in done if j['result'] == 'delivered']
    interrupted = sum(1 for j in jobs if j['start'] and not j['result'] and now - j['last'] > idle_s)
    counted = sum(r in ('delivered', 'failed', 'returned') for r in res) + interrupted
    times = [j['end'] - j['start'] for j in delivered if j['start']]
    legs = [leg for j in jobs for leg in j['legs']]
    secs, metres = sum(s for s, _ in legs), sum(m for _, m in legs)
    lost_s = [s for j in jobs for s in j['lost_s']]
    wh = [j['wh'] for j in delivered if j['wh_known'] and j['wh'] > 0]
    total = lambda k: sum(j[k] for j in jobs)
    return {
        'deliveries': {'delivered': len(delivered), 'failed': res.count('failed'), 'returned': res.count('returned'),
                       'cancelled': res.count('cancelled'), 'interrupted': interrupted},
        'success_pct': round(100.0 * len(delivered) / counted) if counted else None,
        'on_its_own_pct': round(100.0 * sum(1 for j in delivered if not j['estop'] and not j['help']) / len(delivered))
        if delivered else None,
        'avg_s': round(statistics.mean(times)) if times else None,
        'median_s': round(statistics.median(times)) if times else None,
        'legs': len(legs), 'metres': round(metres, 1),
        'avg_speed': round(metres / secs, 3) if secs else None,
        'narrow': {'tries': total('narrow'), 'through': total('narrow_ok')},
        'lost': {'times': total('lost'), 'found': total('found'),
                 'avg_s': round(statistics.mean(lost_s), 1) if lost_s else None},
        'waits': total('waits'), 'around': total('around'), 'detours': total('detours'), 'blocked': total('blocked'),
        'estops': total('estop'), 'motor_restarts': total('motor'),
        'wh_per_delivery': round(statistics.mean(wh), 2) if wh else None,
    }


class DeliveryStats(Node):
    def __init__(self):
        super().__init__('delivery_stats')
        self.declare_parameter('log_file', '~/dev_ws/run_logs/events.jsonl')
        self.declare_parameter('period', 30.0)
        self.declare_parameter('idle_minutes', 10.0)
        self.path = os.path.expanduser(self.get_parameter('log_file').value)
        self.pos = 0                     # bytes of the log already read
        self.jobs = {}                   # website id (or job number) -> counts, in order of the first event
        self.last = None
        self.pub = self.create_publisher(String, '/delivery_stats', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(self.get_parameter('period').value, self._tick)
        self._tick()
        self.get_logger().info('Delivery statistics from %s' % self.path)

    def _read(self):
        try:
            with open(self.path) as f:
                f.seek(self.pos)
                lines = f.readlines()
                if lines and not lines[-1].endswith('\n'):
                    lines.pop()          # half-written last line: read it next time
                self.pos += sum(len(l.encode()) for l in lines)
        except OSError:
            return
        for line in lines:
            try:
                ev = json.loads(line)
            except ValueError:
                continue
            key = ev.get('ref') or ev.get('job')
            if key is None:
                continue
            job = self.jobs.setdefault(key, new_job())
            job.setdefault('first', ev.get('t', 0))
            job['last'] = ev.get('t', 0)
            add(job, ev)

    def _tick(self):
        self._read()
        midnight = time.mktime(time.localtime()[:3] + (0, 0, 0, 0, 0, -1))
        jobs = list(self.jobs.values())
        now, idle = time.time(), 60.0 * self.get_parameter('idle_minutes').value
        body = {'today': summary([j for j in jobs if j.get('first', 0) >= midnight], now, idle),
                'all': summary(jobs, now, idle)}
        if body != self.last:
            self.last = body
            body = dict(body, t=round(time.time()))
            self.pub.publish(String(data=json.dumps(body)))


def main():
    rclpy.init()
    node = DeliveryStats()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
