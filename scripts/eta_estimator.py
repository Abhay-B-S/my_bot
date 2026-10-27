#!/usr/bin/env python3
"""ETA estimator: how long until the robot reaches the sender or the receiver, learned from the robot's own trips.

Learned from the robot's own legs (event_logger.py's "leg" records: planned route length, turn needed at the
start, real time):
    time = fixed part + route length / effective speed + start turn x seconds per degree
fitted by least squares, refitted without the worst 10 %; if that fit is implausible (noisy legs) only a robust
median time per metre is used. Honest result in the simulator (2026-10-27, 16 legs in the narrow loop world, leave-
one-out): the same 2-4 m route took 13-60 s depending on whether the robot got stuck; no model beat "the usual time"
(errors ~70 %), so the website shows a RANGE from the real spread, and the queue estimates (minutes) are the useful
part (88 s predicted, 88 s real). Speeding up, settling at the spot (fixed part), waiting for people,
gap-assist manoeuvres and self-recovery (effective speed) are all included. The range comes
from how far real legs fell from the model (25 % / 75 % quantiles of the ratio). With fewer than `min_samples`
legs a default (fixed part `default_fixed` s, `default_speed` m/s, +-35 %) is used. How long people take to load/unload is learned the same way
(time from arriving at a cabin to leaving it).

  current leg     : remaining length of Nav2's current plan / effective speed
  later legs      : route length between spots on the map (grid search keeping the robot's width) / effective speed
  queued requests : the current job's remaining time + every job before it (urgent ones first, as the manager does)

Topics: in /delivery_status, /plan, /map, /robot_event, /delivery_reload_spots, TF map->base_link;
        out /delivery_eta (String, JSON, every `period` s):
            {"model": {...}, "jobs": {<ref>: {"pickup": [s, lo, hi] | null, "dropoff": [s, lo, hi] | null,
                                              "ahead": n, "phase": "..."}}}
  python3 src/my_bot/scripts/eta_estimator.py --ros-args -p locations_file:=... [-p log_file:=...]
"""
import heapq
import json
import math
import os
import time

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener


def quantile(values, q):
    v = sorted(values)
    return v[min(len(v) - 1, max(0, int(round(q * (len(v) - 1)))))]


class EtaEstimator(Node):
    def __init__(self):
        super().__init__('eta_estimator')
        here = os.path.dirname(os.path.abspath(__file__))
        self.declare_parameter('locations_file', os.path.join(here, '..', 'config', 'locations.yaml'))
        self.declare_parameter('log_file', '~/dev_ws/run_logs/events.jsonl')
        self.declare_parameter('default_speed', 0.15)     # m/s effective (with turns), until enough legs are logged
        self.declare_parameter('default_fixed', 6.0)      # s every leg needs anyway (turn, start, stop)
        self.declare_parameter('default_wait', 30.0)      # s a person needs at the robot, until learned
        self.declare_parameter('min_samples', 3)
        self.declare_parameter('robot_radius', 0.17)      # m: corridor width the route search keeps
        self.declare_parameter('period', 2.0)
        self.p = {k: self.get_parameter(k).value for k in
                  ('locations_file', 'log_file', 'default_speed', 'default_fixed', 'default_wait', 'min_samples',
                   'robot_radius', 'period')}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(String, '/delivery_status', self._on_status, latched)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(String, '/robot_event', self._on_event, 50)
        self.create_subscription(Empty, '/delivery_reload_spots', lambda _m: self._load_spots(), 10)
        self.pub = self.create_publisher(String, '/delivery_eta', 10)

        self.status = None
        self.plan = None
        self.leg_total = None
        self.grid = None
        self.route_cache = {}
        self.speeds, self.waits = [], []
        self.last_leg_end = None       # (time, kind) of the last finished leg, to learn people's waiting time
        self._load_spots()
        self._load_history()
        self.create_timer(self.p['period'], self._tick)
        m = self._model()
        self.get_logger().info('ETA estimator: %.0f s + route / %.2f m/s + %.2f s per degree of start turn (%d legs), '
                               'people need %.0f s (%d waits)' % (m['fixed'], m['speed'], m['per_deg'],
                                                                   len(self.speeds), m['wait'], len(self.waits)))

    # ---------- what the robot has learned ----------
    def _load_spots(self):
        with open(os.path.normpath(os.path.expanduser(self.p['locations_file']))) as f:
            self.spots = (yaml.safe_load(f) or {}).get('locations') or {}
        self.route_cache = {}

    def _load_history(self):
        path = os.path.expanduser(self.p['log_file'])
        if not os.path.exists(path):
            return
        with open(path) as f:
            for line in f:
                try:
                    self._learn(json.loads(line))
                except ValueError:
                    continue

    def _learn(self, ev):
        if ev.get('type') != 'leg':
            return
        start = ev['t'] - ev.get('seconds', 0)
        if self.last_leg_end and self.last_leg_end[1] in ('TO_PICKUP', 'TO_DROPOFF', 'RETURN_TO_SENDER'):
            wait = start - self.last_leg_end[0]
            if 2.0 < wait < 600.0:                         # a person loaded or unloaded the parcel meanwhile
                self.waits.append(wait)
        self.last_leg_end = (ev['t'], ev.get('kind')) if ev.get('arrived') else None
        if ev.get('arrived') and (ev.get('plan_m') or 0) > 0.3 and ev.get('seconds', 0) > 2:
            if ev['seconds'] < 600:
                self.speeds.append((ev['plan_m'], ev.get('turn_deg') or 0.0, ev['seconds']))   # metres, deg, s

    def _on_event(self, msg):
        try:
            self._learn(json.loads(msg.data))
        except ValueError:
            pass

    def _model(self):
        fixed, per_m, per_deg = self.p['default_fixed'], 1.0 / self.p['default_speed'], 0.1
        lo, hi, turn = 1 / 1.35, 1.35, 90.0
        if len(self.speeds) >= self.p['min_samples']:
            a = np.array(self.speeds, dtype=float)
            d, th, t = a[:, 0], a[:, 1], a[:, 2]
            turn = float(np.median(th))                          # typical start turn, for legs not yet planned
            coef = self._fit(d, th, t) if len(a) >= 6 else None
            if coef is not None:
                fixed, per_m, per_deg = coef
            else:                  # few legs, or the fit is implausible (noisy legs): robust time per metre only
                per_deg = 0.0
                per_m = float(np.median(((t - fixed) / np.maximum(d, 0.3)).clip(min=2.0)))
            pred = fixed + per_m * d + per_deg * th
            ratio = t / np.maximum(pred, 1.0)                    # how far real legs fell from the model
            lo, hi = min(quantile(list(ratio), 0.25), 0.95), max(quantile(list(ratio), 0.75), 1.05)
        w = float(np.median(self.waits)) if len(self.waits) >= self.p['min_samples'] else self.p['default_wait']
        return {'speed': 1.0 / per_m, 'fixed': float(fixed), 'per_deg': float(per_deg), 'turn': turn,
                'lo': lo, 'hi': hi, 'wait': w}

    @staticmethod
    def _fit(d, th, t):
        """Least squares time = fixed + per_m d + per_deg th, refitted without the worst 10 %; None if implausible."""
        X = np.stack([np.ones_like(d), d, th], 1)
        keep = np.ones(len(t), bool)
        for _ in range(2):
            coef, *_ = np.linalg.lstsq(X[keep], t[keep], rcond=None)
            err = np.abs(X @ coef - t)
            if keep.sum() >= 8:
                keep = err <= np.quantile(err, 0.9)
        fixed, per_m, per_deg = (float(c) for c in coef)
        if not (0.0 <= fixed <= 30.0 and 2.0 <= per_m <= 50.0 and 0.0 <= per_deg <= 2.0):
            return None
        return fixed, per_m, per_deg

    # ---------- distances ----------
    def _on_plan(self, msg):
        if len(msg.poses) > 1:
            self.plan = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
            if self.leg_total is None:          # the first plan of a leg = its full length
                self.leg_total = float(np.hypot(np.diff(self.plan[:, 0]), np.diff(self.plan[:, 1])).sum())

    def _on_map(self, m):
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        free = (g >= 0) & (g < 50)
        wall = g >= 50
        # cells too close to a wall for the robot (a square approximation of robot_radius is enough here)
        k = max(1, int(self.p['robot_radius'] / m.info.resolution))
        near = np.zeros_like(wall)
        for dy in range(-k, k + 1):
            for dx in range(-k, k + 1):
                near |= np.roll(np.roll(wall, dy, 0), dx, 1)
        self.grid = (free & ~near, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y)
        self.route_cache = {}

    def _pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform.translation
            return t.x, t.y
        except Exception:
            return None

    def _route_from_robot(self, b):
        pose = self._pose()
        if pose is None or b not in self.spots:
            return None
        return self._route({'x': pose[0], 'y': pose[1]}, b, cache=False)

    def _route(self, a, b, cache=True):
        """Route length (m) between two spots (names, or a {'x', 'y'} position for a) on the map, or the straight
        line x 1.3 when it cannot be searched."""
        key = (a, b)
        if cache and key in self.route_cache:
            return self.route_cache[key]
        pa = a if isinstance(a, dict) else self.spots.get(a)
        pb = self.spots.get(b)
        if pa is None or pb is None:
            return 0.0
        straight = math.hypot(float(pa['x']) - float(pb['x']), float(pa['y']) - float(pb['y']))
        length = straight * 1.3
        if self.grid is not None:
            ok, res, ox, oy = self.grid
            cell = lambda p: (int((float(p['y']) - oy) / res), int((float(p['x']) - ox) / res))
            s, g = cell(pa), cell(pb)
            h, w = ok.shape
            if all(0 <= c[0] < h and 0 <= c[1] < w for c in (s, g)):
                ok = ok.copy()
                ok[s] = ok[g] = True
                dist = {s: 0.0}
                pq = [(0.0, s)]
                while pq:
                    d, (r, c) = heapq.heappop(pq)
                    if (r, c) == g:
                        length = d * res
                        break
                    if d > dist.get((r, c), 1e18):
                        continue
                    for dr, dc, step in ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
                                         (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < h and 0 <= cc < w and ok[rr, cc] and d + step < dist.get((rr, cc), 1e18):
                            dist[(rr, cc)] = d + step
                            heapq.heappush(pq, (d + step, (rr, cc)))
        if cache:
            self.route_cache[key] = length
        return length

    def _turn_needed(self):
        """Degrees the robot still has to turn to face its route (plan point ~0.5 m ahead), 0 if unknown."""
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform
        except Exception:
            return 0.0
        if self.plan is None:
            return 0.0
        x, y = t.translation.x, t.translation.y
        yaw = math.degrees(2 * math.atan2(t.rotation.z, t.rotation.w))
        for px, py in self.plan:
            if math.hypot(px - x, py - y) >= 0.5:
                want = math.degrees(math.atan2(py - y, px - x))
                return abs((want - yaw + 180) % 360 - 180)
        return 0.0

    def _remaining_plan(self):
        """Length (m) of the current plan from the point nearest the robot to its end."""
        pose = self._pose()
        if self.plan is None or pose is None:
            return None
        i = int(np.argmin(np.hypot(self.plan[:, 0] - pose[0], self.plan[:, 1] - pose[1])))
        rest = self.plan[i:]
        return float(np.hypot(np.diff(rest[:, 0]), np.diff(rest[:, 1])).sum()) + \
            math.hypot(rest[0, 0] - pose[0], rest[0, 1] - pose[1])

    # ---------- estimates ----------
    def _on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if self.status is None or st.get('state') != self.status.get('state'):
            self.plan, self.leg_total = None, None   # a new leg: the last plan belongs to the previous one
        self.status = st

    def _tick(self):
        st = self.status
        if not st:
            return
        m = self._model()
        drive = lambda metres: m['fixed'] + metres / m['speed'] + m['per_deg'] * m['turn']   # a whole leg (learned)
        total = self.leg_total
        # the leg under way: only the share of the fixed part that is still ahead
        share = lambda metres: min(1.0, metres / total) if total else 1.0
        turn_left = self._turn_needed()                             # still to turn towards the route (deg)
        now_leg = lambda metres: m['fixed'] * share(metres) + metres / m['speed'] + m['per_deg'] * turn_left
        state, home = st.get('state'), 'home'
        jobs = {}

        # the current job: when does the robot reach the sender / the receiver, and when is it free again
        here = None                                                # where the robot will be when the job ends
        free_in = 0.0
        if st.get('ref') and state not in ('IDLE', 'RETURNING'):
            s, d = st.get('sender'), st.get('destination')
            leg = self._remaining_plan()
            if leg is None:                                    # no plan for this leg yet: search our own route
                target = {'TO_PICKUP': s, 'TO_DROPOFF': d, 'RETURN_TO_SENDER': s}.get(state)
                leg = self._route_from_robot(target) if target else None
            pickup = dropoff = None
            if state == 'TO_PICKUP':
                to_s = now_leg(leg) if leg is not None else drive(self._route(home, s))
                pickup = to_s
                dropoff = to_s + m['wait'] + drive(self._route(s, d))
            elif state == 'WAIT_SHIPPED':
                dropoff = m['wait'] + drive(self._route(s, d))
            elif state == 'TO_DROPOFF':
                dropoff = now_leg(leg) if leg is not None else drive(self._route(s, d))
            elif state == 'RETURN_TO_SENDER':
                pickup = now_leg(leg) if leg is not None else drive(self._route(d, s))
            end = {'TO_PICKUP': (dropoff or 0) + m['wait'], 'WAIT_SHIPPED': (dropoff or 0) + m['wait'],
                   'TO_DROPOFF': (dropoff or 0) + m['wait'], 'WAIT_RECEIVED': m['wait'] / 2,
                   'RETURN_TO_SENDER': (pickup or 0) + m['wait'], 'WAIT_RETURNED': m['wait'] / 2}.get(state, 0.0)
            free_in = end
            here = s if state in ('RETURN_TO_SENDER', 'WAIT_RETURNED') else d
            jobs[st['ref']] = {'phase': state, 'ahead': 0,
                               'pickup': self._band(pickup, m), 'dropoff': self._band(dropoff, m)}
        elif state == 'RETURNING':
            here = None                                            # on the way home: next job starts from here

        # queued jobs, in the manager's order (urgent first, then first come; nearest-first can change the
        # order a little, so these are "about" numbers)
        queue = st.get('queue_jobs') or []
        order = [j for j in queue if j.get('urgent')] + [j for j in queue if not j.get('urgent')]
        t, pos = free_in, here
        for n, j in enumerate(order):
            start_pos = pos or self._nearest_spot()
            to_s = t + drive(self._route(start_pos, j['sender'])) if start_pos else t + drive(3.0)
            to_d = to_s + m['wait'] + drive(self._route(j['sender'], j['destination']))
            if j.get('ref'):
                jobs[j['ref']] = {'phase': 'QUEUED', 'ahead': n + (1 if st.get('ref') and state != 'RETURNING' else 0),
                                  'pickup': self._band(to_s, m), 'dropoff': self._band(to_d, m)}
            t, pos = to_d + m['wait'], j['destination']
        self.pub.publish(String(data=json.dumps({
            'model': {'speed': round(m['speed'], 3), 'fixed': round(m['fixed'], 1), 's_per_deg': round(m['per_deg'], 3),
                      'legs': len(self.speeds),
                      'wait': round(m['wait']),
                      'waits': len(self.waits)},
            'jobs': jobs})))

    def _band(self, seconds, m):
        """[estimate, low, high] in whole seconds (low/high from the slow and fast quartile speeds)."""
        if seconds is None:
            return None
        return [round(seconds), round(seconds * m['lo']), round(seconds * m['hi'])]

    def _nearest_spot(self):
        pose = self._pose()
        if pose is None or not self.spots:
            return None
        return min(self.spots, key=lambda n: math.hypot(float(self.spots[n]['x']) - pose[0],
                                                         float(self.spots[n]['y']) - pose[1]))


def main():
    rclpy.init()
    node = EtaEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
