#!/usr/bin/env python3
"""Path guard: when something blocks the planned route, first let Nav2 steer round it; only when there is no way
through, wait a moment and then make Nav2 take another way (or stay calm and say so when there is none).

Why: Nav2 alone drives up to an obstacle until every move looks like a collision; in a ~1 m corridor the robot then
cannot even turn round. Its "clear costmap" recovery also wipes obstacles it has seen, so it tries a blocked way
again and goes back and forth (seen 2026-10-24 at home and in the simulator).

1. Watch. Ten times a second: lidar points that are NOT walls of the map and lie on the planned route ahead
   (within `path_half_width`, up to `look_ahead` m) are "in the way".
2. Go round it if possible: Nav2 re-plans once a second. If its next plan avoids the obstacle on the same route,
   nothing happens. Only if the new plan still runs into it (or it is closer than `hard_stop`) -> stop, still
   `stop_distance` away (room to turn round). A short re-plan round it while stopped -> drive on ("around").
3. No way through: wait up to `wait_seconds` (a person usually steps aside) -> drive on. Still there -> remember it
   and let Nav2 take another way ("detour"), or hold until it has one ("rerouting"). The same wait happens when Nav2
   itself switches to another way because of something within `wait_distance`.
4. No other way within `reroute_seconds`, or Nav2 gives up -> "blocked": stay still and say so; continue once the
   lidar sees the spot free again. The delivery manager decides what to do next (check again, return the parcel).
Remembered ("blocked") spots go to the global costmap on /blocked_points, so Nav2 cannot forget them. They are
forgotten after `memory_seconds`, when the lidar sees through them (obstacle gone), or on /path_guard/forget.

  python3 src/my_bot/scripts/path_guard.py        (real robot and simulation alike: it uses wall-clock timers)
Topics: in /plan, /scan, /map, /navigate_to_pose/_action/status, /path_guard/forget (Empty), TF;
        out /cmd_vel_guard (Twist, twist_mux input), /blocked_points (PointCloud2, map frame), /path_status (String,
        latched): "clear" | "around: ..." | "waiting: ..." | "detour: ..." | "rerouting: ..." | "blocked: ..."
"""
import math
import struct
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

CLEAR, WAITING, REROUTING, BLOCKED = 'clear', 'waiting', 'rerouting', 'blocked'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def length(route):
    return float(np.hypot(np.diff(route[:, 0]), np.diff(route[:, 1])).sum()) if route is not None and len(route) > 1 else 0.0


class PathGuard(Node):
    def __init__(self):
        super().__init__('path_guard')
        defaults = {
            'look_ahead': 1.5,         # m of planned route checked ahead of the robot
            'path_half_width': 0.22,   # m either side of the route (robot is 0.16 wide each side of its centre)
            'stop_distance': 0.8,      # m from the axle: something in the way this close and Nav2's next plan
                                       # still runs into it -> stop (there is still room to turn round)
            'hard_stop': 0.5,          # m: stop at once, without waiting for Nav2's next plan
            'wait_distance': 1.5,      # m: Nav2 detours to another way because of something this close -> wait
            'min_points': 3,           # lidar points needed to count as "in the way" (ignores single noisy points)
            'wait_seconds': 4.0,       # s to wait for it to move (a person steps aside) before looking for another way
            'reroute_seconds': 8.0,    # s allowed for Nav2 to find another way
            'detour_extra': 1.0,       # m: a new route this much longer than the last one = Nav2 detours round something
            'memory_seconds': 180.0,   # s a blocked spot is remembered
            'see_through_range': 2.0,  # m: a remembered point closer than this that the lidar sees through ...
            'see_through_seconds': 2.0,  # ... continuously for this long is forgotten (beams past the edge of a box or
                                         # over a low one for a moment must not erase it: robot went back and forth,
                                         # real test 2026-10-24)
            'plan_fresh': 2.5,         # s: an older plan is not used to start a stop
            'max_range': 4.0,          # m: ignore farther lidar points
            'wall_margin': 0.20,       # m: lidar points this close to anything in the map are the map, not new
                                       # obstacles (10 cm was too little: a sofa edge seen 15-20 cm off while the
                                       # position was slightly wrong became a "block", 2026-10-26)
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._on_goal_status, latched)
        self.create_subscription(Empty, '/path_guard/forget', self._on_forget, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_guard', 10)
        self.status_pub = self.create_publisher(String, '/path_status', latched)
        self.blocked_pub = self.create_publisher(PointCloud2, '/blocked_points', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.near_wall = None       # map mask: map walls grown by wall_margin (lidar points there are walls, not obstacles)
        self.info = None
        self.scan = None
        self.plan = None            # Nx2 array of the latest plan
        self.plan_time = 0.0
        self.plan_count = 0         # plans received (to wait for a NEW one)
        self.goal_active = False    # Nav2 has a navigate_to_pose goal running
        self.aborted_at = -1e9      # when Nav2 last gave up on a goal
        self.goal_states = {}       # goal id -> last status seen
        self.last_plan = None       # previous plan of this goal and its length (to notice detours)
        self.last_len = 0.0
        self.seen_plan = 0
        self.detour_votes = 0
        self.blocked = np.zeros((0, 4))   # remembered blocked points: x, y, time seen, see-through count
        self.snapshot = np.zeros((0, 2))  # what was in the way when the robot stopped
        self.len_at_stop = 0.0      # remaining route length when it stopped (to tell "round it" from "another way")
        self.noticed_plan = None    # plan_count when something on the route was first noticed
        self.gone_since = None
        self.last_report = 0.0
        self.state = CLEAR
        self.since = 0.0            # when the current state started
        self.plan_mark = 0          # plan_count when rerouting started
        self.clear_since = None
        self._status(CLEAR, 'watching')
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_blocked)
        self.get_logger().info('Path guard: stops %.2f m before anything on the route, waits %.0f s, then reroutes'
                               % (self.p['stop_distance'], self.p['wait_seconds']))

    # ---------- inputs ----------
    def _on_map(self, m):
        info = m.info
        grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
        occ = grid >= 65
        grown = occ.copy()
        k = int(round(self.p['wall_margin'] / info.resolution))
        for dx in range(-k, k + 1):
            for dy in range(-k, k + 1):
                grown |= np.roll(np.roll(occ, dx, axis=1), dy, axis=0)
        self.near_wall, self.info = grown, info

    def _on_scan(self, s):
        self.scan = s

    def _on_goal_status(self, msg):
        self.goal_active = any(s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
                               for s in msg.status_list)
        for s in msg.status_list:                    # the list also holds old goals: react only to a NEW abort
            gid = bytes(s.goal_info.goal_id.uuid)
            if s.status == GoalStatus.STATUS_ABORTED and self.goal_states.get(gid) != GoalStatus.STATUS_ABORTED \
                    and gid in self.goal_states:
                self.aborted_at = time.monotonic()
            if gid not in self.goal_states and s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING) \
                    and self.state != CLEAR:
                self._status(CLEAR, 'new destination')    # e.g. the manager gave up and sends the robot elsewhere
            self.goal_states[gid] = s.status

    def _on_plan(self, msg):
        if len(msg.poses) < 2:
            return
        self.plan = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        self.plan_time = time.monotonic()
        self.plan_count += 1

    # ---------- helpers ----------
    def _poses(self):
        """(x, y, yaw) of the laser and (x, y) of the axle in the map at the scan's time, or None."""
        stamp = Time.from_msg(self.scan.header.stamp)
        try:
            tl = self.tf_buffer.lookup_transform('map', self.scan.header.frame_id, stamp)
            tb = self.tf_buffer.lookup_transform('map', 'base_link', stamp)
        except Exception:
            try:                                     # scan newer than the last map transform: use the latest
                tl = self.tf_buffer.lookup_transform('map', self.scan.header.frame_id, Time())
                tb = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            except Exception:
                return None
        return ((tl.transform.translation.x, tl.transform.translation.y, yaw_of(tl.transform.rotation)),
                (tb.transform.translation.x, tb.transform.translation.y))

    def _scan_points(self, laser):
        """New-obstacle lidar points (Nx2, map frame): valid, not too far, not on a wall of the map."""
        s = self.scan
        r = np.array(s.ranges, dtype=np.float64)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, self.p['max_range']))
        lx, ly, lyaw = laser
        xs, ys = lx + r[ok] * np.cos(a[ok] + lyaw), ly + r[ok] * np.sin(a[ok] + lyaw)
        if self.near_wall is not None:
            info = self.info
            cx = ((xs - info.origin.position.x) / info.resolution).astype(np.int64)
            cy = ((ys - info.origin.position.y) / info.resolution).astype(np.int64)
            inside = (cx >= 0) & (cx < info.width) & (cy >= 0) & (cy < info.height)
            wall = np.zeros(len(xs), dtype=bool)
            wall[inside] = self.near_wall[cy[inside], cx[inside]]
            xs, ys = xs[~wall], ys[~wall]
        return np.stack([xs, ys], axis=1)

    def _ahead(self, plan, axle, dist):
        """Plan points from the one nearest the robot onwards, up to `dist` m along the route."""
        if plan is None:
            return None
        i = int(np.argmin(np.hypot(plan[:, 0] - axle[0], plan[:, 1] - axle[1])))
        ahead = plan[i:]
        if len(ahead) < 2:
            return ahead
        along = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(ahead[:, 0]), np.diff(ahead[:, 1])))])
        return ahead[along <= dist]

    def _on_route(self, pts, route):
        """Points (Nx2) lying on the route (within path_half_width of a route point)."""
        if route is None or len(route) == 0 or len(pts) == 0:
            return np.zeros((0, 2))
        d = np.hypot(pts[:, None, 0] - route[None, :, 0], pts[:, None, 1] - route[None, :, 1]).min(axis=1)
        return pts[d <= self.p['path_half_width']]

    def _remember(self, pts, why):
        if len(pts) < self.p['min_points']:
            return
        now = time.monotonic()
        self.blocked = np.vstack([self.blocked, np.column_stack([pts, np.full(len(pts), now), np.zeros(len(pts))])])[-2000:]
        self.get_logger().info('Remembering %d blocked points at (%.1f, %.1f): %s'
                               % (len(pts), pts[:, 0].mean(), pts[:, 1].mean(), why))

    def _forget_seen_through(self, laser, now):
        """Drop remembered points that are too old, or that the lidar now sees through (the obstacle has gone)."""
        if len(self.blocked) == 0:
            return
        keep = now - self.blocked[:, 2] < self.p['memory_seconds']
        s = self.scan
        if s is not None and laser is not None:
            lx, ly, lyaw = laser
            dist = np.hypot(self.blocked[:, 0] - lx, self.blocked[:, 1] - ly)
            bearing = np.arctan2(self.blocked[:, 1] - ly, self.blocked[:, 0] - lx) - lyaw - s.angle_min
            idx = np.round((np.arctan2(np.sin(bearing), np.cos(bearing)) % (2 * math.pi)) / s.angle_increment)
            ranges = np.array(s.ranges, dtype=np.float64)
            r = ranges[np.clip(idx.astype(np.int64), 0, len(ranges) - 1)]
            through = (dist < self.p['see_through_range']) & np.isfinite(r) & (r > s.range_min) & (r > dist + 0.2)
            self.blocked[:, 3] = np.where(through, self.blocked[:, 3] + 1, 0)
            keep &= self.blocked[:, 3] < self.p['see_through_seconds'] * 10    # ticks are 0.1 s apart
        if not keep.all():
            self.get_logger().info('Forgot %d blocked points (%d left)' % (int((~keep).sum()), int(keep.sum())))
        self.blocked = self.blocked[keep]
        if len(keep) and not keep.any():             # every remembered block is gone: always say so (the manager
            self._status(CLEAR, 'every remembered block is gone')   # may be waiting for it)

    def _still_there(self, laser):
        """True when the obstacle seen at the stop (snapshot) is still visible."""
        if len(self.snapshot) == 0:
            return False
        pts = self._scan_points(laser)
        if len(pts) == 0:
            return False
        d = np.hypot(pts[:, None, 0] - self.snapshot[None, :, 0], pts[:, None, 1] - self.snapshot[None, :, 1])
        return int((d.min(axis=1) < 0.15).sum()) >= self.p['min_points']

    def _status(self, state, detail, kind=None):
        """Change state and publish "<kind or state>: detail" ("clear" alone when plainly clear)."""
        if state != self.__dict__.get('state'):
            self.since = time.monotonic()
            self.count_at_block = len(self.blocked)    # BLOCKED ends once some remembered point is forgotten
        self.state = state
        kind = kind or state
        text = 'clear' if kind == CLEAR else '%s: %s' % (kind, detail)
        self.status_pub.publish(String(data=text))
        self.get_logger().info('[%s] %s' % (kind, detail))

    def _publish_blocked(self):
        """The remembered blocked points as a PointCloud2 for the global costmap. It always holds one extra point far
        outside the costmap's obstacle_range: an EMPTY cloud seemed to be ignored by the costmap (it kept marking the
        last non-empty one, so forgotten blocks came back, 2026-10-24 simulator)."""
        xy = np.vstack([self.blocked[:, :2], [[1000.0, 1000.0]]])
        msg = PointCloud2()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(xy)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        msg.is_bigendian, msg.point_step, msg.is_dense = False, 12, True
        msg.row_step = 12 * len(xy)
        msg.data = b''.join(struct.pack('<fff', x, y, 0.2) for x, y in xy)
        self.blocked_pub.publish(msg)

    def _watch_detours(self, laser, axle):
        """A new plan much longer than the last one = Nav2 detours round something on the old route (another way).
        Close by: hold the robot a moment first (a person may step aside). Far away: just remember it."""
        if self.plan_count == self.seen_plan or self.plan is None:
            return
        self.seen_plan = self.plan_count
        new_len = length(self._ahead(self.plan, axle, 1e9))
        if self.last_plan is None or new_len <= self.last_len + self.p['detour_extra']:
            self.last_plan, self.last_len, self.detour_votes = self.plan, new_len, 0
            return
        self.detour_votes += 1
        stuck = self._on_route(self._scan_points(laser), self._ahead(self.last_plan, axle, 3.0))
        close = len(stuck) >= self.p['min_points'] and \
            np.hypot(stuck[:, 0] - axle[0], stuck[:, 1] - axle[1]).min() < self.p['wait_distance']
        if self.detour_votes < 2 and not close:       # one odd plan alone is not a detour
            return
        old_len = self.last_len
        self.last_plan, self.last_len, self.detour_votes = self.plan, new_len, 0
        if len(stuck) < self.p['min_points'] or self.state != CLEAR:
            return
        where = '(%.1f, %.1f)' % tuple(stuck.mean(axis=0))
        if close:
            self.cmd_pub.publish(Twist())
            self.snapshot, self.len_at_stop = stuck, old_len
            self._status(WAITING, 'the route is blocked at %s; waiting a moment before taking another way' % where)
        else:
            self._remember(stuck, 'Nav2 detoured round it')
            self._status(CLEAR, 'route blocked at %s, taking another way' % where, kind='detour')

    def _on_forget(self, _msg):
        """The delivery manager asks to check the routes again: forget every remembered block."""
        n = len(self.blocked)
        self.blocked = np.zeros((0, 4))
        self.get_logger().info('Forgot all %d remembered blocked points (asked to check again)' % n)
        self._publish_blocked()
        if self.state != CLEAR:
            self._status(CLEAR, 'checking the routes again')

    # ---------- main loop ----------
    def _tick(self):
        now = time.monotonic()
        if self.scan is None:
            return
        poses = self._poses()
        laser, axle = poses if poses else (None, None)
        self._forget_seen_through(laser, now)
        if laser is None:
            return

        if not self.goal_active:
            # also drop the last received plan: the first plan seen after a NEW goal starts was otherwise the old
            # goal's finished, ~0 m plan, and the new goal's full route looked like a detour (false "every route
            # blocked" on the way home from cabin D, real test 2026-10-26)
            self.last_plan, self.noticed_plan, self.plan = None, None, None
            if self.state == WAITING and now - self.since > 1.0:
                self._status(CLEAR, 'Nav2 is no longer driving')
            elif self.state == REROUTING or (self.state == CLEAR and now - self.aborted_at < 3.0 and len(self.blocked)):
                self._status(BLOCKED, 'the route is blocked and Nav2 found no other way; please clear the path')
            elif self.state == BLOCKED and len(self.blocked) < max(1, self.count_at_block):
                self._status(CLEAR, 'the way is clear again')
            return

        self._watch_detours(laser, axle)
        fresh_plan = self.plan is not None and now - self.plan_time < self.p['plan_fresh']
        route = self._ahead(self.plan, axle, self.p['look_ahead'])
        pts = self._scan_points(laser)
        if len(self.blocked):
            pts = np.vstack([pts, self.blocked[:, :2]])
        way = self._on_route(pts, route)
        dist = np.hypot(way[:, 0] - axle[0], way[:, 1] - axle[1]) if len(way) else np.zeros(0)
        if now - self.last_report >= 5.0:              # what the guard sees, for checking it works
            self.last_report = now
            self.get_logger().info('driving: plan %s, %d new-obstacle points, %d on the route ahead%s, %d remembered'
                                   % ('%.1f s old' % (now - self.plan_time) if self.plan is not None else 'none',
                                      len(pts) - len(self.blocked), len(way),
                                      ' (nearest %.2f m)' % dist.min() if len(dist) else '', len(self.blocked)))
        near = way[dist < self.p['stop_distance'] + 0.15]
        where = '(%.1f, %.1f)' % tuple(near.mean(axis=0)) if len(near) else ''

        if self.state == CLEAR:
            if not (fresh_plan and len(near) >= self.p['min_points'] and dist.min() < self.p['stop_distance']):
                self.noticed_plan = None
                return
            # give Nav2 one new plan to steer round it in the same corridor (it re-plans once a second); stop only
            # if the new plan still runs into it, or it is already very close
            if self.noticed_plan is None:
                self.noticed_plan = self.plan_count
            if self.plan_count > self.noticed_plan or dist.min() < self.p['hard_stop']:
                self.cmd_pub.publish(Twist())
                self.snapshot, self.noticed_plan = near, None
                self.len_at_stop = length(self._ahead(self.plan, axle, 1e9))
                self._status(WAITING, 'something is in the way at %s, waiting for it to move' % where)
            return

        self.cmd_pub.publish(Twist())               # hold the robot (twist_mux: above Nav2, below teleop)
        free_now = len(way) < self.p['min_points']   # nothing on the route ahead (new plan avoids it, or it left)
        self.clear_since = (self.clear_since or now) if free_now else None
        if self.state == WAITING:
            gone = not self._still_there(laser)
            self.gone_since = (self.gone_since or now) if gone else None
            plan_len = length(self._ahead(self.plan, axle, 1e9))
            if gone and now - self.gone_since >= 1.0:
                self._status(CLEAR, 'the way is free again')
            elif free_now and now - self.clear_since >= 0.5 and plan_len <= self.len_at_stop + self.p['detour_extra']:
                self._status(CLEAR, 'going round it on the same route', kind='around')
            elif now - self.since >= self.p['wait_seconds']:
                self._remember(self.snapshot, 'still in the way')
                if free_now:                          # Nav2 already has another way: take it
                    self._status(CLEAR, 'route blocked at %s, taking another way'
                                 % ('(%.1f, %.1f)' % tuple(self.snapshot.mean(axis=0))), kind='detour')
                else:
                    self.plan_mark = self.plan_count
                    self._status(REROUTING, 'route blocked at %s, looking for another way' % where)
        elif self.state == REROUTING:
            if free_now and self.plan_count > self.plan_mark + 1:
                self._status(CLEAR, 'taking another way', kind='detour')
            elif now - self.since >= self.p['reroute_seconds']:
                self._status(BLOCKED, 'route blocked at %s and no other way; please clear the path' % where)
        elif self.state == BLOCKED:
            if len(self.blocked) < max(1, self.count_at_block) or (free_now and now - self.clear_since >= 1.0):
                self._status(CLEAR, 'the way is clear again')


def main():
    rclpy.init()
    node = PathGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
