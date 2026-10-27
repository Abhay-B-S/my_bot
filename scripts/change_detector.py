#!/usr/bin/env python3
"""What changed: the robot compares what its lidar sees with the saved map on every trip (change detection, a
remote-sensing technique) and reports objects that are NEW (not on the map) and map obstacles that are GONE
(moved furniture, an open door). It also collects TROUBLE spots: where it waited, detoured, was blocked, shunted in a
narrow spot, lost its position or had a motor stop (from event_logger's /robot_event).

Each lidar ray, in map squares (the map's own resolution):
  the squares the ray passes through were seen free   -> evidence against "new" there; if the saved map has an
                                                         obstacle there (and the ray goes on > `pass_margin` beyond
                                                         it) -> evidence for "gone"
  the square where the ray stops was seen occupied    -> evidence for "new" if the saved map is free there and has no
                                                         obstacle within `map_tolerance`; evidence against "gone" for
                                                         map obstacles right there
Evidence is kept as log-odds per square (clamped), so a change has to be seen again and again, and it fades when the
place is seen differently later (the box was taken away). A change is reported once its evidence is high and it has
lasted `persist` s, so people walking past do not count. Nothing is collected while self_recovery says the position is
not certain or while the robot turns fast (a 7 Hz lidar smears during turns).
Before comparing, each scan is ALIGNED to the map: shifts of up to `search_xy` m and `search_deg` degrees from the
localisation's pose are tried and the one where most ray ends land on mapped obstacles is used (a localisation error of
10-20 cm otherwise makes whole walls look "gone", seen in the simulator on 2026-10-27). A scan whose best alignment
still puts fewer than `min_match` of its ray ends on mapped obstacles is skipped (too much has changed, or lost).

Publishes /map_changes (String JSON, latched) every `publish_period` s when something changed:
  {map, t, match (last scan's fit to the map, 0-1), changes: [{kind: new|gone, x, y, w, h, cells, first, last}],
   trouble: [{x, y, n, kinds: {waited: 2, detour: 1, ...}, last, text}]}
Kept in out_dir/map_changes_<map_name>.npz and trouble_<map_name>.json (grows over trips and runs).
Forget everything (e.g. after saving a new map): /map_changes/reset (std_msgs/Empty).

  python3 src/my_bot/scripts/change_detector.py --ros-args -p map_name:=granny_explored6
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

LO_MIN, LO_MAX = -2.0, 4.0
TROUBLE = (  # (event type, text start, label)
    ('path', 'waiting', 'waited'), ('path', 'rerouting', 'detour'), ('path', 'detour', 'detour'),
    ('path', 'blocked', 'blocked'), ('path', 'around', 'went round'),
    ('gap_assist', 'Nav2 is stuck', 'narrow spot'), ('localisation', 'lost', 'position lost'),
    ('motor', 'SAFETY STOP', 'motor stop'), ('motor', 'GAVE UP', 'motor stop'))


def components(mask):
    """8-connected groups of True squares: list of (rows, cols) arrays."""
    seen = np.zeros_like(mask, dtype=bool)
    groups = []
    for r0, c0 in zip(*np.nonzero(mask)):
        if seen[r0, c0]:
            continue
        stack, rows, cols = [(r0, c0)], [], []
        seen[r0, c0] = True
        while stack:
            r, c = stack.pop()
            rows.append(r)
            cols.append(c)
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < mask.shape[0] and 0 <= cc < mask.shape[1] and mask[rr, cc] and not seen[rr, cc]:
                        seen[rr, cc] = True
                        stack.append((rr, cc))
        groups.append((np.array(rows), np.array(cols)))
    return groups


class ChangeDetector(Node):
    def __init__(self):
        super().__init__('change_detector')
        p = self.declare_parameter
        p('map_name', 'default')
        p('out_dir', '~/dev_ws/run_logs')
        p('max_range', 5.0)          # m: rays used up to here (the X2 gets noisy further out)
        p('min_range', 0.30)         # m: closer returns are the robot itself or a person at it
        p('map_tolerance', 0.15)     # m: a return this close to a mapped obstacle is that obstacle (map/pose error)
        p('pass_margin', 0.30)       # m: a ray must go this far beyond a mapped obstacle to call it gone
        p('max_cross', 0.30)         # m: longest run of mapped obstacle a ray may cross to count as passing through it
        p('exit_free', 0.15)         # m of mapped free floor the ray must reach behind it
        p('persist', 6.0)            # s: a change must last this long before it is reported
        p('threshold', 2.0)          # log-odds needed
        p('hit', 0.5)
        p('miss', 0.25)
        p('max_turn_rate', 0.5)      # rad/s: no collecting while turning faster
        p('rate', 2.0)               # scans used per second
        p('search_xy', 0.20)         # m: alignment search around the localisation's pose
        p('search_deg', 8.0)         # degrees
        p('min_match', 0.75)         # share of ray ends that must land on mapped obstacles after alignment
        p('min_new_cells', 5)        # squares (5 cm): smaller marks were lidar noise or chair legs (real run 2026-10-27)
        p('min_gone_cells', 5)
        p('trouble_radius', 0.6)     # m: trouble events closer than this are one spot
        p('publish_period', 5.0)
        g = lambda n: self.get_parameter(n).value
        self.map_name = g('map_name')
        out = os.path.expanduser(g('out_dir'))
        os.makedirs(out, exist_ok=True)
        self.npz_path = os.path.join(out, 'map_changes_%s.npz' % self.map_name)
        self.trouble_path = os.path.join(out, 'trouble_%s.json' % self.map_name)
        self.grid = None
        self.match = None                # fit of the last scan to the map (0-1)
        self.loc_ok = True               # no self_recovery running = trust the position
        self.last_used = 0.0
        self.last_pose = None            # (time, yaw) of the last scan used, for the turn rate
        self.changed = True
        self.stats = {'used': 0, 'turning': 0, 'poor fit': 0, 'position uncertain': 0}
        self.trouble = []
        try:
            with open(self.trouble_path) as f:
                self.trouble = json.load(f)
        except (OSError, ValueError):
            pass
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(String, '/localization_status', self._on_loc, latched)
        self.create_subscription(String, '/robot_event', self._on_event, 50)
        self.create_subscription(Empty, '/map_changes/reset', self._on_reset, 10)
        self.pub = self.create_publisher(String, '/map_changes', latched)
        self.create_timer(g('publish_period'), self._publish)
        self.create_timer(30.0, self._save)
        self.create_timer(30.0, self._log_stats)
        self.get_logger().info('Change detector for map %s' % self.map_name)

    # ---------- inputs ----------
    def _on_map(self, msg):
        info = msg.info
        h, w = info.height, info.width
        data = np.array(msg.data, dtype=np.int16).reshape(h, w)
        occ = data > 50
        tol = int(round(self.get_parameter('map_tolerance').value / info.resolution))
        near = occ.copy()                    # obstacles grown by map_tolerance
        for dr in range(-tol, tol + 1):
            for dc in range(-tol, tol + 1):
                if dr * dr + dc * dc <= tol * tol:
                    near |= np.roll(np.roll(occ, dr, 0), dc, 1)
        close = occ.copy()                   # obstacles grown by 2 squares: for the alignment score
        for dr in range(-2, 3):
            for dc in range(-2, 3):
                if dr * dr + dc * dc <= 4:
                    close |= np.roll(np.roll(occ, dr, 0), dc, 1)
        self.grid = {'res': info.resolution, 'ox': info.origin.position.x, 'oy': info.origin.position.y,
                     'w': w, 'h': h, 'occ': occ, 'near': near, 'close': close, 'free': data == 0}
        self.match = None
        self.new_lo = np.zeros((h, w), np.float32)
        self.gone_lo = np.zeros((h, w), np.float32)
        self.new_first = np.full((h, w), np.nan)
        self.gone_first = np.full((h, w), np.nan)
        self.last_seen = np.full((h, w), np.nan)
        try:
            saved = np.load(self.npz_path)
            if saved['new_lo'].shape == (h, w):
                for k in ('new_lo', 'gone_lo', 'new_first', 'gone_first', 'last_seen'):
                    setattr(self, k, saved[k])
                self.get_logger().info('Loaded earlier changes from %s' % self.npz_path)
        except (OSError, KeyError, ValueError):
            pass
        self.changed = True
        self.get_logger().info('Map %dx%d at %.2f m' % (w, h, info.resolution))

    def _on_loc(self, msg):
        self.loc_ok = msg.data in ('ok', 'recovered')

    def _on_reset(self, _msg):
        if self.grid is not None:
            for a in (self.new_lo, self.gone_lo):
                a[:] = 0
            for a in (self.new_first, self.gone_first, self.last_seen):
                a[:] = np.nan
        self.trouble = []
        self.changed = True
        self._save()
        self.get_logger().info('Forgot all changes and trouble spots')

    def _on_event(self, msg):
        try:
            ev = json.loads(msg.data)
        except ValueError:
            return
        pose, typ, text = ev.get('pose'), ev.get('type'), ev.get('text') or ''
        label = next((lab for t, start, lab in TROUBLE if t == typ and text.startswith(start)), None)
        if label is None or not pose:
            return
        x, y = pose[0], pose[1]
        r = self.get_parameter('trouble_radius').value
        spot = next((s for s in self.trouble if math.hypot(s['x'] - x, s['y'] - y) < r), None)
        if spot is None:
            spot = {'x': x, 'y': y, 'n': 0, 'kinds': {}}
            self.trouble.append(spot)
        spot['x'] = round((spot['x'] * spot['n'] + x) / (spot['n'] + 1), 2)    # centre of all its events
        spot['y'] = round((spot['y'] * spot['n'] + y) / (spot['n'] + 1), 2)
        spot['n'] += 1
        spot['kinds'][label] = spot['kinds'].get(label, 0) + 1
        spot['last'] = round(ev.get('t') or time.time())
        spot['text'] = text.split(':', 1)[-1].strip()[:100]
        self.changed = True

    def _on_scan(self, scan):
        now = time.monotonic()
        if self.grid is None or now - self.last_used < 1.0 / self.get_parameter('rate').value:
            return
        if not self.loc_ok:
            self.stats['position uncertain'] += 1
            self.last_used = now
            return
        try:
            t = self.tf.lookup_transform('map', scan.header.frame_id, Time())
        except Exception:
            return
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        prev, self.last_pose = self.last_pose, (now, yaw)
        self.last_used = now
        if prev is None:
            return
        rate = abs((yaw - prev[1] + math.pi) % (2 * math.pi) - math.pi) / max(now - prev[0], 1e-3)
        if rate > self.get_parameter('max_turn_rate').value:
            self.stats['turning'] += 1
            return
        pose = self._align(scan, t.transform.translation.x, t.transform.translation.y, yaw)
        if pose is None:
            self.stats['poor fit'] += 1
            return
        self.stats['used'] += 1
        self._integrate(scan, *pose)

    def _align(self, scan, lx, ly, yaw):
        """The pose near (lx, ly, yaw) where most ray ends land on mapped obstacles, or None if none fits well."""
        g = self.grid
        r = np.array(scan.ranges, dtype=np.float64)
        ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
        ok = np.isfinite(r) & (r > self.get_parameter('min_range').value) & (r <= self.get_parameter('max_range').value)
        r, ang = r[ok], ang[ok]
        if len(r) < 30:
            return None
        res, xy = g['res'], self.get_parameter('search_xy').value
        offs = np.arange(-xy, xy + res / 2, res)
        dx, dy = [a.ravel() for a in np.meshgrid(offs, offs)]
        best = (-1.0, 0.0, 0.0, 0.0, 0.0)
        for dth in np.radians(np.arange(-self.get_parameter('search_deg').value, self.get_parameter('search_deg').value
                                        + 0.1, 2.0)):
            ex = lx + r * np.cos(ang + yaw + dth)
            ey = ly + r * np.sin(ang + yaw + dth)
            c = np.floor((ex[None, :] + dx[:, None] - g['ox']) / res).astype(np.int64)
            rr = np.floor((ey[None, :] + dy[:, None] - g['oy']) / res).astype(np.int64)
            inside = (c >= 0) & (c < g['w']) & (rr >= 0) & (rr < g['h'])
            hits = np.zeros(c.shape, bool)
            hits[inside] = g['close'][rr[inside], c[inside]]
            # share of ray ends on obstacles, with a small penalty for moving far from the localisation's pose
            score = hits.mean(axis=1) - 0.02 * (np.hypot(dx, dy) / xy + abs(dth) / 0.14)
            k = int(np.argmax(score))
            if score[k] > best[0]:
                best = (float(score[k]), dx[k], dy[k], dth, float(hits[k].mean()))
        self.match = round(best[4], 2)
        if best[4] < self.get_parameter('min_match').value:
            return None
        return lx + best[1], ly + best[2], yaw + best[3]

    # ---------- evidence ----------
    def _integrate(self, scan, lx, ly, yaw):
        g = self.grid
        res, max_r = g['res'], self.get_parameter('max_range').value
        r = np.array(scan.ranges, dtype=np.float32)
        a = scan.angle_min + np.arange(len(r)) * scan.angle_increment + yaw
        valid = np.isfinite(r) & (r > self.get_parameter('min_range').value)
        hit = valid & (r <= max_r)
        reach = np.where(hit, r, max_r)           # no return (or too far): free up to max_range
        keep = valid | ~np.isfinite(r)
        a, reach, hit = a[keep], reach[keep], hit[keep]
        ca, sa = np.cos(a), np.sin(a)
        steps = np.arange(1, int(max_r / res) + 1) * res

        def cells(xs, ys):
            c = np.floor((xs - g['ox']) / res).astype(np.int64)
            rr = np.floor((ys - g['oy']) / res).astype(np.int64)
            ok = (c >= 0) & (c < g['w']) & (rr >= 0) & (rr < g['h'])
            return rr[ok], c[ok], ok

        # squares passed through (stop 2 squares before the end: the end square itself is the obstacle)
        tt = np.broadcast_to(steps, (len(a), len(steps)))
        through = tt < (reach[:, None] - 2 * res)
        xs = lx + ca[:, None] * tt
        ys = ly + sa[:, None] * tt
        rr, cc, ok = cells(xs[through], ys[through])
        np.add.at(self.new_lo, (rr, cc), -self.get_parameter('miss').value)
        # gone: a mapped obstacle the ray passed well beyond, crossing it (at most `max_cross` m of mapped obstacle in
        # a row along the ray; a longer run is a ray sliding along a wall the map draws a bit too thick)
        beyond = through & (tt < (reach[:, None] - self.get_parameter('pass_margin').value)) & hit[:, None]
        bc = np.clip(np.floor((xs - g['ox']) / res).astype(np.int64), 0, g['w'] - 1)
        br = np.clip(np.floor((ys - g['oy']) / res).astype(np.int64), 0, g['h'] - 1)
        mapped = beyond & g['occ'][br, bc]
        max_run = int(round(self.get_parameter('max_cross').value / res))
        exit_n = max(1, int(round(self.get_parameter('exit_free').value / res)))
        for b_i in np.nonzero(mapped.any(axis=1))[0]:
            row = mapped[b_i]
            k = 0
            while k < len(row):
                if not row[k]:
                    k += 1
                    continue
                e = k
                while e < len(row) and row[e]:
                    e += 1
                # it must come out on the other side into mapped free floor for `exit_free` m (a ray sliding along
                # a wall zig-zags in and out of its edge squares but never reaches free floor behind it)
                x_end = e + exit_n
                if e - k <= max_run and x_end <= len(row) and beyond[b_i, x_end - 1] and \
                        g['free'][br[b_i, e:x_end], bc[b_i, e:x_end]].all():
                    np.add.at(self.gone_lo, (br[b_i, k:e], bc[b_i, k:e]), self.get_parameter('hit').value)
                k = e
        # end squares
        ex, ey = lx + ca[hit] * reach[hit], ly + sa[hit] * reach[hit]
        er, ec, _ = cells(ex, ey)
        new = g['free'][er, ec] & ~g['near'][er, ec]      # known free floor on the map, away from its obstacles
        np.add.at(self.new_lo, (er[new], ec[new]), self.get_parameter('hit').value + self.get_parameter('miss').value)
        at_map = g['near'][er, ec]
        for dr in (-1, 0, 1):                     # a return at a mapped obstacle: it is still there
            for dc in (-1, 0, 1):
                r3 = np.clip(er[at_map] + dr, 0, g['h'] - 1)
                c3 = np.clip(ec[at_map] + dc, 0, g['w'] - 1)
                np.add.at(self.gone_lo, (r3, c3), -self.get_parameter('hit').value)
        np.clip(self.new_lo, LO_MIN, LO_MAX, out=self.new_lo)
        np.clip(self.gone_lo, LO_MIN, LO_MAX, out=self.gone_lo)
        wall = time.time()
        for lo, first in ((self.new_lo, self.new_first), (self.gone_lo, self.gone_first)):
            started = (lo > 0) & np.isnan(first)
            first[started] = wall
            first[lo <= 0] = np.nan
        self.last_seen[er, ec] = wall
        self.changed = True

    # ---------- output ----------
    def changes(self):
        if self.grid is None:
            return []
        g, out, wall = self.grid, [], time.time()
        th, persist = self.get_parameter('threshold').value, self.get_parameter('persist').value
        for kind, lo, first, min_cells in (
                ('new', self.new_lo, self.new_first, self.get_parameter('min_new_cells').value),
                ('gone', self.gone_lo, self.gone_first, self.get_parameter('min_gone_cells').value)):
            with np.errstate(invalid='ignore'):
                mask = (lo > th) & (wall - first >= persist)
            for rows, cols in components(mask):
                if len(rows) < min_cells:
                    continue
                x0, x1 = g['ox'] + cols.min() * g['res'], g['ox'] + (cols.max() + 1) * g['res']
                y0, y1 = g['oy'] + rows.min() * g['res'], g['oy'] + (rows.max() + 1) * g['res']
                seen = self.last_seen[rows, cols]
                out.append({'kind': kind, 'x': round((x0 + x1) / 2, 2), 'y': round((y0 + y1) / 2, 2),
                            'w': round(x1 - x0, 2), 'h': round(y1 - y0, 2), 'cells': int(len(rows)),
                            'first': round(float(np.nanmin(first[rows, cols]))),
                            'last': round(float(np.nanmax(seen))) if np.isfinite(seen).any() else None})
        return out

    def _publish(self):
        if not self.changed:
            return
        self.changed = False
        ch = self.changes()
        self.pub.publish(String(data=json.dumps({'map': self.map_name, 't': round(time.time()), 'match': self.match,
                                                 'changes': ch,
                                                 'trouble': self.trouble})))

    def _log_stats(self):
        if sum(self.stats.values()):
            self.get_logger().info('scans in the last 30 s: %s; last fit %s' % (
                ', '.join('%s %d' % kv for kv in self.stats.items()), self.match))
        self.stats = dict.fromkeys(self.stats, 0)

    def _save(self):
        try:
            if self.grid is not None:
                with open(self.npz_path + '.tmp', 'wb') as f:      # a file object: numpy adds no extra ".npz"
                    np.savez_compressed(f, new_lo=self.new_lo, gone_lo=self.gone_lo, new_first=self.new_first,
                                        gone_first=self.gone_first, last_seen=self.last_seen)
                os.replace(self.npz_path + '.tmp', self.npz_path)
            with open(self.trouble_path + '.tmp', 'w') as f:
                json.dump(self.trouble, f)
            os.replace(self.trouble_path + '.tmp', self.trouble_path)
        except OSError as exc:
            self.get_logger().warn('Could not save: %s' % exc)


def main():
    rclpy.init()
    node = ChangeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
