#!/usr/bin/env python3
"""Self-recovery: notices when the robot no longer knows where it is, and finds its position again.

The "kidnapped robot problem": someone picks the robot up and puts it somewhere else, it is pushed or bumped,
or localisation simply drifts away. AMCL then keeps believing the old position.

1. Detect. Twice a second this program checks what share of the lidar points land on a wall of the map at the
   position the robot believes it is at (normally 90-100 %). Below `lost_threshold` for `lost_seconds` = lost.
   It publishes "lost" on /localization_status; the delivery manager pauses the job and Nav2 stops.
2. Search (global scan matching). AMCL's own global localisation spreads too few guesses over a whole room to
   settle reliably (tested: 90 s of turning, only 52 % match). Instead this program tries the CURRENT scan at
   every free spot of the map (every `search_step` m, `search_angles` directions), keeps the best candidates,
   refines them finely, and accepts the best pose when it matches at least `good_threshold` AND clearly beats
   the best different place by `unique_margin` (similar-looking places = ambiguous).
3. If the result is ambiguous or weak: turn `turn_step_deg` on the spot (when there is room) for a new view and
   search again, up to a full circle; then publish "help: ..." and keep watching.
4. Accepted pose -> sent to AMCL as /initialpose. When the live match stays above `good_threshold` for
   `good_checks` checks, publish "recovered"; the manager continues the delivery.
It never turns when something is closer than `clear_radius` to the axle (the back end swings ~0.29 m). A
2D Pose Estimate in RViz that makes the lidar fit the map again also counts as "recovered".

  python3 src/my_bot/scripts/self_recovery.py        (real robot and simulation alike: it uses wall-clock timers)
Topics: in /map, /scan, TF map->laser_frame, base_link->laser_frame;  out /localization_status (String, latched),
        /localization_match (String), /cmd_vel_recovery (Twist, twist_mux input), /initialpose.
"""
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

OK, LOST, TURNING, VERIFY, HELP = 'ok', 'lost', 'turning', 'verifying', 'help'


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SelfRecovery(Node):
    def __init__(self):
        super().__init__('self_recovery')
        defaults = {
            'lost_threshold': 0.50,   # share of scan points on walls below which we may be lost (2026-10-26: was 0.40;
                                      # a 53-80 % match for 40 s while turning on marble drove the robot into the sofa;
                                      # 0.60 stopped it needlessly beside railings, which always match ~55 %)
            'lost_seconds': 2.0,      # ... for this long (was 3.0)
            'good_threshold': 0.75,   # share that counts as "found"
            'good_checks': 3,         # ... this many checks in a row (0.5 s apart)
            'unique_margin': 0.05,    # the best pose must beat the best DIFFERENT place by this much
            'search_step': 0.2,       # m between candidate positions in the coarse search
            'search_angles': 24,      # directions tried at each candidate position
            'turn_speed': 0.4,        # rad/s while turning for a new view
            'turn_step_deg': 60.0,    # degrees turned between searches
            'max_searches': 6,        # searches before asking for help (6 x 60 deg = a full circle)
            'verify_seconds': 5.0,    # time allowed for AMCL to accept the found pose
            'clear_radius': 0.33,     # m around the axle that must be free before turning
            'max_range': 6.0,         # ignore far scan points (less reliable)
            'cooldown': 10.0,         # s after a recovery before another can start
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/localization_status', latched)
        self.match_pub = self.create_publisher(String, '/localization_match', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_recovery', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wall = self.near1 = self.near2 = None   # map masks: walls, within 1 cell, within 2 cells
        self.free_xy = None                          # coarse candidate positions (free floor)
        self.info = None
        self.scan = None
        self.state = OK
        self.low_since = None
        self.good_count = 0
        self.searches = 0
        self.turn_until = 0.0
        self.verify_until = 0.0
        self.cooldown_until = 0.0
        self.last_log = 0.0
        self._status(OK, 'watching')
        self.create_timer(0.5, self._check)
        self.create_timer(0.1, self._drive)
        self.get_logger().info('Self-recovery watching: lost below %.0f %% for %.1f s, found above %.0f %%'
                               % (100 * self.p['lost_threshold'], self.p['lost_seconds'], 100 * self.p['good_threshold']))

    # ---------- inputs ----------
    def _on_map(self, m):
        info = m.info
        grid = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
        occ = grid >= 65

        def grow(mask, k):
            out = mask.copy()
            for dx in range(-k, k + 1):
                for dy in range(-k, k + 1):
                    out |= np.roll(np.roll(mask, dx, axis=1), dy, axis=0)
            return out
        self.wall, self.near1, self.near2, self.info = occ, grow(occ, 1), grow(occ, 2), info
        free = (grid >= 0) & (grid < 25) & ~grow(occ, 3)          # free floor, at least 15 cm from walls
        step = max(1, int(round(self.p['search_step'] / info.resolution)))
        ys, xs = np.nonzero(free[::step, ::step])
        self.free_xy = np.stack([info.origin.position.x + (xs * step + 0.5) * info.resolution,
                                 info.origin.position.y + (ys * step + 0.5) * info.resolution], axis=1)
        self.get_logger().info('Map received: %d x %d cells, %d search positions'
                               % (info.width, info.height, len(self.free_xy)))

    def _on_scan(self, s):
        self.scan = s

    # ---------- scan helpers ----------
    def _scan_polar(self, max_points=None):
        """Valid scan (ranges, angles) in the laser frame, optionally thinned to max_points."""
        s = self.scan
        r = np.array(s.ranges, dtype=np.float64)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, self.p['max_range']))
        r, a = r[ok], a[ok]
        if max_points and len(r) > max_points:
            idx = np.linspace(0, len(r) - 1, max_points).astype(int)
            r, a = r[idx], a[idx]
        return r, a

    def _points(self, frame):
        """Scan points (x, y) in `frame`, or None when the transform is not available."""
        try:
            t = self.tf_buffer.lookup_transform(frame, self.scan.header.frame_id, Time())
        except Exception:
            return None
        r, a = self._scan_polar()
        yaw = yaw_of(t.transform.rotation)
        return t.transform.translation.x + r * np.cos(a + yaw), t.transform.translation.y + r * np.sin(a + yaw)

    def _score(self, xs, ys, mask):
        """Share of points (arrays, any shape; last axis = points) that land on `mask`."""
        info = self.info
        cx = ((xs - info.origin.position.x) / info.resolution).astype(np.int64)
        cy = ((ys - info.origin.position.y) / info.resolution).astype(np.int64)
        inside = (cx >= 0) & (cx < info.width) & (cy >= 0) & (cy < info.height)
        hit = np.zeros(cx.shape, dtype=bool)
        hit[inside] = mask[cy[inside], cx[inside]]
        return hit.mean(axis=-1)

    def _match(self):
        if self.scan is None or self.near2 is None:
            return None
        pts = self._points('map')
        if pts is None or len(pts[0]) < 20:
            return None
        return float(self._score(pts[0], pts[1], self.near2))

    def _space_to_turn(self):
        """True when nothing is closer than clear_radius to the axle (base_link)."""
        pts = self._points('base_link')
        if pts is None:
            return False
        d = np.hypot(pts[0], pts[1])
        return len(d) == 0 or float(d.min()) > self.p['clear_radius']

    # ---------- global scan matching ----------
    def _search(self):
        """Best laser pose (x, y, yaw, score, runner_up_score) for the current scan over the whole map."""
        r, a = self._scan_polar(max_points=90)
        if len(r) < 20 or self.free_xy is None:
            return None
        cand = self.free_xy
        n_ang = int(self.p['search_angles'])
        yaws = np.arange(n_ang) * 2 * math.pi / n_ang
        results = []                                             # (score, x, y, yaw)
        for yaw in yaws:
            ox, oy = r * np.cos(a + yaw), r * np.sin(a + yaw)   # scan offsets for this direction
            sc = self._score(cand[:, 0:1] + ox, cand[:, 1:2] + oy, self.near2)
            best = np.argsort(sc)[-8:]
            results += [(sc[i], cand[i, 0], cand[i, 1], yaw) for i in best]
        results.sort(reverse=True)
        # refine the 10 best coarse candidates: +-0.2 m in 5 cm steps, +-10 deg in 2.5 deg steps, all points
        rf, af = self._scan_polar()
        refined = []
        d = np.arange(-0.2, 0.2001, 0.05)
        dyaws = np.radians(np.arange(-10, 10.01, 2.5))
        gx, gy = np.meshgrid(d, d)
        for _, x0, y0, yaw0 in results[:10]:
            best = (-1, 0, 0, 0)
            for dyaw in dyaws:
                yaw = yaw0 + dyaw
                ox, oy = rf * np.cos(af + yaw), rf * np.sin(af + yaw)
                xs = (x0 + gx.ravel())[:, None] + ox
                ys = (y0 + gy.ravel())[:, None] + oy
                sc = self._score(xs, ys, self.near1)
                i = int(np.argmax(sc))
                if sc[i] > best[0]:
                    best = (float(sc[i]), x0 + gx.ravel()[i], y0 + gy.ravel()[i], yaw)
            refined.append(best)
        refined.sort(reverse=True)
        top = refined[0]
        # runner-up: best candidate that is a DIFFERENT place (> 0.5 m away or > 30 deg turned)
        other = 0.0
        for s, x, y, yaw in refined[1:]:
            dyaw = abs(math.atan2(math.sin(yaw - top[3]), math.cos(yaw - top[3])))
            if math.hypot(x - top[1], y - top[2]) > 0.5 or dyaw > math.radians(30):
                other = s
                break
        return top[1], top[2], top[3], top[0], other

    def _laser_offset(self):
        """(x, y, yaw) of the laser in base_link."""
        try:
            t = self.tf_buffer.lookup_transform('base_link', self.scan.header.frame_id, Time())
            return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        except Exception:
            return -0.1995, 0.0, 0.0                               # the delivery body's lidar mount

    def _set_pose(self, lx, ly, lyaw):
        ox, oy, oyaw = self._laser_offset()
        yaw = lyaw - oyaw                                          # robot heading
        x = lx - (ox * math.cos(yaw) - oy * math.sin(yaw))         # axle = laser minus its offset
        y = ly - (ox * math.sin(yaw) + oy * math.cos(yaw))
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.02
        cov[35] = 0.02
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)
        return x, y, yaw

    # ---------- the state machine ----------
    def _status(self, state, detail):
        self.state = state
        if state == OK:
            text = 'recovered' if detail == 'recovered' else 'ok'
        elif state == LOST:
            text = 'lost'
        else:
            text = '%s: %s' % (state, detail)
        self.status_pub.publish(String(data=text))
        self.get_logger().info('[%s] %s' % (state, detail))

    def _check(self):
        now = time.monotonic()
        m = self._match()
        if m is not None:
            self.match_pub.publish(String(data='%.2f' % m))
            if now - self.last_log > 10.0:
                self.last_log = now
                self.get_logger().info('match %.0f %% (%s)' % (100 * m, self.state))

        if self.state == OK:
            if m is not None and m < self.p['lost_threshold'] and now > self.cooldown_until:
                self.low_since = self.low_since or now
                if now - self.low_since >= self.p['lost_seconds']:
                    self._status(LOST, 'only %.0f %% of the lidar matches the map: position lost' % (100 * m))
                    self.searches = 0
                    self.turn_until = now + 1.0                    # let the robot stop before the first search
                    self.state = TURNING
            else:
                self.low_since = None
            return

        # any non-OK state: a good live match ends the recovery (also after a 2D Pose Estimate by hand)
        self.good_count = self.good_count + 1 if (m is not None and m >= self.p['good_threshold']) else 0
        if self.good_count >= self.p['good_checks']:
            self.cmd_pub.publish(Twist())
            self.cooldown_until = now + self.p['cooldown']
            self.low_since = None
            self.good_count = 0
            self._status(OK, 'recovered')
            return

        if self.state == TURNING and now >= self.turn_until:
            self.cmd_pub.publish(Twist())
            self._search_now()
        elif self.state == VERIFY and now >= self.verify_until:
            self.get_logger().warn('AMCL did not settle on the found pose; searching again')
            self._next_view()

    def _search_now(self):
        if self.scan is None or self.info is None:
            return
        self.searches += 1
        t0 = time.monotonic()
        res = self._search()
        if res is None:
            self._next_view()
            return
        lx, ly, lyaw, score, other = res
        self.get_logger().info('search %d: best %.0f %% at laser x=%.2f y=%.2f yaw=%.0f deg; best other place %.0f %% '
                               '(%.1f s)' % (self.searches, 100 * score, lx, ly, math.degrees(lyaw), 100 * other,
                                             time.monotonic() - t0))
        if score >= self.p['good_threshold'] and score - other >= self.p['unique_margin']:
            x, y, yaw = self._set_pose(lx, ly, lyaw)
            self._status(VERIFY, 'found x=%.2f y=%.2f facing %.0f deg (%.0f %% match); checking'
                         % (x, y, math.degrees(yaw), 100 * score))
            self.verify_until = time.monotonic() + self.p['verify_seconds']
        else:
            why = 'two places look alike' if score >= self.p['good_threshold'] else 'no place matches well'
            self.get_logger().info('not sure (%s): turning for a new view' % why)
            self._next_view()

    def _next_view(self):
        if self.searches >= self.p['max_searches']:
            self._status(HELP, 'could not find the position; set a 2D Pose Estimate in RViz')
            return
        if not self._space_to_turn():
            self._status(HELP, 'too close to something to turn for a new view; move the robot into open '
                               'space or set a 2D Pose Estimate in RViz')
            return
        self.turn_until = time.monotonic() + math.radians(self.p['turn_step_deg']) / self.p['turn_speed']
        self._status(TURNING, 'turning %.0f deg for a new view' % self.p['turn_step_deg'])

    def _drive(self):
        if self.state == TURNING and time.monotonic() < self.turn_until and self.searches > 0:
            t = Twist()
            t.angular.z = self.p['turn_speed']
            self.cmd_pub.publish(t)


def main():
    rclpy.init()
    node = SelfRecovery()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
