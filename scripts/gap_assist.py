#!/usr/bin/env python3
"""Gap assist: when Nav2 is stuck in a narrow gap (doorway, between furniture), line the robot up with the gap and
drive straight through, then hand back to Nav2.

Why: Nav2 Foxy plans for a round robot and its controller only checks a padded body shape on a 5 cm grid. Our wheels
sit near the FRONT of a 33 cm long body, so the back swings out when turning. In 70 cm doorways Nav2 turned too early,
then found "No valid trajectories" and gave up, although the real robot fitted (real tests 2026-10-26). A person
would turn on the spot until the robot points straight through the gap and then drive straight: that is what this does.

1. Trigger: a Nav2 goal is active, the robot has not moved for `stuck_seconds`, and the controller reported
   "No valid trajectories" / "Failed to make progress" lately. Never while the path guard holds the robot (a person
   in the way) or self-recovery searches for the position.
2. Choose: headings within `max_turn_deg` of the planned route are scored by how far the robot can drive straight
   (a strip as wide as the robot, up to `drive_distance`) using the LIVE lidar points, not the costmap grid. A heading
   only counts if the real body (+ `margin`) can turn to it without any lidar point inside it.
3. Act: turn in short steps (`step_deg`) and choose again after each step (wheel slip on smooth floors cannot fool
   it), until the best heading is straight ahead; then drive straight, checking the strip ahead all the time.
4. Hand back: Nav2 continues from freer space. At most `max_assists` per goal; if no heading is safe it says so.

  python3 src/my_bot/scripts/gap_assist.py
Topics: in /scan, /plan, /odom, /rosout, /navigate_to_pose/_action/status, /path_status, /localization_status, TF;
        out /cmd_vel_assist (Twist, twist_mux input above the guard, below self-recovery), /gap_assist_status (String)
"""
import math
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

IDLE, TURNING, SETTLE, DRIVING = 'idle', 'turning', 'settle', 'driving'


def grow(body, margin):
    """Body shape (front, back, box_half, wheel_half_len, wheel_out) with `margin` m added all round."""
    f, b, box, wl, wo = body
    return (f + margin, b + margin, box + margin, wl + margin, wo + margin)


def body_hits(pts, shape, angle=0.0):
    """True if any point (robot frame, Nx2) lies inside the body turned by `angle` round the axle.
    The body is the box (front..back, +-box_half) plus the two wheels (+-wheel_half_len along, out to wheel_out):
    only the wheels stick out past the box, so next to a wall the box can still turn (real test 2026-10-26)."""
    if len(pts) == 0:
        return False
    f, b, box, wl, wo = shape
    c, s = math.cos(angle), math.sin(angle)
    x = pts[:, 0] * c + pts[:, 1] * s           # points in the turned body's frame
    y = np.abs(-pts[:, 0] * s + pts[:, 1] * c)
    in_box = (x < f) & (x > -b) & (y < box)
    in_wheel = (np.abs(x) < wl) & (y < wo)
    return bool(np.any(in_box | in_wheel))


def can_turn(pts, shape, angle, step=math.radians(3)):
    """True if the body can turn on the spot from 0 to `angle` without touching any point."""
    n = max(1, int(abs(angle) / step))
    return not any(body_hits(pts, shape, angle * i / n) for i in range(1, n + 1))


def straight_free(pts, shape, angle, limit):
    """How far (m, up to `limit`) the body can drive straight after turning to `angle` (strip as wide as the
    wheels, starting at the front of the box or the wheels, whichever a point would meet first)."""
    f, b, box, wl, wo = shape
    c, s = math.cos(angle), math.sin(angle)
    x = pts[:, 0] * c + pts[:, 1] * s
    y = np.abs(-pts[:, 0] * s + pts[:, 1] * c)
    gaps = []
    in_box_lane = (y < box) & (x >= f)
    if in_box_lane.any():
        gaps.append((x[in_box_lane] - f).min())
    in_wheel_lane = (y >= box) & (y < wo) & (x >= wl)
    if in_wheel_lane.any():
        gaps.append((x[in_wheel_lane] - wl).min())
    return float(min([limit] + gaps))


def choose(pts, route_angle, body, margin, max_turn, min_drive, limit, turn_sign=0):
    """Best heading (rad, robot frame) and its free distance, or None. Prefers headings close to the route.
    turn_sign -1/+1: only turns to the right/left (away from a wall the robot already touches)."""
    shape = grow(body, margin)
    best = None
    for deg in range(-int(max_turn), int(max_turn) + 1, 5):
        a = route_angle + math.radians(deg)
        a = math.atan2(math.sin(a), math.cos(a))
        if turn_sign and a * turn_sign <= 0:
            continue
        if not can_turn(pts, shape, a):
            continue
        free = straight_free(pts, shape, a, limit)
        if free < min_drive:
            continue
        score = free - 0.35 * abs(math.radians(deg)) - 0.1 * abs(a)    # long straight, close to route, little turning
        if best is None or score > best[2]:
            best = (a, free, score)
    return best


class GapAssist(Node):
    def __init__(self):
        super().__init__('gap_assist')
        defaults = {
            'body_front': 0.085,      # m in front of the axle (measured 2026-10-26)
            'body_back': 0.245,       # m behind the axle
            'box_half_width': 0.1225, # m: the box is 0.245 m wide (measured 2026-10-26)
            'wheel_half_length': 0.035,  # m: wheels (radius 0.034 m) stick out only this far ahead/behind the axle
            'body_half_width': 0.156, # m from the centre to the outside of a wheel
            'margin': 0.03,           # m kept free round the real body
            'margin_tight': 0.015,    # m, tried only when nothing fits with `margin` (lidar X2: about +-1-2 cm)
            'margin_away': -0.006,    # m, last try, only turning AWAY from the nearest point: a robot whose wheel
                                      # already touches a frame (lidar: 0.16 m from the axle, wheel 0.156 m; real test
                                      # 2026-10-26) cannot keep any margin, but a wheel turning on the spot stays on
                                      # its own circle, and a 15 deg turn away by hand freed it. -6 mm = lidar noise.
            'stuck_seconds': 2.0,     # no movement for this long while Nav2 is "stuck"
            'max_turn_deg': 60.0,     # headings tried on each side of the planned route
            'route_look_ahead': 0.7,  # m along the plan for the route direction
            'drive_distance': 0.5,    # m driven straight once lined up
            'min_drive': 0.20,        # m: a heading needs at least this much straight free space
            'step_deg': 15.0,         # turn in steps of at most this many degrees, then choose again
            'aligned_deg': 6.0,       # best heading this close to straight ahead = lined up
            'turn_speed': 0.45,       # rad/s (slower does not move the heavy robot on marble)
            'drive_speed': 0.12,      # m/s
            'max_assists': 4,         # per Nav2 goal
            'cooldown': 6.0,          # s after an assist before the next one
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.body = (self.p['body_front'], self.p['body_back'], self.p['box_half_width'],
                     self.p['wheel_half_length'], self.p['body_half_width'])

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', self._on_plan, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Log, '/rosout', self._on_log, 50)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self._on_goal_status, latched)
        self.create_subscription(String, '/path_status', lambda m: setattr(self, 'guard', m.data), latched)
        self.create_subscription(String, '/localization_status', lambda m: setattr(self, 'loc', m.data), latched)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_assist', 10)
        self.status_pub = self.create_publisher(String, '/gap_assist_status', latched)

        self.scan = self.plan = self.odom = None
        self.guard, self.loc = 'clear', 'ok'
        self.goal_id, self.assists = None, 0
        self.nav_stuck_at = 0.0
        self.still_since = None
        self.last_pose = None
        self.cooldown_until = 0.0
        self.state, self.target, self.until = IDLE, 0.0, 0.0
        self.create_timer(0.05, self._tick)
        self._status('watching')
        self.get_logger().info('Gap assist ready: box %.3f/%.3f/%.3f m, wheels +-%.3f m to %.3f m, + %.2f m margin'
                               % (*self.body, self.p['margin']))

    # ---------- inputs ----------
    def _on_scan(self, s):
        self.scan = s

    def _on_plan(self, msg):
        if len(msg.poses) >= 2:
            self.plan = msg

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        self.odom = (m.pose.pose.position.x, m.pose.pose.position.y, 2 * math.atan2(q.z, q.w))

    def _on_log(self, m):
        if m.name.endswith('controller_server') and ('No valid trajectories' in m.msg or 'Failed to make progress' in m.msg):
            self.nav_stuck_at = time.monotonic()

    def _on_goal_status(self, msg):
        active = [s for s in msg.status_list if s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)]
        gid = bytes(active[-1].goal_info.goal_id.uuid) if active else None
        if gid != self.goal_id:
            self.goal_id, self.assists = gid, 0
        if gid is None and self.state != IDLE:
            self._finish('goal ended')

    # ---------- helpers ----------
    def _status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _points(self):
        """Live lidar points in the axle (base_link) frame, Nx2, without hits on the robot itself."""
        s = self.scan
        try:
            t = self.tf.lookup_transform('base_link', s.header.frame_id, Time())
        except Exception:
            return None
        r = np.array(s.ranges, dtype=float)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment
        ok = np.isfinite(r) & (r > max(s.range_min, 0.05)) & (r < 3.0)
        px, py = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
        q = t.transform.rotation
        yaw = 2 * math.atan2(q.z, q.w)
        tx, ty = t.transform.translation.x, t.transform.translation.y
        pts = np.stack([tx + px * math.cos(yaw) - py * math.sin(yaw), ty + px * math.sin(yaw) + py * math.cos(yaw)], 1)
        f, b, box = self.body[:3]
        inside = (pts[:, 0] < f) & (pts[:, 0] > -b) & (np.abs(pts[:, 1]) < box)   # the robot's own parts
        return pts[~inside]

    def _route_angle(self):
        """Direction (rad, robot frame) of the plan `route_look_ahead` m ahead of the robot, or None."""
        try:
            t = self.tf.lookup_transform('base_link', self.plan.header.frame_id, Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = 2 * math.atan2(q.z, q.w)
        tx, ty = t.transform.translation.x, t.transform.translation.y
        p = np.array([[ps.pose.position.x, ps.pose.position.y] for ps in self.plan.poses])
        x = tx + p[:, 0] * math.cos(yaw) - p[:, 1] * math.sin(yaw)
        y = ty + p[:, 0] * math.sin(yaw) + p[:, 1] * math.cos(yaw)
        i0 = int(np.argmin(np.hypot(x, y)))
        d = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x[i0:]), np.diff(y[i0:])))])
        j = i0 + int(np.searchsorted(d, self.p['route_look_ahead']))
        j = min(j, len(x) - 1)
        if math.hypot(x[j], y[j]) < 0.15:
            return None
        return math.atan2(y[j], x[j])

    def _stop(self):
        self.cmd_pub.publish(Twist())

    def _finish(self, why):
        self._stop()
        self.state = IDLE
        self.cooldown_until = time.monotonic() + self.p['cooldown']
        self._status('done: %s' % why)

    # ---------- main loop ----------
    def _tick(self):
        now = time.monotonic()
        if self.odom is None or self.scan is None:
            return
        if self.state == IDLE:
            self._watch(now)
        elif self.state == TURNING:
            self._turn(now)
        elif self.state == SETTLE:
            self._stop()
            if now > self.until:
                self._plan_step(now)
        elif self.state == DRIVING:
            self._drive(now)

    def _watch(self, now):
        moved = self.last_pose is None or math.hypot(self.odom[0] - self.last_pose[0], self.odom[1] - self.last_pose[1]) > 0.03 \
            or abs(math.atan2(math.sin(self.odom[2] - self.last_pose[2]), math.cos(self.odom[2] - self.last_pose[2]))) > 0.12
        if moved:
            self.last_pose, self.still_since = self.odom, now
        held = not self.guard.startswith(('clear', 'around')) or self.loc not in ('ok', 'recovered')
        if (self.goal_id is None or held or self.plan is None or now < self.cooldown_until
                or self.assists >= self.p['max_assists'] or now - self.nav_stuck_at > 3.0
                or now - self.still_since < self.p['stuck_seconds']):
            return
        self.assists += 1
        self._status('Nav2 is stuck in a narrow spot: lining up with the gap (try %d of %d)'
                     % (self.assists, self.p['max_assists']))
        self.steps = 0
        self._plan_step(now)

    def _plan_step(self, now):
        """Choose the best heading from the live scan; turn a step towards it, or drive if already lined up."""
        pts, route = self._points(), self._route_angle()
        if pts is None or route is None:
            return self._finish('no scan/route data')
        for margin in (self.p['margin'], self.p['margin_tight']):    # the tight margin only if nothing else fits
            best = choose(pts, route, self.body, margin, self.p['max_turn_deg'], self.p['min_drive'],
                          self.p['drive_distance'])
            if best is not None:
                break
        if best is None and len(pts):                 # already at a wall: only turn away from it
            near = pts[int(np.argmin(np.hypot(pts[:, 0], pts[:, 1])))]
            away = -1 if near[1] > 0 else 1           # wall on the left -> turn right, and the other way round
            best = choose(pts, route, self.body, self.p['margin_away'], 90.0, self.p['min_drive'],
                          self.p['drive_distance'], turn_sign=away)
            if best is not None:
                self.get_logger().info('touching on the %s: turning away from it' % ('left' if away < 0 else 'right'))
        if best is None:
            return self._finish('no safe way to line up here; Nav2 or a person must help')
        angle, free, _ = best
        self.steps += 1
        if self.steps > 12:
            return self._finish('could not line up after 12 steps')
        if abs(angle) <= math.radians(self.p['aligned_deg']):
            self.drive_left = min(free - 0.05, self.p['drive_distance'])
            if self.drive_left < 0.1:
                return self._finish('lined up, but too little room to drive on')
            self.start = self.odom
            self.state, self.until = DRIVING, now + self.drive_left / self.p['drive_speed'] + 3.0
            self._status('lined up: driving straight %.2f m' % self.drive_left)
            return
        step = math.copysign(min(abs(angle), math.radians(self.p['step_deg'])), angle)
        self.target, self.turned, self.last_yaw = step, 0.0, self.odom[2]
        self.state, self.until = TURNING, now + abs(step) / self.p['turn_speed'] + 2.5
        self.get_logger().info('route %+.0f deg, best heading %+.0f deg (%.2f m straight): turning %+.0f deg'
                               % (math.degrees(route), math.degrees(angle), free, math.degrees(step)))

    def _turn(self, now):
        d = self.odom[2] - self.last_yaw
        self.turned += math.atan2(math.sin(d), math.cos(d))
        self.last_yaw = self.odom[2]
        shape = grow(self.body, min(0.0, self.p['margin_away']))       # live check while turning
        pts = self._points()
        nxt = math.copysign(math.radians(4), self.target)
        if pts is not None and body_hits(pts, shape, nxt):
            return self._finish('stopped turning: something is close to the body')
        if abs(self.turned) >= abs(self.target) - math.radians(3) or now > self.until:
            self._stop()
            self.state, self.until = SETTLE, now + 0.4      # let it come to rest, then choose again (fresh scan)
            return
        c = Twist()
        c.angular.z = math.copysign(self.p['turn_speed'], self.target)
        self.cmd_pub.publish(c)

    def _drive(self, now):
        done = math.hypot(self.odom[0] - self.start[0], self.odom[1] - self.start[1])
        pts = self._points()
        if pts is not None and straight_free(pts, grow(self.body, self.p['margin'] / 2), 0.0, 0.2) < 0.06:
            return self._finish('stopped: something ahead (drove %.2f m)' % done)
        if done >= self.drive_left or now > self.until:
            return self._finish('through the gap (drove %.2f m); Nav2 continues' % done)
        c = Twist()
        c.linear.x = self.p['drive_speed']
        self.cmd_pub.publish(c)


def main():
    rclpy.init()
    node = GapAssist()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
