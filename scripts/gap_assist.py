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
2. Plan: from the LIVE lidar points (not the costmap grid) and the real body shape (box + wheels), search the
   cheapest sequence of moves after which the robot points along the planned route with `goal_free` m free straight
   ahead. Moves: turns of 15 or 45 deg on the spot (round the middle of the axle), round the left wheel or round the
   right wheel (2026-10-26, the user's idea: an arm with shoulder, elbow and wrist - keep one wheel still and let the
   other swing the robot), or 10 / 30 cm ahead; no straight reversing (it upsets the motor Arduino). The whole arc
   each move sweeps must stay clear, like a driver shunting in a tight spot (a corner needs a small turn one way,
   then the other). The free space round the body (front, back, left, right) is logged when it starts.
3. Act: do only the first move, stop, take a fresh scan and plan again (wheel slip on smooth floors cannot add up),
   until the robot points out along the route; then drive straight out, checking the strip ahead all the time.
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


def pivot_pose(angle, yc):
    """Pose (x, y, heading) of the axle after turning by `angle` round the point (0, yc) of the robot frame.
    yc = 0: on the spot round the middle of the axle; yc = +-half the wheel spacing: round the left/right wheel
    (that wheel stands still, the other one drives: "shoulder fixed, the wrist moves")."""
    return yc * math.sin(angle), yc - yc * math.cos(angle), angle


def to_body(pts, pose):
    """Points (robot frame, Nx2) seen from the body at `pose` (x, y, heading)."""
    x0, y0, th = pose
    c, s = math.cos(th), math.sin(th)
    dx, dy = pts[:, 0] - x0, pts[:, 1] - y0
    return dx * c + dy * s, -dx * s + dy * c


def body_hits(pts, shape, pose=(0.0, 0.0, 0.0)):
    """True if any point (robot frame, Nx2) lies inside the body standing at `pose`.
    The body is the box (front..back, +-box_half) plus the two wheels (+-wheel_half_len along, out to wheel_out):
    only the wheels stick out past the box, so next to a wall the box can still turn (real test 2026-10-26)."""
    if len(pts) == 0:
        return False
    f, b, box, wl, wo = shape
    x, y = to_body(pts, pose)
    y = np.abs(y)
    in_box = (x < f) & (x > -b) & (y < box)
    in_wheel = (np.abs(x) < wl) & (y < wo)
    return bool(np.any(in_box | in_wheel))


def can_turn(pts, shape, angle, yc=0.0, step=math.radians(3)):
    """True if the body can turn by `angle` round (0, yc) without touching any point anywhere on the arc it sweeps."""
    n = max(1, int(abs(angle) / step))
    return not any(body_hits(pts, shape, pivot_pose(angle * i / n, yc)) for i in range(1, n + 1))


def straight_free(pts, shape, pose, limit):
    """How far (m, up to `limit`) the body standing at `pose` can drive straight ahead (strip as wide as the
    wheels, starting at the front of the box or the wheels, whichever a point would meet first)."""
    f, b, box, wl, wo = shape
    x, y = to_body(pts, pose)
    y = np.abs(y)
    gaps = []
    in_box_lane = (y < box) & (x >= f)
    if in_box_lane.any():
        gaps.append((x[in_box_lane] - f).min())
    in_wheel_lane = (y >= box) & (y < wo) & (x >= wl)
    if in_wheel_lane.any():
        gaps.append((x[in_wheel_lane] - wl).min())
    return float(min([limit] + gaps))


def clearances(pts, body):
    """Free space (m) in front of, behind, left and right of the real body, from the live points."""
    f, b, box, wl, wo = body
    x, y = pts[:, 0], pts[:, 1]
    side = (x < f) & (x > -b)
    lane = np.abs(y) < wo
    def least(v):
        return float(v.min()) if len(v) else 9.9
    return {'front': least(x[lane & (x >= f)] - f), 'back': least(-x[lane & (x <= -b)] - b),
            'left': least(y[side & (y > 0)] - box), 'right': least(-y[side & (y < 0)] - box)}


def compose(pose, move):
    """Pose (start frame) after a move done from `pose`: ('turn', angle, yc) or ('ahead', distance)."""
    x, y, th = pose
    if move[0] == 'turn':
        dx, dy, dth = pivot_pose(move[1], move[2])
    else:
        dx, dy, dth = move[1], 0.0, 0.0
    c, s = math.cos(th), math.sin(th)
    return x + c * dx - s * dy, y + s * dx + c * dy, th + dth


def move_ok(pts, shape, pose, move):
    """True if the move from `pose` (start frame) keeps the whole body clear of the points all the way."""
    local = np.stack(to_body(pts, pose), 1)
    if move[0] == 'turn':
        return can_turn(local, shape, move[1], move[2])
    n = max(1, int(move[1] / 0.025))
    return not any(body_hits(local, shape, (move[1] * i / n, 0.0, 0.0)) for i in range(1, n + 1))


def move_cost(m):
    """Effort of a move: grows with its size; turning round a wheel and arcs that move backwards cost a bit more."""
    backwards = m[0] == 'turn' and m[2] * m[1] < 0
    size = abs(m[1]) * 0.5 if m[0] == 'turn' else m[1]
    return 0.3 + size + (0.2 if m[0] == 'turn' and m[2] else 0.0) + (0.3 if backwards else 0.0)


def seq_cost(seq):
    return sum(move_cost(m) for m in seq)


def plan_moves(pts, route, body, margin, pivots, step, goal_free, route_tol, max_moves=6, max_nodes=4000,
               turn_sign=0, backward_arcs=True):
    """Cheapest sequence of moves (each: turn by +-step or +-3 step round the middle or a wheel, or 10 / 30 cm
    ahead; no straight reversing: it upsets the motor Arduino) after which
    the robot points within `route_tol` of the route with `goal_free` m free straight ahead, never touching a point.
    Like a driver shunting in a tight spot: a corner often needs a small turn one way, then the other.
    Returns (moves, free) or None. turn_sign -1/+1: the first move must turn right/left (away from a wall)."""
    import heapq
    shape = grow(body, margin)
    moves = ([('turn', d * k * step, yc) for yc in pivots for d in (1, -1) for k in (1, 3)
              if backward_arcs or yc * d >= 0]      # yc * angle < 0: the axle moves back, one wheel runs backwards
             + [('ahead', 0.10), ('ahead', 0.30)])
    start = (0.0, 0.0, 0.0)
    pq, seen, count = [(0.0, 0, start, [])], set(), 0
    while pq and count < max_nodes:
        _, _, pose, seq = heapq.heappop(pq)
        cost = seq_cost(seq)
        dev = math.atan2(math.sin(pose[2] - route), math.cos(pose[2] - route))
        if seq and abs(dev) <= route_tol:
            free = straight_free(pts, shape, pose, goal_free)
            if free >= goal_free:
                return seq, free
        if len(seq) >= max_moves:
            continue
        for m in moves:
            if not seq and turn_sign and (m[0] != 'turn' or m[1] * turn_sign <= 0):
                continue
            nxt = compose(pose, m)
            key = (round(nxt[0] / 0.03), round(nxt[1] / 0.03), round(math.atan2(math.sin(nxt[2]), math.cos(nxt[2])) / 0.13))
            if key in seen:
                continue
            count += 1
            if not move_ok(pts, shape, pose, m):
                continue
            seen.add(key)
            c = cost + move_cost(m)
            ndev = math.atan2(math.sin(nxt[2] - route), math.cos(nxt[2] - route))
            togo = 0.5 * max(0.0, abs(ndev) - route_tol)   # turning still needed: try moves towards the route first
            heapq.heappush(pq, (c + togo, count, nxt, seq + [m]))
    return None


def describe(move):
    if move[0] == 'ahead':
        return 'drive %.0f cm ahead' % (move[1] * 100)
    return 'turn %+.0f deg %s' % (math.degrees(move[1]), pivot_name(move[2]))


def pivot_name(yc):
    return 'on the spot' if not yc else ('round the left wheel' if yc > 0 else 'round the right wheel')


class GapAssist(Node):
    def __init__(self):
        super().__init__('gap_assist')
        defaults = {
            'body_front': 0.085,      # m in front of the axle (measured 2026-10-26)
            'body_back': 0.245,       # m behind the axle
            'box_half_width': 0.1225, # m: the box is 0.245 m wide (measured 2026-10-26)
            'wheel_half_length': 0.035,  # m: wheels (radius 0.034 m) stick out only this far ahead/behind the axle
            'body_half_width': 0.156, # m from the centre to the outside of a wheel
            'wheel_separation': 0.288,  # m between the wheel centres: turning round one wheel pivots this far aside
            'margin': 0.03,           # m kept free round the real body
            'margin_tight': 0.015,    # m, tried only when nothing fits with `margin` (lidar X2: about +-1-2 cm)
            'margin_away': -0.006,    # m, last try, only turning AWAY from the nearest point: a robot whose wheel
                                      # already touches a frame (lidar: 0.16 m from the axle, wheel 0.156 m; real test
                                      # 2026-10-26) cannot keep any margin, but a wheel turning on the spot stays on
                                      # its own circle, and a 15 deg turn away by hand freed it. -6 mm = lidar noise.
            'stuck_seconds': 2.0,     # no movement for this long while Nav2 is "stuck"
            'route_look_ahead': 0.7,  # m along the plan for the route direction
            'drive_distance': 0.5,    # m driven straight once lined up
            'step_deg': 15.0,         # turn in steps of at most this many degrees, then choose again
            'goal_free': 0.40,        # m free straight ahead along the route = way out found
            'route_tol_deg': 30.0,    # ... while pointing within this of the route
            'max_moves': 6,           # longest move sequence searched (moves of 15 or 45 deg, 10 or 30 cm)
            'backward_arcs': False,   # turns round a wheel that drive the other wheel BACKWARDS at full speed: a
                                      # motor Arduino reset followed one (real test 2026-10-26); on after the wiring fix
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
        h = self.p['wheel_separation'] / 2
        self.pivots = (0.0, h, -h)
        self.laser_tf = None          # the lidar is fixed on the robot: keep its position once known
        self.goal_xy = None           # end of the latest plan (route direction when the plan is too short)

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
        self.turn_margin = 0.0
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
            self.goal_xy = (msg.header.frame_id, msg.poses[-1].pose.position.x, msg.poses[-1].pose.position.y)

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
            self.laser_tf = self.tf.lookup_transform('base_link', s.header.frame_id, Time())
        except Exception:
            if self.laser_tf is None:
                return None
        t = self.laser_tf
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
        if math.hypot(x[j], y[j]) < 0.15:          # plan too short here: aim at its end (the goal) instead
            j = len(x) - 1
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
        self.steps, self.then_plan, self.no_straight = 0, False, False
        self._plan_step(now)

    def _plan_step(self, now):
        """From a fresh scan: already pointing out along the route with room ahead -> drive straight out;
        otherwise search the shortest safe move sequence and do only its first move (then plan again)."""
        pts, route = self._points(), self._route_angle()
        if pts is None:
            return self._finish('no lidar data in the robot frame (TF)')
        wide = route is None                          # no usable plan: look all round, prefer straight ahead
        if wide:
            route = 0.0
            self.get_logger().warn('no route direction from the plan: looking all round')
        if self.steps == 0:
            c = clearances(pts, self.body)
            self.get_logger().info('free space round the body: front %.2f m, back %.2f m, left %.2f m, right %.2f m'
                                   % (c['front'], c['back'], c['left'], c['right']))
        tol = math.pi if wide else math.radians(self.p['route_tol_deg'])
        step = math.radians(self.p['step_deg'])
        ahead = straight_free(pts, grow(self.body, self.p['margin'] / 2), (0.0, 0.0, 0.0), self.p['drive_distance'])
        # (the same margin as the check while driving: with 3 cm here and 1.5 cm there, a wall 3 cm beside the
        #  front wheel counted as "free ahead" but stopped the drive at once, real test 2026-10-26)
        if abs(route) <= tol and ahead >= self.p['goal_free'] and not self.no_straight:
            self.drive_left, self.start, self.then_plan = min(ahead - 0.05, self.p['drive_distance']), self.odom, False
            self.state, self.until = DRIVING, now + self.drive_left / self.p['drive_speed'] + 3.0
            self._status('lined up: driving straight %.2f m' % self.drive_left)
            return
        tight = min(clearances(pts, self.body).values()) < 0.10
        found = None
        for margin in (self.p['margin'], self.p['margin_tight']):    # the tight margin only if nothing else fits
            found = plan_moves(pts, route, self.body, margin, self.pivots, step, self.p['goal_free'], tol,
                               self.p['max_moves'], backward_arcs=self.p['backward_arcs'])
            if found:
                self.turn_margin = margin / 2         # kept while turning (live check)
                break
        if found is None and len(pts):                # already at a wall: the first move must turn away from it
            near = pts[int(np.argmin(np.hypot(pts[:, 0], pts[:, 1])))]
            away = -1 if near[1] > 0 else 1           # wall on the left -> turn right, and the other way round
            found = plan_moves(pts, route, self.body, self.p['margin_away'], self.pivots, step, self.p['goal_free'],
                               tol, self.p['max_moves'], turn_sign=away, backward_arcs=self.p['backward_arcs'])
            if found:
                self.turn_margin = self.p['margin_away']
                self.get_logger().info('touching on the %s: turning away from it first' % ('left' if away < 0 else 'right'))
        if found is None:
            return self._finish('no safe way out found (tried up to %d moves: turns on the spot or round either '
                                'wheel, 10 cm ahead); Nav2 or a person must help' % self.p['max_moves'])
        seq, free = found
        self.steps += 1
        if self.steps > 12:
            return self._finish('could not get out after 12 moves')
        self.get_logger().info('route %+.0f deg; way out: %s (then %.2f m straight)'
                               % (math.degrees(route), ', '.join(describe(m) for m in seq), free))
        m = seq[0]
        if m[0] == 'ahead':
            self.drive_left, self.start, self.then_plan = m[1], self.odom, True
            self.state, self.until = DRIVING, now + m[1] / self.p['drive_speed'] + 3.0
            return
        angle = m[1]
        if tight:              # do at most 15 deg, then look again: on marble the robot turns up to 40 % more than
            angle = math.copysign(min(abs(angle), math.radians(15)), angle)   # the wheels say (45 -> 60 deg)
        self.target, self.turned, self.last_yaw, self.pivot = angle, 0.0, self.odom[2], m[2]
        self.state, self.until = TURNING, now + abs(angle) / self.p['turn_speed'] + 2.5

    def _turn(self, now):
        d = self.odom[2] - self.last_yaw
        self.turned += math.atan2(math.sin(d), math.cos(d))
        self.last_yaw = self.odom[2]
        shape = grow(self.body, self.turn_margin)    # live check while turning: the margin the plan was made with
                                                      # (the -6 mm "touching" allowance only when turning away;
                                                      # with it for every turn the back touched a wall, 2026-10-26)
        pts = self._points()
        nxt = pivot_pose(math.copysign(math.radians(4), self.target), self.pivot)   # the pivot moves with the robot
        if pts is not None and body_hits(pts, shape, nxt):
            return self._finish('stopped turning: something is close to the body')
        if abs(self.turned) >= abs(self.target) - math.radians(3) or now > self.until:
            self._stop()
            self.state, self.until = SETTLE, now + 0.4      # let it come to rest, then choose again (fresh scan)
            return
        c = Twist()
        c.angular.z = math.copysign(self.p['turn_speed'], self.target)
        c.linear.x = c.angular.z * self.pivot      # round a wheel: that wheel stands still, the other one drives
        self.cmd_pub.publish(c)

    def _drive(self, now):
        done = math.hypot(self.odom[0] - self.start[0], self.odom[1] - self.start[1])
        pts = self._points()
        if pts is not None and straight_free(pts, grow(self.body, self.p['margin'] / 2), (0.0, 0.0, 0.0), 0.2) < 0.06:
            if done < 0.03 and not self.then_plan:   # could not even start: look for a sequence of moves instead
                self._stop()
                self.state, self.until, self.no_straight = SETTLE, now + 0.4, True
                return
            return self._finish('stopped: something ahead (drove %.2f m)' % done)
        if done >= self.drive_left or now > self.until:
            if self.then_plan:                         # a 10 cm move of a sequence: settle, then plan again
                self._stop()
                self.state, self.until = SETTLE, now + 0.4
                return
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
