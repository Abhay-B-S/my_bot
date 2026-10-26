#!/usr/bin/env python3
"""Autonomous exploration: the robot maps an unknown place by itself, drives back to its start and saves the map.

Why: the reviewer asked for mapping without a person driving the robot round. Ready-made explorers
(explore_lite, frontier_exploration_ros2) are not available for Foxy, so this is a small one of our own, built on
slam_toolbox (live map) and Nav2 (driving).

How (frontier exploration, choosing the next target like the MRTSP idea: near, large and little turning):
1. Frontiers = free map cells next to unknown cells, grouped into clusters; clusters shorter than `min_frontier`
   m are noise.
2. A grid search from the robot over known free cells that keep `clearance` m from walls gives, for each cluster,
   the nearest reachable standing point and the real driving distance to it (not the straight line).
3. Score = `gain_weight` x cluster length - driving distance - `turn_weight` x turn needed. The best one becomes
   the Nav2 goal, facing the unknown side.
4. While driving: if the target cluster has become known (the lidar saw it on the way), switch to the next target at
   once. Targets Nav2 cannot reach are blacklisted (`blacklist_radius`).
5. No frontier left (or `max_minutes`): drive back to the start, save the map (`map_name`, .pgm/.yaml and the
   slam_toolbox pose graph) and report the explored area and time on /explore_status.

  ros2 launch slam_toolbox online_async_launch.py params_file:=.../mapper_params_mapping.yaml   (mapping)
  ros2 launch my_bot navigation_launch.py                                                         (Nav2, no AMCL)
  python3 src/my_bot/scripts/explorer.py --ros-args -p map_name:=$HOME/dev_ws/maps/explored
Topics: in /map (slam_toolbox), TF map->base_link; out NavigateToPose goals, /explore_status (String),
        /explore_frontiers (Marker, for RViz).
"""
import heapq
import math
import time
from collections import deque

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from slam_toolbox.srv import SerializePoseGraph
from std_msgs.msg import ColorRGBA, String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

FREE, UNKNOWN = 0, -1
NB8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def wall_distance(occ, res, limit):
    """Distance (m, capped at `limit`) of every cell to the nearest occupied cell (8-neighbour wavefront)."""
    h, w = occ.shape
    cap = int(limit / res) + 1
    dist = np.full((h, w), cap, dtype=np.int32)
    q = deque()
    for r, c in zip(*np.where(occ)):
        dist[r, c] = 0
        q.append((r, c))
    while q:
        r, c = q.popleft()
        d = dist[r, c] + 1
        if d >= cap:
            continue
        for dr, dc in NB8:
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and dist[rr, cc] > d:
                dist[rr, cc] = d
                q.append((rr, cc))
    return dist * res


def frontier_clusters(grid, min_cells, piece_cells, ignore=None):
    """Clusters (lists of (row, col)) of free cells that touch unknown cells. Long clusters (the whole edge of
    the first scan is one ring) are cut into square pieces of `piece_cells`, so each target is one opening.
    `ignore` = boolean mask of cells that do not count (the blind ring round the robot itself)."""
    free = grid == FREE
    unknown = grid == UNKNOWN
    touch = np.zeros_like(free)
    touch[1:, :] |= unknown[:-1, :]
    touch[:-1, :] |= unknown[1:, :]
    touch[:, 1:] |= unknown[:, :-1]
    touch[:, :-1] |= unknown[:, 1:]
    front = free & touch
    if ignore is not None:
        front &= ~ignore
    seen = np.zeros_like(front)
    clusters = []
    for r0, c0 in zip(*np.where(front)):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        q, cells = deque([(r0, c0)]), []
        while q:
            r, c = q.popleft()
            cells.append((r, c))
            for dr, dc in NB8:
                rr, cc = r + dr, c + dc
                if 0 <= rr < front.shape[0] and 0 <= cc < front.shape[1] and front[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    q.append((rr, cc))
        pieces = {}
        for r, c in cells:
            pieces.setdefault((r // piece_cells, c // piece_cells), []).append((r, c))
        clusters += [p for p in pieces.values() if len(p) >= min_cells]
    return clusters


def drive_distances(ok, start, max_cells):
    """Driving distance in cells from `start` over cells where `ok` is True (Dijkstra, 8 neighbours)."""
    h, w = ok.shape
    dist = np.full((h, w), np.inf)
    dist[start] = 0.0
    pq = [(0.0, start)]
    while pq:
        d, (r, c) = heapq.heappop(pq)
        if d > dist[r, c] or d > max_cells:
            continue
        for dr, dc in NB8:
            rr, cc = r + dr, c + dc
            if 0 <= rr < h and 0 <= cc < w and ok[rr, cc]:
                nd = d + (1.4142 if dr and dc else 1.0)
                if nd < dist[rr, cc]:
                    dist[rr, cc] = nd
                    heapq.heappush(pq, (nd, (rr, cc)))
    return dist


def write_map(m, name):
    """Save an OccupancyGrid as name.pgm + name.yaml (the map_server format), without slam_toolbox's map_saver
    (it timed out after 2 s on a busy laptop, 2026-10-26)."""
    g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    img = np.full(g.shape, 205, dtype=np.uint8)       # unknown
    img[(g >= 0) & (g < 25)] = 254                     # free
    img[g >= 65] = 0                                   # occupied
    img = img[::-1]                                    # image rows go top-down, map rows bottom-up
    with open(name + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())
    o = m.info.origin.position
    with open(name + '.yaml', 'w') as f:
        f.write('image: %s.pgm\nmode: trinary\nresolution: %.3f\norigin: [%.3f, %.3f, 0]\nnegate: 0\n'
                'occupied_thresh: 0.65\nfree_thresh: 0.25\n' % (name.split('/')[-1], m.info.resolution, o.x, o.y))


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        defaults = {
            'clearance': 0.35,         # m from walls for standing points (the back swings 0.29 m when turning; 0.30 put
                                       # targets next to furniture where Nav2 could not turn, real test 2026-10-26)
            'min_frontier': 0.40,      # m: shorter frontier clusters are ignored
            'piece': 1.5,              # m: long frontiers are cut into pieces this size (one target each)
            'blind_radius': 0.6,       # m round the robot: unknown cells there are its own blind spot, not a frontier
            'min_goal_distance': 0.5,  # m: a target must be at least this far from the robot
            'gain_weight': 1.5,        # score per m of frontier
            'turn_weight': 0.4,        # score per rad of turning towards the target
            'blacklist_radius': 0.5,   # m round a target Nav2 could not reach
            'replan_seconds': 3.0,     # how often the current target is checked while driving
            'goal_timeout': 90.0,      # s for one target
            'max_minutes': 20.0,       # stop exploring after this long
            'tries': 2,                # attempts per target before it is blacklisted
            'second_pass': True,       # when only blacklisted frontiers are left, forget the list once and retry
            'start_delay': 5.0,        # s for slam_toolbox to publish a first map
            'map_name': '',            # save the map here when done ('' = do not save)
            'return_home': True,
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        # a motor fault restarts the wheel odometry at zero, and the map is then glued together wrongly (three
        # rotated pieces, real test 2026-10-26): stop at once and keep the map made so far
        self.create_subscription(String, '/safety_event', self._on_safety, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/explore_status', latched)
        self.marker_pub = self.create_publisher(Marker, '/explore_frontiers', 10)
        self.save_graph = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

        self.map = None
        self.start_pose = None
        self.blacklist = []           # (x, y) in the map frame
        self.failures = {}            # rounded target -> failed attempts
        self.passes = 1
        self.goal_handle = None
        self.goal_sent_at = 0.0
        self.target = None            # (x, y, cluster cells as map points) being driven to
        self.state = 'starting'
        self.t0 = time.monotonic()
        self.targets_done = 0
        self.distance = 0.0
        self.last_xy = None
        self.create_timer(1.0, self._tick)
        self._status('starting: waiting for the map and Nav2')

    # ---------- helpers ----------
    def _status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(text)

    def _on_map(self, m):
        self.map = m

    def _on_safety(self, msg):
        if msg.data.startswith('SAFETY STOP') and self.state in ('choose', 'driving'):
            self.p['return_home'] = False          # the odometry is about to jump: no trip home on a broken map
            self._finish('motor fault (%s): map saved up to here; it may be broken after this point, please map '
                         'again' % msg.data[13:80])

    def _robot(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
        except Exception:
            return None
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, 2 * math.atan2(q.z, q.w)

    def _grid(self):
        m = self.map
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        g[(g > 0) & (g < 65)] = FREE                 # slam_toolbox: 0 free, 100 occupied, -1 unknown
        return g, m.info.resolution, m.info.origin.position.x, m.info.origin.position.y

    def _send(self, x, y, yaw):
        g = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        g.pose = p
        self.goal_handle, self.goal_result = None, None
        self.goal_sent_at = time.monotonic()
        fut = self.nav.send_goal_async(g)
        fut.add_done_callback(self._on_accept)

    def _on_accept(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.goal_result = GoalStatus.STATUS_ABORTED
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(lambda f: setattr(self, 'goal_result', f.result().status))

    def _cancel(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.goal_handle, self.goal_result = None, None

    # ---------- choosing a target ----------
    def _choose(self, robot):
        g, res, ox, oy = self._grid()
        h, w = g.shape
        occupied = g >= 65
        clear = wall_distance(occupied, res, self.p['clearance'] + res) >= self.p['clearance']
        ok = (g == FREE) & clear
        rc = (int((robot[1] - oy) / res), int((robot[0] - ox) / res))
        if not (0 <= rc[0] < h and 0 <= rc[1] < w):
            return None
        if not ok[rc]:                                # robot stands close to a wall: start from the nearest ok cell
            rr, cc = np.where(ok)
            if len(rr) == 0:
                return None
            i = int(np.argmin((rr - rc[0]) ** 2 + (cc - rc[1]) ** 2))
            if math.hypot(rr[i] - rc[0], cc[i] - rc[1]) * res > 0.6:
                return None
            rc = (int(rr[i]), int(cc[i]))
        dist = drive_distances(ok, rc, 4000)
        rows, cols = np.indices(g.shape)
        rob_r, rob_c = (robot[1] - oy) / res, (robot[0] - ox) / res
        blind = np.hypot(rows - rob_r, cols - rob_c) * res < self.p['blind_radius']
        clusters = frontier_clusters(g, max(2, int(self.p['min_frontier'] / res)), max(4, int(self.p['piece'] / res)),
                                     ignore=blind)
        too_near = np.hypot(rows - rob_r, cols - rob_c) * res < self.p['min_goal_distance']
        dist = np.where(too_near, np.inf, dist)      # standing points right where the robot is do not count
        best, shown = None, []
        for cells in clusters:
            cells = np.array(cells)
            cx, cy = ox + (cells[:, 1].mean() + 0.5) * res, oy + (cells[:, 0].mean() + 0.5) * res
            if any(math.hypot(cx - bx, cy - by) < self.p['blacklist_radius'] for bx, by in self.blacklist):
                continue                     # (the blacklist holds both the frontier centre and the standing point)
            # nearest reachable standing point within 1.2 m of the cluster
            r0, c0 = cells[:, 0].min(), cells[:, 1].min()
            r1, c1 = cells[:, 0].max(), cells[:, 1].max()
            pad = int(1.2 / res)
            sub = dist[max(0, r0 - pad):r1 + pad + 1, max(0, c0 - pad):c1 + pad + 1]
            if not np.isfinite(sub).any():
                continue
            rr, cc = np.where(np.isfinite(sub))
            rr, cc = rr + max(0, r0 - pad), cc + max(0, c0 - pad)
            near = np.min((rr[:, None] - cells[None, :, 0]) ** 2 + (cc[:, None] - cells[None, :, 1]) ** 2, axis=1)
            near = np.sqrt(near) * res
            close = near < 1.2
            if not close.any():
                continue
            # among standing points close to the frontier, the one with the shortest drive
            cand = np.where(close)[0]
            j = cand[int(np.argmin(dist[rr[cand], cc[cand]] + near[cand] / res))]
            gx, gy = ox + (cc[j] + 0.5) * res, oy + (rr[j] + 0.5) * res
            drive = dist[rr[j], cc[j]] * res
            # arrive in the driving direction: the lidar sees all round, and a final turn on the spot next to
            # furniture is what the robot (long back) and Nav2 Foxy fail at (7 of 10 targets, real test 2026-10-26)
            heading = math.atan2(gy - robot[1], gx - robot[0]) if drive > 0.2 else robot[2]
            yaw = heading
            turn = abs(math.atan2(math.sin(heading - robot[2]), math.cos(heading - robot[2])))
            length = len(cells) * res
            score = self.p['gain_weight'] * length - drive - self.p['turn_weight'] * turn
            shown.append((cx, cy))
            if any(math.hypot(gx - bx, gy - by) < self.p['blacklist_radius'] for bx, by in self.blacklist):
                continue
            if best is None or score > best[0]:
                best = (score, gx, gy, yaw, cells, length, drive, (cx, cy))
        self._show(shown, best)
        return best

    def _show(self, pts, best):
        m = Marker()
        m.header.frame_id = 'map'
        m.ns, m.id, m.type, m.action = 'frontiers', 0, Marker.SPHERE_LIST, Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.pose.orientation.w = 1.0
        for x, y in pts:
            m.points.append(Point(x=x, y=y, z=0.1))
            good = best is not None and math.hypot(x - best[1], y - best[2]) < 1.3
            m.colors.append(ColorRGBA(r=0.1 if good else 0.9, g=0.8 if good else 0.4, b=0.2, a=1.0))
        self.marker_pub.publish(m)

    def _target_still_open(self):
        """True while some of the target's frontier cells are still next to unknown space."""
        g, res, ox, oy = self._grid()
        h, w = g.shape
        open_cells = 0
        for x, y in self.target[2]:
            r, c = int((y - oy) / res), int((x - ox) / res)
            if 1 <= r < h - 1 and 1 <= c < w - 1 and (g[r - 1:r + 2, c - 1:c + 2] == UNKNOWN).any():
                open_cells += 1
        return open_cells * res >= self.p['min_frontier'] / 2

    # ---------- main loop ----------
    def _tick(self):
        now = time.monotonic()
        robot = self._robot()
        if robot is not None:
            if self.last_xy is not None:
                self.distance += math.hypot(robot[0] - self.last_xy[0], robot[1] - self.last_xy[1])
            self.last_xy = robot[:2]
        if self.state == 'starting':
            if self.map is None or robot is None or not self.nav.server_is_ready() \
                    or now - self.t0 < self.p['start_delay']:
                return
            self.start_pose, self.t0 = robot, now
            self.state = 'choose'
            self._status('exploring: started at (%.2f, %.2f)' % robot[:2])
        if self.state == 'choose':
            if robot is None:
                return
            if (now - self.t0) / 60 > self.p['max_minutes']:
                return self._finish('time limit reached')
            best = self._choose(robot)
            if best is None and self.p['second_pass'] and self.passes == 1 and self.blacklist:
                self.passes = 2
                self.get_logger().info('only failed targets left: trying them once more (%d)' % (len(self.blacklist) // 2))
                self.blacklist, self.failures = [], {}
                best = self._choose(robot)
            if best is None:
                return self._finish('no unexplored frontier left')
            score, gx, gy, yaw, cells, length, drive, centre = best
            g, res, ox, oy = self._grid()
            pts = [(ox + (c + 0.5) * res, oy + (r + 0.5) * res) for r, c in cells]
            self.target = (gx, gy, pts, centre)
            self._send(gx, gy, yaw)
            self.state, self.last_check = 'driving', now
            self._status('exploring: target %d at (%.2f, %.2f), frontier %.1f m long, %.1f m drive'
                         % (self.targets_done + 1, gx, gy, length, drive))
        elif self.state == 'driving':
            result = getattr(self, 'goal_result', None)
            if result is not None:
                if result == GoalStatus.STATUS_SUCCEEDED and self._target_still_open():
                    # reached, but the unknown area is still unknown (behind furniture, out of the lidar's sight):
                    # going there again only nudges the robot along the wall (targets 45-47, real test 2026-10-26)
                    self.blacklist += [self.target[:2], self.target[3]]
                    self.get_logger().info('target reached but its area cannot be seen from there: skipped')
                elif result != GoalStatus.STATUS_SUCCEEDED:
                    key = (round(self.target[0], 1), round(self.target[1], 1))
                    self.failures[key] = self.failures.get(key, 0) + 1
                    if self.failures[key] >= self.p['tries']:
                        self.blacklist += [self.target[:2], self.target[3]]
                        self.get_logger().warn('target (%.2f, %.2f) not reached (status %d): blacklisted'
                                               % (self.target[0], self.target[1], result))
                    else:
                        self.get_logger().info('target (%.2f, %.2f) not reached (status %d): will try again'
                                               % (self.target[0], self.target[1], result))
                self.targets_done += 1
                self.state = 'choose'
            elif now - self.goal_sent_at > self.p['goal_timeout']:
                self._cancel()
                self.blacklist += [self.target[:2], self.target[3]]
                self.get_logger().warn('target timed out: blacklisted')
                self.state = 'choose'
            elif now - self.last_check > self.p['replan_seconds']:
                self.last_check = now
                if not self._target_still_open():
                    self._cancel()
                    self.targets_done += 1
                    self.get_logger().info('target area already mapped on the way: choosing the next one')
                    self.state = 'choose'
        elif self.state == 'returning':
            result = getattr(self, 'goal_result', None)
            if result is not None or now - self.goal_sent_at > self.p['goal_timeout']:
                self._save_and_report()

    def _finish(self, why):
        self._cancel()
        self.finish_reason = why
        if self.p['return_home'] and self.start_pose is not None:
            self.state = 'returning'
            self._status('exploring done (%s): driving back to the start' % why)
            self._send(*self.start_pose)
        else:
            self._save_and_report()

    def _save_and_report(self):
        self.state = 'done'
        g, res, _, _ = self._grid()
        known = float(((g == FREE) | (g >= 65)).sum()) * res * res
        free = float((g == FREE).sum()) * res * res
        name = self.p['map_name']
        saved = ''
        if name:
            try:
                write_map(self.map, name)
                saved = ', map saved as %s.yaml' % name
            except OSError as exc:
                saved = ', MAP NOT SAVED (%s)' % exc
            req2 = SerializePoseGraph.Request()
            req2.filename = name + '_serial'
            if self.save_graph.wait_for_service(timeout_sec=5.0):
                self.save_graph.call_async(req2)
        self._status('done: %s; %.1f min, %d targets, %.1f m driven, %.1f m2 mapped (%.1f m2 free)%s'
                     % (self.finish_reason, (time.monotonic() - self.t0) / 60, self.targets_done, self.distance,
                        known, free, saved))


def main():
    rclpy.init()
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._cancel()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
