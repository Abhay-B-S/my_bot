#!/usr/bin/env python3
"""Bridge between the Firebase Realtime Database (the website) and the delivery manager.

Firebase is read and written with plain web requests (its REST interface, Python's standard library),
so no Firebase package has to be installed and no key file is needed.

Database layout (see web/index.html for the writer side):
  /requests/<id>  {sender, destination, senderName, receiverName, status, detail, shipped, received, created,
                   lidSender, lidReceiver, returned, lidReturn, urgent, reason, cancel}
  /robot          {state, detail, queue, ref, lid, lidButtons, estop, heartbeat}      written by this bridge
  /control        {estop, by, t}   emergency stop, written by any member on the website
  /spots          {home|cabin_X: {x, y, yaw, by, t}}   spots on the current map; written by the Admin on the website
                  (seeded from locations_file when the map changes; /spotsMap = the map they belong to)
  /spotStatus     {name: {ok, msg, t}}   the robot's check of each spot (free floor, distance to walls, reachable)
  /eta            {t, model, jobs: {<request id>: {pickup: [s, lo, hi], dropoff: [s, lo, hi], ahead, phase}}}
                  from eta_estimator.py, at most every 3 s (the website counts down from t)
  /battery        {source, v, a, w, pct, wh, state, r_ohm, over_range, t} from battery_monitor.py, every 10 s or
                  when the % or state changes (at most every 3 s); source 'sim' = simulated, no sensor fitted yet
  /wifiMap        {map, cell, t, cells: {i_j: [mean_dbm, worst_dbm, readings, rtt_ms, lost_pct]}} Wi-Fi coverage
                  from wifi_mapper.py, at most every 15 s
  /mapChanges     {map, t, changes: [{kind new|gone, x, y, w, h, cells, first, last}], trouble: [{x, y, n, kinds,
                  last, text}]} from change_detector.py, at most every 15 s
  /pose           {x, y, yaw, t}   robot position on the map (map -> base_link), about once a second while moving
  /map            {png, resolution, originX, originY, width, height, spots, name}   written once at start-up
                  when -p map_yaml:=<map .yaml> is given (the website draws the map, the spots and the robot)

Flow:
  website writes a request with status 'pending'   -> bridge publishes "sender,destination,<id>" on /delivery_request
  manager status (/delivery_status)                -> bridge updates /robot and the request's status
  website sets shipped/received = true             -> bridge publishes /delivery_shipped, /delivery_received
                                                      (only while the manager is waiting for that very request)
  website sets lidSender/lidReceiver = true        -> bridge publishes /delivery_open_lid (same check)
  undeliverable parcel back at the sender: website sets lidReturn / returned = true
                                                   -> /delivery_open_lid, /delivery_returned (same check)
  the request's detail follows the manager's detail while the job runs (e.g. "the route is blocked ...")
  website sets urgent = true (with a reason)         -> the request is forwarded as "sender,destination,<id>,urgent"
  the sender sets cancel = true                      -> /delivery_cancel <id> (not yet forwarded: cancelled here)
  /control/estop true/false                          -> /delivery_estop "stop"/"resume" (repeated until the manager's
                                                        status shows the same; also after a restart of either)
  the Admin changes /spots                           -> each spot is checked on the map; accepted ones are written to
                                                        locations_file and /delivery_reload_spots tells the manager

Login: the bridge signs in as robot@example.com; its password is read from ~/.delivery_robot_password
(a private file outside the repo). Fallback to the old open database: --ros-args -p use_auth:=false

Run:  python3 src/my_bot/scripts/firebase_bridge.py [--ros-args -p database_url:=https://...]
Live map on the website (same map and spots as Nav2 and the delivery manager):
      python3 src/my_bot/scripts/firebase_bridge.py --ros-args -p map_yaml:=./maps/<name>.yaml \
          -p locations_file:=./src/my_bot/config/locations_real.yaml
(the map picture needs the Python imaging library PIL, which the laptop has; without it only the position is sent)
"""
import base64
import io
import shutil
import json
import math
import os
import queue
import re
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

DEFAULT_DB = 'https://delivery-robot-demo-default-rtdb.asia-southeast1.firebasedatabase.app'
API_KEY = 'AIzaSyByOIQUag5MRhQiflwGldqNYPJGWaO-QTo'      # public web key of the project (not a secret)
SIGN_IN_URL = 'https://identitytoolkit.googleapis.com/v1/accounts:signInWithPassword?key=' + API_KEY
REFRESH_URL = 'https://securetoken.googleapis.com/v1/token?key=' + API_KEY
NAME_RE = re.compile(r'^[A-Za-z0-9_]+$')          # location names we accept from the website

# manager state -> status text stored on the request
STATE_TO_STATUS = {'TO_PICKUP': 'going_to_sender', 'WAIT_SHIPPED': 'wait_shipped',
                   'TO_DROPOFF': 'going_to_destination', 'WAIT_RECEIVED': 'wait_received',
                   'RETURN_TO_SENDER': 'returning', 'WAIT_RETURNED': 'wait_returned'}
UNFINISHED = ('queued', 'going_to_sender', 'wait_shipped', 'going_to_destination', 'wait_received',
              'returning', 'wait_returned')
FINAL = ('delivered', 'returned', 'failed', 'rejected', 'cancelled')


class FirebaseBridge(Node):
    def __init__(self):
        super().__init__('firebase_bridge')
        self.declare_parameter('database_url', DEFAULT_DB)
        self.declare_parameter('poll_period', 1.5)         # seconds between reads of /requests
        self.declare_parameter('heartbeat_period', 5.0)    # seconds between "I am alive" writes
        self.declare_parameter('use_auth', True)           # False = old open database (fallback)
        self.declare_parameter('robot_email', 'robot@example.com')
        self.declare_parameter('password_file', '~/.delivery_robot_password')
        here = os.path.dirname(os.path.abspath(__file__))
        self.declare_parameter('map_yaml', '')              # map to show on the website ('' = none)
        self.declare_parameter('locations_file', os.path.join(here, '..', 'config', 'locations.yaml'))
        self.declare_parameter('pose_period', 1.0)          # seconds between position checks
        self.declare_parameter('spot_clearance', 0.25)      # m a spot must keep from walls/furniture on the map
        self.declare_parameter('robot_radius', 0.17)        # m: free corridor needed to reach a spot from home
        self.db = self.get_parameter('database_url').value.rstrip('/')
        self.poll_period = self.get_parameter('poll_period').value
        self.heartbeat_period = self.get_parameter('heartbeat_period').value
        self.use_auth = self.get_parameter('use_auth').value
        self.token_lock = threading.Lock()
        self.id_token = None
        self.refresh_token = None
        self.token_expires = 0.0
        if self.use_auth:
            self._load_credentials()

        self.request_pub = self.create_publisher(String, '/delivery_request', 10)
        self.shipped_pub = self.create_publisher(Empty, '/delivery_shipped', 10)
        self.received_pub = self.create_publisher(Empty, '/delivery_received', 10)
        self.returned_pub = self.create_publisher(Empty, '/delivery_returned', 10)
        self.open_lid_pub = self.create_publisher(Empty, '/delivery_open_lid', 10)
        self.cancel_pub = self.create_publisher(String, '/delivery_cancel', 10)
        self.estop_pub = self.create_publisher(String, '/delivery_estop', 10)
        self.reload_pub = self.create_publisher(Empty, '/delivery_reload_spots', 10)
        self.create_subscription(String, '/delivery_eta', self._on_eta, 10)
        self.last_eta, self.last_eta_sent = None, 0.0
        self.create_subscription(String, '/battery_status', self._on_battery, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.last_battery, self.last_battery_sent = None, 0.0
        self.create_subscription(String, '/wifi_map', self._on_wifi_map, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.wifi_pending, self.wifi_sent = None, 0.0
        self.create_subscription(String, '/map_changes', self._on_map_changes, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.changes_pending, self.changes_sent = None, 0.0
        self.create_timer(5.0, self._send_wifi_map)
        self.grid = None                 # map for checking spots: (free bool array, wall distance m, res, ox, oy, h)
        self.spots_applied = None        # the /spots content last applied (to notice changes)
        self.spots_seeded = False
        self.control = {}                # last /control read from the website (emergency stop)
        self.last_estop_send = 0.0
        # transient local like the manager's publisher: a bridge started after the manager still gets its last
        # status (with the default QoS the website showed "UNKNOWN" until the next status, 2026-10-26)
        self.create_subscription(String, '/delivery_status', self._on_status,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                            reliability=ReliabilityPolicy.RELIABLE))

        self.lock = threading.Lock()
        self.last_status = None          # last manager status (dict)
        self.forwarded = set()           # request ids already sent to the manager
        self.sent_shipped = set()
        self.sent_received = set()
        self.sent_lid_sender = set()
        self.sent_lid_receiver = set()
        self.sent_returned = set()
        self.sent_lid_return = set()
        self.sent_cancel = set()
        self.written = {}                # request id -> last status we wrote
        self.written_detail = {}         # request id -> last detail we wrote
        self.out = queue.Queue()         # pending writes to Firebase
        self.net_ok = None
        self.first_poll = True
        self.running = True

        # live position: robot pose on the map from TF, sent when it changed (or every 10 s as a keep-alive)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_pose = None
        self.last_pose_sent = 0.0
        self.create_timer(self.get_parameter('pose_period').value, self._send_pose)
        map_body = self._map_body()
        if map_body:
            self.out.put(('PUT', 'map', map_body, None))

        threading.Thread(target=self._poll_loop, daemon=True).start()
        threading.Thread(target=self._writer_loop, daemon=True).start()
        self.get_logger().info('Bridge started, database: %s' % self.db)

    # ---------- Firebase REST ----------
    def _load_credentials(self):
        """Read the robot's password and sign in once, so a wrong password stops us at start-up."""
        self.robot_email = self.get_parameter('robot_email').value
        path = os.path.expanduser(self.get_parameter('password_file').value)
        try:
            with open(path) as f:
                self.password = f.read().strip()
        except OSError as exc:
            raise RuntimeError('Cannot read the robot password file %s (%s). Create it first, see docs/RUNBOOK.md.'
                               % (path, exc))
        if not self.password:
            raise RuntimeError('The robot password file %s is empty.' % path)
        try:
            self._token()
        except urllib.error.HTTPError as exc:
            reason = self._error_reason(exc)
            raise RuntimeError('Firebase refused the robot login (%s). Check the email, the password in %s, and '
                               'that the user exists in Authentication.' % (reason, path))
        except Exception as exc:                  # no internet yet: the loops below keep retrying
            self.get_logger().warn('Could not sign in yet (%s); will retry.' % exc)

    @staticmethod
    def _error_reason(exc):
        try:
            return json.loads(exc.read()).get('error', {}).get('message', str(exc))
        except Exception:
            return str(exc)

    def _post(self, url, body, form=False):
        data = urllib.parse.urlencode(body).encode() if form else json.dumps(body).encode()
        ctype = 'application/x-www-form-urlencoded' if form else 'application/json'
        req = urllib.request.Request(url, data=data, headers={'Content-Type': ctype})
        with urllib.request.urlopen(req, timeout=8) as resp:
            return json.loads(resp.read())

    def _token(self):
        """A valid sign-in token; renewed a few minutes before it expires."""
        with self.token_lock:
            now = time.time()
            if self.id_token and now < self.token_expires - 300:
                return self.id_token
            if self.refresh_token:
                try:
                    r = self._post(REFRESH_URL, {'grant_type': 'refresh_token',
                                                 'refresh_token': self.refresh_token}, form=True)
                    self.id_token, self.refresh_token = r['id_token'], r['refresh_token']
                    self.token_expires = now + int(r['expires_in'])
                    return self.id_token
                except Exception:
                    self.refresh_token = None      # fall through to a fresh sign-in
            r = self._post(SIGN_IN_URL, {'email': self.robot_email, 'password': self.password,
                                         'returnSecureToken': True})
            self.id_token, self.refresh_token = r['idToken'], r['refreshToken']
            self.token_expires = now + int(r['expiresIn'])
            return self.id_token

    def _http(self, method, path, body=None, timeout=8):
        data = None if body is None else json.dumps(body).encode()
        for attempt in (0, 1):
            url = '%s/%s.json' % (self.db, path)
            if self.use_auth:
                url += '?auth=' + self._token()
            req = urllib.request.Request(url, data=data, method=method,
                                         headers={'Content-Type': 'application/json'})
            try:
                with urllib.request.urlopen(req, timeout=timeout) as resp:
                    raw = resp.read()
                return json.loads(raw) if raw else None
            except urllib.error.HTTPError as exc:
                if exc.code == 401 and self.use_auth and attempt == 0:
                    with self.token_lock:          # token rejected: sign in again and retry once
                        self.id_token, self.refresh_token = None, None
                    continue
                raise

    def _queue_patch(self, rid, status, detail):
        """Write a request's status; remember it only after the write succeeded."""
        body = {'status': status, 'detail': detail}

        def done():
            self.written[rid], self.written_detail[rid] = status, detail
        self.out.put(('PATCH', 'requests/%s' % rid, body, done))

    # ---------- incoming from the website ----------
    def _poll_loop(self):
        while self.running:
            try:
                reqs = self._http('GET', 'requests') or {}
                control = self._http('GET', 'control') or {}
                self._check_spots()
                if self.net_ok is False:
                    self.get_logger().info('Connection to Firebase restored')
                self.net_ok = True
                self._process(reqs)
                self._check_estop(control if isinstance(control, dict) else {})
            except Exception as exc:                      # network down, bad JSON, ...
                if self.net_ok is not False:
                    self.get_logger().warn('Cannot reach Firebase (%s). Retrying...' % exc)
                self.net_ok = False
            time.sleep(self.poll_period)

    def _process(self, reqs):
        if self.first_poll:
            self.first_poll = False
            for rid, r in reqs.items():
                if isinstance(r, dict) and r.get('status') in UNFINISHED:
                    self._queue_patch(rid, 'failed', 'The robot system was restarted. Please request again.')
        for rid in sorted(reqs):                          # push ids sort by time
            r = reqs[rid]
            if not isinstance(r, dict):
                continue
            status = r.get('status')
            if r.get('cancel') is True and rid not in self.sent_cancel and status not in FINAL:
                self.sent_cancel.add(rid)
                if status == 'pending' and rid not in self.forwarded:     # never reached the robot
                    self.forwarded.add(rid)
                    self._queue_patch(rid, 'cancelled', 'Cancelled by the sender.')
                else:
                    self.cancel_pub.publish(String(data=rid))
                    self.get_logger().info('Cancel forwarded for %s' % rid)
                continue
            if status == 'pending' and rid not in self.forwarded:
                self._forward(rid, r)
            # "Open lid" first, so presses reach the manager in the order people make them
            if r.get('lidSender') is True and rid not in self.sent_lid_sender:
                self._press('WAIT_SHIPPED', rid, self.open_lid_pub, self.sent_lid_sender)
            if r.get('lidReceiver') is True and rid not in self.sent_lid_receiver:
                self._press('WAIT_RECEIVED', rid, self.open_lid_pub, self.sent_lid_receiver)
            if r.get('shipped') is True and rid not in self.sent_shipped:
                self._press('WAIT_SHIPPED', rid, self.shipped_pub, self.sent_shipped)
            if r.get('received') is True and rid not in self.sent_received:
                self._press('WAIT_RECEIVED', rid, self.received_pub, self.sent_received)
            if r.get('lidReturn') is True and rid not in self.sent_lid_return:
                self._press('WAIT_RETURNED', rid, self.open_lid_pub, self.sent_lid_return)
            if r.get('returned') is True and rid not in self.sent_returned:
                self._press('WAIT_RETURNED', rid, self.returned_pub, self.sent_returned)

    def _forward(self, rid, r):
        sender, dest = str(r.get('sender', '')), str(r.get('destination', ''))
        self.forwarded.add(rid)
        if not (NAME_RE.match(sender) and NAME_RE.match(dest)):
            self._queue_patch(rid, 'rejected', 'Invalid request')
            return
        urgent = r.get('urgent') is True
        self.request_pub.publish(String(data='%s,%s,%s%s' % (sender, dest, rid, ',urgent' if urgent else '')))
        self.get_logger().info('%sRequest %s forwarded: %s -> %s' % ('URGENT ' if urgent else '', rid, sender, dest))
        self._queue_patch(rid, 'queued', 'In the queue')

    def _check_estop(self, control):
        """Pass the website's emergency stop on until the manager reports the same state."""
        self.control = control
        want = control.get('estop') is True
        with self.lock:
            st = self.last_status
        if st is None or bool(st.get('estop')) == want or time.monotonic() - self.last_estop_send < 2.0:
            return
        self.last_estop_send = time.monotonic()
        self.estop_pub.publish(String(data='stop' if want else 'resume'))
        self.get_logger().warn('Emergency stop %s (pressed by %s)' % ('ON' if want else 'released',
                                                                    str(control.get('by', '?'))[:60]))

    # ---------- spots set by the Admin ----------
    def _load_grid(self, img, res, ox, oy):
        """Free cells and each cell's distance to the nearest wall (m), for checking spots."""
        free = img > 250
        wall = img < 100
        dist = np.full(img.shape, np.inf)
        dist[wall] = 0.0
        # two-pass chamfer distance (approximate Euclidean), plenty for a 25 cm check
        h, w = img.shape
        for r in range(h):
            for c in range(w):
                if dist[r, c]:
                    best = dist[r, c]
                    if r > 0:
                        best = min(best, dist[r - 1, c] + 1, dist[r - 1, c - 1] + 1.414 if c > 0 else np.inf,
                                   dist[r - 1, c + 1] + 1.414 if c < w - 1 else np.inf)
                    if c > 0:
                        best = min(best, dist[r, c - 1] + 1)
                    dist[r, c] = best
        for r in range(h - 1, -1, -1):
            for c in range(w - 1, -1, -1):
                if dist[r, c]:
                    best = dist[r, c]
                    if r < h - 1:
                        best = min(best, dist[r + 1, c] + 1, dist[r + 1, c + 1] + 1.414 if c < w - 1 else np.inf,
                                   dist[r + 1, c - 1] + 1.414 if c > 0 else np.inf)
                    if c < w - 1:
                        best = min(best, dist[r, c + 1] + 1)
                    dist[r, c] = best
        self.grid = (free, dist * res, res, ox, oy, h)

    def _cell(self, x, y):
        free, dist, res, ox, oy, h = self.grid
        return int(h - 1 - (y - oy) / res), int((x - ox) / res)

    def _spot_problem(self, name, p, home):
        """None if the spot is fine for the robot, else a short reason (shown to the Admin)."""
        if self.grid is None:
            return None                                   # no map loaded: cannot check
        free, dist, res, ox, oy, h = self.grid
        r, c = self._cell(p['x'], p['y'])
        if not (0 <= r < free.shape[0] and 0 <= c < free.shape[1]):
            return 'outside the map'
        if not free[r, c]:
            return 'not on free floor (on a wall, furniture or an unmapped area)'
        need = self.get_parameter('spot_clearance').value
        if dist[r, c] < need:
            return 'too close to a wall or furniture (%.0f cm, needs %.0f cm)' % (dist[r, c] * 100, need * 100)
        if home is not None and name != 'home':
            if not self._reachable(home, p):
                return 'cannot be reached from home (no corridor wide enough for the robot)'
        return None

    def _reachable(self, a, b):
        """True if a corridor at least robot_radius wide connects spot a to spot b on the map."""
        free, dist, res, ox, oy, h = self.grid
        ok = free & (dist >= self.get_parameter('robot_radius').value)
        start, goal = self._cell(a['x'], a['y']), self._cell(b['x'], b['y'])
        if not (ok[start] and ok[goal]):
            return False
        seen = np.zeros_like(ok)
        seen[start] = True
        todo = [start]
        while todo:
            r, c = todo.pop()
            if (r, c) == goal:
                return True
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                rr, cc = r + dr, c + dc
                if 0 <= rr < ok.shape[0] and 0 <= cc < ok.shape[1] and ok[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    todo.append((rr, cc))
        return False

    def _read_locations(self):
        path = os.path.normpath(os.path.expanduser(self.get_parameter('locations_file').value))
        with open(path) as f:
            return path, yaml.safe_load(f) or {}

    def _check_spots(self):
        """Seed /spots from the spot file once per map; afterwards apply the Admin's changes (checked) to the file."""
        map_name = os.path.basename(self.get_parameter('map_yaml').value) or 'no map'
        if not self.spots_seeded:
            current = self._http('GET', 'spotsMap')
            path, data = self._read_locations()
            locs = data.get('locations') or {}
            if current != map_name or not self._http('GET', 'spots'):
                spots = {n: {'x': round(float(p['x']), 3), 'y': round(float(p['y']), 3),
                             'yaw': round(float(p.get('yaw', 0.0)), 3), 'by': 'robot', 't': {'.sv': 'timestamp'}}
                         for n, p in locs.items() if re.match(r'^(home|cabin_[A-Z])$', n)}
                self._http('PUT', 'spots', spots)
                self._http('PUT', 'spotsMap', map_name)
                self._http('PUT', 'spotStatus', {n: {'ok': True, 'msg': 'from the spot file', 't': {'.sv': 'timestamp'}}
                                                 for n in spots})
                self.get_logger().info('Spots of %s copied to the website (%d)' % (map_name, len(spots)))
            self.spots_seeded = True
        spots = self._http('GET', 'spots') or {}
        if not isinstance(spots, dict) or spots == self.spots_applied:
            return
        path, data = self._read_locations()
        locs = dict(data.get('locations') or {})
        status, changed = {}, False
        home = spots.get('home') or locs.get('home')
        for name, p in spots.items():
            if not (re.match(r'^(home|cabin_[A-Z])$', name) and isinstance(p, dict)
                    and all(isinstance(p.get(k), (int, float)) for k in ('x', 'y', 'yaw'))):
                continue
            new = {'x': round(float(p['x']), 3), 'y': round(float(p['y']), 3), 'yaw': round(float(p['yaw']), 3)}
            old = locs.get(name)
            if old and all(abs(float(old.get(k, 0)) - new[k]) < 0.002 for k in new):
                continue                                   # unchanged
            why = self._spot_problem(name, new, None if name == 'home' else home)
            if why:
                status[name] = {'ok': False, 'msg': 'Rejected: ' + why, 't': {'.sv': 'timestamp'}}
                self.get_logger().warn('Spot %s rejected: %s' % (name, why))
                continue
            locs[name] = new
            changed = True
            status[name] = {'ok': True, 'msg': 'Accepted', 't': {'.sv': 'timestamp'}}
            self.get_logger().info('Spot %s set by %s: x=%.2f y=%.2f yaw=%.0f deg'
                                   % (name, p.get('by', '?'), new['x'], new['y'], math.degrees(new['yaw'])))
        for name in [n for n in locs if n != 'home' and n not in spots]:
            del locs[name]                                 # deleted by the Admin
            changed = True
            status[name] = None
            self.get_logger().info('Spot %s deleted' % name)
        if changed:
            if not os.path.exists(path + '.bak'):
                shutil.copy(path, path + '.bak')           # the file as it was before the first change
            data['locations'] = locs
            tmp = path + '.tmp'
            with open(tmp, 'w') as f:
                yaml.safe_dump(data, f, sort_keys=False)
            os.replace(tmp, path)
            self.reload_pub.publish(Empty())
            self.out.put(('PATCH', 'map', {'spots': {n: {'x': p['x'], 'y': p['y']} for n, p in locs.items()}}, None))
        if status:
            self._http('PATCH', 'spotStatus', status)
        self.spots_applied = spots

    def _press(self, needed_state, rid, publisher, sent_set):
        """Forward a button press only while the manager waits for exactly this request."""
        with self.lock:
            st = self.last_status
        if st and st.get('state') == needed_state and st.get('ref') == rid:
            publisher.publish(Empty())
            sent_set.add(rid)
            self.get_logger().info('Button %s forwarded for %s' % (needed_state, rid))

    # ---------- outgoing to the website ----------
    def _on_status(self, msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.last_status = st
        rej = st.get('rejected_ref')
        if rej:
            self._queue_patch(rej, 'rejected', st.get('detail', 'Rejected'))
        can = st.get('cancelled_ref')
        if can:
            self._queue_patch(can, 'cancelled', 'Cancelled by the sender.')
        req = st.get('requeued_ref')           # battery very low before pick-up: the job waits in the queue again
        if req and self.written.get(req) not in FINAL:
            self._queue_patch(req, 'queued', 'Back in the queue: the robot went home to charge')
        ref, result = st.get('ref'), st.get('result')
        if ref:
            if result:
                status = result if result in ('delivered', 'returned', 'cancelled') else 'failed'
            else:
                status = STATE_TO_STATUS.get(st.get('state'))
            detail = st.get('detail', '')
            if status and self.written.get(ref) not in FINAL and \
                    (self.written.get(ref) != status or self.written_detail.get(ref) != detail):
                self._queue_patch(ref, status, detail)
        self.out.put(('PUT', 'robot', self._robot_body(), None))

    def _on_eta(self, msg):
        """Forward the ETA estimate to the website when it changed, at most every 3 s."""
        try:
            eta = json.loads(msg.data)
        except ValueError:
            return
        now = time.monotonic()
        if eta.get('jobs') == self.last_eta and now - self.last_eta_sent < 15.0 or now - self.last_eta_sent < 3.0:
            return
        self.last_eta, self.last_eta_sent = eta.get('jobs'), now
        eta['t'] = {'.sv': 'timestamp'}
        self.out.put(('PUT', 'eta', eta, None))

    def _on_battery(self, msg):
        """Forward the battery to the website: every 10 s, or sooner (at most every 3 s) when % or state changed."""
        try:
            b = json.loads(msg.data)
        except ValueError:
            return
        now = time.monotonic()
        key = (b.get('pct'), b.get('state'), b.get('error'))
        if now - self.last_battery_sent < 3.0 or key == self.last_battery and now - self.last_battery_sent < 10.0:
            return
        self.last_battery, self.last_battery_sent = key, now
        b['t'] = {'.sv': 'timestamp'}
        self.out.put(('PUT', 'battery', b, None))

    def _on_wifi_map(self, msg):
        try:
            self.wifi_pending = json.loads(msg.data)
        except ValueError:
            pass

    def _on_map_changes(self, msg):
        try:
            self.changes_pending = json.loads(msg.data)
        except ValueError:
            pass

    def _send_wifi_map(self):
        """The newest Wi-Fi coverage map and map changes, each at most every 15 s."""
        now = time.monotonic()
        if self.wifi_pending is not None and now - self.wifi_sent >= 15.0:
            body, self.wifi_pending, self.wifi_sent = self.wifi_pending, None, now
            self.out.put(('PUT', 'wifiMap', body, None))
        if self.changes_pending is not None and now - self.changes_sent >= 15.0:
            body, self.changes_pending, self.changes_sent = self.changes_pending, None, now
            self.out.put(('PUT', 'mapChanges', body, None))

    # ---------- live map ----------
    def _map_body(self):
        """The map picture (cropped to the explored part), its scale and position, and the spots, for /map."""
        yaml_path = self.get_parameter('map_yaml').value
        if not yaml_path:
            return None
        try:
            from PIL import Image
            yaml_path = os.path.expanduser(yaml_path)
            with open(yaml_path) as f:
                meta = yaml.safe_load(f)
            img_path = meta['image']
            if not os.path.isabs(img_path):
                img_path = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_path)
            res, (ox, oy) = float(meta['resolution']), meta['origin'][:2]
            img = Image.open(img_path).convert('L')
            w, h = img.size
            px = img.load()
            self._load_grid(np.array(img), res, ox, oy)
            # map_server colours: 0 = wall, 254 = free floor, 205 = unknown. Keep the explored part plus a margin.
            known = [(x, y) for y in range(h) for x in range(w) if px[x, y] != 205]
            if not known:
                raise ValueError('the map has no explored area')
            m = 10
            x0, x1 = max(min(p[0] for p in known) - m, 0), min(max(p[0] for p in known) + m + 1, w)
            y0, y1 = max(min(p[1] for p in known) - m, 0), min(max(p[1] for p in known) + m + 1, h)
            crop = img.crop((x0, y0, x1, y1)).point(lambda v: 40 if v < 100 else 255 if v > 250 else 225)
            buf = io.BytesIO()
            crop.save(buf, format='PNG', optimize=True)
            spots = {}
            loc_path = os.path.normpath(os.path.expanduser(self.get_parameter('locations_file').value))
            with open(loc_path) as f:
                for name, p in (yaml.safe_load(f).get('locations') or {}).items():
                    spots[name] = {'x': round(float(p['x']), 3), 'y': round(float(p['y']), 3)}
            body = {'png': base64.b64encode(buf.getvalue()).decode(), 'resolution': res,
                    'originX': ox + x0 * res, 'originY': oy + (h - y1) * res,   # bottom-left corner of the crop
                    'width': x1 - x0, 'height': y1 - y0, 'spots': spots,
                    'name': os.path.basename(yaml_path), 'updated': {'.sv': 'timestamp'}}
            self.get_logger().info('Map %s (%dx%d, %d bytes) and %d spots will be shown on the website'
                                   % (body['name'], body['width'], body['height'], len(buf.getvalue()), len(spots)))
            return body
        except Exception as exc:                       # no PIL, missing file, ...: the rest still works
            self.get_logger().warn('Map not sent to the website (%s)' % exc)
            return None

    def _send_pose(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time())
        except Exception:
            return                                     # not localised yet (no map frame)
        tr, q = t.transform.translation, t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        pose = (round(tr.x, 3), round(tr.y, 3), round(yaw, 3))
        now = time.monotonic()
        last = self.last_pose
        moved = last is None or math.hypot(pose[0] - last[0], pose[1] - last[1]) > 0.02 or \
            abs(math.atan2(math.sin(pose[2] - last[2]), math.cos(pose[2] - last[2]))) > 0.05
        if moved or now - self.last_pose_sent > 10.0:
            self.last_pose, self.last_pose_sent = pose, now
            self.out.put(('PUT', 'pose', {'x': pose[0], 'y': pose[1], 'yaw': pose[2],
                                          't': {'.sv': 'timestamp'}}, None))

    def _robot_body(self):
        with self.lock:
            st = self.last_status or {}
        return {'state': st.get('state', 'UNKNOWN'), 'detail': st.get('detail', ''),
                'queue': st.get('queue', 0), 'ref': st.get('ref'),
                'lid': st.get('cargo'), 'lidButtons': bool(st.get('lid_buttons')), 'estop': bool(st.get('estop')),
                'heartbeat': {'.sv': 'timestamp'}}

    def _writer_loop(self):
        last_beat = 0.0
        while self.running:
            try:
                method, path, body, done = self.out.get(timeout=1.0)
            except queue.Empty:
                method = None
            if method:
                for attempt in range(3):
                    try:
                        self._http(method, path, body)
                        if done:
                            done()
                        break
                    except Exception as exc:
                        if attempt == 2:
                            self.get_logger().warn('Write to %s failed: %s' % (path, exc))
                        else:
                            time.sleep(1.0)
            if time.monotonic() - last_beat >= self.heartbeat_period:
                last_beat = time.monotonic()
                try:
                    self._http('PUT', 'robot', self._robot_body())    # also repairs a lost status write
                except Exception:
                    pass

    def shutdown(self):
        self.running = False
        try:
            self._http('PUT', 'robot', {'state': 'OFFLINE', 'detail': 'Robot system is off',
                                        'queue': 0, 'heartbeat': 0}, timeout=3)
        except Exception:
            pass


def main():
    rclpy.init()
    node = FirebaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
