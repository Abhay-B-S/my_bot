#!/usr/bin/env python3
"""Bridge between the Firebase Realtime Database (the website) and the delivery manager.

Only Python's standard library is used: Firebase is read and written with plain web requests
(its REST interface), so nothing has to be installed and no key file is needed.

Database layout (see web/index.html for the writer side):
  /requests/<id>  {sender, destination, senderName, receiverName, status, detail, shipped, received, created}
  /robot          {state, detail, queue, ref, heartbeat}      written by this bridge

Flow:
  website writes a request with status 'pending'   -> bridge publishes "sender,destination,<id>" on /delivery_request
  manager status (/delivery_status)                -> bridge updates /robot and the request's status
  website sets shipped/received = true             -> bridge publishes /delivery_shipped, /delivery_received
                                                      (only while the manager is waiting for that very request)

Run:  python3 src/my_bot/scripts/firebase_bridge.py [--ros-args -p database_url:=https://...]
"""
import json
import queue
import re
import threading
import time
import urllib.request

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

DEFAULT_DB = 'https://delivery-robot-demo-default-rtdb.asia-southeast1.firebasedatabase.app'
NAME_RE = re.compile(r'^[A-Za-z0-9_]+$')          # location names we accept from the website

# manager state -> status text stored on the request
STATE_TO_STATUS = {'TO_PICKUP': 'going_to_sender', 'WAIT_SHIPPED': 'wait_shipped',
                   'TO_DROPOFF': 'going_to_destination', 'WAIT_RECEIVED': 'wait_received'}
UNFINISHED = ('queued', 'going_to_sender', 'wait_shipped', 'going_to_destination', 'wait_received')
FINAL = ('delivered', 'failed', 'rejected')


class FirebaseBridge(Node):
    def __init__(self):
        super().__init__('firebase_bridge')
        self.declare_parameter('database_url', DEFAULT_DB)
        self.declare_parameter('poll_period', 1.5)         # seconds between reads of /requests
        self.declare_parameter('heartbeat_period', 5.0)    # seconds between "I am alive" writes
        self.db = self.get_parameter('database_url').value.rstrip('/')
        self.poll_period = self.get_parameter('poll_period').value
        self.heartbeat_period = self.get_parameter('heartbeat_period').value

        self.request_pub = self.create_publisher(String, '/delivery_request', 10)
        self.shipped_pub = self.create_publisher(Empty, '/delivery_shipped', 10)
        self.received_pub = self.create_publisher(Empty, '/delivery_received', 10)
        self.create_subscription(String, '/delivery_status', self._on_status, 10)

        self.lock = threading.Lock()
        self.last_status = None          # last manager status (dict)
        self.forwarded = set()           # request ids already sent to the manager
        self.sent_shipped = set()
        self.sent_received = set()
        self.written = {}                # request id -> last status we wrote
        self.out = queue.Queue()         # pending writes to Firebase
        self.net_ok = None
        self.first_poll = True
        self.running = True
        threading.Thread(target=self._poll_loop, daemon=True).start()
        threading.Thread(target=self._writer_loop, daemon=True).start()
        self.get_logger().info('Bridge started, database: %s' % self.db)

    # ---------- Firebase REST ----------
    def _http(self, method, path, body=None, timeout=8):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request('%s/%s.json' % (self.db, path), data=data, method=method,
                                     headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read()
        return json.loads(raw) if raw else None

    def _queue_patch(self, rid, status, detail):
        """Write a request's status; remember it only after the write succeeded."""
        body = {'status': status, 'detail': detail}
        self.out.put(('PATCH', 'requests/%s' % rid, body, lambda: self.written.__setitem__(rid, status)))

    # ---------- incoming from the website ----------
    def _poll_loop(self):
        while self.running:
            try:
                reqs = self._http('GET', 'requests') or {}
                if self.net_ok is False:
                    self.get_logger().info('Connection to Firebase restored')
                self.net_ok = True
                self._process(reqs)
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
            if status == 'pending' and rid not in self.forwarded:
                self._forward(rid, r)
            if r.get('shipped') is True and rid not in self.sent_shipped:
                self._press('WAIT_SHIPPED', rid, self.shipped_pub, self.sent_shipped)
            if r.get('received') is True and rid not in self.sent_received:
                self._press('WAIT_RECEIVED', rid, self.received_pub, self.sent_received)

    def _forward(self, rid, r):
        sender, dest = str(r.get('sender', '')), str(r.get('destination', ''))
        self.forwarded.add(rid)
        if not (NAME_RE.match(sender) and NAME_RE.match(dest)):
            self._queue_patch(rid, 'rejected', 'Invalid request')
            return
        self.request_pub.publish(String(data='%s,%s,%s' % (sender, dest, rid)))
        self.get_logger().info('Request %s forwarded: %s -> %s' % (rid, sender, dest))
        self._queue_patch(rid, 'queued', 'In the queue')

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
        ref, result = st.get('ref'), st.get('result')
        if ref:
            if result:
                status = 'delivered' if result == 'delivered' else 'failed'
            else:
                status = STATE_TO_STATUS.get(st.get('state'))
            if status and self.written.get(ref) != status and self.written.get(ref) not in FINAL:
                self._queue_patch(ref, status, st.get('detail', ''))
        self.out.put(('PUT', 'robot', self._robot_body(), None))

    def _robot_body(self):
        with self.lock:
            st = self.last_status or {}
        return {'state': st.get('state', 'UNKNOWN'), 'detail': st.get('detail', ''),
                'queue': st.get('queue', 0), 'ref': st.get('ref'),
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
