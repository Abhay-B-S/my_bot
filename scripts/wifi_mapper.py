#!/usr/bin/env python3
"""Wi-Fi coverage map: where in the building the robot's Wi-Fi is strong, weak or laggy, built on every trip.

Runs on the laptop. Every /wifi_signal (from wifi_probe.py on the Pi) is put into a square of `cell` m on the map at
the robot's position (TF map -> base_link). Once a second it also sends /wifi_ping and times the answer on /wifi_pong:
the round-trip time (lag) of the robot's link and the share of pings with no answer within `ping_timeout` s (lost).

Per square: mean and worst signal (dBm), number of readings, mean round-trip time (ms), lost pings (%).
The squares are kept in out_dir/wifi_map_<map_name>.json, so the map grows over trips and runs (one file per map).
Publishes /wifi_map (String JSON, latched) every `publish_period` s when something changed:
  {map, cell, t, cells: {"i_j": [mean_dbm, worst_dbm, readings, rtt_ms or null, lost_pct or null]}}
  square i_j covers x in [i*cell, (i+1)*cell), y in [j*cell, (j+1)*cell) on the map
The website (via firebase_bridge.py) colours the map with it.

  python3 src/my_bot/scripts/wifi_mapper.py --ros-args -p map_name:=granny_explored6
"""
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


class WifiMapper(Node):
    def __init__(self):
        super().__init__('wifi_mapper')
        self.declare_parameter('map_name', 'default')
        self.declare_parameter('out_dir', '~/dev_ws/run_logs')
        self.declare_parameter('cell', 0.5)
        self.declare_parameter('ping_period', 1.0)
        self.declare_parameter('ping_timeout', 2.0)      # s: a ping without an answer by then counts as lost
        self.declare_parameter('publish_period', 10.0)
        self.declare_parameter('save_period', 30.0)
        self.map_name = self.get_parameter('map_name').value
        self.cell = self.get_parameter('cell').value
        self.timeout = self.get_parameter('ping_timeout').value
        out = os.path.expanduser(self.get_parameter('out_dir').value)
        os.makedirs(out, exist_ok=True)
        self.path = os.path.join(out, 'wifi_map_%s.json' % self.map_name)
        # square -> [sum dBm, readings, worst dBm, sum rtt ms, answered pings, lost pings]
        self.cells = {}
        try:
            with open(self.path) as f:
                saved = json.load(f)
            if saved.get('cell') == self.cell:
                self.cells = saved['raw']
                self.get_logger().info('Loaded %d squares from %s' % (len(self.cells), self.path))
        except (OSError, ValueError, KeyError):
            pass
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(String, '/wifi_signal', self._on_signal, 10)
        self.create_subscription(String, '/wifi_pong', self._on_pong, 10)
        self.ping_pub = self.create_publisher(String, '/wifi_ping', 10)
        self.map_pub = self.create_publisher(String, '/wifi_map', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.seq = 0
        self.pending = {}          # ping number -> (sent time, square)
        self.changed = True
        self.last_signal = 0.0
        self.create_timer(self.get_parameter('ping_period').value, self._ping)
        self.create_timer(self.get_parameter('publish_period').value, self._publish)
        self.create_timer(self.get_parameter('save_period').value, self._save)
        self.get_logger().info('Wi-Fi mapper for map %s, %.2f m squares' % (self.map_name, self.cell))

    def _square(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform.translation
        except Exception:
            return None
        return '%d_%d' % (math.floor(t.x / self.cell), math.floor(t.y / self.cell))

    def _entry(self, key):
        return self.cells.setdefault(key, [0.0, 0, 0.0, 0.0, 0, 0])

    def _on_signal(self, msg):
        try:
            dbm = float(json.loads(msg.data)['dbm'])
        except (ValueError, KeyError, TypeError):
            return
        self.last_signal = time.monotonic()
        key = self._square()
        if key is None:
            return
        e = self._entry(key)
        e[2] = dbm if e[1] == 0 else min(e[2], dbm)
        e[0] += dbm
        e[1] += 1
        self.changed = True

    def _ping(self):
        now = time.monotonic()
        for n, (sent, key) in list(self.pending.items()):       # no answer in time: lost
            if now - sent > self.timeout:
                del self.pending[n]
                if key is not None:
                    self._entry(key)[5] += 1
                    self.changed = True
        if now - self.last_signal > 5.0:      # the probe is not running (Pi off): do not count its silence as loss
            self.pending.clear()
            return
        self.seq += 1
        self.pending[self.seq] = (now, self._square())
        self.ping_pub.publish(String(data=str(self.seq)))

    def _on_pong(self, msg):
        try:
            n = int(msg.data)
        except ValueError:
            return
        if n not in self.pending:
            return
        sent, key = self.pending.pop(n)
        if key is not None:
            e = self._entry(key)
            e[3] += (time.monotonic() - sent) * 1000.0
            e[4] += 1
            self.changed = True

    def summary(self):
        out = {}
        for key, (s, n, worst, rtt, ok, lost) in self.cells.items():
            if n == 0 and ok + lost == 0:
                continue
            out[key] = [round(s / n, 1) if n else None, round(worst, 1) if n else None, n,
                        round(rtt / ok) if ok else None, round(100.0 * lost / (ok + lost)) if ok + lost else None]
        return out

    def _publish(self):
        if not self.changed:
            return
        self.changed = False
        self.map_pub.publish(String(data=json.dumps({'map': self.map_name, 'cell': self.cell,
                                                     't': round(time.time()), 'cells': self.summary()})))

    def _save(self):
        try:
            with open(self.path + '.tmp', 'w') as f:
                json.dump({'map': self.map_name, 'cell': self.cell, 'raw': self.cells}, f)
            os.replace(self.path + '.tmp', self.path)
        except OSError as exc:
            self.get_logger().warn('Could not save %s: %s' % (self.path, exc))


def main():
    rclpy.init()
    node = WifiMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
