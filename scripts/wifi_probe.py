#!/usr/bin/env python3
"""Wi-Fi probe: the robot's Wi-Fi signal strength, for the Wi-Fi coverage map (wifi_mapper.py on the laptop).

Runs ON THE PI (it travels with the robot). Reads /proc/net/wireless once a second (no sudo, no extra package) and
publishes /wifi_signal (String JSON {dbm, quality_pct, iface, t}). It also answers /wifi_ping with the same text on
/wifi_pong at once, so the laptop can measure the real round-trip time (lag) of the robot's link.

  python3 wifi_probe.py                                   (Pi; interface: the first one in /proc/net/wireless)
  python3 wifi_probe.py --ros-args -p source:=sim -p use_sim_time:=true   (Gazebo: signal made up from the position)

Simulation: one access point at (sim_ap_x, sim_ap_y); the signal falls with distance (-45 dBm at 1 m, -30 dB per
tenfold distance) and 8 dB per wall cell crossed on the map, so the coverage map shows something to look at.
"""
import json
import math
import random
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


def read_proc(iface=''):
    """(iface, dBm, link quality 0-70) of the first (or the named) wireless interface, or None."""
    try:
        with open('/proc/net/wireless') as f:
            lines = f.readlines()[2:]
    except OSError:
        return None
    for line in lines:
        name, _, rest = line.partition(':')
        name = name.strip()
        if iface and name != iface:
            continue
        cols = rest.split()
        if len(cols) >= 3:
            return name, float(cols[2].rstrip('.')), float(cols[1].rstrip('.'))
    return None


class WifiProbe(Node):
    def __init__(self):
        super().__init__('wifi_probe')
        self.declare_parameter('source', 'proc')      # proc | sim
        self.declare_parameter('iface', '')
        self.declare_parameter('period', 1.0)
        self.declare_parameter('sim_ap_x', 0.0)
        self.declare_parameter('sim_ap_y', 0.0)
        self.source = self.get_parameter('source').value
        self.iface = self.get_parameter('iface').value
        self.pub = self.create_publisher(String, '/wifi_signal', 10)
        self.pong = self.create_publisher(String, '/wifi_pong', 10)
        self.create_subscription(String, '/wifi_ping', lambda m: self.pong.publish(m), 10)
        if self.source == 'sim':
            self.tf = Buffer()
            TransformListener(self.tf, self)
            self.grid = None
            self.create_subscription(OccupancyGrid, '/map', self._on_map, QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
        self.warned = False
        self.create_timer(self.get_parameter('period').value, self._tick)
        self.get_logger().info('Wi-Fi probe: source %s' % self.source)

    def _on_map(self, msg):
        self.grid = msg

    def _sim(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform.translation
        except Exception:
            return None
        ax, ay = self.get_parameter('sim_ap_x').value, self.get_parameter('sim_ap_y').value
        d = max(1.0, math.hypot(t.x - ax, t.y - ay))
        walls = 0
        g = self.grid
        if g is not None:                                # count wall cells on the straight line to the access point
            res, ox, oy, w = g.info.resolution, g.info.origin.position.x, g.info.origin.position.y, g.info.width
            steps = int(d / res)
            inside = False
            for k in range(steps):
                x, y = ax + (t.x - ax) * k / steps, ay + (t.y - ay) * k / steps
                i, j = int((x - ox) / res), int((y - oy) / res)
                wall = 0 <= i < w and 0 <= j < g.info.height and g.data[j * w + i] > 50
                walls += wall and not inside               # one wall = one run of occupied cells
                inside = wall
        dbm = -45 - 30 * math.log10(d) - 8 * walls + random.gauss(0, 1.5)
        return 'sim', dbm, max(0.0, min(70.0, dbm + 110))

    def _tick(self):
        r = self._sim() if self.source == 'sim' else read_proc(self.iface)
        if r is None:
            if not self.warned and self.source != 'sim':
                self.get_logger().warn('No wireless interface found in /proc/net/wireless')
                self.warned = True
            return
        iface, dbm, quality = r
        self.pub.publish(String(data=json.dumps({'dbm': round(dbm, 1), 'quality_pct': round(quality / 70 * 100),
                                                 'iface': iface, 't': round(time.time(), 2)})))


def main():
    rclpy.init()
    node = WifiProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
