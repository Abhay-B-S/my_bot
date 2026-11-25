#!/usr/bin/env python3
# Usage (from ~/dev_ws, ROS sourced): python3 src/my_bot/tools/test_battery_prediction.py src/my_bot/scripts/delivery_manager.py src/my_bot/config/locations_<map>.yaml
"""Scenario test for the battery prediction in delivery_manager (dry run: fake 3 s drives, fake battery readings)."""
import json
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String

MGR = sys.argv[1]
LOC = sys.argv[2]
latched = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)


class T(Node):
    def __init__(self):
        super().__init__('battery_test')
        self.bat = self.create_publisher(String, '/battery_status', latched)
        self.req = self.create_publisher(String, '/delivery_request', 10)
        self.shipped = self.create_publisher(Empty, '/delivery_shipped', 10)
        self.received = self.create_publisher(Empty, '/delivery_received', 10)
        self.returned = self.create_publisher(Empty, '/delivery_returned', 10)
        self.log = []
        self.create_subscription(String, '/delivery_status', self.on_status, latched)
        self.pct = 80.0
        self.create_timer(0.5, self.pub_bat)

    def pub_bat(self):
        self.bat.publish(String(data=json.dumps({'source': 'ina219', 'pct': self.pct, 'usable_wh': 24.5, 'wh': 0.0,
                                                 'state': 'ok', 'v': 11.5, 'a': 0.3})))

    def on_status(self, m):
        d = json.loads(m.data)
        self.log.append((time.monotonic(), d['state'], d['detail'], d.get('result'), d.get('rejected_ref')))
        print('   [%s] %s%s%s' % (d['state'], d['detail'][:150], ' | result=%s' % d['result'] if d.get('result') else '',
                                 ' | rejected=%s' % d['rejected_ref'] if d.get('rejected_ref') else ''), flush=True)


def spin_until(node, cond, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cond():
            return True
    return False


def last_state(n):
    return n.log[-1][1] if n.log else None


def main():
    rclpy.init()
    n = T()
    mgr = subprocess.Popen(['python3', MGR, '--ros-args', '-p', 'dry_run:=true', '-p', 'battery_aware:=true',
                            '-p', 'locations_file:=' + LOC, '-p', 'energy_check_period:=0.3',
                            '-p', 'energy_shortfall_s:=0.8', '-p', 'energy_file:=/tmp/energy_test.json'],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    ok = {}
    try:
        spin_until(n, lambda: any('Ready' in l[2] for l in n.log), 15)

        print('\n1) Normal job at 80 %: must complete untouched')
        n.pct = 80.0
        n.req.publish(String(data='cabin_A,cabin_B,ref1'))
        spin_until(n, lambda: last_state(n) == 'WAIT_SHIPPED', 15); n.shipped.publish(Empty())
        spin_until(n, lambda: last_state(n) == 'WAIT_RECEIVED', 15); n.received.publish(Empty())
        ok['1'] = spin_until(n, lambda: any(l[3] == 'delivered' for l in n.log), 10) and \
            not any('Sorry' in l[2] for l in n.log)
        spin_until(n, lambda: last_state(n) == 'IDLE', 15)

        print('\n2) Request while the battery is almost empty (2 %): must be declined with an apology')
        n.pct = 2.0
        spin_until(n, lambda: False, 1.5)
        n.req.publish(String(data='cabin_A,cabin_E,ref2'))
        ok['2'] = spin_until(n, lambda: any(l[4] == 'ref2' and 'Sorry' in l[2] for l in n.log), 10)

        print('\n3) Battery fails while going to the sender: job stopped before pick-up, robot goes home')
        n.pct = 80.0
        spin_until(n, lambda: False, 1.5)
        n.req.publish(String(data='cabin_B,cabin_E,ref3'))
        spin_until(n, lambda: last_state(n) == 'TO_PICKUP', 10)
        n.pct = 1.0
        ok['3'] = spin_until(n, lambda: any(l[3] == 'failed' and 'before pick-up' in l[2] for l in n.log), 10)
        spin_until(n, lambda: last_state(n) == 'IDLE', 15)

        print('\n4) Parcel loaded but not enough for the trip: parcel handed back at the sender')
        n.pct = 80.0
        spin_until(n, lambda: False, 1.5)
        n.req.publish(String(data='cabin_A,cabin_D,ref4'))
        spin_until(n, lambda: last_state(n) == 'WAIT_SHIPPED', 15)
        n.pct = 1.5
        spin_until(n, lambda: False, 1.0)
        n.shipped.publish(Empty())
        ok['4a'] = spin_until(n, lambda: last_state(n) == 'WAIT_RETURNED' and 'take the parcel back' in n.log[-1][2], 10)
        n.returned.publish(Empty())
        ok['4b'] = spin_until(n, lambda: any(l[3] == 'returned' and 'battery was too low' in l[2] for l in n.log), 10)
        spin_until(n, lambda: last_state(n) == 'IDLE', 15)

        print('\n5) Battery drops on the way to the receiver: cheapest safe option is chosen')
        n.pct = 80.0
        spin_until(n, lambda: False, 1.5)
        n.req.publish(String(data='cabin_E,cabin_A,ref5'))
        spin_until(n, lambda: last_state(n) == 'WAIT_SHIPPED', 15); n.shipped.publish(Empty())
        spin_until(n, lambda: last_state(n) == 'TO_DROPOFF', 10)
        n.pct = 3.0
        ok['5'] = spin_until(n, lambda: any('Bringing the parcel back' in l[2] for l in n.log[-6:]), 10)
        spin_until(n, lambda: last_state(n) == 'WAIT_RETURNED', 10); n.returned.publish(Empty())
        ok['5b'] = spin_until(n, lambda: any(l[3] == 'returned' and 'battery was too low' in l[2] for l in n.log), 10)
    finally:
        mgr.terminate()
        mgr.wait()
        rclpy.shutdown()
    print('\nRESULTS:', ok, 'ALL PASS' if all(ok.values()) else 'SOME FAILED')


if __name__ == '__main__':
    main()
