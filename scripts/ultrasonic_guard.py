#!/usr/bin/env python3
"""Ultrasonic guard (run ON THE PI): one HC-SR04 at the front, about 10 cm above the floor, sees low things the lidar
(22 cm up) misses: sofa edges, bags, shoes.

Wiring (2026-10-29): VCC pin 2 (5 V), GND pin 9, Trig pin 16 (GPIO 23), Echo pin 18 (GPIO 24) through a 1k/2k divider
(the Echo pin gives 5 V, the Pi takes 3.3 V). Needs python3-rpi.gpio and /dev/gpiomem readable by the dialout group.

  out /ultrasonic           (sensor_msgs/Range)   the distance, 10 per second
      /ultrasonic_points    (sensor_msgs/PointCloud2, base_link) the obstacle as points for the Nav2 costmaps; with
                            nothing in range one point beyond obstacle_range, which only clears the old marks
      /cmd_vel_ultrasonic   (Twist, twist_mux priority 47) while something is closer than stop_distance and the
                            winning drive command goes FORWARD: the same command without the forward part, so the
                            robot can still turn and back away (gap assist backs up by itself)
  in  /cmd_vel /cmd_vel_guard /cmd_vel_assist   the drive requests below this one in twist_mux
Two HW-201 IR sensors (2026-10-29) low at the front corners catch what passes under the ultrasonic beam: OUT left
pin 29 (GPIO 5), right pin 31 (GPIO 6), 3.3 V on pins 1/17, GND 20/25; output LOW = something within ~10 cm (set by
the blue screw). Either one: no driving forward, and a mark at that corner so Nav2 plans round on the other side.
"""
import math
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, PointCloud2, PointField, Range

try:
    import RPi.GPIO as GPIO
except ImportError:                     # laptop / simulation: the node starts but reads nothing
    GPIO = None

SPEED_OF_SOUND_HALF = 171.5             # m/s: the echo goes there and back


class UltrasonicGuard(Node):
    def __init__(self):
        super().__init__('ultrasonic_guard')
        defaults = {
            'trig_gpio': 23, 'echo_gpio': 24,
            'ir_left_gpio': -1, 'ir_right_gpio': -1,  # IR off for the expo (2026-10-29: unreliable); left was
                                                     # pin 31 (GPIO 6), right pin 29 (GPIO 5)
            'ir_x': 0.17, 'ir_y': 0.11,  # m: where an IR detection is marked (front corner, ~8 cm ahead of the box)
            'ir_hold': 3.0,            # s the stop stays after the IR clears: touching an object the IR flickers to
                                       # 'clear', and a short hold let Nav2 creep on and push it (real test 2026-10-29)
            'backoff_after': 2.0,      # s of forward requests while blocked before the guard backs the robot off
            'backoff_distance': 0.12,  # m reversed at least, nose turning away from the side that saw the object;
            'backoff_max': 0.35,       # then on until front_clear is free in front (or this far, or the back is close)
            'front_clear': 0.25,       # m free in front of the box: it turns round the front axle and its front
                                       # corners sweep 0.15 m from it (0.065 m past the front face) - room to turn
            'back_free': 0.10,         # m that must stay free behind the box (lidar, checked live)
            'laser_x': -0.1995,        # m: the lidar sits this far ahead of the axle (behind it); box back edge -0.245
            'body_back': 0.245, 'body_half_width': 0.16,
            'backoff_speed': 0.12, 'backoff_turn': 0.3,
            'rate': 10.0,              # readings per second
            'sensor_x': 0.085,         # m in front of the axle (front face of the box)
            'sensor_z': 0.15,          # m above the floor (photo 2026-10-29)
            'min_range': 0.03, 'max_range': 2.0,
            'stop_distance': 0.20,     # m from the sensor: no driving forward closer than this
            'mark_range': 1.0,         # m: only closer things are put into the costmaps
            'beam_half_width': 0.15,   # m each side of the beam centre marked as obstacle (30 cm wide: plan clearly round)
            'hold_seconds': 3.0,       # keep blocking this long when no echo comes back at all
            'release_margin': 0.05,    # m: a real reading this much past stop_distance frees the way at once
            # "no echo" is NOT "free": rounded or soft things (a helmet, real test 2026-10-29) often give no echo,
            # and treating that as free released the stop after 1 s and erased the obstacle from the costmaps
        }
        for k, v in defaults.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in defaults}
        self.range_pub = self.create_publisher(Range, '/ultrasonic', qos_profile_sensor_data)
        self.points_pub = self.create_publisher(PointCloud2, '/ultrasonic_points', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_ultrasonic', 10)
        # the drive requests below this guard in twist_mux, with their priorities and timeouts
        self.inputs = {'cmd_vel': (10, 0.5), 'cmd_vel_guard': (40, 0.3), 'cmd_vel_assist': (45, 0.5)}
        self.last = {}
        for topic in self.inputs:
            self.create_subscription(Twist, '/' + topic, lambda m, t=topic: self._on_cmd(t, m), 10)
        self.history = []
        self.blocked_until = 0.0
        self.blocking, self.near, self.reason = False, 0.0, ''
        self.ir = (False, False)
        self.side = 0              # +1 something on the left, -1 on the right, 0 straight ahead
        self.block_start = None    # when forward requests started being blocked
        self.backoff_until = 0.0
        self.backoff_start = None
        self.odom = None
        self.back_gap = 9.9
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', (m.pose.pose.position.x, m.pose.pose.position.y)), 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        if GPIO is not None:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(self.p['trig_gpio'], GPIO.OUT, initial=GPIO.LOW)
            GPIO.setup(self.p['echo_gpio'], GPIO.IN)
            for k in ('ir_left_gpio', 'ir_right_gpio'):
                if self.p[k] >= 0:
                    GPIO.setup(self.p[k], GPIO.IN)
            time.sleep(0.1)
        else:
            self.get_logger().warn('RPi.GPIO not available: no readings (run this on the Pi)')
        self.create_timer(1.0 / self.p['rate'], self._tick)
        self.create_timer(0.05, self._guard)            # 20 Hz: faster than the 0.3-0.5 s twist_mux timeouts
        self.get_logger().info('Ultrasonic guard: stops forward driving closer than %.0f cm, marks obstacles up to %.1f m'
                               % (100 * self.p['stop_distance'], self.p['mark_range']))

    def _on_scan(self, s):
        """Free space behind the box from the lidar (the lidar looks over low things, fine for walls and furniture)."""
        best, a = 9.9, s.angle_min
        edge = self.p['body_back'] + self.p['laser_x']          # back edge, in front of (-) / behind the lidar
        for r in s.ranges:
            if s.range_min < r < s.range_max:
                x, y = r * math.cos(a) + self.p['laser_x'], r * math.sin(a)
                if x < -self.p['body_back'] and abs(y) < self.p['body_half_width']:
                    best = min(best, -x - self.p['body_back'])
            a += s.angle_increment
        self.back_gap = best

    def _front_free(self):
        """Free distance in front of the box from the ultrasonic and IR (None = no echo, counted as free)."""
        if self.ir[0] or self.ir[1]:
            return 0.0
        valid = [x for x in self.history if x is not None]
        return min(valid) if valid else 9.9

    def _on_cmd(self, topic, msg):
        self.last[topic] = (time.monotonic(), msg)

    def _read(self):
        """One measurement in metres, or None when no echo came back (nothing in range, soft or angled surface)."""
        trig, echo = self.p['trig_gpio'], self.p['echo_gpio']
        GPIO.output(trig, GPIO.HIGH)
        time.sleep(10e-6)
        GPIO.output(trig, GPIO.LOW)
        t0 = time.monotonic()
        while not GPIO.input(echo):
            if time.monotonic() - t0 > 0.02:
                return None
        start = time.monotonic()
        while GPIO.input(echo):
            if time.monotonic() - start > 0.03:           # HC-SR04 holds Echo ~59 ms when it hears nothing
                return None
        d = (time.monotonic() - start) * SPEED_OF_SOUND_HALF
        return d if self.p['min_range'] <= d <= self.p['max_range'] else None

    def _tick(self):
        if GPIO is None:
            return
        d = self._read()
        self.history = (self.history + [d])[-3:]
        valid = sorted(x for x in self.history if x is not None)
        dist = valid[len(valid) // 2] if len(valid) >= 2 else None    # median of the last 3: one stray echo is ignored
        now = self.get_clock().now().to_msg()
        r = Range()
        r.header.stamp, r.header.frame_id = now, 'base_link'
        r.radiation_type, r.field_of_view = Range.ULTRASOUND, math.radians(30)
        r.min_range, r.max_range = float(self.p['min_range']), float(self.p['max_range'])
        r.range = float(dist) if dist is not None else math.inf
        self.range_pub.publish(r)
        left = self.p['ir_left_gpio'] >= 0 and GPIO.input(self.p['ir_left_gpio']) == GPIO.LOW
        right = self.p['ir_right_gpio'] >= 0 and GPIO.input(self.p['ir_right_gpio']) == GPIO.LOW
        if (left, right) != self.ir:
            self.get_logger().info('IR: left %s, right %s' % ('SEES' if left else 'clear', 'SEES' if right else 'clear'))
        self.ir = (left, right)
        self._publish_points(now, dist if dist is not None else d, d is not None or dist is not None, left, right)
        if left or right:
            self.blocked_until = max(self.blocked_until, time.monotonic() + self.p['ir_hold'])
            self.reason = 'IR ' + ('front-left and front-right' if left and right else 'front-left' if left else 'front-right')
            self.side = 0 if left and right else (1 if left else -1)
        if dist is not None and dist < self.p['stop_distance']:
            self.blocked_until = max(self.blocked_until, time.monotonic() + self.p['hold_seconds'])
            self.near = dist
            self.reason = 'ultrasonic, %.0f cm' % (100 * dist)
            if not (left or right):
                self.side = 0
        elif dist is not None and dist > self.p['stop_distance'] + self.p['release_margin'] and not (left or right):
            self.blocked_until = 0.0                  # a real echo from farther away: the way is free

    def _publish_points(self, stamp, dist, heard, left, right):
        x0, z = self.p['sensor_x'], self.p['sensor_z']
        far = x0 + self.p['mark_range'] + 0.2               # beyond the marking range: such a point only clears
        pts = []
        if dist is not None and dist < self.p['mark_range']:
            w = self.p['beam_half_width']
            pts += [(x0 + dist, w * k / 6.0, z) for k in range(-6, 7)]
        elif heard:                                      # a real far echo clears straight ahead; no echo clears nothing
            pts.append((far, 0.0, z))
        for side, seen in ((1, left), (-1, right)):
            for y in (self.p['ir_y'] + 0.12, self.p['ir_y'] + 0.06, self.p['ir_y'],
                      self.p['ir_y'] - 0.06, self.p['ir_y'] - 0.12):     # a 25 cm wide mark at that corner
                if seen:                                 # marks stay until the lidar or a far echo clears them
                    pts.append((self.p['ir_x'], side * y, 0.05))
        if not pts:
            return
        msg = PointCloud2()
        msg.header.stamp, msg.header.frame_id = stamp, 'base_link'
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                      for i, n in enumerate('xyz')]
        msg.is_bigendian, msg.point_step, msg.row_step, msg.is_dense = False, 12, 12 * len(pts), True
        msg.data = b''.join(struct.pack('<fff', *p) for p in pts)
        self.points_pub.publish(msg)

    def _winner(self):
        """The drive request twist_mux would pass on right now (highest priority, not timed out)."""
        now, best = time.monotonic(), None
        for topic, (prio, timeout) in self.inputs.items():
            if topic in self.last and now - self.last[topic][0] < timeout and (best is None or prio > best[0]):
                best = (prio, self.last[topic][1])
        return best[1] if best else None

    def _guard(self):
        now = time.monotonic()
        if now < self.backoff_until:                  # backing off: reverse, nose away from the object
            moved = math.hypot(self.odom[0] - self.backoff_start[0], self.odom[1] - self.backoff_start[1]) \
                if self.odom and self.backoff_start else 0.0
            front = self._front_free()
            enough = moved >= self.p['backoff_distance'] and front >= self.p['front_clear']
            if enough or moved >= self.p['backoff_max'] or self.back_gap < self.p['back_free']:
                self.backoff_until = 0.0
                self.cmd_pub.publish(Twist())
                self.get_logger().info('Backed off %.2f m: %s free in front, %.2f m behind'
                                       % (moved, 'open' if front > 2 else '%.2f m' % front, self.back_gap))
                return
            c = Twist()
            c.linear.x = -self.p['backoff_speed']
            c.angular.z = -self.side * self.p['backoff_turn'] if self.side else 0.0
            self.cmd_pub.publish(c)
            return
        want = self._winner()
        # everything but reversing is held: the wheels are at the front, so even turning on the spot sweeps a front
        # corner forwards/sideways into an object there (a cushion at the front-right corner, real test 2026-10-29)
        block = now < self.blocked_until and want is not None and want.linear.x > -0.01
        if block:
            if self.block_start is None:
                self.block_start = now
            elif now - self.block_start > self.p['backoff_after']:
                self.block_start = None
                if self.back_gap < self.p['back_free'] + 0.05:
                    self.get_logger().warn('Blocked ahead and no room behind (%.2f m): waiting' % self.back_gap)
                    return
                self.backoff_until = now + self.p['backoff_max'] / self.p['backoff_speed'] + 1.0
                self.backoff_start = self.odom
                self.get_logger().warn('Still pushed towards it: backing off until %.0f cm free in front%s' % (100 * self.p['front_clear'],
                                       '' if not self.side else ', nose turning ' + ('right' if self.side > 0 else 'left')))
                return
            self.cmd_pub.publish(Twist())
            if not self.blocking:
                self.get_logger().warn('Stopped: something low in front (%s)' % self.reason)
        else:
            self.block_start = None
            if self.blocking:
                self.get_logger().info('Ultrasonic: the way ahead is free again')
        self.blocking = block

    def destroy_node(self):
        if GPIO is not None:
            GPIO.cleanup(tuple(self.p[k] for k in ('trig_gpio', 'echo_gpio', 'ir_left_gpio', 'ir_right_gpio') if self.p[k] >= 0))
        super().destroy_node()


def main():
    rclpy.init()
    node = UltrasonicGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
