#!/usr/bin/env python3
"""Cargo box controller: locks and unlocks the parcel box with servos on the Arduino Nano (firmware/servo_nano).

Runs ON THE PI (the Nano is plugged into the Pi by USB).
  python3 src/my_bot/scripts/cargo_controller.py
  python3 src/my_bot/scripts/cargo_controller.py --ros-args -p simulate:=true     # no hardware: just pretend

Topics:
  in : /cargo_cmd    std_msgs/String   "lock" or "unlock"
  out: /cargo_state  std_msgs/String   "unknown", "moving", "locked", "unlocked" or "error: ..."   (latched)

"unlock" = lock servo to its unlocked angle, then (if a lid servo is enabled) open the lid.
The lid can be lifted by TWO servos together (config lid.servo2 = 1, lid.servo2_mirror for a mirror-image mount):
both move one degree at a time with the firmware's "p" command (servos 0 and 1 only).
"lock"   = (if a lid servo is enabled) close the lid, wait, then lock servo to its locked angle.
At start-up the box is locked (fail-safe), unless -p lock_on_start:=false.
Angles come from config/cargo.yaml (placeholders until calibrated on the finished box).
"""
import glob
import os
import queue
import threading
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

GREETING = 'servo_nano ready'


class SimLink:
    """Stands in for the Nano when there is no hardware."""
    port = 'simulated'

    def move(self, servo, angle):
        time.sleep(0.4)
        return True, 'OK %d' % angle

    def move_both(self, a0, a1):
        time.sleep(0.4)
        return True, 'OK %d %d' % (a0, a1)

    def release(self, servo):
        return True, 'OK'

    def close(self):
        pass


class NanoLink:
    """Serial connection to the servo Nano."""

    def __init__(self, ser):
        self.ser = ser
        self.port = ser.port

    def move(self, servo, angle):
        self.ser.reset_input_buffer()
        self.ser.write(('s %d %d\n' % (servo, angle)).encode())
        end = time.time() + 6.0                      # a full sweep takes about 2 s
        while time.time() < end:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line.startswith('OK'):
                return True, line
            if line.startswith('ERR'):
                return False, line
        return False, 'no answer from the Nano'

    def move_both(self, a0, a1):
        """Move servo 0 and servo 1 together (firmware 'p' command)."""
        self.ser.reset_input_buffer()
        self.ser.write(('p %d %d\n' % (a0, a1)).encode())
        end = time.time() + 10.0
        while time.time() < end:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line.startswith('OK'):
                return True, line
            if line.startswith('ERR'):
                return False, line
        return False, 'no answer from the Nano'

    def release(self, servo):
        """Tell the servo to stop holding (it goes limp and draws no current)."""
        self.ser.reset_input_buffer()
        self.ser.write(('d %d\n' % servo).encode())
        end = time.time() + 2.0
        while time.time() < end:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line.startswith('OK'):
                return True, line
        return False, 'no answer from the Nano'

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def open_nano(dev, baud=57600):
    """Open dev and return a NanoLink if the board there is the servo Nano, else None."""
    import serial
    try:
        ser = serial.Serial(dev, baud, timeout=1.0)
    except Exception:
        return None
    time.sleep(2.5)                                   # opening the port resets the Nano; it then says hello
    seen = ser.read(ser.in_waiting or 1).decode(errors='ignore')
    if GREETING not in seen:
        ser.reset_input_buffer()
        ser.write(b'?\n')
        seen += ser.readline().decode(errors='ignore')
    if GREETING in seen:
        return NanoLink(ser)
    ser.close()
    return None


class CargoController(Node):
    def __init__(self):
        super().__init__('cargo_controller')
        here = os.path.dirname(os.path.abspath(__file__))
        self.declare_parameter('config_file', os.path.join(here, '..', 'config', 'cargo.yaml'))
        self.declare_parameter('port', 'auto')            # 'auto' = /dev/servo_nano, else search the free USB ports
        self.declare_parameter('simulate', False)
        self.declare_parameter('lock_on_start', True)
        with open(os.path.normpath(self.get_parameter('config_file').value)) as f:
            self.cfg = yaml.safe_load(f)
        self.simulate = self.get_parameter('simulate').value
        self.link = SimLink() if self.simulate else None
        self.state = 'unknown'
        self.queue = queue.Queue()
        self.started_lock = False

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.state_pub = self.create_publisher(String, '/cargo_state', qos)
        self.create_subscription(String, '/cargo_cmd', self._on_cmd, 10)
        self._set_state('unknown')
        threading.Thread(target=self._worker, daemon=True).start()
        self.get_logger().info('Cargo controller started%s' % (' [SIMULATED]' if self.simulate else ''))

    # ---------- state ----------
    def _set_state(self, text):
        self.state = text
        self.state_pub.publish(String(data=text))
        self.get_logger().info('cargo state: %s' % text)

    def _on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd not in ('lock', 'unlock'):
            self.get_logger().warn("Ignoring unknown cargo command '%s' (use lock or unlock)" % msg.data)
            return
        self.queue.put(cmd)

    # ---------- finding and opening the Nano ----------
    def _find_link(self):
        port = self.get_parameter('port').value
        if port != 'auto':
            return open_nano(port)
        if os.path.exists('/dev/servo_nano'):
            return open_nano('/dev/servo_nano')
        busy = {os.path.realpath(p) for p in ('/dev/arduino', '/dev/ydlidar') if os.path.exists(p)}
        for dev in sorted(glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')):
            if os.path.realpath(dev) in busy:             # never touch the motor Arduino or the lidar
                continue
            link = open_nano(dev)
            if link:
                return link
        return None

    # ---------- the actions ----------
    def _move(self, name, angle_key, release_key=None):
        """Move one servo, then (if the config says so) let go of it, so only one servo draws current at a time."""
        part = self.cfg.get(name, {})
        if not part.get('enabled', False):
            return True
        angle = int(part[angle_key])
        servos = [int(part['servo'])]
        if part.get('servo2') is not None:               # two servos lift together (servos 0 and 1)
            servos.append(int(part['servo2']))
            a2 = 180 - angle if part.get('servo2_mirror', False) else angle
            a0, a1 = (angle, a2) if servos[0] == 0 else (a2, angle)
            ok, reply = self.link.move_both(a0, a1)
        else:
            ok, reply = self.link.move(servos[0], angle)
        if not ok:
            self.get_logger().error('%s servo: %s' % (name, reply))
            return False
        if release_key and part.get(release_key, False):
            for n in servos:
                ok, reply = self.link.release(n)
                if not ok:
                    self.get_logger().error('%s servo %d release: %s' % (name, n, reply))
        return ok

    def _do(self, cmd):
        self._set_state('moving')
        if cmd == 'unlock':
            ok = self._move('lock', 'unlocked', 'release_after_move') and self._move('lid', 'open', 'release_when_open')
        else:
            ok = self._move('lid', 'closed', 'release_when_closed')
            if ok:
                time.sleep(float(self.cfg.get('settle_seconds', 0.6)))
                ok = self._move('lock', 'locked', 'release_after_move')
        self._set_state(('unlocked' if cmd == 'unlock' else 'locked') if ok else 'error: servo did not answer')
        return ok

    def _worker(self):
        while rclpy.ok():
            if self.link is None:                         # (re)connect to the Nano
                self.link = self._find_link()
                if self.link is None:
                    if self.state != 'error: servo Nano not found':
                        self._set_state('error: servo Nano not found')
                    time.sleep(3.0)
                    continue
                self.get_logger().info('Servo Nano found on %s' % self.link.port)
            if not self.started_lock and self.get_parameter('lock_on_start').value:
                self.started_lock = True
                self._do('lock')
                continue
            self.started_lock = True
            try:
                cmd = self.queue.get(timeout=1.0)
            except queue.Empty:
                continue
            try:
                self._do(cmd)
            except Exception as exc:                      # e.g. the USB cable was pulled
                self.get_logger().error('Lost the Nano (%s); will reconnect' % exc)
                if self.link:
                    self.link.close()
                self.link = None
                self.started_lock = False
                self._set_state('error: servo Nano lost')
                self.queue.put(cmd)                       # try this command again once reconnected


def main():
    rclpy.init()
    node = CargoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
