#!/usr/bin/env python3
"""Safety watchdog: stops the motors if the motor Arduino misbehaves. Runs ON THE PI next to launch_robot.

Why: with the heavy delivery body, the motor Arduino sometimes restarts when the motor current changes
direction (stopping after reversing, turning). Its wheel counters then jump to zero, the wheel readings freeze,
and the motors could keep running for a minute. This program notices that and stops everything.

It trips when
  - a wheel reading changes impossibly fast (default > 30 rad/s; the motors manage ~15 at full power)
    = the Arduino restarted, or
  - a drive command is active but neither wheel reading changes for 3 s
    = the Arduino stopped answering, or a wheel is stalled or jammed.
Then it ends the motor controller process (ros2_control_node), so no more commands reach the Arduino, opens
/dev/arduino itself and sends stop commands (opening the port also restarts the Arduino into a stopped state),
and publishes the reason on /safety_event (std_msgs/String, latched). The robot stays stopped until
launch_robot is started again by hand.

  python3 safety_watchdog.py                         # normal use, on the Pi, after launch_robot
  python3 safety_watchdog.py --ros-args -p dry_run:=true   # only report what it WOULD do (testing)
"""
import os
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

WHEELS = ('left_wheel_joint', 'right_wheel_joint')


class SafetyWatchdog(Node):
    def __init__(self):
        super().__init__('safety_watchdog')
        self.declare_parameter('max_wheel_speed', 30.0)     # rad/s; above this the reading is not physical
        self.declare_parameter('frozen_seconds', 3.0)       # commanded but no wheel movement for this long
        self.declare_parameter('cmd_timeout', 0.6)          # a drive command counts as active this long
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('dry_run', False)
        self.max_speed = self.get_parameter('max_wheel_speed').value
        self.frozen_s = self.get_parameter('frozen_seconds').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.dry_run = self.get_parameter('dry_run').value

        self.last_pos = None          # (time, {wheel: position})
        self.last_change = time.monotonic()
        self.cmd_active_until = 0.0
        self.cmd_since = None         # when the current stretch of active commands began
        self.tripped = False

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.event_pub = self.create_publisher(String, '/safety_event', latched)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(Twist, '/diff_cont/cmd_vel_unstamped', self._on_cmd, 10)
        self.create_timer(0.2, self._check_frozen)
        self.get_logger().info('Safety watchdog running%s: trips above %.0f rad/s, or %.1f s commanded without '
                               'wheel movement' % (' [DRY RUN]' if self.dry_run else '', self.max_speed, self.frozen_s))

    # ---------- inputs ----------
    def _on_cmd(self, msg):
        moving = abs(msg.linear.x) > 0.02 or abs(msg.angular.z) > 0.1
        now = time.monotonic()
        if moving:
            if self.cmd_since is None or now > self.cmd_active_until:
                self.cmd_since = now
            self.cmd_active_until = now + self.cmd_timeout

    def _on_joints(self, msg):
        now = time.monotonic()
        pos = {n: p for n, p in zip(msg.name, msg.position) if n in WHEELS}
        if len(pos) < 2:
            return
        if self.last_pos is not None:
            t0, p0 = self.last_pos
            dt = max(now - t0, 1e-3)
            for n in WHEELS:
                speed = abs(pos[n] - p0[n]) / dt
                if speed > self.max_speed and abs(pos[n] - p0[n]) > 1.0:
                    self._trip('%s reading jumped %.1f rad in %.2f s (%.0f rad/s): the motor Arduino restarted?'
                               % (n, pos[n] - p0[n], dt, speed))
                    return
            if any(abs(pos[n] - p0[n]) > 0.01 for n in WHEELS):
                self.last_change = now
        self.last_pos = (now, pos)

    def _check_frozen(self):
        now = time.monotonic()
        commanded = now < self.cmd_active_until
        if not commanded:
            self.cmd_since = None
            return
        # frozen = commanded for a while AND no wheel movement for frozen_s
        if self.cmd_since is not None and now - self.cmd_since > self.frozen_s and now - self.last_change > self.frozen_s:
            self._trip('drive command active for %.1f s but the wheel readings have not changed: the Arduino '
                       'stopped answering, or a wheel is stalled' % (now - self.cmd_since))

    # ---------- the stop ----------
    def _trip(self, reason):
        if self.tripped:
            return
        self.tripped = True
        self.get_logger().error('SAFETY STOP: %s' % reason)
        self.event_pub.publish(String(data='SAFETY STOP: ' + reason))
        if self.dry_run:
            self.get_logger().warn('(dry run) would now end ros2_control_node and send stop to the Arduino')
            return
        # 1. end the motor controller so nothing keeps commanding the motors
        try:
            out = subprocess.run(['pgrep', '-f', '^/opt/ros/foxy/lib/controller_manager/ros2_control_node'],
                                 capture_output=True, text=True).stdout.split()
            for pid in out:
                os.kill(int(pid), signal.SIGKILL)
            self.get_logger().error('Ended the motor controller (pid %s)' % ', '.join(out) if out else
                                    'Motor controller was not running')
        except Exception as exc:
            self.get_logger().error('Could not end the motor controller: %s' % exc)
        # 2. stop the motors directly (opening the port restarts the Arduino into a stopped state)
        try:
            import serial
            time.sleep(0.3)
            ser = serial.Serial(self.get_parameter('port').value, 57600, timeout=0.5)
            time.sleep(2.5)
            for cmd in (b'm 0 0\r', b'o 0 0\r', b'o 0 0\r'):
                ser.write(cmd)
                time.sleep(0.15)
            ser.close()
            self.get_logger().error('Stop sent to the motor Arduino. Start launch_robot again to drive.')
        except Exception as exc:
            self.get_logger().error('Could not send stop to the Arduino (%s): SWITCH OFF THE MOTOR BATTERY' % exc)


def main():
    rclpy.init()
    node = SafetyWatchdog()
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
