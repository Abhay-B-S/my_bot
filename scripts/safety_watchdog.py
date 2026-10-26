#!/usr/bin/env python3
"""Safety watchdog: stops the motors if the motor Arduino misbehaves. Runs ON THE PI next to launch_robot.

Why: with the heavy delivery body, the motor Arduino sometimes restarts when the motor current changes
direction (stopping after reversing, turning). Its wheel counters then jump to zero, the wheel readings freeze,
and the motors could keep running for a minute. This program notices that and stops everything.

It trips when
  - a wheel reading changes impossibly fast (default > 30 rad/s; the motors manage ~15 at full power)
    = the Arduino restarted, or
  - a real drive command (>= 0.08 m/s or 0.3 rad/s; any reverse >= 0.03 m/s) is active but neither wheel reading
    changes for 5 s = the Arduino stopped answering, or a wheel is stalled or jammed, or
  - while REVERSING, a wheel turns much faster than asked (> reverse_burst_factor x the commanded wheel speed, and
    > 4 rad/s) = the "backward burst" seen in the lifted test of 2026-10-26 (13 rad/s for a 1.5 rad/s request).
Then it ends the motor controller process (ros2_control_node), so no more commands reach the Arduino, opens
/dev/arduino itself and sends stop commands (opening the port also restarts the Arduino into a stopped state),
and publishes the reason on /safety_event (std_msgs/String, latched).

Self-healing (2026-10-26, `auto_restart`): `restart_delay` s after a stop it starts the motor controller
(launch_robot) again and watches as before once wheel readings arrive ("RESTARTED" on /safety_event). The delivery
manager pauses the job meanwhile and self_recovery finds the position again (the odometry starts from zero). At most
`max_restarts` within `restart_window` s: after that it stays stopped ("GAVE UP"), so a real fault (dead battery,
jammed wheel) cannot cause an endless loop. Four Arduino restarts on 2026-10-26 each needed a manual restart.

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
        self.declare_parameter('frozen_seconds', 5.0)       # commanded but no wheel movement for this long
        self.declare_parameter('min_cmd_linear', 0.08)      # m/s: smaller commands may not move the heavy robot at all,
        self.declare_parameter('min_cmd_angular', 0.3)      # rad/s  so they do not count as "driving" (2026-10-23)
        self.declare_parameter('cmd_timeout', 0.6)          # a drive command counts as active this long
        self.declare_parameter('min_cmd_reverse', 0.03)     # m/s: reverse commands from this speed count as driving
        self.declare_parameter('reverse_burst_factor', 3.0)  # wheel faster than this x the commanded speed = burst
        self.declare_parameter('wheel_radius', 0.034)       # m (to turn the commanded m/s into wheel rad/s)
        self.declare_parameter('port', '/dev/arduino')
        self.declare_parameter('dry_run', False)
        self.declare_parameter('auto_restart', True)        # start the motor controller again after a stop
        self.declare_parameter('restart_delay', 3.0)        # s after the stop
        self.declare_parameter('max_restarts', 3)           # ... at most this many times ...
        self.declare_parameter('restart_window', 600.0)     # ... within this many seconds
        self.declare_parameter('restart_timeout', 45.0)     # s to wait for wheel readings after a restart
        self.max_speed = self.get_parameter('max_wheel_speed').value
        self.frozen_s = self.get_parameter('frozen_seconds').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.dry_run = self.get_parameter('dry_run').value

        self.last_pos = None          # (time, {wheel: position})
        self.last_change = time.monotonic()
        self.cmd_active_until = 0.0
        self.cmd_since = None         # when the current stretch of active commands began
        self.cmd_x = 0.0              # last commanded linear speed (m/s), and until when it counts
        self.cmd_x_until = 0.0
        self.tripped = False
        self.restarts = []            # times of automatic restarts
        self.restart_at = None        # when to start the motor controller again
        self.restarting_until = None  # waiting for wheel readings after a restart

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.event_pub = self.create_publisher(String, '/safety_event', latched)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 10)
        self.create_subscription(Twist, '/diff_cont/cmd_vel_unstamped', self._on_cmd, 10)
        self.create_timer(0.2, self._check_frozen)
        self.event_pub.publish(String(data='READY: watchdog started (motor controller started by hand)'))
        self.get_logger().info('Safety watchdog running%s: trips above %.0f rad/s, or %.1f s commanded without '
                               'wheel movement' % (' [DRY RUN]' if self.dry_run else '', self.max_speed, self.frozen_s))

    # ---------- inputs ----------
    def _on_cmd(self, msg):
        moving = (abs(msg.linear.x) >= self.get_parameter('min_cmd_linear').value or
                  abs(msg.angular.z) >= self.get_parameter('min_cmd_angular').value or
                  msg.linear.x <= -self.get_parameter('min_cmd_reverse').value)
        now = time.monotonic()
        self.cmd_x, self.cmd_x_until = msg.linear.x, now + self.cmd_timeout
        if moving:
            if self.cmd_since is None or now > self.cmd_active_until:
                self.cmd_since = now
            self.cmd_active_until = now + self.cmd_timeout

    def _on_joints(self, msg):
        now = time.monotonic()
        pos = {n: p for n, p in zip(msg.name, msg.position) if n in WHEELS}
        if len(pos) < 2 or (self.tripped and self.restarting_until is None):
            return
        if self.restarting_until is not None:      # first readings from the restarted controller: watch again
            self.restarting_until, self.tripped = None, False
            self.last_pos, self.last_change, self.cmd_since = (now, pos), now, None
            self.get_logger().info('Motor controller restarted: watching again')
            self.event_pub.publish(String(data='RESTARTED: motor controller running again'))
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
            if self.cmd_x < -0.005 and now < self.cmd_x_until and dt > 0.02:      # reversing: watch for a burst
                asked = abs(self.cmd_x) / self.get_parameter('wheel_radius').value
                limit = max(4.0, self.get_parameter('reverse_burst_factor').value * asked)
                for n in WHEELS:
                    speed = abs(pos[n] - p0[n]) / dt
                    if speed > limit:
                        self._trip('while reversing at %.2f m/s, %s turned at %.1f rad/s (limit %.1f): backward burst'
                                   % (self.cmd_x, n, speed, limit))
                        return
        self.last_pos = (now, pos)

    def _check_frozen(self):
        now = time.monotonic()
        if self.restart_at is not None and now >= self.restart_at:
            self.restart_at = None
            self._restart()
        if self.restarting_until is not None and now > self.restarting_until:
            self.restarting_until = None
            self._give_up('the restarted motor controller sent no wheel readings within %.0f s'
                          % self.get_parameter('restart_timeout').value)
        if self.tripped:
            return
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
            self.get_logger().error('Stop sent to the motor Arduino.')
        except Exception as exc:
            self.get_logger().error('Could not send stop to the Arduino (%s): SWITCH OFF THE MOTOR BATTERY' % exc)
            return self._give_up('could not stop the motor Arduino')
        # 3. self-healing: start the motor controller again a moment later (limited)
        if not self.get_parameter('auto_restart').value:
            self.get_logger().error('auto_restart is off: start launch_robot again to drive.')
            return
        now = time.monotonic()
        window = self.get_parameter('restart_window').value
        self.restarts = [t for t in self.restarts if now - t < window]
        if len(self.restarts) >= self.get_parameter('max_restarts').value:
            return self._give_up('%d automatic restarts within %.0f min already' % (len(self.restarts), window / 60))
        self.restarts.append(now)
        self.restart_at = now + self.get_parameter('restart_delay').value
        self.get_logger().warn('Restarting the motor controller in %.0f s (automatic restart %d of %d)'
                               % (self.get_parameter('restart_delay').value, len(self.restarts),
                                  self.get_parameter('max_restarts').value))

    def _restart(self):
        """End what is left of launch_robot and start it again (detached, logging to ~/robot.log)."""
        for pattern in ('^/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch my_bot launch_robot.launch.py',
                        '^/opt/ros/foxy/lib/robot_state_publisher/robot_state_publisher',
                        '^/opt/ros/foxy/lib/controller_manager/'):
            subprocess.run(['pkill', '-TERM', '-f', pattern])
        time.sleep(3.0)
        for pattern in ('launch_robot.launch.py', '^/opt/ros/foxy/lib/robot_state_publisher/',
                        '^/opt/ros/foxy/lib/controller_manager/'):
            subprocess.run(['pkill', '-KILL', '-f', pattern])
        log = open(os.path.expanduser('~/robot.log'), 'a')
        subprocess.Popen(['ros2', 'launch', 'my_bot', 'launch_robot.launch.py'], stdout=log, stderr=log,
                         stdin=subprocess.DEVNULL, cwd=os.path.expanduser('~/robot_ws'), start_new_session=True)
        self.last_pos = None
        self.restarting_until = time.monotonic() + self.get_parameter('restart_timeout').value
        self.get_logger().warn('Motor controller started again; waiting for wheel readings')
        self.event_pub.publish(String(data='RESTARTING: motor controller starting again'))

    def _give_up(self, why):
        self.get_logger().error('NOT restarting (%s): check the battery and wiring, then start launch_robot by hand'
                                % why)
        self.event_pub.publish(String(data='GAVE UP: ' + why))


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
