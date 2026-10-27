#!/usr/bin/env python3
"""Delivery manager: runs delivery jobs one at a time with Nav2.

A job is "sender,destination" (names from config/locations.yaml). For each job the robot:
  home -> sender -> (waits for "shipped") -> destination -> (waits for "received") -> home
If another job is already waiting when one ends, the robot does NOT go home first: it drives straight
from where it is to the next sender (also when a new job arrives while it is on its way home).
Set go_home_between_jobs:=true for the old behaviour (always home first), e.g. to compare the two.

Which job next (2026-10-26): urgent jobs first, in the order they came; otherwise the job whose sender is nearest
to the robot, but a job waiting longer than `max_wait` s goes first (oldest first), so nobody waits for ever.
Cancel (/delivery_cancel, website id): a queued job is dropped; before the parcel is on board the robot simply
does not go (next job or home); with the parcel on board it brings it back to the sender (WAIT_RETURNED).
Emergency stop (/delivery_estop "stop"/"resume"): stops the robot at once (zero speed at teleop priority, Nav2
goal cancelled) and continues the same leg on "resume".

Topics (all plain ROS 2, so a website bridge can use them later):
  in : /delivery_request   std_msgs/String  "cabin_A,cabin_D"  (optional 3rd field = website id: "cabin_A,cabin_D,-Nabc",
                                             optional 4th field "urgent")
  in : /delivery_cancel    std_msgs/String  website id of the job to cancel
  in : /delivery_estop     std_msgs/String  "stop" or "resume"
  in : /delivery_reload_spots std_msgs/Empty  read locations_file again (the Admin moved/added/deleted a spot)
  in : /delivery_shipped   std_msgs/Empty   sender has loaded the parcel
  in : /delivery_received  std_msgs/Empty   receiver has taken the parcel
  in : /delivery_open_lid  std_msgs/Empty   the person at the robot wants the lid open (lid_buttons mode)
  in : /delivery_returned  std_msgs/Empty   the sender has taken an undeliverable parcel back out
  out: /delivery_status    std_msgs/String  JSON: state, job, sender, destination, detail, queue, cargo, lid_buttons,
                                             estop, cancelled_ref (a queued job that was dropped)
  out: /cmd_vel_key        geometry_msgs/Twist  zero speed, 10 times a second, while the emergency stop is on

Parcel box (needs cargo_controller.py running):
  -p cargo_lock:=true                       box opens by itself on arrival, closes before every drive
  -p cargo_lock:=true -p lid_buttons:=true  box opens only when the person presses "Open lid" on the website;
                                            "shipped"/"received" close it, and the robot never drives until the
                                            box reports "locked" (it keeps retrying and says so instead)

Self-recovery (needs scripts/self_recovery.py running):  -p self_recovery:=true
  on /localization_status "lost" the job is paused (Nav2 goal cancelled); on "recovered" the robot continues to the
  same spot (or drives back to the cabin it was waiting at, if someone moved it); "help: ..." is shown on the website

Blocked routes (needs scripts/path_guard.py running):  -p path_guard:=true
  /path_status "waiting"/"rerouting"/"detour" are shown on the website. "blocked" (no way at all), or blocks at
  `max_detours` different places on one leg (the same box seen twice counts once), ends the leg at once:
    on the way to the sender                 -> job failed, the robot goes home
    on the way to the receiver (parcel)      -> it brings the parcel back to the sender (RETURN_TO_SENDER), waits
                                                there for "returned" (WAIT_RETURNED; website: "Parcel taken back")
    on the way back to the sender (parcel)   -> job failed, it goes home with the parcel
    on the way home                          -> it stops, asks for the path to be cleared and looks at every route
                                                again every `recheck_wait` s; continues once a way is clear
  (Optional: -p blocked_rechecks:=N waits `recheck_wait` s and looks at every route again N times before giving up;
  0 by default, the user found re-checking too slow on 2026-10-24.)
  The same giving-up rules apply when Nav2 fails a leg `max_nav_retries` times for any other reason.

Battery (needs scripts/battery_monitor.py running):  -p battery_aware:=true
  Before starting a job the robot estimates the energy it needs for the worst case: from where it is to the sender,
  to the receiver, back to the sender (route blocked, parcel returned) and home: straight-line metres x `wh_per_m`
  (learned from its own trips, kept in energy_file) + `wait_wh` for the waiting. If the battery would end below
  `reserve_pct`, the job stays in the queue, the robot goes home and asks to be charged; it starts by itself once the
  battery is high enough again. If the battery turns "critical" on the way to a sender (no parcel on board yet),
  that job goes back to the front of the queue and the robot goes home. With a parcel on board it always finishes.
  Motor power switched off (battery "off"): no job starts. A simulated battery (no sensor fitted) is ignored unless
  -p battery_trust_sim:=true (Gazebo tests only).

Run (sim, Nav2 already running):
  python3 src/my_bot/scripts/delivery_manager.py --ros-args -p use_sim_time:=true
Test without Nav2:
  python3 src/my_bot/scripts/delivery_manager.py --ros-args -p dry_run:=true
"""
import json
import math
import os
import re
import time
from collections import deque

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Empty, String
from tf2_ros import Buffer, TransformListener

# States of the state machine
IDLE = 'IDLE'
TO_PICKUP = 'TO_PICKUP'
WAIT_SHIPPED = 'WAIT_SHIPPED'
TO_DROPOFF = 'TO_DROPOFF'
WAIT_RECEIVED = 'WAIT_RECEIVED'
RETURNING = 'RETURNING'
RETURN_TO_SENDER = 'RETURN_TO_SENDER'    # could not deliver: bringing the parcel back
WAIT_RETURNED = 'WAIT_RETURNED'          # back at the sender, waiting for "returned"


class DeliveryManager(Node):
    def __init__(self):
        super().__init__('delivery_manager')
        here = os.path.dirname(os.path.abspath(__file__))
        self.declare_parameter('locations_file', os.path.join(here, '..', 'config', 'locations.yaml'))
        self.declare_parameter('home_name', 'home')
        self.declare_parameter('dry_run', False)             # fake the driving (3 s per trip)
        self.declare_parameter('wait_shipped_timeout', 600.0)   # seconds
        self.declare_parameter('wait_received_timeout', 600.0)  # seconds
        self.declare_parameter('max_nav_retries', 2)
        self.declare_parameter('nav_timeout', 180.0)          # seconds allowed for one leg (per attempt)
        self.declare_parameter('cargo_lock', False)           # True = use the parcel-box lock (cargo_controller on the Pi)
        self.declare_parameter('cargo_lock_timeout', 10.0)    # seconds to wait for "locked" before driving anyway
        self.declare_parameter('go_home_between_jobs', False)  # True = old behaviour: always return home first
        self.declare_parameter('lid_buttons', False)          # True = the person opens the lid from the website
        self.declare_parameter('self_recovery', False)        # True = pause/resume on /localization_status
        self.declare_parameter('path_guard', False)           # True = report/pause on /path_status (blocked routes)
        self.declare_parameter('blocked_rechecks', 0)         # times all routes are checked again before giving up
        self.declare_parameter('recheck_wait', 15.0)          # s to wait (blocked) before checking the routes again
        self.declare_parameter('return_to_sender', True)      # undeliverable parcel goes back to the sender
        self.declare_parameter('max_detours', 2)              # route switches on one leg before "every route blocked"
        self.declare_parameter('max_wait', 300.0)             # s: a normal job waiting this long goes before nearer ones
        self.declare_parameter('battery_aware', False)        # True = check the battery before each job
        self.declare_parameter('battery_wh', 27.0)            # usable energy of a full pack: 3 x 3.6 V x 2.5 Ah
        self.declare_parameter('wh_per_m', 0.15)              # first guess, then learned (Wh per straight-line metre)
        self.declare_parameter('wait_wh', 0.5)                # energy for the waiting at the two cabins
        self.declare_parameter('reserve_pct', 12.0)           # battery % that must be left after the worst case
        self.declare_parameter('energy_file', '~/dev_ws/run_logs/energy_model.json')
        self.declare_parameter('battery_trust_sim', False)    # act on a SIMULATED battery too (tests in Gazebo only;
                                                              # never on the real robot: its numbers are made up)
        self.declare_parameter('arrival_check', 0.5)          # metres; Nav2 "arrived" is only believed if the robot
                                                              # is really this close to the spot (0 = do not check)

        self.locations_path = os.path.normpath(self.get_parameter('locations_file').value)
        path = self.locations_path
        with open(path) as f:
            data = yaml.safe_load(f)
        self.frame = data.get('frame_id', 'map')
        self.locations = data['locations']
        self.home = self.get_parameter('home_name').value
        if self.home not in self.locations:
            raise RuntimeError("home location '%s' not in %s" % (self.home, path))
        self.dry_run = self.get_parameter('dry_run').value
        self.get_logger().info('Locations: %s (frame %s)%s' % (
            ', '.join(self.locations), self.frame, '  [DRY RUN]' if self.dry_run else ''))

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.tf_buffer = Buffer()                 # the robot's real position, to double-check Nav2's "arrived"
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(String, '/delivery_request', self._on_request, 10)
        self.create_subscription(Empty, '/delivery_shipped', self._on_shipped, 10)
        self.create_subscription(Empty, '/delivery_received', self._on_received, 10)
        self.create_subscription(Empty, '/delivery_returned', self._on_returned, 10)
        self.create_subscription(String, '/delivery_cancel', self._on_cancel, 10)
        self.create_subscription(String, '/delivery_estop', self._on_estop, 10)
        self.create_subscription(Empty, '/delivery_reload_spots', self._on_reload_spots, 10)
        self.stop_pub = self.create_publisher(Twist, '/cmd_vel_key', 10)   # emergency stop: above everything
        self.estop = False
        self.estop_target = None         # the leg to continue after "resume"
        self.create_timer(0.1, self._hold_estop)
        # transient local: a late subscriber (e.g. the website bridge) still gets the last status
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/delivery_status', status_qos)

        # optional parcel-box lock: unlock when the robot reaches a person, lock before every drive
        self.cargo_enabled = self.get_parameter('cargo_lock').value
        self.lid_buttons = self.cargo_enabled and self.get_parameter('lid_buttons').value
        self.cargo_state = 'unknown'
        self.lock_deadline = 0.0
        if self.cargo_enabled:
            self.cargo_pub = self.create_publisher(String, '/cargo_cmd', 10)
            self.create_subscription(String, '/cargo_state', self._on_cargo_state, status_qos)
            self.get_logger().info('Cargo lock ENABLED (needs cargo_controller running)%s'
                                   % (', lid opened by the website buttons' if self.lid_buttons else ''))
        elif self.get_parameter('lid_buttons').value:
            self.get_logger().warn('lid_buttons needs cargo_lock:=true: ignoring it')
        self.create_subscription(Empty, '/delivery_open_lid', self._on_open_lid, 10)
        self.recovery_enabled = self.get_parameter('self_recovery').value
        self.lost = False                # self_recovery reported the position lost: no driving until recovered
        self.resume_target = None        # where to go once the position is found again
        if self.recovery_enabled:
            self.create_subscription(String, '/localization_status', self._on_localization, status_qos)
            self.get_logger().info('Self-recovery ENABLED (needs self_recovery.py running)')
        self.motor_fault = False         # safety_watchdog stopped the motors and restarts them: no driving until then
        # only NEW events (not latched): an old "SAFETY STOP" from before a manual restart must not block the robot
        self.create_subscription(String, '/safety_event', self._on_safety_event, 10)
        self.path_blocked = False        # path_guard says the route is blocked with no other way: wait
        self.path_state = 'clear'
        self.blocked_since = 0.0
        self.rechecks = 0                # route checks already done on this leg
        self.detours = 0                 # blocked routes met since the leg (or the last check) started
        self.detour_spots = []           # where they were (the same box seen twice counts once)
        self.hold_until = 0.0            # no new goal before this time
        if self.get_parameter('path_guard').value:
            self.create_subscription(String, '/path_status', self._on_path_status, status_qos)
            self.forget_pub = self.create_publisher(Empty, '/path_guard/forget', 10)
            self.clear_costmap = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
            self.get_logger().info('Path guard ENABLED (needs path_guard.py running)')

        self.battery_aware = self.get_parameter('battery_aware').value
        self.battery = None              # last /battery_status (dict) and when it came
        self.battery_t = 0.0
        self.battery_hold = False        # a job waits for charging: do not skip the trip home
        self.battery_msg = ''            # last "please charge" message (published again only when it changes)
        self.leg_energy = None           # (Wh counter, x, y) at the start of the current leg, for learning wh_per_m
        self.energy_path = os.path.expanduser(self.get_parameter('energy_file').value)
        self.wh_per_m = self.get_parameter('wh_per_m').value
        try:
            with open(self.energy_path) as f:
                self.wh_per_m = float(json.load(f)['wh_per_m'])
        except (OSError, ValueError, KeyError):
            pass
        if self.battery_aware:
            self.create_subscription(String, '/battery_status', self._on_battery, status_qos)
            self.get_logger().info('Battery check ENABLED (needs battery_monitor.py), %.3f Wh per metre'
                                   % self.wh_per_m)

        self.state = IDLE
        self.lid_opened = False       # someone opened the lid at the current stop (lid_buttons mode)
        self.last_spot = self.home    # the last spot reached (for "nearest" when the live position is unknown)
        self.queue = deque()
        self.job = None
        self.next_id = 1
        self.seen_refs = set()     # website ids already accepted (a repeated send is ignored)
        self.want_goal = None     # location name we still have to send to Nav2
        self.navigating = False    # a goal is in flight
        self.goal_handle = None
        self.goal_gen = 0          # counts goals sent, to ignore callbacks of abandoned goals
        self.goal_started = 0.0
        self.retries = 0
        self.deadline = None
        self.dry_until = 0.0
        self.last_wait_log = 0.0
        self.last_publish = 0.0    # time of the last status message (two in a row could be lost)
        self.last_detail = ''      # repeated when only the box state changes

        self.create_timer(0.5, self._tick)
        self._publish('Ready at %s. Waiting for requests.' % self.home)

    # ---------- status ----------
    def _publish(self, detail, result=None, rejected_ref=None, cancelled_ref=None, requeued_ref=None):
        # ref = the website's id for the CURRENT job; result = 'delivered' or 'failed' when a job ends;
        # rejected_ref = the id of a request that was refused (so the website can show why)
        msg = {'state': self.state,
               'job': self.job['id'] if self.job else None,
               'ref': self.job.get('ref') if self.job else None,
               'sender': self.job['sender'] if self.job else None,
               'destination': self.job['destination'] if self.job else None,
               'detail': detail, 'queue': len(self.queue),
               'queue_jobs': [{'ref': j.get('ref'), 'sender': j['sender'], 'destination': j['destination'],
                               'urgent': bool(j.get('urgent'))} for j in self.queue],   # for eta_estimator
               'result': result, 'rejected_ref': rejected_ref, 'cancelled_ref': cancelled_ref,
               'requeued_ref': requeued_ref,
               'urgent': bool(self.job and self.job.get('urgent')), 'estop': self.estop,
               'cargo': self.cargo_state if self.cargo_enabled else None, 'lid_buttons': self.lid_buttons}
        self.status_pub.publish(String(data=json.dumps(msg)))
        self.last_publish = time.monotonic()
        if result is None and rejected_ref is None and cancelled_ref is None and requeued_ref is None:
            self.last_detail = detail
        self.get_logger().info('[%s] %s' % (self.state, detail))

    # ---------- incoming messages ----------
    def _on_request(self, msg):
        parts = [p.strip() for p in msg.data.split(',')]
        urgent = len(parts) == 4 and parts[3] == 'urgent'
        if urgent:
            parts = parts[:3]
        ref = parts[2] if len(parts) == 3 else None   # optional website id
        if ref is not None and ref in self.seen_refs:
            self.get_logger().info('Ignoring duplicate request %s' % ref)
            return
        if len(parts) not in (2, 3) or parts[0] not in self.locations or parts[1] not in self.locations:
            self._publish("Rejected request '%s': use 'sender,destination' with names from %s"
                          % (msg.data, ', '.join(self.locations)), rejected_ref=ref)
            return
        if parts[0] == parts[1] or self.home in parts[:2]:
            self._publish("Rejected request '%s': sender and destination must differ and not be %s"
                          % (msg.data, self.home), rejected_ref=ref)
            return
        if ref is not None:
            self.seen_refs.add(ref)
        job = {'id': self.next_id, 'sender': parts[0], 'destination': parts[1], 'ref': ref, 'urgent': urgent,
               'queued_at': time.monotonic()}
        self.next_id += 1
        self.queue.append(job)
        self._publish('%sJob %d queued: %s -> %s (%d waiting)'
                      % ('URGENT ' if urgent else '', job['id'], job['sender'], job['destination'], len(self.queue)))

    def _on_shipped(self, _msg):
        if self.state != WAIT_SHIPPED:
            self.get_logger().warn('Ignoring "shipped": state is %s' % self.state)
            return
        self.state = TO_DROPOFF
        self._publish('Shipped. Going to %s' % self.job['destination'])
        self._go(self.job['destination'])

    def _on_received(self, _msg):
        if self.state != WAIT_RECEIVED:
            self.get_logger().warn('Ignoring "received": state is %s' % self.state)
            return
        self.state = RETURNING
        if self.queue and not self.get_parameter('go_home_between_jobs').value:
            # no trip home: the main loop starts the waiting job in about a second, from right here
            self._publish('Delivered job %d. Going straight to the next job (%d waiting)'
                          % (self.job['id'], len(self.queue)), result='delivered')
        else:
            self._publish('Delivered job %d. Returning to %s' % (self.job['id'], self.home),
                          result='delivered')
            self._go(self.home)      # the main loop cancels this if a job arrives on the way

    def _on_returned(self, _msg):
        if self.state != WAIT_RETURNED:
            self.get_logger().warn('Ignoring "returned": state is %s' % self.state)
            return
        self.state = RETURNING
        result = 'cancelled' if self.job.get('cancelled') else 'returned'
        if self.queue and not self.get_parameter('go_home_between_jobs').value:
            self._publish('Parcel of job %d returned to %s. Going straight to the next job (%d waiting)'
                          % (self.job['id'], self.job['sender'], len(self.queue)), result=result)
        else:
            self._publish('Parcel of job %d returned to %s. Returning to %s'
                          % (self.job['id'], self.job['sender'], self.home), result=result)
            self._go(self.home)

    def _on_cancel(self, msg):
        """The sender cancels a job (website id): drop it, skip it, or bring the parcel back."""
        ref = msg.data.strip()
        for job in list(self.queue):
            if job.get('ref') == ref:
                self.queue.remove(job)
                self._publish('Job %d cancelled by the sender before it started.' % job['id'], cancelled_ref=ref)
                return
        if not self.job or self.job.get('ref') != ref:
            self.get_logger().warn('Ignoring cancel for %s: not a waiting or current job' % ref)
            return
        if self.state == WAIT_SHIPPED and self.lid_buttons and self.lid_opened:
            # the sender opened the lid and may have put the parcel in: never drive off with it locked inside
            self.job['cancelled'] = True
            self.state = WAIT_RETURNED
            self.deadline = time.monotonic() + self.get_parameter('wait_received_timeout').value
            self._publish('Cancelled by the sender. If the parcel is in the box, take it out, then tap '
                          '"Close lid & done".')
        elif self.state in (TO_PICKUP, WAIT_SHIPPED):            # nothing on board yet
            self._abandon_goal()
            self.estop_target = None
            self.state = RETURNING
            if self.queue and not self.get_parameter('go_home_between_jobs').value:
                self._publish('Job %d cancelled by the sender. Going to the next job (%d waiting)'
                              % (self.job['id'], len(self.queue)), result='cancelled')
            else:
                self._publish('Job %d cancelled by the sender. Returning to %s' % (self.job['id'], self.home),
                              result='cancelled')
                self._go(self.home)
        elif self.state in (TO_DROPOFF, WAIT_RECEIVED):        # parcel on board: bring it back to the sender
            self._abandon_goal()
            self.job['cancelled'] = True
            self.state = RETURN_TO_SENDER
            self.estop_target = self.job['sender'] if self.estop else None
            self._publish('Cancelled by the sender: bringing the parcel back to %s.' % self.job['sender'])
            self._go(self.job['sender'])
        else:
            self.get_logger().info('Cancel for job %d ignored: the parcel is already going back' % self.job['id'])

    def _on_estop(self, msg):
        cmd = msg.data.strip().lower()
        if cmd == 'stop' and not self.estop:
            self.estop = True
            moving = self.navigating or self.want_goal
            self.estop_target = (self.want_goal or self._current_target()) if moving else None
            self._abandon_goal()
            self.stop_pub.publish(Twist())
            self._publish('EMERGENCY STOP: the robot is stopped. Press Resume to continue.')
        elif cmd == 'resume' and self.estop:
            self.estop = False
            target, self.estop_target = self.estop_target, None
            if target:
                self._publish('Emergency stop released. Continuing to %s.' % target)
                self._go(target)
            else:
                self._publish('Emergency stop released.')

    def _on_reload_spots(self, _msg):
        """The Admin changed the spots on the website: use the new positions from now on (the next goal sent uses
        them); jobs for a deleted cabin are dropped, the current one is cancelled like by its sender."""
        try:
            with open(self.locations_path) as f:
                locs = (yaml.safe_load(f) or {}).get('locations') or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error('Could not read the spots again: %s' % exc)
            return
        if self.home not in locs:
            self.get_logger().error('New spot file has no %s: keeping the old spots' % self.home)
            return
        gone = set(self.locations) - set(locs)
        old, self.locations = self.locations, locs
        self.get_logger().info('Spots reloaded: %s%s' % (', '.join(locs), ' (deleted: %s)' % ', '.join(sorted(gone))
                                                          if gone else ''))
        for job in list(self.queue):
            if job['sender'] in gone or job['destination'] in gone:
                self.queue.remove(job)
                self._publish('Job %d dropped: its cabin was removed by the Admin.' % job['id'], cancelled_ref=job['ref'])
        if self.job and (self.job['sender'] in gone or self.job['destination'] in gone) and self.job.get('ref'):
            if self.job['sender'] in gone and self.state in (TO_DROPOFF, WAIT_RECEIVED, RETURN_TO_SENDER, WAIT_RETURNED):
                for name in (self.job['sender'], self.job['destination']):
                    if name in gone:                   # keep the old position until this parcel is handed over
                        self.locations[name] = old[name]
                self.get_logger().warn('The sender\'s cabin of the current job was removed while the parcel is on '
                                       'board: finishing the job with the old spots')
            else:
                self._on_cancel(String(data=self.job['ref']))
        if self.last_spot not in self.locations:
            self.last_spot = self.home

    def _hold_estop(self):
        if self.estop:                   # zero speed above Nav2, the guard, gap assist and self-recovery
            self.stop_pub.publish(Twist())

    # ---------- parcel-box lock ----------
    def _on_localization(self, msg):
        text = msg.data
        if text == 'lost' and not self.lost:
            self.lost = True
            if self.state in (TO_PICKUP, TO_DROPOFF, RETURNING, WAIT_SHIPPED, WAIT_RECEIVED, RETURN_TO_SENDER,
                              WAIT_RETURNED):
                self.resume_target = self._current_target()
            self._abandon_goal()
            self._publish('Robot lost its position (moved or bumped?). Finding it again...')
        elif text in ('recovered', 'ok') and self.lost:     # 'ok' = a restarted self_recovery is localised
            self.lost = False
            if self.motor_fault:                           # wait for the motors as well (_on_safety_event goes on)
                self._publish('Position found again. Waiting for the motors to restart...')
                return
            target, self.resume_target = self.resume_target, None
            if target:
                self._publish('Position found again. Continuing to %s' % target)
                self._go(target)
            else:
                self._publish('Position found again.')
        elif text.startswith('help') and self.lost:
            self._publish('Robot needs help: %s' % text.split(':', 1)[-1].strip())

    def _on_safety_event(self, msg):
        """safety_watchdog (on the Pi): SAFETY STOP -> pause; RESTARTED -> go on (once the position is known too)."""
        text = msg.data
        if text.startswith('SAFETY STOP') and not self.motor_fault:
            self.motor_fault = True
            if self.resume_target is None and self.state in (TO_PICKUP, TO_DROPOFF, RETURNING, RETURN_TO_SENDER):
                self.resume_target = self._current_target()
            self._abandon_goal()
            self._publish('Motor fault: the motors were stopped and restart automatically (about 20 s)...')
        elif text.startswith(('RESTARTED', 'READY')) and self.motor_fault:     # READY = started again by hand
            self.motor_fault = False
            if self.lost:
                self._publish('Motors running again. Finding the position...')
                return
            target, self.resume_target = self.resume_target, None
            if target:
                self._publish('Motors running again. Continuing to %s' % target)
                self._go(target)
            else:
                self._publish('Motors running again.')
        elif text.startswith('GAVE UP'):
            self.motor_fault = True
            self._publish('Robot needs help: motor fault (%s). Check the battery and wiring.' % text.split(':', 1)[-1].strip())

    def _on_path_status(self, msg):
        kind = msg.data.split(':', 1)[0]
        previous, self.path_state = self.path_state, kind
        if (kind == previous and kind != 'detour') or self.state in (IDLE, WAIT_SHIPPED, WAIT_RECEIVED, WAIT_RETURNED):
            return
        target = self._current_target()
        if kind == 'around':
            self.get_logger().info('Going round something on the same route')
        elif kind == 'waiting':
            self._publish('Something is in the way. Waiting for it to move...')
        elif kind == 'rerouting':
            self._publish('The route is blocked. Looking for another way to %s...' % target)
        elif kind == 'detour':
            where = self._where(msg.data)
            if where is not None and any(math.hypot(where[0] - x, where[1] - y) < 0.8 for x, y in self.detour_spots):
                self.get_logger().info('Same block as before at (%.1f, %.1f): not counted as another route' % where)
                return                           # the same box seen again (e.g. while turning round)
            if where is not None:
                self.detour_spots.append(where)
            self.detours += 1
            if self.detours < self.get_parameter('max_detours').value:
                self._publish('The route is blocked. Taking another way to %s.' % target)
            else:                                # blocks on different routes: every way must be blocked
                self.get_logger().warn('%d blocked routes on this leg: treating every route as blocked' % self.detours)
                kind = 'blocked'
        if kind == 'blocked' and not self.path_blocked:
            self._abandon_goal()
            if self.state != RETURNING and self.rechecks >= self.get_parameter('blocked_rechecks').value:
                self._give_up(target, 'every route is blocked')     # no more checking: act now
                return
            self.path_blocked = True
            self.blocked_since = time.monotonic()
            self.want_goal = target              # continue to the same spot once the way is clear
            if self.state == RETURNING:
                self._publish('Every route to %s is blocked. Please clear the path; checking again every %.0f s.'
                              % (target, self.get_parameter('recheck_wait').value))
            else:
                self._publish('Every route to %s is blocked. Please clear the path. Checking again in %.0f s.'
                              % (target, self.get_parameter('recheck_wait').value))
        elif kind == 'clear' and self.path_blocked:
            self.path_blocked = False
            self.retries = 0
            self._publish('The way is clear again. Continuing to %s.' % target)
        elif kind == 'clear' and previous in ('waiting', 'rerouting'):
            self._publish('The way is free again. Continuing to %s.' % target)

    @staticmethod
    def _where(text):
        """(x, y) from a guard message like "detour: route blocked at (1.5, 2.4), ...", or None."""
        m = re.search(r'\((-?[0-9.]+), (-?[0-9.]+)\)', text)
        return (float(m.group(1)), float(m.group(2))) if m else None

    def _recheck_or_give_up(self):
        """Blocked for recheck_wait s: look at every route again, or give the leg up after blocked_rechecks checks."""
        target = self._current_target()
        self.path_blocked = False
        if self.rechecks < self.get_parameter('blocked_rechecks').value or self.state == RETURNING:
            self.rechecks += 1
            self.detours, self.detour_spots = 0, []
            self.forget_pub.publish(Empty())         # the guard forgets what it saw, so Nav2 tries every way again
            if self.clear_costmap.service_is_ready():
                self.clear_costmap.call_async(ClearEntireCostmap.Request())
            self.want_goal, self.retries = target, 0
            self.hold_until = time.monotonic() + 1.5     # let the costmap take the forgotten blocks first
            self._publish('Checking the routes to %s again (check %d)...' % (target, self.rechecks))
        else:
            self._give_up(target, 'every route is blocked')

    def _give_up(self, target, why):
        """A leg cannot be done: decide what happens to the job (see the module description)."""
        home = self.home
        if self.state == TO_PICKUP:
            failed = self.job['id']
            self.state = RETURNING
            self._publish('FAILED job %d: could not reach %s (%s). Returning to %s'
                          % (failed, target, why, home), result='failed')
            self._go(home)
        elif self.state == TO_DROPOFF and self.get_parameter('return_to_sender').value:
            self.state = RETURN_TO_SENDER
            self._publish('Could not reach %s (%s). Bringing the parcel back to %s.'
                          % (target, why, self.job['sender']))
            self._go(self.job['sender'])
        elif self.state in (TO_DROPOFF, RETURN_TO_SENDER):
            failed = self.job['id']
            self.state = RETURNING
            self._publish('FAILED job %d: could not reach %s (%s). Returning to %s with the parcel'
                          % (failed, target, why, home), result='failed')
            self._go(home)
        else:                                    # on the way home: never just stay somewhere (real test 2026-10-26:
            # wedged at a door frame, "Staying here"); wait and try again, like a blocked route home
            self.path_blocked, self.blocked_since = True, time.monotonic()
            self._publish('Could not reach %s (%s). Trying again in %.0f s.'
                          % (target, why, self.get_parameter('recheck_wait').value))

    def _on_open_lid(self, _msg):
        if not self.lid_buttons:
            self.get_logger().warn('Ignoring "open lid": lid_buttons is off')
            return
        if self.state not in (WAIT_SHIPPED, WAIT_RECEIVED, WAIT_RETURNED):
            self.get_logger().warn('Ignoring "open lid": state is %s' % self.state)
            return
        self.lid_opened = True
        self._cargo('unlock')

    def _on_cargo_state(self, msg):
        changed = msg.data != self.cargo_state
        self.cargo_state = msg.data
        # tell the website what the lid is doing while a person stands at the robot
        if changed and self.lid_buttons and self.state in (WAIT_SHIPPED, WAIT_RECEIVED, WAIT_RETURNED):
            self._publish(self.last_detail)

    def _cargo(self, cmd):
        """Ask the cargo controller to lock or unlock the box (no-op when the lock is not enabled)."""
        if not self.cargo_enabled:
            return
        self.cargo_state = 'pending'          # until the controller reports the real state
        if cmd == 'lock':
            self.lock_deadline = time.monotonic() + self.get_parameter('cargo_lock_timeout').value
        self.cargo_pub.publish(String(data=cmd))
        self.get_logger().info('Cargo box: %s' % cmd)

    def _cargo_ready(self, now):
        """True when it is safe to drive: the box is locked (or the lock is off / did not answer in time)."""
        if not self.cargo_enabled or self.cargo_state == 'locked':
            return True
        if now > self.lock_deadline and self.lid_buttons:
            # people open this lid, so never drive with it possibly open: say so and try again
            self._publish('The lid did not close (box: %s). Trying again; check the box.' % self.cargo_state)
            self._cargo('lock')
            return False
        if now > self.lock_deadline:
            self.get_logger().warn('Box lock not confirmed within %.0f s (state: %s): driving anyway'
                                   % (self.get_parameter('cargo_lock_timeout').value, self.cargo_state))
            return True
        return False

    # ---------- driving ----------
    def _go(self, name):
        self.want_goal = name
        self.leg_energy = None
        here = self._here()
        if self.battery and self.battery.get('wh') is not None and here is not None:
            self.leg_energy = (self.battery['wh'], here[0], here[1])
        self.retries = 0
        self.rechecks = 0
        self.detours, self.detour_spots = 0, []
        if self.cargo_enabled and self.cargo_state != 'locked':
            self._cargo('lock')               # always lock before leaving a person (and at the start of a job)

    def _abandon_goal(self):
        """Forget the current leg (used when the robot skips its trip home for a waiting job)."""
        self.want_goal = None
        if self.navigating:
            self.goal_gen += 1               # ignore the old goal's late callbacks
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.navigating = False
            self.goal_handle = None

    def _pose(self, name):
        p = self.locations[name]
        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(p['x'])
        msg.pose.position.y = float(p['y'])
        msg.pose.orientation.z = math.sin(float(p['yaw']) / 2.0)
        msg.pose.orientation.w = math.cos(float(p['yaw']) / 2.0)
        return msg

    def _send_goal(self, name):
        self.navigating = True
        self.goal_started = time.monotonic()
        if self.dry_run:
            self.dry_until = time.monotonic() + 3.0
            self.get_logger().info('(dry run) driving to %s' % name)
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(name)
        self.get_logger().info('Sending Nav2 goal: %s' % name)
        self.goal_gen += 1
        gen = self.goal_gen    # lets us ignore late callbacks of a goal we already gave up on
        self.client.send_goal_async(goal).add_done_callback(lambda f: self._on_goal_response(f, gen))

    def _on_goal_response(self, future, gen):
        handle = future.result()
        if gen != self.goal_gen:             # we already gave up on this goal
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.navigating = False
            self._nav_failed('goal rejected by Nav2')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self._on_result(f, gen))

    def _on_result(self, future, gen):
        if gen != self.goal_gen or not self.navigating:
            return
        self._finish_goal(future.result().status)

    def _finish_goal(self, status):
        self.navigating = False
        self.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            off = self._distance_to(self._current_target())
            if off is not None and off > self.get_parameter('arrival_check').value > 0:
                # seen 2026-10-23: Foxy reported a new goal reached at once while the robot was elsewhere
                self._nav_failed('Nav2 said "arrived" but the robot is %.2f m from %s' % (off, self._current_target()))
                return
            self._arrived()
        else:
            self._nav_failed('Nav2 finished with status %d' % status)

    def _distance_to(self, name):
        """Distance (m) from the robot's real map position to a spot, or None when unknown (dry run, no TF)."""
        if self.dry_run:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.frame, 'base_link', Time()).transform.translation
        except Exception:
            return None
        p = self.locations[name]
        return math.hypot(float(p['x']) - t.x, float(p['y']) - t.y)

    def _watch_goal(self, now):
        """Safety nets for a goal whose result never arrives."""
        h = self.goal_handle
        ended = (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED)
        if h is not None and h.status in ended and now - self.goal_started > 2.0:
            self.get_logger().warn('Nav2 reports status %d but the result message is missing: using it' % h.status)
            self.goal_gen += 1               # ignore the real result if it still shows up
            self._finish_goal(h.status)
        elif now - self.goal_started > self.get_parameter('nav_timeout').value:
            self.get_logger().warn('Leg to %s took longer than nav_timeout: cancelling it' % self._current_target())
            if h is not None:
                h.cancel_goal_async()
            self.goal_gen += 1
            self.navigating = False
            self.goal_handle = None
            self._nav_failed('leg timed out')

    def _nav_failed(self, reason):
        name = self._current_target()
        if self.path_blocked:                # the guard said so: wait for the way to clear, keep the retries
            self.want_goal = name
            return
        self.retries += 1
        if self.retries <= self.get_parameter('max_nav_retries').value:
            self.get_logger().warn('%s while going to %s: retry %d' % (reason, name, self.retries))
            self.want_goal = name
            return
        self._give_up(name, reason)

    def _current_target(self):
        if self.state in (TO_PICKUP, WAIT_SHIPPED, RETURN_TO_SENDER, WAIT_RETURNED):
            return self.job['sender']
        if self.state in (TO_DROPOFF, WAIT_RECEIVED):
            return self.job['destination']
        return self.home

    def _arrived(self):
        self.last_spot = self._current_target()
        self._learn_energy(self.last_spot)
        self.lid_opened = False
        if self.state == TO_PICKUP:
            self.state = WAIT_SHIPPED
            self.deadline = time.monotonic() + self.get_parameter('wait_shipped_timeout').value
            self._publish('At %s. Waiting for "OK shipped".' % self.job['sender'])
            if not self.lid_buttons:
                self._cargo('unlock')         # the sender can now put the parcel in
        elif self.state == TO_DROPOFF:
            self.state = WAIT_RECEIVED
            self.deadline = time.monotonic() + self.get_parameter('wait_received_timeout').value
            self._publish('At %s. Waiting for "received".' % self.job['destination'])
            if not self.lid_buttons:
                self._cargo('unlock')         # the receiver can now take it out
        elif self.state == RETURN_TO_SENDER:
            self.state = WAIT_RETURNED
            self.deadline = time.monotonic() + self.get_parameter('wait_received_timeout').value
            if self.job.get('cancelled'):
                self._publish('Back at %s with the parcel (delivery cancelled). Please take it out.' % self.job['sender'])
            else:
                self._publish('Back at %s with the parcel (could not deliver it to %s). Please take it out.'
                              % (self.job['sender'], self.job['destination']))
            if not self.lid_buttons:
                self._cargo('unlock')
        elif self.state == RETURNING:
            self.state = IDLE
            self.job = None
            self._publish('Back at %s%s' % (self.home, ' to charge.' if self.battery_hold else '. Ready.'))

    # ---------- main loop (twice a second) ----------
    def _tick(self):
        now = time.monotonic()

        # a job is waiting while the robot heads home: skip the trip home and start the job from here.
        # (at least 1 s after the last status, so the website surely got the previous job's result)
        if (self.state == RETURNING and self.queue and now - self.last_publish >= 1.0 and not self.estop
                and not self.battery_hold and not self.get_parameter('go_home_between_jobs').value):
            self._abandon_goal()
            self.state = IDLE
            self.job = None
            self.get_logger().info('Next job is waiting: not returning to %s' % self.home)

        # fake driving
        if self.dry_run and self.navigating and now >= self.dry_until:
            self.navigating = False
            self._arrived()

        if self.navigating and not self.dry_run:
            self._watch_goal(now)

        # send a wanted goal as soon as Nav2 is ready
        if self.want_goal and not self.navigating and not self.lost and not self.motor_fault and not self.estop \
                and not self.path_blocked \
                and now >= self.hold_until \
                and self._cargo_ready(now):
            if self.dry_run or self.client.server_is_ready():
                name, self.want_goal = self.want_goal, None
                self._send_goal(name)
            elif now - self.last_wait_log > 5.0:
                self.last_wait_log = now
                self.get_logger().warn('Waiting for the Nav2 navigate_to_pose action server...')

        # blocked with no way at all for a while: check the routes again, or give the leg up
        if self.path_blocked and now - self.blocked_since >= self.get_parameter('recheck_wait').value:
            self._recheck_or_give_up()

        # timeouts while waiting for people
        if self.state in (WAIT_SHIPPED, WAIT_RECEIVED, WAIT_RETURNED) and now > self.deadline:
            what = {WAIT_SHIPPED: '"shipped"', WAIT_RECEIVED: '"received"'}.get(self.state, '"parcel taken back"')
            self.state = RETURNING
            self._publish('Timed out waiting for %s (job %d). Returning to %s'
                          % (what, self.job['id'], self.home), result='failed')
            self._go(self.home)

        # start the next job (see _next_job), not in the same instant as the previous status
        if self.state == IDLE and self.queue and not self.lost and not self.motor_fault and not self.estop \
                and now - self.last_publish >= 1.0:
            job, why = self._next_job(now)
            if not self._battery_ok(job):
                return
            self.battery_hold, self.battery_msg = False, ''
            self.job = job
            self.queue.remove(self.job)
            self.state = TO_PICKUP
            self._publish('Starting %sjob %d: %s -> %s%s'
                          % ('URGENT ' if self.job['urgent'] else '', self.job['id'], self.job['sender'],
                             self.job['destination'], why))
            self._go(self.job['sender'])

    # ---------- battery ----------
    def _on_battery(self, msg):
        try:
            b = json.loads(msg.data)
        except ValueError:
            return
        if b.get('source') == 'sim' and not self.get_parameter('battery_trust_sim').value:
            return
        self.battery, self.battery_t = b, time.monotonic()
        if self.battery.get('state') == 'critical' and self.state == TO_PICKUP and self.job is not None:
            # no parcel on board yet: put the job back and go home to charge
            job, self.job = self.job, None
            self.queue.appendleft(job)
            self._abandon_goal()
            self.battery_hold = True
            self.state = RETURNING
            self._publish('Battery very low (%s %%): job %d goes back to the queue, going home to charge'
                          % (self.battery.get('pct'), job['id']), requeued_ref=job.get('ref'))
            self._go(self.home)

    def _here(self):
        """(x, y) of the robot on the map, or of the last spot reached when the live position is unknown."""
        if not self.dry_run:
            try:
                t = self.tf_buffer.lookup_transform(self.frame, 'base_link', Time()).transform.translation
                return t.x, t.y
            except Exception:
                pass
        p = self.locations[self.last_spot]
        return float(p['x']), float(p['y'])

    def _job_pct(self, job):
        """Battery % the job needs in the worst case: here -> sender -> receiver -> sender -> home."""
        pts = [self._here()] + [(float(self.locations[n]['x']), float(self.locations[n]['y']))
                                for n in (job['sender'], job['destination'], job['sender'], self.home)]
        metres = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        wh = metres * self.wh_per_m + self.get_parameter('wait_wh').value
        return 100.0 * wh / self.get_parameter('battery_wh').value, metres

    def _battery_ok(self, job):
        """True = the job may start. Otherwise the robot goes (or stays) home and says why."""
        if not self.battery_aware:
            return True
        b = self.battery
        if b is None or time.monotonic() - self.battery_t > 15.0 or b.get('error'):
            return True                      # no (real) battery reading: never block the deliveries
        if b.get('state') == 'off':
            msg = 'Motor power is switched off. Switch it on to start job %d.' % job['id']
        else:
            need, metres = self._job_pct(job)
            left = b['pct'] - need
            if left >= self.get_parameter('reserve_pct').value:
                return True
            msg = ('Battery %d %%: job %d (%s -> %s, about %.0f m with a possible return) needs about %.0f %%, '
                   'which would leave %.0f %%. Please charge the robot; the job starts by itself afterwards.'
                   % (b['pct'], job['id'], job['sender'], job['destination'], metres, need, max(left, 0)))
        self.battery_hold = True
        if self.state == IDLE and self.last_spot != self.home and not self.want_goal and not self.navigating:
            self.state = RETURNING
            self._publish('Not enough battery for the next job: going home to charge.')
            self._go(self.home)
            return False
        if msg != self.battery_msg:
            self.battery_msg = msg
            self._publish(msg)
        return False

    def _learn_energy(self, target):
        """Update wh_per_m from the leg just finished (energy used / straight-line distance)."""
        start, self.leg_energy = self.leg_energy, None
        if start is None or self.dry_run or not self.battery or self.battery.get('wh') is None:
            return
        p = self.locations[target]
        metres = math.hypot(float(p['x']) - start[1], float(p['y']) - start[2])
        used = self.battery['wh'] - start[0]
        if metres < 1.0 or used <= 0:
            return
        self.wh_per_m = min(1.0, max(0.03, 0.8 * self.wh_per_m + 0.2 * used / metres))
        try:
            os.makedirs(os.path.dirname(self.energy_path), exist_ok=True)
            with open(self.energy_path, 'w') as f:
                json.dump({'wh_per_m': round(self.wh_per_m, 4), 'source': self.battery.get('source'),
                           't': round(time.time())}, f)
        except OSError:
            pass
        self.get_logger().info('Leg to %s: %.2f Wh for %.1f m -> %.3f Wh per metre'
                               % (target, used, metres, self.wh_per_m))

    def _next_job(self, now):
        """Urgent jobs first (first come, first served among them); then any job waiting longer than max_wait
        (oldest first); otherwise the job whose sender is nearest to the robot (straight line)."""
        urgent = [j for j in self.queue if j.get('urgent')]
        if urgent:
            return urgent[0], ''
        waited = [j for j in self.queue if now - j['queued_at'] > self.get_parameter('max_wait').value]
        if waited:
            return waited[0], ' (waiting longest)'
        if len(self.queue) == 1:
            return self.queue[0], ''
        dist = {j['id']: self._distance_to(j['sender']) for j in self.queue}
        if any(d is None for d in dist.values()):          # live position unknown (dry run): from the last spot
            here = self.locations[self.last_spot]
            dist = {j['id']: math.hypot(float(self.locations[j['sender']]['x']) - float(here['x']),
                                        float(self.locations[j['sender']]['y']) - float(here['y']))
                    for j in self.queue}
        job = min(self.queue, key=lambda j: dist[j['id']])
        return job, (' (nearest sender, %.1f m away)' % dist[job['id']]) if job is not self.queue[0] else ''

    def stop(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.client.destroy()   # before the node, or Foxy prints a harmless InvalidHandle error


def main():
    rclpy.init()
    node = DeliveryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
