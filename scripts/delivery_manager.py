#!/usr/bin/env python3
"""Delivery manager: runs delivery jobs one at a time with Nav2.

A job is "sender,destination" (names from config/locations.yaml). For each job the robot:
  home -> sender -> (waits for "shipped") -> destination -> (waits for "received") -> home
If another job is already waiting when one ends, the robot does NOT go home first: it drives straight
from where it is to the next sender (also when a new job arrives while it is on its way home).
Set go_home_between_jobs:=true for the old behaviour (always home first), e.g. to compare the two.

Topics (all plain ROS 2, so a website bridge can use them later):
  in : /delivery_request   std_msgs/String  "cabin_A,cabin_D"  (optional 3rd field = website id: "cabin_A,cabin_D,-Nabc")
  in : /delivery_shipped   std_msgs/Empty   sender has loaded the parcel
  in : /delivery_received  std_msgs/Empty   receiver has taken the parcel
  in : /delivery_open_lid  std_msgs/Empty   the person at the robot wants the lid open (lid_buttons mode)
  in : /delivery_returned  std_msgs/Empty   the sender has taken an undeliverable parcel back out
  out: /delivery_status    std_msgs/String  JSON: state, job, sender, destination, detail, queue, cargo, lid_buttons

Parcel box (needs cargo_controller.py running):
  -p cargo_lock:=true                       box opens by itself on arrival, closes before every drive
  -p cargo_lock:=true -p lid_buttons:=true  box opens only when the person presses "Open lid" on the website;
                                            "shipped"/"received" close it, and the robot never drives until the
                                            box reports "locked" (it keeps retrying and says so instead)

Self-recovery (needs scripts/self_recovery.py running):  -p self_recovery:=true
  on /localization_status "lost" the job is paused (Nav2 goal cancelled); on "recovered" the robot continues to the
  same spot (or drives back to the cabin it was waiting at, if someone moved it); "help: ..." is shown on the website

Blocked routes (needs scripts/path_guard.py running):  -p path_guard:=true
  /path_status "waiting"/"rerouting"/"detour" are shown on the website. "blocked" (no way at all), or `max_detours`
  route switches on one leg (the other way was blocked too), ends the leg at once:
    on the way to the sender                 -> job failed, the robot goes home
    on the way to the receiver (parcel)      -> it brings the parcel back to the sender (RETURN_TO_SENDER), waits
                                                there for "returned" (WAIT_RETURNED; website: "Parcel taken back")
    on the way back to the sender (parcel)   -> job failed, it goes home with the parcel
    on the way home                          -> it stops and asks for the path to be cleared; continues once clear
  (Optional: -p blocked_rechecks:=N waits `recheck_wait` s and looks at every route again N times before giving up;
  0 by default, the user found re-checking too slow on 2026-10-24.)
  The same giving-up rules apply when Nav2 fails a leg `max_nav_retries` times for any other reason.

Run (sim, Nav2 already running):
  python3 src/my_bot/scripts/delivery_manager.py --ros-args -p use_sim_time:=true
Test without Nav2:
  python3 src/my_bot/scripts/delivery_manager.py --ros-args -p dry_run:=true
"""
import json
import math
import os
import time
from collections import deque

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
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
        self.declare_parameter('arrival_check', 0.5)          # metres; Nav2 "arrived" is only believed if the robot
                                                              # is really this close to the spot (0 = do not check)

        path = os.path.normpath(self.get_parameter('locations_file').value)
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
        self.path_blocked = False        # path_guard says the route is blocked with no other way: wait
        self.path_state = 'clear'
        self.blocked_since = 0.0
        self.rechecks = 0                # route checks already done on this leg
        self.detours = 0                 # route switches since the leg (or the last check) started
        self.hold_until = 0.0            # no new goal before this time
        if self.get_parameter('path_guard').value:
            self.create_subscription(String, '/path_status', self._on_path_status, status_qos)
            self.forget_pub = self.create_publisher(Empty, '/path_guard/forget', 10)
            self.clear_costmap = self.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
            self.get_logger().info('Path guard ENABLED (needs path_guard.py running)')

        self.state = IDLE
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
    def _publish(self, detail, result=None, rejected_ref=None):
        # ref = the website's id for the CURRENT job; result = 'delivered' or 'failed' when a job ends;
        # rejected_ref = the id of a request that was refused (so the website can show why)
        msg = {'state': self.state,
               'job': self.job['id'] if self.job else None,
               'ref': self.job.get('ref') if self.job else None,
               'sender': self.job['sender'] if self.job else None,
               'destination': self.job['destination'] if self.job else None,
               'detail': detail, 'queue': len(self.queue),
               'result': result, 'rejected_ref': rejected_ref,
               'cargo': self.cargo_state if self.cargo_enabled else None, 'lid_buttons': self.lid_buttons}
        self.status_pub.publish(String(data=json.dumps(msg)))
        self.last_publish = time.monotonic()
        if result is None and rejected_ref is None:
            self.last_detail = detail
        self.get_logger().info('[%s] %s' % (self.state, detail))

    # ---------- incoming messages ----------
    def _on_request(self, msg):
        parts = [p.strip() for p in msg.data.split(',')]
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
        job = {'id': self.next_id, 'sender': parts[0], 'destination': parts[1], 'ref': ref}
        self.next_id += 1
        self.queue.append(job)
        self._publish('Job %d queued: %s -> %s (%d waiting)'
                      % (job['id'], job['sender'], job['destination'], len(self.queue)))

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
        if self.queue and not self.get_parameter('go_home_between_jobs').value:
            self._publish('Parcel of job %d returned to %s. Going straight to the next job (%d waiting)'
                          % (self.job['id'], self.job['sender'], len(self.queue)), result='returned')
        else:
            self._publish('Parcel of job %d returned to %s. Returning to %s'
                          % (self.job['id'], self.job['sender'], self.home), result='returned')
            self._go(self.home)

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
            target, self.resume_target = self.resume_target, None
            if target:
                self._publish('Position found again. Continuing to %s' % target)
                self._go(target)
            else:
                self._publish('Position found again.')
        elif text.startswith('help') and self.lost:
            self._publish('Robot needs help: %s' % text.split(':', 1)[-1].strip())

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
            self.detours += 1
            if self.detours < self.get_parameter('max_detours').value:
                self._publish('The route is blocked. Taking another way to %s.' % target)
            else:                                # switched back and forth: every way must be blocked
                self.get_logger().warn('%d route switches on this leg: treating every route as blocked' % self.detours)
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
                self._publish('Every route to %s is blocked. Please clear the path; the robot continues as soon '
                              'as it is clear.' % target)
            else:
                self._publish('Every route to %s is blocked. Please clear the path. Checking again in %.0f s.'
                              % (target, self.get_parameter('recheck_wait').value))
        elif kind == 'clear' and self.path_blocked:
            self.path_blocked = False
            self.retries = 0
            self._publish('The way is clear again. Continuing to %s.' % target)
        elif kind == 'clear' and previous in ('waiting', 'rerouting'):
            self._publish('The way is free again. Continuing to %s.' % target)

    def _recheck_or_give_up(self):
        """Blocked for recheck_wait s: look at every route again, or give the leg up after blocked_rechecks checks."""
        target = self._current_target()
        self.path_blocked = False
        if self.rechecks < self.get_parameter('blocked_rechecks').value:
            self.rechecks += 1
            self.detours = 0
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
        else:                                    # on the way home
            self.state = IDLE
            self.job = None
            self._publish('FAILED to reach %s (%s). Staying here.' % (target, why))

    def _on_open_lid(self, _msg):
        if not self.lid_buttons:
            self.get_logger().warn('Ignoring "open lid": lid_buttons is off')
            return
        if self.state not in (WAIT_SHIPPED, WAIT_RECEIVED, WAIT_RETURNED):
            self.get_logger().warn('Ignoring "open lid": state is %s' % self.state)
            return
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
        self.retries = 0
        self.rechecks = 0
        self.detours = 0
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
            self._publish('Back at %s with the parcel (could not deliver it to %s). Please take it out.'
                          % (self.job['sender'], self.job['destination']))
            if not self.lid_buttons:
                self._cargo('unlock')
        elif self.state == RETURNING:
            self.state = IDLE
            self.job = None
            self._publish('Back at %s. Ready.' % self.home)

    # ---------- main loop (twice a second) ----------
    def _tick(self):
        now = time.monotonic()

        # a job is waiting while the robot heads home: skip the trip home and start the job from here.
        # (at least 1 s after the last status, so the website surely got the previous job's result)
        if (self.state == RETURNING and self.queue and now - self.last_publish >= 1.0
                and not self.get_parameter('go_home_between_jobs').value):
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
        if self.want_goal and not self.navigating and not self.lost and not self.path_blocked and now >= self.hold_until \
                and self._cargo_ready(now):
            if self.dry_run or self.client.server_is_ready():
                name, self.want_goal = self.want_goal, None
                self._send_goal(name)
            elif now - self.last_wait_log > 5.0:
                self.last_wait_log = now
                self.get_logger().warn('Waiting for the Nav2 navigate_to_pose action server...')

        # blocked with no way at all for a while: check the routes again, or give the leg up
        if self.path_blocked and self.state != RETURNING and \
                now - self.blocked_since >= self.get_parameter('recheck_wait').value:
            self._recheck_or_give_up()

        # timeouts while waiting for people
        if self.state in (WAIT_SHIPPED, WAIT_RECEIVED, WAIT_RETURNED) and now > self.deadline:
            what = {WAIT_SHIPPED: '"shipped"', WAIT_RECEIVED: '"received"'}.get(self.state, '"parcel taken back"')
            self.state = RETURNING
            self._publish('Timed out waiting for %s (job %d). Returning to %s'
                          % (what, self.job['id'], self.home), result='failed')
            self._go(self.home)

        # start the next job (first come, first served), not in the same instant as the previous status
        if self.state == IDLE and self.queue and not self.lost and now - self.last_publish >= 1.0:
            self.job = self.queue.popleft()
            self.state = TO_PICKUP
            self._publish('Starting job %d: %s -> %s'
                          % (self.job['id'], self.job['sender'], self.job['destination']))
            self._go(self.job['sender'])

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
