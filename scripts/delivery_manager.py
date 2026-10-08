#!/usr/bin/env python3
"""Delivery manager: runs delivery jobs one at a time with Nav2.

A job is "sender,destination" (names from config/locations.yaml). For each job the robot:
  home -> sender -> (waits for "shipped") -> destination -> (waits for "received") -> home

Topics (all plain ROS 2, so a website bridge can use them later):
  in : /delivery_request   std_msgs/String  "cabin_A,cabin_D"
  in : /delivery_shipped   std_msgs/Empty   sender has loaded the parcel
  in : /delivery_received  std_msgs/Empty   receiver has taken the parcel
  out: /delivery_status    std_msgs/String  JSON: state, job, sender, destination, detail, queue

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
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Empty, String

# States of the state machine
IDLE = 'IDLE'
TO_PICKUP = 'TO_PICKUP'
WAIT_SHIPPED = 'WAIT_SHIPPED'
TO_DROPOFF = 'TO_DROPOFF'
WAIT_RECEIVED = 'WAIT_RECEIVED'
RETURNING = 'RETURNING'


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
        self.create_subscription(String, '/delivery_request', self._on_request, 10)
        self.create_subscription(Empty, '/delivery_shipped', self._on_shipped, 10)
        self.create_subscription(Empty, '/delivery_received', self._on_received, 10)
        # transient local: a late subscriber (e.g. the website bridge) still gets the last status
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/delivery_status', status_qos)

        self.state = IDLE
        self.queue = deque()
        self.job = None
        self.next_id = 1
        self.want_goal = None      # location name we still have to send to Nav2
        self.navigating = False    # a goal is in flight
        self.goal_handle = None
        self.retries = 0
        self.deadline = None
        self.dry_until = 0.0
        self.last_wait_log = 0.0

        self.create_timer(0.5, self._tick)
        self._publish('Ready at %s. Waiting for requests.' % self.home)

    # ---------- status ----------
    def _publish(self, detail):
        msg = {'state': self.state,
               'job': self.job['id'] if self.job else None,
               'sender': self.job['sender'] if self.job else None,
               'destination': self.job['destination'] if self.job else None,
               'detail': detail, 'queue': len(self.queue)}
        self.status_pub.publish(String(data=json.dumps(msg)))
        self.get_logger().info('[%s] %s' % (self.state, detail))

    # ---------- incoming messages ----------
    def _on_request(self, msg):
        parts = [p.strip() for p in msg.data.split(',')]
        if len(parts) != 2 or parts[0] not in self.locations or parts[1] not in self.locations:
            self._publish("Rejected request '%s': use 'sender,destination' with names from %s"
                          % (msg.data, ', '.join(self.locations)))
            return
        if parts[0] == parts[1] or self.home in parts:
            self._publish("Rejected request '%s': sender and destination must differ and not be %s"
                          % (msg.data, self.home))
            return
        job = {'id': self.next_id, 'sender': parts[0], 'destination': parts[1]}
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
        self._publish('Delivered job %d. Returning to %s' % (self.job['id'], self.home))
        self._go(self.home)

    # ---------- driving ----------
    def _go(self, name):
        self.want_goal = name
        self.retries = 0

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
        if self.dry_run:
            self.dry_until = time.monotonic() + 3.0
            self.get_logger().info('(dry run) driving to %s' % name)
            return
        goal = NavigateToPose.Goal()
        goal.pose = self._pose(name)
        self.get_logger().info('Sending Nav2 goal: %s' % name)
        self.client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.navigating = False
            self._nav_failed('goal rejected by Nav2')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        status = future.result().status
        self.navigating = False
        self.goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._arrived()
        else:
            self._nav_failed('Nav2 finished with status %d' % status)

    def _nav_failed(self, reason):
        name = self._current_target()
        self.retries += 1
        if self.retries <= self.get_parameter('max_nav_retries').value:
            self.get_logger().warn('%s while going to %s: retry %d' % (reason, name, self.retries))
            self.want_goal = name
            return
        if self.state == RETURNING:
            self.state = IDLE
            self.job = None
            self._publish('FAILED to reach %s (%s). Staying here.' % (name, reason))
            return
        failed = self.job['id']
        self.state = RETURNING
        self._publish('FAILED job %d: could not reach %s (%s). Returning to %s'
                      % (failed, name, reason, self.home))
        self._go(self.home)

    def _current_target(self):
        if self.state == TO_PICKUP:
            return self.job['sender']
        if self.state == TO_DROPOFF:
            return self.job['destination']
        return self.home

    def _arrived(self):
        if self.state == TO_PICKUP:
            self.state = WAIT_SHIPPED
            self.deadline = time.monotonic() + self.get_parameter('wait_shipped_timeout').value
            self._publish('At %s. Waiting for "OK shipped".' % self.job['sender'])
        elif self.state == TO_DROPOFF:
            self.state = WAIT_RECEIVED
            self.deadline = time.monotonic() + self.get_parameter('wait_received_timeout').value
            self._publish('At %s. Waiting for "received".' % self.job['destination'])
        elif self.state == RETURNING:
            self.state = IDLE
            self.job = None
            self._publish('Back at %s. Ready.' % self.home)

    # ---------- main loop (twice a second) ----------
    def _tick(self):
        now = time.monotonic()

        # fake driving
        if self.dry_run and self.navigating and now >= self.dry_until:
            self.navigating = False
            self._arrived()

        # send a wanted goal as soon as Nav2 is ready
        if self.want_goal and not self.navigating:
            if self.dry_run or self.client.server_is_ready():
                name, self.want_goal = self.want_goal, None
                self._send_goal(name)
            elif now - self.last_wait_log > 5.0:
                self.last_wait_log = now
                self.get_logger().warn('Waiting for the Nav2 navigate_to_pose action server...')

        # timeouts while waiting for people
        if self.state in (WAIT_SHIPPED, WAIT_RECEIVED) and now > self.deadline:
            what = '"shipped"' if self.state == WAIT_SHIPPED else '"received"'
            self.state = RETURNING
            self._publish('Timed out waiting for %s (job %d). Returning to %s'
                          % (what, self.job['id'], self.home))
            self._go(self.home)

        # start the next job (first come, first served)
        if self.state == IDLE and self.queue:
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
