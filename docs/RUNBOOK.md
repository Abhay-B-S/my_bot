# my_bot runbook: delivery robot (ROS 2 Foxy)

Run the delivery system. For plain Nav2 / SLAM / mapping (no delivery) use `RUNBOOK_NAV2_SLAM.md`.

Flow: `home -> sender -> (sender presses "OK shipped") -> destination -> (receiver presses "received") -> home`.
Status: full cycle tested OK in the simulation on 2026-10-08 (cabin_A -> cabin_D, legs took 11 s, 21 s, 9 s; steps 1-12 below as written).
Not yet tested: two queued jobs on the real Nav2, a failing leg, the real robot. Firebase: not built.

One terminal per command. Every terminal starts with:

```
cd ~/dev_ws && source /opt/ros/foxy/setup.bash && source install/setup.bash
```

(A terminal without it says `Package 'my_bot' not found`.)

## Rules
- Never run sim and real robot together; never run slam_toolbox and AMCL together (see `RUNBOOK_NAV2_SLAM.md`).
- Close Firefox / WhatsApp web first; laptop overloads easily (lost `/odom`, stalled sim, Nav2 crashes).
- Start the sim, close its window, wait 30 s, check `/odom`, THEN start the rest.
- Stop ROS commands with Ctrl-C, not `timeout`/kill -9. If everything goes silent: stop all, `rm -f /dev/shm/fastrtps_*`, restart.
- Teach AMCL the start pose accurately (laser arcs on the obstacle edges) before sending any delivery.

## A. Simulation, step by step
1. Clean start: `killall -q gzserver gzclient rviz2` then `rm -f /dev/shm/fastrtps_*` (only if nothing ROS is running).
2. Sim (wait for `Configured and started diff_cont`):
   `ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/obstacles.world`
3. `killall gzclient` (closes the Gazebo window), wait 30 s.
4. Check: `ros2 topic hz /odom` should show about 29 Hz; Ctrl-C.
5. RViz (untick **Camera** at once):
   `ros2 run rviz2 rviz2 -d src/my_bot/config/main.rviz --ros-args -p use_sim_time:=true`
6. Localisation: `ros2 launch my_bot localization_launch.py map:=./my_map_save.yaml use_sim_time:=true`
   RViz: Fixed Frame `map`, **2D Pose Estimate** on the robot (untick/re-tick Map if it is not drawn). Check laser arcs sit on the pillars.
7. Nav2: `ros2 launch my_bot navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true`
8. Delivery manager (must NOT say `[DRY RUN]`):
   `python3 src/my_bot/scripts/delivery_manager.py --ros-args -p use_sim_time:=true`
9. Status viewer (start it AFTER the manager, or it cannot find the topic):
   `ros2 topic echo /delivery_status`
10. Send a request (`sender,destination`):
    `ros2 topic pub --once /delivery_request std_msgs/String "{data: 'cabin_A,cabin_D'}"`
11. When the manager prints `Waiting for "OK shipped"` (the sender's button):
    `ros2 topic pub -t 5 -r 2 /delivery_shipped std_msgs/Empty "{}"`
12. When it prints `Waiting for "received"` (the receiver's button):
    `ros2 topic pub -t 5 -r 2 /delivery_received std_msgs/Empty "{}"`
    The robot then drives home.

Step 10 uses `--once`, which also gets lost sometimes (2 of 3 `cabin_B,cabin_C` requests vanished on 2026-10-08): always check the manager prints `Job N queued`, and send again if it does not. A long-running publisher (the future Firebase bridge) does not have this problem.
Queue tested OK: a request sent while a job is running waits, and starts by itself after the robot is back home.

Steps 11/12 use repeated sends (5 times, 2 per second) because single `--once` messages were lost once on the loaded laptop (extra copies are ignored by the manager). Typing the words "OK shipped" in a terminal does nothing.

## B. Teleop / recording positions (optional)
- Teleop (slow it down with `x`/`c`, `k` stops):
  `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_key`
- Save the robot's current map pose as a named spot (robot stopped, localisation running):
  `python3 src/my_bot/scripts/record_location.py cabin_A` (overwrites an entry of the same name)
- Saved spots: `src/my_bot/config/locations.yaml` (map frame: x, y in metres, yaw in radians). Keep spots at least 0.45 m from obstacles.
  These are SIMULATION positions (sim map `my_map_save.yaml`). The real staffroom needs its own map and its own recording.

## C. Testing the manager without Nav2 or the sim
`python3 src/my_bot/scripts/delivery_manager.py --ros-args -p dry_run:=true` fakes each trip with a 3 s wait.
Same topics as above. Tested OK: queue order, rejected requests, shipped/received handling, return home.

## Manager facts (`scripts/delivery_manager.py`)
- In: `/delivery_request` (String `a,b`), `/delivery_shipped`, `/delivery_received` (Empty). Out: `/delivery_status` (String, JSON; latched).
- Jobs run one at a time, first come first served. Unknown names, sender = destination, or home as an endpoint are rejected (reason in status).
- A failed Nav2 leg is retried twice (`max_nav_retries`); if it still fails the job is dropped and the robot returns home.
- Each wait for a person times out after 600 s (`wait_shipped_timeout`, `wait_received_timeout`), then the robot returns home.
- Parameters: `locations_file`, `home_name`, `dry_run`, `max_nav_retries`, the two timeouts (set with `-p name:=value`).

## Shutdown
`k` in teleop, then Ctrl-C in this order: delivery manager, status viewer, Nav2, localisation, teleop, RViz, sim.
Then `killall -q gzserver gzclient rviz2`, check `ps aux | grep -E "ros2|gzserver|rviz2|amcl" | grep -v grep` is empty, `rm -f /dev/shm/fastrtps_*`.

## Not committed yet (by decision, 2026-10-08)
`docs/`, `scripts/`, `config/locations.yaml` are uncommitted on purpose. Commit milestones: (1) simulation delivery works end to end, (2) Firebase linked in simulation, (3) real robot.
