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
- Stop ROS commands with Ctrl-C, not `timeout`/kill -9. If everything goes silent: stop all, `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`, restart.
- Teach AMCL the start pose accurately (laser arcs on the obstacle edges) before sending any delivery.

## A. Simulation, step by step
1. Clean start: `killall -q gzserver gzclient rviz2` then `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` (only if nothing ROS is running).
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

## A2. Website (Firebase) on top of the simulation  (tested OK 2026-10-08)
Replaces steps 9 to 12 above (no `ros2 topic pub` buttons). Needs internet. Firebase project `delivery-robot-demo`, Realtime Database in test mode (open; 30 days from 2026-10-08, then the rules must be updated).
After steps 1 to 8 (sim ... delivery manager, with the REAL locations file), in order:
1. Bridge (only after the manager says `Ready at home`):
   `python3 src/my_bot/scripts/firebase_bridge.py`
   Prints `Bridge started` and, for each website request, `Request <id> forwarded`. Buttons are forwarded only while the manager waits for that very request.
2. Web server for a local preview (own terminal): `cd ~/dev_ws/src/my_bot/web && python3 -m http.server 8000`, then open `localhost:8000` in Firefox.
3. In the page: pick "Who are you?" and "Send to", press **Request delivery**. When the robot reaches the sender the card shows **OK shipped**; at the receiver (switch "Who are you?" to the receiver, or use another phone) it shows **Received**. The robot card turns green while the bridge is alive.
Notes: names are edited in the `TEACHERS` list at the top of `web/index.html`. A request left `pending` in the database is picked up when the bridge starts. On bridge start, half-finished requests are marked failed. Requests are never forwarded twice (the manager remembers website ids). Hosting for phones on cell data: not done yet (`firebase.json`, `.firebaserc` are ready; deploy with `npx firebase-tools` from `src/my_bot`).
Observed: several Nav2 goals abort once or twice (status 6) and succeed on a retry; cause not investigated.

## A3. Website login (Firebase Authentication)  (set up and tested 2026-10-08)
Who: `abhaybs` (cabin A), `darshanraju` (B), `khushal` (C), `abhisheksj` (D), plus `robot` (the bridge). Login emails are `<id>@example.com` (fake; Firebase rejects `.demo`). The page accepts the short ID or the full address.
- Accounts: Firebase console > Authentication > Users > Add user (password >= 6 characters). Create the `robot` account BEFORE deploying the rules, or anyone could register that name.
- People list: `members.json` (id -> cabin/name) and the `TEACHERS` list in `web/index.html` must match. Import: `npx firebase-tools database:set /members members.json --project delivery-robot-demo --force`.
- Rules (`database.rules.json`): only members read; a person creates requests only from their own cabin and presses OK shipped / Received only as that request's sender / receiver at the right status; only the robot account writes statuses. Deploy: `npx firebase-tools deploy --only database`. Tested live with 34 break-in attempts, all refused as intended.
- Page: `npx firebase-tools deploy --only hosting` from `src/my_bot`, live at https://delivery-robot-demo.web.app (browsers may cache it: reload or reopen the tab).
- Robot password (never in git): `read -s -p "Robot password: " p; echo "$p" > ~/.delivery_robot_password; chmod 600 ~/.delivery_robot_password; unset p; echo`. The bridge reads it at start and refuses to run if it is wrong or missing.
- Forgotten password: console cannot show it; delete the user and add it again.
- FALLBACK to the old open database (no login): `npx firebase-tools deploy --only database` with `database.rules.open.json` copied over `database.rules.json`, run the bridge with `--ros-args -p use_auth:=false`, and use the old page from git history (commit 0d23485).

## A4. When the laser arcs stop matching (AMCL lost the robot)  (simulation)
Symptom: red arcs far from the obstacles, Nav2 says "failed to generate a valid path" / "Trajectory Hits Obstacle", deliveries stop. It does NOT fix itself and needs no restart:
`python3 src/my_bot/scripts/sim_set_pose_from_gazebo.py` reads the robot's true pose from Gazebo and sets AMCL to it (sim only). On the real robot click 2D Pose Estimate instead.
Seen on 2026-10-08: AMCL was 3.7 m off. The sim room has only 8 small pillars and no walls, so the laser has little to lock on to; the real staffroom with walls should be better. Not yet done: enable AMCL recovery particles (`recovery_alpha_fast/slow` are 0).

## A5. REAL ROBOT delivery  (tested 2026-10-08 in a practice house: full delivery from two phones worked)
Stop the simulation first (same ROS domain 0). Laptop and Pi on the SAME hotspot (Pi address: see RUNBOOK_NAV2_SLAM.md B). Every laptop terminal starts with the usual first line (`cd ~/dev_ws && source /opt/ros/foxy/setup.bash && source install/setup.bash`).
Pi (two `ssh ubuntu@<pi-ip>` terminals, same commands as RUNBOOK_NAV2_SLAM.md B.1 and B.2): `launch_robot.launch.py`, then `ydlidar.launch.py`. Check from the laptop: `/scan` about 11 Hz, `/odom` 30 Hz.
Laptop, in this order:
1. RViz without sim time: `ros2 run rviz2 rviz2 -d src/my_bot/config/main.rviz` (untick Camera).
2. twist_mux by hand (not installed on the Pi) and teleop (press `x` about 10 times first, speed about 0.15, `c` a few times for turning): commands in RUNBOOK_NAV2_SLAM.md B.
3. ONE TIME per place: map it with slam_toolbox (RUNBOOK_NAV2_SLAM.md B.5: mapping launch, drive slowly along the walls, return to the start, save_map + serialize_map to `maps/<name>`). Start mapping with the robot on the spot that will be `home`. Check there are no doubled walls.
4. Localisation: `ros2 launch my_bot localization_launch.py map:=./maps/<name>.yaml use_sim_time:=false`; RViz Fixed Frame `map`; 2D Pose Estimate at the start pose (map origin); the red scan must sit on the black walls.
5. ONE TIME per place: record the spots into their OWN file, robot stopped and scan matching:
   `python3 src/my_bot/scripts/record_location.py cabin_A --file src/my_bot/config/locations_real.yaml` (also `home`, `cabin_B`, `cabin_C`, `cabin_D`). Then check them:
   `python3 src/my_bot/scripts/check_locations.py maps/<name>.yaml src/my_bot/config/locations_real.yaml`
   Aim for clearance >= 0.45 m from walls (0.25 m was too close in the first try) and every spot reaching every other.
6. Nav2: `ros2 launch my_bot navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true`. First test: one short 2D Goal Pose with a hand on the power switch.
7. Delivery manager with the REAL spots: `python3 src/my_bot/scripts/delivery_manager.py --ros-args -p locations_file:=./src/my_bot/config/locations_real.yaml`. Test once from the command line (A.10 to A.12 above) before using the phones.
8. Bridge (needs internet through the hotspot): `python3 src/my_bot/scripts/firebase_bridge.py`. Phones: https://delivery-robot-demo.web.app (see A3 for logins).
Quick stops: power switch; or Ctrl-C in the Nav2 terminal (the robot stops within about 0.5 s).
The practice-house spots (`config/locations_real.yaml`) and map (`maps/practice_room.*`) are NOT in git; the real staffroom needs its own map and spots. Not yet done: a single start script for all of this.

## A5b. REAL ROBOT with the new delivery body  (tested 2026-10-23 in a friend's room: deliveries from phones, incl. two requests in a row)
**Never drive backward (`,` in teleop).** Stopping after reversing (sometimes also after turning) can restart the motor Arduino; the motors then run on their own. Nav2 never reverses in this setup. Keep the motor battery switch within reach. Open hardware fix: secure the Dupont jumpers, 10 kOhm pull-downs on IN1-IN4/ENA/ENB, 1000 uF on the 12 V input, 100 nF across each motor.

Pi (one command; survives a dropped SSH link): lidar `ros2 launch my_bot ydlidar.launch.py` (as in B.2), then `~/start_robot.sh` = launch_robot + **safety watchdog** (`scripts/safety_watchdog.py`, copied to the Pi home). The watchdog ends the motor controller and sends a stop to the Arduino when a wheel reading jumps (> 30 rad/s = Arduino restart) or a drive command gets no wheel movement for 3 s; then restart `~/start_robot.sh`. Its log: `~/watchdog.log`.

Laptop, in this order (same map and spots everywhere):
1. `ros2 run twist_mux twist_mux --ros-args --params-file ./src/my_bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped`
2. Mapping a new room: `ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot/config/mapper_params_mapping.yaml use_sim_time:=false`, drive slowly (teleop on `/cmd_vel_key`, `z` until speed ~0.14), then save with the two service calls in RUNBOOK_NAV2_SLAM.md B.5 and stop slam.
3. Localisation: `ros2 launch my_bot localization_launch.py map:=./maps/<room>.yaml use_sim_time:=false`, RViz 2D Pose Estimate.
4. Spots: `python3 src/my_bot/scripts/record_location.py home --file src/my_bot/config/locations_<room>.yaml` (and cabin_A..D), then `python3 src/my_bot/scripts/check_locations.py maps/<room>.yaml src/my_bot/config/locations_<room>.yaml` (needs >= 0.45 m clearance: the back end swings 0.29 m around the axle).
5. Nav2: `ros2 launch my_bot navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true` (wait for "Managed nodes are active"; if it hangs at "Configuring controller_server", stop it and start again).
6. Manager: `python3 src/my_bot/scripts/delivery_manager.py --ros-args -p locations_file:=./src/my_bot/config/locations_<room>.yaml`
7. Bridge with the live map: `python3 src/my_bot/scripts/firebase_bridge.py --ros-args -p map_yaml:=./maps/<room>.yaml -p locations_file:=./src/my_bot/config/locations_<room>.yaml`
Check that the request reaches the manager: `ros2 topic info /delivery_request` must show 1 publisher and 1 subscription (once the manager started but never connected; restarting manager and bridge fixed it).

Fixed 2026-10-23: Foxy's goal checker with `stateful: True` declared a new goal reached at once (the robot "arrived home" while still at cabin C); now `stateful: False`, and the manager only believes "arrived" when the robot is within `arrival_check` (0.5 m) of the spot, else it retries.

## A6. Cargo box: lid servo + lock servo (written 2026-10-20, hardware not yet on the robot)
Parts: an Arduino Nano running `firmware/servo_nano/servo_nano.ino` (upload from the Arduino IDE; Board Arduino Nano, Processor ATmega328P, Port /dev/ttyUSBx; if the IDE says permission denied, start it with `sg dialout -c ~/Downloads/arduino-1.8.19/arduino` or log out and in after `sudo usermod -aG dialout $USER`). Servo 0 (pin D9) = lid, servo 1 (pin D10) = lock. Serial 57600, commands `s <n> <angle>`, `g <n>`, `d <n>`, `?`. Power the servos from their OWN 5 V supply (2 A or more, a 470 uF capacitor across it), with its GND joined to the Nano's GND; only the Nano's USB goes to the Pi.
Pi side: `scripts/cargo_controller.py` (topics `/cargo_cmd` = `lock` / `unlock`, `/cargo_state` = `locked` / `unlocked` / `moving` / `error: ...`). Angles live in `config/cargo.yaml` (placeholders until calibrated). The delivery manager uses it with `-p cargo_lock:=true` (off by default): unlock on arrival at a person, lock (and wait for the confirmation, 10 s at most) before every drive.

**Lid buttons on the website** (2026-10-23): add `-p lid_buttons:=true` (together with `cargo_lock:=true`). The box then stays shut on arrival; the person taps **Open lid**, loads or unloads, and taps **Close lid & send** (sender) or **Close lid & done** (receiver). The page shows the lid state (moving / open / closed / problem). In this mode the robot NEVER drives until the box reports `locked`: if it does not, the website says "The lid did not close ... check the box" and it retries every 10 s. Pressing the final button without opening the lid is fine (the lid simply stays shut). Without `lid_buttons` the website shows the old OK shipped / Received buttons (fallback). Tested 2026-10-23 with `cargo_controller.py -p simulate:=true` and the dry-run manager (normal flow, send without opening, cargo controller dead after opening = robot waits).
IMPORTANT udev: the Nano and the motor Uno are both CH340 1a86:7523 without serial numbers, so they are told apart only by the USB socket. `udev/99-robot-serial.rules` names the Nano's socket (`KERNELS=="1-1.4"`): find the real socket with `bash scripts/usb_ports.sh` on the Pi, edit the rule, then `sudo cp udev/99-robot-serial.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger` (ask first, needs sudo). Check `ls -l /dev/arduino /dev/servo_nano` afterwards. Never plug the Nano into the Pi before this rule is installed, or `/dev/arduino` may point at the Nano.
Calibration (box built, servos powered): use `python3 scripts/servo_test.py <port> <angle> <servo>` or the Serial Monitor. Lid: find `closed` and `open` (5 degree steps, a few degrees of margin). Lock: find `unlocked` and `locked` (about 90 degrees apart), the arm must sit firmly over the hook with the lid closed. Write the numbers in `config/cargo.yaml`, then set `lid.enabled: true`. Test with `ros2 topic pub --once /cargo_cmd std_msgs/String "{data: unlock}"` (and `lock`).
Mechanics: let the hook take the pull (a slot or catch in the frame), not the servo gears; keep the lid light and put the lid servo's lever close to the hinge (an SG90 is only about 1.5 kg cm); closing is slow (1 degree per 12 ms), keep fingers out of the gap. At controller start-up the lid jumps to "closed" at once.

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
- No trip home between jobs (2026-10-23): when a job ends and another is waiting, the robot drives straight from where it is to the next sender; a job that arrives while it is on its way home turns it around. `-p go_home_between_jobs:=true` restores the old "always home first" behaviour (for comparisons). Dry-run test, 3 jobs (fake 3 s trips): 3rd parcel delivered at 30 s instead of 37 s (two trips home saved).
- Parameters: `locations_file`, `home_name`, `dry_run`, `max_nav_retries`, `nav_timeout`, `cargo_lock`, `go_home_between_jobs`, the two timeouts (set with `-p name:=value`).

## Self-recovery: the robot finds its position again (2026-10-23)
"Kidnapped robot problem": the robot is picked up and put elsewhere, bumped, or localisation drifts; AMCL keeps believing the old position. Run on the LAPTOP next to Nav2 (needs /map, /scan, TF):
`python3 src/my_bot/scripts/self_recovery.py` (no use_sim_time, also in the simulator), and start the manager with `-p self_recovery:=true`.
- Detect: share of lidar points on map walls at the believed pose (normally 90-100 %); below 40 % for 3 s = lost -> `/localization_status` "lost"; the manager cancels the Nav2 goal and the website shows "Robot lost its position...".
- Search (global scan matching): the current scan is tried at every free spot (20 cm grid, 24 directions), the 10 best are refined (5 cm, 2.5 deg); accepted when >= 75 % AND >= 5 points better than any other place; sent to AMCL as /initialpose; "recovered" after 3 good checks -> the manager continues to the same spot (or back to the cabin it was waiting at).
- Unsure (similar-looking places or weak match): turns 60 deg via twist_mux input `cmd_vel_recovery` (priority 50: above Nav2, below the keyboard) and searches again, up to a full circle; never turns with anything within 0.33 m of the axle; then "help: ..." (2D Pose Estimate in RViz also ends it).
- Tested in Gazebo (robot teleported mid-delivery with `gz model -m my_bot -x .. -y .. -Y ..`): noticed after 3.6-4.0 s, search 0.6 s, position error 6-7 cm, delivery completed. AMCL's own global localisation (first attempt) failed: 90 s of turning, 52 % match.
- Simulator note: the real robot's diff_cont speed/acceleration limits stopped odometry in Gazebo; `config/gaz_ros2_ctl_use_sim.yaml` switches them off for the simulation only.

## Live robot position on the website (2026-10-23)
Start the bridge with the SAME map and spots that Nav2 and the manager use:
`python3 src/my_bot/scripts/firebase_bridge.py --ros-args -p map_yaml:=./maps/<room>.yaml -p locations_file:=./src/my_bot/config/locations_real.yaml`
At start-up it uploads the map (cropped to the explored part) and the spots to `/map`; then, about once a second while the robot moves, its map position (TF map -> base_link) to `/pose`. The page shows a "Where is the robot" card below My deliveries: the map, the spots with names, the current pickup in orange and destination in green, and the robot as a blue dot with a direction line (grey when the position is older than 15 s or the robot system is offline). Only logged-in team members can read `/map` and `/pose`; only the robot account writes them. Without `map_yaml` the bridge works as before and the page shows no map. Tested 2026-10-23 with the sim map and a fake moving robot, viewed on a phone.

## Website sound (2026-10-23)
Phones block sound until the page is tapped, so after login a "Turn on sound" card covers the page until it is tapped (it comes back if the phone mutes the page again). The page then keeps the screen awake, beeps three times when the robot reaches that person (and every 15 s until they press the button), and plays two tones when their parcel is delivered. While the robot waits for that person the whole page also pulses orange (1 s cycle) and the tab title alternates with "Robot is here!", until they press the button: this works on a phone in silent mode. The small "Sound on (tap to test)" link plays a test beep. The phone must stay on this page with the MEDIA volume up (Android ringer silent/vibrate usually still allows the beep; an iPhone's silent switch mutes it).

## Failure handling tested (sim, 2026-10-08, with a temporary far-away `cabin_X`)
- Impossible drop-off (A -> X) and impossible pickup (X -> D): 3 attempts (a bad goal took about 41 s to abort the first time), then `FAILED job N`, return home, back to IDLE. About 80 to 120 s per failed job.
- KNOWN LIMITATION (not tested, not fixed): if Nav2 dies mid-trip the manager waits forever (no per-leg time limit). Recovery: restart Nav2 and the manager. Fix idea: a `nav_timeout` per leg that cancels the goal and counts as a failure. Revisit before a live demo.
- The parcel stays on board after a FAILED job: the website must tell someone to unload it.

## Shutdown
`k` in teleop, then Ctrl-C in this order: delivery manager, status viewer, Nav2, localisation, teleop, RViz, sim.
Then `killall -q gzserver gzclient rviz2`, check `ps aux | grep -E "ros2|gzserver|rviz2|amcl" | grep -v grep` is empty, `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*`.

## Commit milestones (decision 2026-10-08)
(1) simulation delivery works end to end: DONE, commit 017e5ae; (2) Firebase linked in simulation; (3) real robot. The Pi has not pulled these yet.
