# my_bot runbook: plain Nav2 + SLAM (ROS 2 Foxy)

Use this file to run the robot WITHOUT the delivery system (mapping, plain Nav2 goals, fallback if
the delivery work breaks). For the delivery robot see `RUNBOOK.md`.

Commands that were tested during setup. One terminal per command. Every **laptop** terminal needs:

```
cd ~/dev_ws && source /opt/ros/foxy/setup.bash && source install/setup.bash
```

(`P:` below means "paste after that line".) If a terminal says `Package 'my_bot' not found`, the `source install/setup.bash` part was missing.

## Rules that avoid problems
- **Never run the sim and the real robot together** on ROS domain 0: teleop would drive both.
- **Never run slam_toolbox and AMCL together**: both publish `map -> odom`.
- Laptop overloads easily: close Firefox/video, close the Gazebo window (`killall gzclient`), untick RViz's Camera display. Overload caused `/odom` to vanish at sim start and a `bt_navigator` segfault.
- Start the sim, **wait ~30 s**, then start the rest. Set the 2D Pose Estimate **before** starting Nav2.
- `ros2 daemon stop; ros2 daemon start` fixes a stale "Node not found".
- Check nodes with `ros2 node list` (not `pgrep`).
- If the sim suddenly stops publishing (`/clock`, `/odom`, `/scan` silent, even `/rosout`) after ROS tools were killed abruptly: stop everything, then `rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*` (stale Fast DDS shared-memory files; only when no ROS program is running) and restart. Avoid `timeout` on ROS commands; stop them with Ctrl-C.
- The 2D Pose Estimate must match the laser: red scan arcs should sit on the obstacle edges. If they are off, click again, or turn the robot slowly in place. Drive slowly (teleop `x`/`c` until about speed 0.2, turn 0.5): fast turns make AMCL lose the robot.

## A. Simulation (laptop only)
twist_mux is started by `launch_sim.launch.py` itself: do not start it by hand.

1. Sim (wait for `Configured and started diff_cont`, then ~20 s):
   `ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/obstacles.world`
   then `killall gzclient` (optional, saves ~100 % of a core)
2. RViz: `ros2 run rviz2 rviz2 -d src/my_bot/config/main.rviz --ros-args -p use_sim_time:=true`
   (Fixed Frame `odom` first; Map display: Reliable + Transient Local)
3. Localisation (map server + AMCL):
   `ros2 launch my_bot localization_launch.py map:=./my_map_save.yaml use_sim_time:=true`
   RViz: Fixed Frame `map`, **2D Pose Estimate** at the robot's spawn point.
4. Nav2:
   `ros2 launch my_bot navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true`
5. Keyboard teleop (optional, overrides Nav2 while pressed):
   `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_key`
6. RViz: add `/global_costmap/costmap` as a Map (Color Scheme `costmap`), then **2D Goal Pose** / Navigation2 Goal / waypoint mode.

## B. Real robot (Pi `ubuntu@<pi-ip>` + laptop)
The Pi's address depends on the network. On the phone hotspot `SJBIT 2.0` it was `10.16.139.17` (2026-10-08); earlier, on another network, `10.154.23.17`. Find it with `getent hosts ubuntu.local` or by pinging the hotspot range and looking for a Raspberry Pi hardware address (`d8:3a:dd:...`) in `ip neigh`. A Pi needs 1 to 2 minutes after power-on before it answers. Laptop and Pi must be on the SAME hotspot.

Pi terminals (`ssh ubuntu@10.154.23.17`, key login):

1. `cd ~/robot_ws && source /opt/ros/foxy/setup.bash && source install/setup.bash && ros2 launch my_bot launch_robot.launch.py`
2. `cd ~/robot_ws && source /opt/ros/foxy/setup.bash && source ~/ydlidar_ros2_ws/install/setup.bash && source install/setup.bash && ros2 launch my_bot ydlidar.launch.py`

Laptop terminals (no sim time):

3. RViz: `ros2 run rviz2 rviz2 -d src/my_bot/config/main.rviz`
4. twist_mux **by hand** (not installed on the Pi; stays on the laptop):
   `ros2 run twist_mux twist_mux --ros-args --params-file ./src/my_bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped`
5. Either build a NEW map with slam_toolbox (mapping mode):
   `ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot/config/mapper_params_mapping.yaml use_sim_time:=false`
   save it later with:
   `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/abhayros/dev_ws/maps/NAME'}}"`
   and `ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '/home/abhayros/dev_ws/maps/NAME_serial'}"`
   Or localise on the saved real-room map with AMCL:
   `ros2 launch my_bot localization_launch.py map:=./maps/real_room.yaml use_sim_time:=false`
   RViz: Fixed Frame `map`, **2D Pose Estimate** where the robot really is.
6. Nav2: `ros2 launch my_bot navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true`
7. Teleop (low speed: press `x` several times; `k` stops): same command as in A.5

## Troubleshooting: the motors do not turn (found 2026-10-17)
Symptom: the Arduino answers `OK` to `o 150 150` and the wheel counters read `0 0`, or the motor only vibrates; a new driver and a new Arduino change nothing; the Pi logs USB over-current on all ports and the Arduino drops off USB.
Cause found: **no common ground between the motor driver and the Arduino.** One wire from the driver's GND to an Arduino GND pin fixed it. Check this FIRST (multimeter in continuity mode between the Arduino GND and the driver GND screw: it must beep).
Other checks, in order: driver power LED and battery under load; the two enable pins (12, 13, or the ENA/ENB jumpers); the signal wires on pins 5, 6, 9, 10; the buck converter holds its voltage when the motors start; no 5 V wire between the driver and the Arduino (the Arduino is powered by USB only).
Direct test without ROS (on the Pi, controller stopped, wheels off the floor): `python3 ~/motor_raw_test.py 150 0 20` (left only), `0 150 20` (right only). The script is `scripts/motor_raw_test.py` (copy it to the Pi's home folder). Or `python3 -m serial.tools.miniterm -e --eol CR /dev/arduino 57600` and type `e`, `o 150 0`, `o 0 150`, `r`. The firmware stops the motors 2 s after the last command.
Pi power health: `vcgencmd get_throttled` (0x0 = no under-voltage) and `dmesg | grep -i over-current`.

## Shutdown
`k` in teleop, Ctrl-C each terminal (Nav2, twist_mux, SLAM/AMCL, RViz, then the two Pi terminals), then
`killall -q gzserver gzclient rviz2`.

## Facts worth remembering
- Odometry calibrated by hand tests: `wheel_separation_multiplier 1.055`, radius multipliers 1.0, 1024 counts/rev.
- Stable device names on the Pi (udev rule in `udev/`): `/dev/arduino`, `/dev/ydlidar`.
- Maps: sim `~/dev_ws/my_map_save.yaml`; real room `~/dev_ws/maps/real_room.yaml` (not in git).
- Foxy = plain (unstamped) `Twist` everywhere; no `twist_stamper` needed.
