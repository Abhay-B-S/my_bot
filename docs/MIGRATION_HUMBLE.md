# Moving to Ubuntu 22.04 + ROS 2 Humble (plan, written 2026-10-26)

Goal: the robot runs on Humble by **swapping the Pi's SD card** and **booting the laptop into Ubuntu 22.04**. The Foxy
setup stays untouched on its own SD card and laptop partition, so going back takes 5 minutes (swap the card back,
boot 20.04). Do it **after the exhibition on 2026-10-30**; the full switch needs about 2.5-3.5 days including tuning.

Why: Nav2 Foxy (2020) plans for a round robot and has no "turn first, then drive". Humble adds what our robot (wheels
near the front, long back that swings out) needs:

| Humble feature | What it fixes for us |
|---|---|
| **Smac Lattice planner** (diff-drive motion primitives, real footprint) | Plans with the real body shape and turning on the spot, so it does not plan routes the robot cannot follow through doorways |
| **Rotation Shim controller** | Turns on the spot to face the path first, then drives (what `scripts/gap_assist.py` does by hand on Foxy) |
| **MPPI controller** (or Regulated Pure Pursuit) | Smoother than DWB, no slalom, better in tight spaces |
| **Collision Monitor** | Stops/slows on lidar points inside a zone round the body, independent of planning |
| **Velocity smoother** | Gentle acceleration in one place (now split between diff_cont and Nav2 limits) |
| **BackUp / DriveOnHeading behaviours** | A reliable short reverse once the motor wiring is fixed |

Foxy (Pi) and Humble (laptop) cannot reliably talk to each other: **both machines switch together**.

---

## Phase 0 - before touching anything (on Foxy, 30 min)

1. Commit and push all code (`main` = the working Foxy version). Tag it: `git tag foxy-final && git push --tags`.
2. Copy to a USB stick (these are NOT in git):
   - `~/dev_ws/maps/` (all maps), `src/my_bot/config/locations_*.yaml`
   - `~/.delivery_robot_password` (keep mode 600), the Firebase web config file if kept outside git
   - Pi: `/etc/udev/rules.d/99-robot-serial.rules`, `~/start_robot.sh`, `~/start_cargo.sh`, `~/stop_all.sh`
   - Claude Code memory: `~/.claude/projects/-home-abhayros-dev-ws/` (see the last section)
3. Buy/prepare a **second SD card** (32 GB+, A1/A2). Keep the Foxy card labelled and untouched.

## Phase 1 - laptop on Ubuntu 22.04 (2-3 h)

1. Install ROS 2 Humble desktop (apt, official instructions) and:
   `ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-twist-mux
   ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
   ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-teleop-twist-keyboard ros-humble-robot-localization
   python3-colcon-common-extensions libserial-dev`
2. Python: `pip3 install numpy pillow pyserial` plus whatever `scripts/firebase_bridge.py` imports.
3. Workspace `~/dev_ws/src`: clone `my_bot` on a new branch **`humble`**; `diffdrive_arduino` from joshnewans on its
   **`humble`** branch (it uses `libserial-dev`, so the old `serial` package is not needed).
4. Copy maps and spot files back from the USB stick.
5. Get the **simulation** working first (Gazebo + Nav2) - no robot needed. Then do the code changes below.

## Phase 2 - Pi on the new SD card (3-5 h)

1. Flash **Ubuntu Server 22.04 (64-bit)** with Raspberry Pi Imager; set hostname, user `ubuntu`, WiFi "SJBIT 2.0"
   (or the exhibition hotspot) and SSH in the imager settings.
2. Install `ros-humble-ros-base` + `ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-xacro libserial-dev`.
3. `~/robot_ws/src`: `my_bot` (branch `humble`), `diffdrive_arduino` (branch `humble`).
   `~/ydlidar_ros2_ws`: YDLidar-SDK (build + install) and `ydlidar_ros2_driver` on its **`humble`** branch.
4. Copy `99-robot-serial.rules` to `/etc/udev/rules.d/`, reload udev, add `ubuntu` to `dialout`. Check that
   `/dev/arduino`, `/dev/servo_nano`, `/dev/ydlidar` appear (same USB sockets as now).
5. Copy `start_robot.sh` / `start_cargo.sh` / `stop_all.sh` and change `foxy` -> `humble` in them.
6. The Arduino firmware does not change.
7. Test step by step: lidar `/scan` -> motors with teleop (lifted) -> `/odom` -> watchdog -> lid servos.

## Code changes on the `humble` branch (3-4 h)

| File | Change |
|---|---|
| `description/ros2_control.xacro` | Hardware plugin name/params of the Humble diffdrive_arduino (`diffdrive_arduino/DiffDriveArduinoHardware`, `device`, `baud_rate`, `timeout_ms`, `enc_counts_per_rev`, wheel joint names) |
| `config/my_controllers.yaml` | Keep `wheel_separation_multiplier` 1.27 and the acceleration limits; add `use_stamped_vel: false` (we send unstamped Twist) |
| `launch/launch_robot.launch.py` | `spawner.py` -> `spawner`; pass `robot_description` to `ros2_control_node` as now |
| `config/nav2_params.yaml` | Start from Humble's `nav2_bringup/params/nav2_params.yaml` and re-apply ours: AMCL (`robot_model_type: nav2_amcl::DifferentialMotionModel`, alpha1 0.8, update_min 0.1), footprint, inflation, `/blocked_points` source, speeds/accelerations. `recoveries_server` -> `behavior_server`; `goal_checker_plugin` -> `goal_checker_plugins`; `default_bt_xml_filename` -> `default_nav_to_pose_bt_xml` |
| Planner | `nav2_smac_planner/SmacPlannerLattice` with the diff-drive primitive file for 0.05 m resolution |
| Controller | `nav2_rotation_shim_controller::RotationShimController` wrapping MPPI (or Regulated Pure Pursuit); `min/max` turn speeds 0.4/0.5 rad/s (slower does not move the robot on marble) |
| New | `collision_monitor` with a stop polygon = real body + 3 cm, slowdown polygon + 15 cm |
| `config/nav2_bt_*.xml` | Check node names against Humble's default trees (mostly the same) |
| `scripts/*.py` | Should run unchanged (rclpy, tf2_ros, nav2_msgs). Test each: delivery_manager, path_guard, self_recovery, gap_assist (may become unnecessary with Rotation Shim + Smac), safety_watchdog, cargo_controller, firebase_bridge, calibrate_turn |
| `demo/*.sh`, `tools/*.sh` | `/opt/ros/foxy` -> `/opt/ros/humble` |

## Phase 3 - real robot on Humble (1-2 days)

1. Run `scripts/calibrate_turn.py` on the floor (multiplier), drive test for distance.
2. New SLAM map (or reuse the old one: map files are the same format), record spots.
3. Tune Smac + Rotation Shim + MPPI in the simulator first, then on the robot: doorway, narrow gap, corridor, the
   sofa loop, blocked route, return to sender, lid.
4. Website end-to-end test (Firebase bridge).

**Success = the same test list as on Foxy passes:** A->B, C->D through doorways without getting stuck, reroute with
one route blocked, every route blocked, return to sender, person as obstacle, lost position + self-recovery.

## Going back to Foxy (5 min)

Shut the Pi down, swap in the Foxy SD card, boot the laptop into Ubuntu 20.04, `git checkout main`. Nothing on the
Foxy side was changed.

## Moving Claude Code to the 22.04 laptop

1. Install Claude Code on 22.04 and log in (do not copy login files).
2. Copy `~/.claude/projects/-home-abhayros-dev-ws/` from the USB stick to the same path. The `memory/` notes hold the
   project history; the `.jsonl` files are the old conversations (`claude --resume` lists them).
3. This only lines up if the user name and the folder are the same (`/home/abhayros/dev_ws`). If not, the folder
   name changes (it is the path with `/` -> `-`): copy `memory/` into the new folder.
4. Do not rely on `/tmp`: helper scripts are in `src/my_bot/tools/`, logs in `~/dev_ws/run_logs/`.
