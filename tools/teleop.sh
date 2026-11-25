#!/bin/bash
# Drive the robot with the keyboard (priority 100 in twist_mux: overrides Nav2). Run it in its OWN terminal.
# Keys: i forward, , back, j/l turn, k stop, q/z faster/slower (press x a few times first to go slow).
cd ~/dev_ws; source /opt/ros/foxy/setup.bash; source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/dev_ws/src/my_bot/tools/udp_only.xml
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_key
