#!/bin/bash
# Run ON THE PI (copy to ~/start_robot.sh): starts the motor controller, then the safety watchdog.
# Start it detached so a dropped SSH link cannot interrupt it:
#   nohup setsid ~/start_robot.sh > /dev/null 2>&1 < /dev/null &
# The lidar is started separately (ros2 launch my_bot ydlidar.launch.py). Logs: ~/robot.log, ~/watchdog.log
source /opt/ros/foxy/setup.bash
source ~/robot_ws/install/setup.bash
cd ~/robot_ws
nohup ros2 launch my_bot launch_robot.launch.py > ~/robot.log 2>&1 &
sleep 18
nohup python3 -u ~/robot_ws/src/my_bot/scripts/safety_watchdog.py > ~/watchdog.log 2>&1 &
