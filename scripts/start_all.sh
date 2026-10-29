#!/bin/bash
# Run ON THE PI (copied to ~/start_all.sh by demo/1_robot_start.sh); start it detached:
#   nohup setsid ~/start_all.sh [lid] > /dev/null 2>&1 < /dev/null &
# Starts the lidar, the motor controller, the safety watchdog, the Wi-Fi probe and, with "lid", the lid controller.
# Anything already running is left alone. Logs: ~/lidar.log ~/robot.log ~/watchdog.log ~/wifi.log ~/cargo.log
# Result line: /tmp/start_all.result
source /opt/ros/foxy/setup.bash
source ~/ydlidar_ros2_ws/install/setup.bash
source ~/robot_ws/install/setup.bash
cd ~/robot_ws
running() { ps -eo cmd | grep -qE "$1"; }
LIDAR='^/home/ubuntu/ydlidar_ros2_ws/install/ydlidar_ros2_driver/lib/'
CTRL='^/opt/ros/foxy/lib/controller_manager/ros2_control_node'
WATCH='^python3 -u /home/ubuntu/robot_ws/src/my_bot/scripts/safety_watchdog.py'
LIDC='^python3 -u /home/ubuntu/robot_ws/src/my_bot/scripts/cargo_controller.py'
WIFI='^python3 -u /home/ubuntu/robot_ws/src/my_bot/scripts/wifi_probe.py'
SONAR='^python3 -u /home/ubuntu/robot_ws/src/my_bot/scripts/ultrasonic_guard.py'
# nothing ROS running yet: clear leftovers of force-stopped programs (they can freeze ROS messaging)
if ! running "$LIDAR|$CTRL|$WATCH|$LIDC|$WIFI|$SONAR"; then
    ros2 daemon stop > /dev/null 2>&1
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
fi
running "$LIDAR" || nohup ros2 launch my_bot ydlidar.launch.py > ~/lidar.log 2>&1 &
if ! running "$CTRL"; then
    nohup ros2 launch my_bot launch_robot.launch.py > ~/robot.log 2>&1 &
    sleep 18
fi
running "$WATCH" || nohup python3 -u ~/robot_ws/src/my_bot/scripts/safety_watchdog.py > ~/watchdog.log 2>&1 &
# Wi-Fi signal for the coverage map (wifi_mapper.py on the laptop); harmless if the file is not there yet
[ -f ~/robot_ws/src/my_bot/scripts/wifi_probe.py ] && ! running "$WIFI" && \
    nohup python3 -u ~/robot_ws/src/my_bot/scripts/wifi_probe.py > ~/wifi.log 2>&1 &
# front ultrasonic sensor (GPIO 23/24): stops forward driving into low things, marks them for Nav2
[ -f ~/robot_ws/src/my_bot/scripts/ultrasonic_guard.py ] && ! running "$SONAR" && \
    nohup python3 -u ~/robot_ws/src/my_bot/scripts/ultrasonic_guard.py > ~/ultrasonic.log 2>&1 &
if [ "$1" = "lid" ] && ! running "$LIDC"; then
    nohup python3 -u ~/robot_ws/src/my_bot/scripts/cargo_controller.py > ~/cargo.log 2>&1 &
fi
sleep 3
echo "$(date +%T) started" > /tmp/start_all.result
