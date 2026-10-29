#!/bin/bash
# Run ON THE PI (copy to ~/stop_all.sh); start detached so a dropped SSH link cannot interrupt it. Result in /tmp/stop_all.result
# stops every robot program on the Pi (lid controller, watchdog, motor controller, lidar) and sends a motor stop
sel() { ps -eo pid,cmd | grep -E "^ *[0-9]+ (python3 -u /home/ubuntu/robot_ws/src/my_bot/scripts/(cargo_controller|safety_watchdog|wifi_probe|ultrasonic_guard).py|/usr/bin/python3 /opt/ros/foxy/bin/ros2 launch my_bot|/opt/ros/foxy/lib/(controller_manager|robot_state_publisher)/|/home/ubuntu/ydlidar_ros2_ws/install/)" | awk '{print $1}'; }
p=$(sel); [ -n "$p" ] && kill -INT $p; sleep 6
p=$(sel); [ -n "$p" ] && kill -TERM $p; sleep 3
p=$(sel); [ -n "$p" ] && kill -KILL $p; sleep 1
python3 - <<PY
import serial, time
s = serial.Serial('/dev/arduino', 57600, timeout=0.5); time.sleep(2.5)
for c in (b'm 0 0\r', b'o 0 0\r'):
    s.write(c); time.sleep(0.15)
s.close()
PY
echo "$(date +%T) stopped; robot programs left: $(sel | wc -w)" > /tmp/stop_all.result
