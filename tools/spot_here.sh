#!/bin/bash
# Save the robot's CURRENT position as a spot (home or cabin_A..cabin_Z) on the running map.
#   bash ~/dev_ws/src/my_bot/tools/spot_here.sh cabin_A
# Needs the delivery stack running (expo start MAP): the website bridge checks the spot like an Admin tap,
# writes config/locations_MAP.yaml, tells the manager to reload, and the website shows it.
# Drive the robot to the spot first (keyboard: tools/teleop.sh), facing the way it should wait.
[ -z "$1" ] && { echo "usage: spot_here.sh home|cabin_A|cabin_B|..."; exit 1; }
cd ~/dev_ws; source /opt/ros/foxy/setup.bash; source install/setup.bash
ros2 topic pub -t 3 -r 2 /record_spot std_msgs/msg/String "{data: $1}" > /dev/null
sleep 4
grep -E "record_spot|Spot $1 " ~/dev_ws/run_logs/logs/g_bridge.log | tail -2 | sed 's/.*firebase_bridge\]: /  /'
grep "Spots reloaded" ~/dev_ws/run_logs/logs/g_manager.log | tail -1 | sed 's/.*delivery_manager\]: /  manager: /'
