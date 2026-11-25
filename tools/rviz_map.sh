#!/bin/bash
# RViz with the robot view (set Fixed Frame = map to watch mapping or localisation).
cd ~/dev_ws; source /opt/ros/foxy/setup.bash; source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/dev_ws/src/my_bot/tools/udp_only.xml
exec rviz2 -d src/my_bot/config/drive_bot.rviz
