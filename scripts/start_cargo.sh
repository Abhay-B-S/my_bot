#!/bin/bash
# Run ON THE PI (copy to ~/start_cargo.sh); start detached: nohup setsid ~/start_cargo.sh > /dev/null 2>&1 < /dev/null &
# starts the parcel-box lid controller (servo Nano), output in ~/cargo.log
source /opt/ros/foxy/setup.bash
source ~/robot_ws/install/setup.bash
exec python3 -u ~/robot_ws/src/my_bot/scripts/cargo_controller.py > ~/cargo.log 2>&1
