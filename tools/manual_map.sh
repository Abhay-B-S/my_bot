#!/bin/bash
# Map by hand: starts twist_mux + slam_toolbox on the laptop (Pi robot side must already be running: expo robot).
# Then drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_key
# Save with: bash src/my_bot/tools/manual_map.sh save NAME   (from ~/dev_ws)
cd ~/dev_ws; source /opt/ros/foxy/setup.bash; source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/dev_ws/src/my_bot/tools/udp_only.xml
S=$HOME/dev_ws/run_logs; mkdir -p $S/logs
if [ "$1" = save ]; then
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '$HOME/dev_ws/maps/$2'}}"
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/dev_ws/maps/${2}_serial'}"
  F=src/my_bot/config/locations_$2.yaml
  [ -f $F ] || printf 'locations:\n  home:\n    x: 0.0\n    y: 0.0\n    yaw: 0.0\n' > $F
  exit 0
fi
ros2 daemon stop >/dev/null 2>&1; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
setsid ros2 run twist_mux twist_mux --ros-args --params-file ./src/my_bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped > $S/logs/m_mux.log 2>&1 < /dev/null &
setsid ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot/config/mapper_params_mapping.yaml use_sim_time:=false > $S/logs/m_slam.log 2>&1 < /dev/null &
for i in $(seq 30); do grep -q 'Registering sensor' $S/logs/m_slam.log && { echo "MAPPER READY"; exit 0; }; sleep 2; done
echo "mapper failed: see $S/logs/m_slam.log"; exit 1
