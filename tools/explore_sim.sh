#!/bin/bash
# Autonomous exploration in the simulator: Gazebo (worlds/explore.world) + slam_toolbox (mapping) + Nav2 + explorer.
#   bash src/my_bot/tools/explore_sim.sh            (from ~/dev_ws; Ctrl-C stops everything)
# The map is saved to ~/dev_ws/maps/explored_sim.{pgm,yaml}. Logs: ~/dev_ws/run_logs/logs/x_*.log
S=${S:-$HOME/dev_ws/run_logs}; mkdir -p $S/logs
WORLD=${WORLD:-explore}
cd ~/dev_ws; source /opt/ros/foxy/setup.bash; source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/dev_ws/src/my_bot/tools/udp_only.xml   # no shared memory (it hangs here)
ros2 daemon stop > /dev/null 2>&1; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
wait_for() { timeout $3 bash -c "until grep -q '$2' $1 2>/dev/null; do sleep 2; done"; }
stop_sim() { kill $(ps -eo pid,args | awk '$2 ~ /^gz(server|client)$/ {print $1}') 2>/dev/null; kill 0; }
trap stop_sim EXIT                     # everything this script started (pkill -f would also hit this script)
ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/$WORLD.world > $S/logs/x_sim.log 2>&1 &
wait_for $S/logs/x_sim.log 'Configured and started' 90 || { echo "simulator failed"; exit 1; }
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot/config/mapper_params_mapping.yaml \
    use_sim_time:=true > $S/logs/x_slam.log 2>&1 &
wait_for $S/logs/x_slam.log 'Registering sensor' 60 || { echo "mapper failed"; exit 1; }
ros2 launch my_bot navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true > $S/logs/x_nav2.log 2>&1 &
wait_for $S/logs/x_nav2.log 'Managed nodes are active' 120 || { echo "nav2 failed"; exit 1; }
python3 -u src/my_bot/scripts/gap_assist.py --ros-args -p use_sim_time:=true > $S/logs/x_assist.log 2>&1 &
python3 -u src/my_bot/scripts/explorer.py --ros-args -p use_sim_time:=true \
    -p map_name:=$HOME/dev_ws/maps/explored_sim > $S/logs/x_explorer.log 2>&1 &
echo "EXPLORING $(date +%T)"
wait
