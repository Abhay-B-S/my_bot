#!/bin/bash
# Autonomous exploration with the REAL robot: the Pi runs launch_robot + lidar + watchdog; this starts on the laptop
# twist_mux, slam_toolbox (mapping), Nav2, gap assist and the explorer.
#   bash src/my_bot/tools/explore_real.sh [map_name]        (from ~/dev_ws; Ctrl-C stops everything)
# Keep someone next to the robot (motor battery switch = emergency stop). The map is saved as
# ~/dev_ws/maps/<map_name>.{pgm,yaml} (default explored_real). Logs: ~/dev_ws/run_logs/logs/r_*.log
NAME=${1:-explored_real}
S=${S:-$HOME/dev_ws/run_logs}; mkdir -p $S/logs
cd ~/dev_ws; source /opt/ros/foxy/setup.bash; source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/dev_ws/src/my_bot/tools/udp_only.xml   # no shared memory (it hangs here)
ros2 daemon stop > /dev/null 2>&1; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
wait_for() { timeout $3 bash -c "until grep -q '$2' $1 2>/dev/null; do sleep 2; done"; }
trap 'kill 0' EXIT
ros2 run twist_mux twist_mux --ros-args --params-file ./src/my_bot/config/twist_mux.yaml \
    -r cmd_vel_out:=diff_cont/cmd_vel_unstamped > $S/logs/r_mux.log 2>&1 &
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_bot/config/mapper_params_mapping.yaml \
    use_sim_time:=false > $S/logs/r_slam.log 2>&1 &
wait_for $S/logs/r_slam.log 'Registering sensor' 60 || { echo "mapper failed (is the lidar running on the Pi?)"; exit 1; }
ros2 launch my_bot navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true > $S/logs/r_nav2.log 2>&1 &
wait_for $S/logs/r_nav2.log 'Managed nodes are active' 120 || { echo "nav2 failed"; exit 1; }
python3 -u src/my_bot/scripts/gap_assist.py > $S/logs/r_assist.log 2>&1 &
python3 -u src/my_bot/scripts/explorer.py --ros-args -p max_minutes:=${MINUTES:-15.0} \
    -p map_name:=$HOME/dev_ws/maps/$NAME > $S/logs/r_explorer.log 2>&1 &
echo "EXPLORING $(date +%T)"
wait
