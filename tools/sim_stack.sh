#!/bin/bash
# The whole delivery system in the simulator (Gazebo loop.world, map sim_loop, spots locations_sim_loop):
# twist_mux (inside launch_sim), localisation, Nav2, self-recovery, path guard, gap assist, delivery manager,
# event logger, ETA estimator, simulated battery (manager checks it before each job), simulated Wi-Fi map, map changes. No website bridge (BRIDGE=1 adds it).
#   bash src/my_bot/tools/sim_stack.sh            (from ~/dev_ws; Ctrl-C stops everything)
# Logs: ~/dev_ws/run_logs/logs/s_*.log, events: ~/dev_ws/run_logs/sim_events.jsonl
S=${S:-$HOME/dev_ws/run_logs}; mkdir -p $S/logs
MAP=${MAP:-sim_loop}; SPOTS=${SPOTS:-./src/my_bot/config/locations_sim_loop.yaml}; WORLD=${WORLD:-loop}
cd ~/dev_ws; source /opt/ros/foxy/setup.bash; source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/dev_ws/src/my_bot/tools/udp_only.xml   # no shared memory (it hangs here)
ros2 daemon stop > /dev/null 2>&1; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
wait_for() { timeout $3 bash -c "until grep -q '$2' $1 2>/dev/null; do sleep 2; done"; }
stop_sim() { kill $(ps -eo pid,args | awk '$2 ~ /^gz(server|client)$/ {print $1}') 2>/dev/null; kill 0; }
trap stop_sim EXIT
SIMT="--ros-args -p use_sim_time:=true"
ros2 launch my_bot launch_sim.launch.py world:=./src/my_bot/worlds/$WORLD.world gui:=${GUI:-true} > $S/logs/s_sim.log 2>&1 &
wait_for $S/logs/s_sim.log 'Configured and started' 90 || { echo "simulator failed"; exit 1; }
sleep 8      # the wheel odometry (odom frame) appears a few seconds later; localisation started before it hung
ros2 launch my_bot localization_launch.py map:=./maps/$MAP.yaml use_sim_time:=true > $S/logs/s_loc.log 2>&1 &
wait_for $S/logs/s_loc.log 'Managed nodes are active' 60 || { echo "localisation failed"; exit 1; }
python3 src/my_bot/scripts/set_pose.py home --file $SPOTS --ros-args -p use_sim_time:=true
ros2 launch my_bot navigation_launch.py use_sim_time:=true map_subscribe_transient_local:=true > $S/logs/s_nav2.log 2>&1 &
wait_for $S/logs/s_nav2.log 'Managed nodes are active' 120 || { echo "nav2 failed"; exit 1; }
python3 -u src/my_bot/scripts/self_recovery.py $SIMT > $S/logs/s_recovery.log 2>&1 &
python3 -u src/my_bot/scripts/path_guard.py $SIMT > $S/logs/s_guard.log 2>&1 &
python3 -u src/my_bot/scripts/gap_assist.py $SIMT > $S/logs/s_assist.log 2>&1 &
python3 -u src/my_bot/scripts/battery_monitor.py --ros-args -p source:=sim > $S/logs/s_battery.log 2>&1 &   # simulated battery
python3 -u src/my_bot/scripts/event_logger.py $SIMT -p log_file:=$S/sim_events.jsonl > $S/logs/s_events.log 2>&1 &
python3 -u src/my_bot/scripts/wifi_probe.py $SIMT -p source:=sim > $S/logs/s_wifi_probe.log 2>&1 &     # made-up Wi-Fi
python3 -u src/my_bot/scripts/wifi_mapper.py $SIMT -p map_name:=$MAP -p out_dir:=$S > $S/logs/s_wifi_map.log 2>&1 &
python3 -u src/my_bot/scripts/change_detector.py $SIMT -p map_name:=$MAP -p out_dir:=$S > $S/logs/s_changes.log 2>&1 &
python3 -u src/my_bot/scripts/eta_estimator.py $SIMT -p locations_file:=$SPOTS -p log_file:=$S/sim_events.jsonl \
    > $S/logs/s_eta.log 2>&1 &
python3 -u src/my_bot/scripts/delivery_manager.py $SIMT -p locations_file:=$SPOTS -p self_recovery:=true \
    -p path_guard:=true -p battery_aware:=true -p battery_trust_sim:=true -p energy_file:=$S/sim_energy.json \
    > $S/logs/s_manager.log 2>&1 &
wait_for $S/logs/s_manager.log 'Ready at home' 30 || { echo "manager failed"; exit 1; }
if [ "${BRIDGE:-0}" = "1" ]; then
  python3 -u src/my_bot/scripts/firebase_bridge.py --ros-args -p map_yaml:=./maps/$MAP.yaml -p locations_file:=$SPOTS \
      > $S/logs/s_bridge.log 2>&1 &
fi
echo "SIM READY $(date +%T)"
wait
