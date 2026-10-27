#!/bin/bash
# laptop side for the granny_home map: twist_mux, localisation, Nav2, self-recovery, path guard, manager, bridge
S=${S:-$HOME/dev_ws/run_logs}; mkdir -p $S/logs   # logs survive reboots (was a /tmp scratch folder)
MAP=granny_home; SPOTS=./src/my_bot/config/locations_granny_home.yaml
cd ~/dev_ws; source /opt/ros/foxy/setup.bash; source install/setup.bash
wait_for() { timeout $3 bash -c "until grep -q '$2' $1 2>/dev/null; do sleep 2; done"; }
ros2 run twist_mux twist_mux --ros-args --params-file ./src/my_bot/config/twist_mux.yaml -r cmd_vel_out:=diff_cont/cmd_vel_unstamped > $S/logs/g_mux.log 2>&1 &
ros2 launch my_bot localization_launch.py map:=./maps/$MAP.yaml use_sim_time:=false > $S/logs/g_loc.log 2>&1 &
wait_for $S/logs/g_loc.log 'Managed nodes are active' 60 || { echo "localisation failed"; exit 1; }
python3 src/my_bot/scripts/set_pose.py ${START:-home} --file ${START_FILE:-$SPOTS}
ros2 launch my_bot navigation_launch.py use_sim_time:=false map_subscribe_transient_local:=true > $S/logs/g_nav2.log 2>&1 &
wait_for $S/logs/g_nav2.log 'Managed nodes are active' 120 || { echo "nav2 failed"; exit 1; }
sleep 8
python3 -u src/my_bot/scripts/self_recovery.py > $S/logs/g_recovery.log 2>&1 &
python3 -u src/my_bot/scripts/path_guard.py > $S/logs/g_guard.log 2>&1 &
python3 -u src/my_bot/scripts/gap_assist.py > $S/logs/g_assist.log 2>&1 &
python3 -u src/my_bot/scripts/event_logger.py > $S/logs/g_events.log 2>&1 &
python3 -u src/my_bot/scripts/wifi_mapper.py --ros-args -p map_name:=$MAP > $S/logs/g_wifi_map.log 2>&1 &
python3 -u src/my_bot/scripts/eta_estimator.py --ros-args -p locations_file:=$SPOTS > $S/logs/g_eta.log 2>&1 &
if [ "${LID:-0}" = "sim" ]; then
  python3 -u src/my_bot/scripts/cargo_controller.py --ros-args -p simulate:=true > $S/logs/g_cargo_sim.log 2>&1 &
  LIDARGS="-p cargo_lock:=true -p lid_buttons:=true"
fi
python3 -u src/my_bot/scripts/delivery_manager.py --ros-args -p locations_file:=$SPOTS -p self_recovery:=true -p path_guard:=true $LIDARGS > $S/logs/g_manager.log 2>&1 &
wait_for $S/logs/g_manager.log 'Ready at home' 30 || { echo "manager failed"; exit 1; }
python3 -u src/my_bot/scripts/firebase_bridge.py --ros-args -p map_yaml:=./maps/$MAP.yaml -p locations_file:=$SPOTS > $S/logs/g_bridge.log 2>&1 &
wait_for $S/logs/g_bridge.log 'Bridge started' 30 && echo "STACK READY $(date +%T)"
wait
