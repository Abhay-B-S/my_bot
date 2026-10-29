#!/bin/bash
# Expo helper: everything needed on the day, one command each (run from any folder).
#   bash ~/dev_ws/src/my_bot/tools/expo.sh find            find the robot's Pi on this Wi-Fi (prints its address)
#   bash ~/dev_ws/src/my_bot/tools/expo.sh robot           start the robot side on the Pi (lidar, motors, watchdog,
#                                                          ultrasonic guard, lid servo, Wi-Fi probe)
#   bash ~/dev_ws/src/my_bot/tools/expo.sh map  NAME       NEW PLACE: the robot explores and maps by itself (~5 min),
#                                                          saves maps/NAME. Put it in open space first. Ctrl-C = stop.
#   bash ~/dev_ws/src/my_bot/tools/expo.sh start NAME      deliveries on map NAME (website live). Leave this running.
#   bash ~/dev_ws/src/my_bot/tools/expo.sh status          is everything running? Wi-Fi delay? position match?
#   bash ~/dev_ws/src/my_bot/tools/expo.sh stop            stop everything (laptop + Pi)
# Options: LID=sim (website shows the lid, the servo does not move). Default: LID=real.
SELF=$(readlink -f "$0"); cd ~/dev_ws || exit 1
source /opt/ros/foxy/setup.bash; source install/setup.bash
L=~/dev_ws/run_logs/logs; mkdir -p $L
PIF=~/.expo_pi_ip

pi() {                                   # the Pi's address: remembered, else searched on this network
    if [ -s $PIF ] && ssh -o ConnectTimeout=4 -o BatchMode=yes ubuntu@$(cat $PIF) true 2>/dev/null; then cat $PIF; return; fi
    ip=$(bash src/my_bot/scripts/find_pi.sh 2>/dev/null | grep -oE 'ubuntu@[0-9.]+' | head -1 | cut -d@ -f2)
    [ -n "$ip" ] && echo $ip > $PIF && echo $ip
}
stop_laptop() {
    pkill -TERM -f "[p]ython3 -u src/my_bot/scripts/" ; pkill -TERM -f "[r]os2 launch" ; pkill -TERM -f "[t]wist_mux --ros-args"
    pkill -TERM -f "tools/[g]ranny_stack" ; pkill -TERM -f "tools/[e]xplore_real" ; sleep 5
    pkill -KILL -f "[c]ontroller_server|[p]lanner_server|[b]t_navigator|[a]mcl|[m]ap_server|[l]ifecycle_manager|[r]ecoveries_server|[w]aypoint_follower|[s]lam_toolbox" 2>/dev/null
    true
}

case "$1" in
find)
    IP=$(pi); [ -n "$IP" ] && echo "Robot found: ubuntu@$IP" || echo "Robot NOT found: is it on, and on the SAME Wi-Fi as this laptop? Wait 1-2 min after power-on."
    ;;
robot)
    IP=$(pi) || true; [ -z "$IP" ] && { echo "Robot not found (see: expo.sh find)"; exit 1; }
    echo "Starting the robot side on $IP (about 40 s) ..."
    ssh ubuntu@$IP 'nohup setsid ~/start_all.sh lid > /dev/null 2>&1 < /dev/null & sleep 38
        for p in ydlidar_ros2_driver ros2_control_node safety_watchdog ultrasonic_guard cargo_controller; do
            ps -eo args | grep -v -e "^bash" -e "^/bin/bash" -e "^-bash" | grep -q "$p" && echo "  OK   $p" || echo "  MISSING  $p"; done
        ls /dev/ydlidar /dev/arduino /dev/servo_nano > /dev/null 2>&1 && echo "  OK   USB names" || echo "  CHECK USB cables (lidar, Uno, Nano in their usual ports)"'
    ping -c 4 -q $IP | awk -F/ '/rtt/{printf "  Wi-Fi delay: %d ms (good below 50)\n", $5}'
    ;;
map)
    [ -z "$2" ] && { echo "usage: expo.sh map NAME   (e.g. expo.sh map venue)"; exit 1; }
    stop_laptop
    echo "Mapping '$2': the robot drives by itself. Stay near it (motor switch = emergency stop). Ctrl-C stops."
    : > $L/r_explorer.log
    setsid bash src/my_bot/tools/explore_real.sh "$2" > $L/r_out.log 2>&1 < /dev/null &
    timeout 900 bash -c "until grep -qE 'done:|stopped' $L/r_explorer.log 2>/dev/null; do sleep 3; done"
    grep -E 'done:' $L/r_explorer.log | tail -1 | sed 's/.*explorer\]: /  /'
    stop_laptop
    if [ -f maps/$2.yaml ]; then
        F=src/my_bot/config/locations_$2.yaml
        [ -f $F ] || printf 'locations:\n  home:\n    x: 0.0\n    y: 0.0\n    yaw: 0.0\n' > $F   # where mapping started
        echo "Map saved: maps/$2. Home = where the robot started mapping (it drove back there)."
        echo "Next: expo.sh start $2, then set the cabins (and Home if you like) on the website's Admin page."
    else
        echo "No map saved (see $L/r_explorer.log)."
    fi
    ;;
start)
    [ -z "$2" ] && { echo "usage: expo.sh start NAME   (maps: $(ls maps/*.yaml | xargs -n1 basename | sed 's/.yaml//' | tr '\n' ' '))"; exit 1; }
    [ -f maps/$2.yaml ] || { echo "No map maps/$2.yaml"; exit 1; }
    F=src/my_bot/config/locations_$2.yaml
    [ -f $F ] || printf 'locations:\n  home:\n    x: 0.0\n    y: 0.0\n    yaw: 0.0\n' > $F
    stop_laptop
    echo "Starting deliveries on '$2' (about 1.5 min) ..."
    MAP=$2 LID=${LID:-real} nohup bash src/my_bot/tools/granny_stack.sh > $L/g_stack_out.log 2>&1 < /dev/null &
    timeout 280 bash -c "until grep -qE 'STACK READY|failed' $L/g_stack_out.log; do sleep 3; done"
    tail -1 $L/g_stack_out.log
    sleep 15
    grep -E 'match|recovered|start:' $L/g_recovery.log | tail -1 | sed 's/.*self_recovery\]: /  position: /'
    echo "Open https://delivery-robot-demo.web.app and send a delivery. 'expo.sh status' to check."
    ;;
status)
    IP=$(cat $PIF 2>/dev/null)
    for p in delivery_manager firebase_bridge self_recovery path_guard gap_assist controller_server amcl; do
        pgrep -f "$p" > /dev/null && echo "  OK   $p" || echo "  --   $p"; done
    [ -n "$IP" ] && ping -c 3 -q $IP | awk -F/ '/rtt/{printf "  Wi-Fi delay: %d ms (good below 50)\n", $5}'
    tail -1 $L/g_recovery.log 2>/dev/null | sed 's/.*self_recovery\]: /  position: /'
    tail -2 $L/g_manager.log 2>/dev/null | sed 's/.*delivery_manager\]: /  robot: /'
    ;;
stop)
    stop_laptop
    IP=$(cat $PIF 2>/dev/null); [ -n "$IP" ] && ssh -o ConnectTimeout=5 ubuntu@$IP 'bash ~/stop_all.sh > /dev/null 2>&1' && echo "Pi programs stopped."
    echo "Laptop programs stopped."
    ;;
*)
    sed -n "2,10p" "$SELF" | sed "s/^# \{0,1\}//"
    ;;
esac
