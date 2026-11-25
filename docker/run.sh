#!/bin/bash
# Start the NavaYantra container (host networking so ROS 2 reaches the Pi; X11 so RViz/Gazebo windows open).
#   bash docker/run.sh shell     -> a terminal inside (then: bash ~/expo find / robot / start home ...)
#   bash docker/run.sh sim       -> the full delivery simulation (Gazebo + Nav2 + manager)
# Persistent data lives on the host in $NAVA_DATA (default ~/navayantra_data):
#   maps/      <- copy ~/dev_ws/maps from the backup here
#   run_logs/  <- logs and events.jsonl
#   config/    <- my_bot/config (created from the image on the first run; put your locations_*.yaml here)
# SSH key for the Pi: your ~/.ssh is copied into the container at start (read-only mount, never changed).
set -e
MODE=${1:-shell}
DATA=${NAVA_DATA:-$HOME/navayantra_data}
mkdir -p "$DATA/maps" "$DATA/run_logs" "$DATA/config"
if [ -z "$(ls -A "$DATA/config")" ]; then        # first run: start from the repo's config folder
  docker run --rm -v "$DATA/config":/out navayantra:foxy cp -a /root/dev_ws/src/my_bot/config/. /out/
fi
xhost +local:root > /dev/null 2>&1 || true
ARGS=(--rm -it --network host --ipc host --privileged
      -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix
      -v "$DATA/maps":/root/dev_ws/maps -v "$DATA/run_logs":/root/dev_ws/run_logs
      -v "$DATA/config":/root/dev_ws/src/my_bot/config
      -v "$HOME/.ssh":/tmp/host_ssh:ro)
[ -f "$HOME/.delivery_robot_password" ] && ARGS+=(-v "$HOME/.delivery_robot_password":/tmp/host_pw:ro)
PREP='cp -r /tmp/host_ssh /root/.ssh 2>/dev/null; chmod 700 /root/.ssh; chmod 600 /root/.ssh/* 2>/dev/null;
      [ -f /tmp/host_pw ] && cp /tmp/host_pw /root/.delivery_robot_password && chmod 600 /root/.delivery_robot_password;'
case "$MODE" in
  sim)         exec docker run "${ARGS[@]}" navayantra:foxy bash -ic "$PREP bash src/my_bot/tools/sim_stack.sh" ;;
  shell|robot) exec docker run "${ARGS[@]}" navayantra:foxy bash -ic "$PREP exec bash" ;;
  *) echo "usage: docker/run.sh shell|sim"; exit 1 ;;
esac
