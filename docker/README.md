# Running the NavaYantra laptop side in Docker (written 25 Nov 2026, NOT yet tested)

Why: the Ubuntu 20.04 laptop install (native ROS 2 Foxy) is deleted after December 2026. ROS 2 Foxy only installs
natively on Ubuntu 20.04, so on Ubuntu 22.04 (or any Linux) it runs in this container instead. The Pi keeps its own
Ubuntu 20.04 + Foxy SD card and is not changed.

Works on: Linux hosts (Ubuntu 22.04) for both simulation and the real robot (`--network host` lets ROS 2 discovery
reach the Pi). Windows Docker Desktop: simulation at most; the real robot is unreliable there (no host networking for DDS).

## One-time setup on Ubuntu 22.04
```bash
sudo apt install docker.io && sudo usermod -aG docker $USER      # then log out and in again
git clone git@github.com:Abhay-B-S/my_bot.git ~/projects/my_bot && cd ~/projects/my_bot
docker build -t navayantra:foxy -f docker/Dockerfile .            # 15-30 min, about 4-5 GB
mkdir -p ~/navayantra_data && cd ~/navayantra_data                 # restore maps + logs from the backup:
#   rclone copy gdrive:Claude_Archive/abhayros_Inspiron_5590_Linux/backup_before_delete/dev_ws_full_with_git.tar.gz .
#   tar -xzf dev_ws_full_with_git.tar.gz dev_ws/maps dev_ws/run_logs && mv dev_ws/maps dev_ws/run_logs . && rmdir dev_ws
bash ~/projects/my_bot/docker/run.sh shell      # first run creates ~/navayantra_data/config; then exit
cp <backup>/dev_ws/src/my_bot/config/locations_*.yaml ~/navayantra_data/config/     # your spots files
# the Pi's SSH login uses your ~/.ssh key (add it to the Pi once: ssh-copy-id ubuntu@<PI_IP>)
# the website bridge needs ~/.delivery_robot_password on the host (private; recreate it from your password manager)
```

## Daily use (same commands as the 08 Quick Commands PDF, inside the container)
```bash
bash ~/projects/my_bot/docker/run.sh shell       # opens a shell inside; then:
bash ~/expo find ; bash ~/expo robot ; bash ~/expo start home ; bash ~/expo status ; bash ~/expo stop
```
Extra terminals into the SAME container: `docker exec -it $(docker ps -q -f ancestor=navayantra:foxy) bash`
(for teleop: `bash src/my_bot/tools/teleop.sh`, RViz: `bash src/my_bot/tools/rviz_map.sh`).
Simulation: `bash ~/projects/my_bot/docker/run.sh sim`.

## Test plan (do this BEFORE deleting Ubuntu 20.04)
1. `run.sh sim` -> Gazebo + RViz windows open, a simulated delivery completes.
2. Pi on: `run.sh shell` -> `expo find`, `expo robot` (all OK), `expo start home` (match >= 80 %), one real delivery.
3. If ROS topics from the Pi do not appear: check `ROS_DOMAIN_ID=0` on both, same Wi-Fi, and that the host
   firewall allows UDP (`sudo ufw status`); FastDDS profile: `FASTRTPS_DEFAULT_PROFILES_FILE=.../tools/udp_only.xml`.
