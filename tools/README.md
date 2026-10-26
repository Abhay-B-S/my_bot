# tools/ - helper scripts used during real-robot tests (2026-10)

| Script | What it does |
|---|---|
| `granny_stack.sh` | Starts the laptop side for one map: twist_mux, localisation, Nav2, self-recovery, path guard, delivery manager, Firebase bridge. `LID=sim` = website lid buttons without moving the servos. `START=here START_FILE=pose.yaml` = start pose other than home. Edit `MAP`/`SPOTS` for another place. Logs: `~/dev_ws/run_logs/logs/`. |
| `udp_only.xml` | Fast DDS profile without shared memory. Use `FASTRTPS_DEFAULT_PROFILES_FILE=tools/udp_only.xml` for helper commands that cannot see running nodes. |
| `match_now.py` | Prints how much of the live lidar scan matches the map (%). |
| `maptools.py` | Map checks: loops round obstacles (`loops`), route length between spots (`route`). |
| `live_rec.py` | Records pose, wheel commands and delivery state every 0.5 s to a CSV, for analysing a run. |
| `turn_right.py`, `drive_straight.py` | Manual moves through twist_mux (keyboard priority): turn right on the spot / drive straight N metres. |
| `drive_test.py`, `turn_test.py`, `lidar_turn_check.py`, `reverse_test.py` | Motor and odometry tests from 2026-10-26. |
| `send.py` | Sends a delivery request without the website. |
