#!/usr/bin/env python3
"""Drive the robot a measured distance straight ahead (or back) using the Arduino's encoder speed control.

Run ON THE PI with the robot controller (launch_robot) STOPPED, the motor driver switched on, the robot on the
floor with clear space, and your hand near the power switch.

  python3 drive_distance.py 1.0            # 1 m forward
  python3 drive_distance.py -0.5           # 0.5 m backward
  python3 drive_distance.py 1.0 20         # 1 m at a cruise speed of 20 encoder counts per 1/30 s (default 22)

Safety: first a 15 cm nudge at a LOW FIXED power (open loop, so a dead encoder cannot make the motors run away);
only after both encoders have counted correctly does it switch to the encoder speed control. It stops at once if a
wheel does not move, runs the wrong way, or the two wheels disagree strongly, and after a time limit. The distance is computed from the encoder counts with the
wheel radius and counts per revolution below, so check it with a tape measure (new wheels may need a new radius).
"""
import math
import sys
import time

import serial

PORT = '/dev/arduino'
BAUD = 57600
COUNTS_PER_REV = 1024.0          # config/my_controllers + ros2_control.xacro (enc_counts_per_rev)
WHEEL_RADIUS = 0.034             # metres, calibrated effective radius of the OLD wheels
M_PER_COUNT = 2 * math.pi * WHEEL_RADIUS / COUNTS_PER_REV
NUDGE_M = 0.15
NUDGE_POWER = 160               # raw motor power (0..255) used until the encoders have been proven


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    target_m = float(sys.argv[1])
    cruise = int(sys.argv[2]) if len(sys.argv) > 2 else 22
    nudge_power = int(sys.argv[3]) if len(sys.argv) > 3 else NUDGE_POWER   # raw power for the first 15 cm
    sign = 1 if target_m >= 0 else -1
    target = abs(target_m) / M_PER_COUNT
    ser = serial.Serial(PORT, BAUD, timeout=0.4)
    time.sleep(2.5)                                    # opening the port restarts the Arduino
    ser.read(ser.in_waiting or 1)

    def counters():
        ser.reset_input_buffer()
        ser.write(b'e\r')
        parts = ser.readline().decode(errors='ignore').split()
        return int(parts[0]), int(parts[1])

    def drive(speed, raw=False):                        # speed in counts per frame, or raw power if raw=True
        cmd = 'o' if raw else 'm'
        value = nudge_power if raw else speed
        ser.write(('%s %d %d\r' % (cmd, sign * value, sign * value)).encode())

    def stop(reason):
        for _ in range(3):
            ser.write(b'm 0 0\r')
            time.sleep(0.15)
        ser.write(b'o 0 0\r')
        time.sleep(0.2)
        l1, r1 = counters()
        dl, dr = (l1 - l0) * sign, (r1 - r0) * sign
        print('STOPPED: %s' % reason)
        print('encoders moved: left %d, right %d counts  =>  left %.3f m, right %.3f m, mean %.3f m' % (
            dl, dr, dl * M_PER_COUNT, dr * M_PER_COUNT, (dl + dr) / 2 * M_PER_COUNT))
        if abs(dl) > 200 and abs(dr) > 200:
            print('veer: right wheel went %.1f%% of the left wheel\'s distance' % (100.0 * dr / dl))
        ser.close()
        sys.exit(0)

    l0, r0 = counters()
    print('start counters: left %d right %d; target %.2f m = %.0f counts' % (l0, r0, abs(target_m), target))
    t_start = time.time()
    nudged = False
    last_l, last_r, last_move = l0, r0, time.time()
    try:
        while True:
            l, r = counters()
            dl, dr = (l - l0) * sign, (r - r0) * sign
            mean = (dl + dr) / 2.0
            elapsed = time.time() - t_start
            # progress watchdog: the counters must change
            if abs(l - last_l) + abs(r - last_r) > 3:
                last_l, last_r, last_move = l, r, time.time()
            elif time.time() - last_move > 1.2:
                stop('the encoders did not count for 1.2 s: motor driver off, battery, ground wire, a blocked wheel, '
                     'or the ENCODER wires are not connected')
            if elapsed > 30:
                stop('time limit of 30 s reached')
            # the nudge check
            if not nudged and mean >= NUDGE_M / M_PER_COUNT:
                if dl < 300 or dr < 300 or not 0.5 < (dr / max(dl, 1)) < 2.0:
                    stop('after the nudge the wheels disagree or one barely moved (left %d, right %d counts)' % (dl, dr))
                nudged = True
                print('nudge OK: left %d right %d counts, continuing' % (dl, dr))
            if dl < -300 or dr < -300:
                stop('a wheel is counting the WRONG way (left %d, right %d)' % (dl, dr))
            # done?
            if mean >= target:
                stop('reached the distance')
            remaining = target - mean
            # soft start, cruise, slow down for the last 15 cm
            speed = min(cruise, 6 + int(elapsed * 20))
            if remaining < 0.15 / M_PER_COUNT:
                speed = max(6, int(cruise * remaining / (0.15 / M_PER_COUNT)))
            if not nudged:
                drive(0, raw=True)                     # low fixed power until the encoders are proven
            else:
                drive(speed)
            time.sleep(0.12)
    except KeyboardInterrupt:
        stop('stopped by you (Ctrl-C)')
    except Exception as exc:
        try:
            stop('error: %s' % exc)
        except SystemExit:
            raise


if __name__ == '__main__':
    main()
