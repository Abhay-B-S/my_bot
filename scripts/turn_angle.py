#!/usr/bin/env python3
"""Turn the robot on the spot by an angle, measured from the wheel encoders. Run ON THE PI, controller stopped.

  python3 turn_angle.py -90      # 90 degrees to the RIGHT (clockwise)
  python3 turn_angle.py 90       # 90 degrees to the LEFT
  python3 turn_angle.py -90 190  # same, with raw motor power 190 (default 170; raise it if the load is heavy)
  python3 turn_angle.py 360 s20  # SPEED CONTROL like Nav2: each wheel held at 20 counts per 1/30 s by the Arduino
                                 # (a weaker wheel gets more power automatically); slows to s8 near the end

Stops at once if a wheel does not move or runs the wrong way, and after 15 s. The motors coast a few degrees after
the stop, so it stops LEAD degrees early. The angle uses wheel_separation x multiplier from config/my_controllers.yaml.
"""
import math
import sys
import time

import serial

PORT = '/dev/arduino'
BAUD = 57600
M_PER_COUNT = 2 * math.pi * 0.034 / 1024.0
SEPARATION = 0.288 * 0.9              # wheel_separation x multiplier, config/my_controllers.yaml
LEAD_DEG = 3.0
SLOW_DEG = 45.0         # slow down over the last 45 degrees: a heavy robot slides on a smooth floor if it stops abruptly
MIN_POWER = 120         # lowest power used while slowing down (enough to keep a moving robot turning)


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    target = float(sys.argv[1])
    arg = sys.argv[2] if len(sys.argv) > 2 else '170'
    speed_mode = arg.startswith('s')      # 's20' = Arduino speed control ('m'), else raw power ('o')
    power = int(arg[1:] if speed_mode else arg)
    low = 8 if speed_mode else MIN_POWER
    sign = 1 if target >= 0 else -1          # +1 = left turn: right wheel forward, left wheel backward
    ser = serial.Serial(PORT, BAUD, timeout=0.4)
    time.sleep(2.5)
    ser.read(ser.in_waiting or 1)

    def counters():
        ser.reset_input_buffer()
        ser.write(b'e\r')
        parts = ser.readline().decode(errors='ignore').split()
        return int(parts[0]), int(parts[1])

    l0, r0 = counters()

    def angle():
        l, r = counters()
        return math.degrees(((r - r0) - (l - l0)) * M_PER_COUNT / SEPARATION), l - l0, r - r0

    def stop(reason):
        for _ in range(3):
            ser.write(b'm 0 0\r' if speed_mode else b'o 0 0\r')
            time.sleep(0.15)
        ser.write(b'o 0 0\r')
        time.sleep(0.6)
        a, dl, dr = angle()
        print('STOPPED: %s' % reason)
        print('turned %.1f degrees (%s); wheel counts left %d, right %d' % (abs(a), 'left' if a > 0 else 'right', dl, dr))
        if max(abs(dl), abs(dr)) > 1000 and min(abs(dl), abs(dr)) < 0.2 * max(abs(dl), abs(dr)):
            print('WARNING: the %s wheel hardly turned: the robot pivoted around it (stuck or too weak under load)'
                  % ('left' if abs(dl) < abs(dr) else 'right'))
        ser.close()
        sys.exit(0)

    t0 = time.time()
    last_move, last_sum = time.time(), 0
    try:
        while True:
            a, dl, dr = angle()
            if abs(dl) + abs(dr) > last_sum + 3:
                last_sum, last_move = abs(dl) + abs(dr), time.time()
            elif time.time() - last_move > 1.5:
                stop('wheels did not turn for 1.5 s (too heavy for this power? raise it, e.g. 200)')
            if time.time() - t0 > 15:
                stop('time limit')
            if a * sign < -15:
                stop('turning the WRONG way')
            if a * sign >= abs(target) - LEAD_DEG:
                stop('reached the angle')
            left = abs(target) - LEAD_DEG - a * sign
            p = power if left > SLOW_DEG else int(low + (power - low) * max(left, 0) / SLOW_DEG)
            ser.write(('%s %d %d\r' % ('m' if speed_mode else 'o', -sign * p, sign * p)).encode())
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop('stopped by you')


if __name__ == '__main__':
    main()
