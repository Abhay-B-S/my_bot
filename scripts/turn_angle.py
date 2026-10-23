#!/usr/bin/env python3
"""Turn the robot on the spot by an angle, measured from the wheel encoders. Run ON THE PI, controller stopped.

  python3 turn_angle.py -90      # 90 degrees to the RIGHT (clockwise)
  python3 turn_angle.py 90       # 90 degrees to the LEFT
  python3 turn_angle.py -90 190  # same, with raw motor power 190 (default 170; raise it if the load is heavy)

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
SEPARATION = 0.288 * 1.0              # wheel_separation x multiplier, config/my_controllers.yaml
LEAD_DEG = 3.0


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    target = float(sys.argv[1])
    power = int(sys.argv[2]) if len(sys.argv) > 2 else 170
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
            ser.write(b'o 0 0\r')
            time.sleep(0.15)
        time.sleep(0.6)
        a, dl, dr = angle()
        print('STOPPED: %s' % reason)
        print('turned %.1f degrees (%s); wheel counts left %d, right %d' % (abs(a), 'left' if a > 0 else 'right', dl, dr))
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
            ser.write(('o %d %d\r' % (-sign * power, sign * power)).encode())
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop('stopped by you')


if __name__ == '__main__':
    main()
