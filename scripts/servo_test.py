#!/usr/bin/env python3
"""Move the servo on the Nano (firmware/servo_nano) from the command line.

Usage:
  python3 servo_test.py <port> <angle>        # servo 0 to <angle> (0..180), e.g. /dev/ttyUSB0 90
  python3 servo_test.py <port> sweep          # slowly 20 -> 160 -> 20 degrees, twice
  python3 servo_test.py <port> <angle> <n>    # servo number n (0, 1 or 2)
The Nano restarts when the port is opened, so the script waits about 2.5 s before the first command.
"""
import sys
import time

import serial


def send(ser, text):
    ser.reset_input_buffer()
    ser.write((text + '\n').encode())
    time.sleep(0.2)
    return ser.readline().decode(errors='ignore').strip()


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(1)
    port, what = sys.argv[1], sys.argv[2]
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 0
    ser = serial.Serial(port, 57600, timeout=3)
    time.sleep(2.5)
    ser.reset_input_buffer()
    print('hello:', send(ser, '?'))
    if what == 'sweep':
        for _ in range(2):
            for a in (20, 160, 20):
                print('servo %d to %d ->' % (n, a), send(ser, 's %d %d' % (n, a)))
                time.sleep(0.3)
    else:
        print('servo %d to %s ->' % (n, what), send(ser, 's %d %d' % (n, int(what))))
    ser.close()


if __name__ == '__main__':
    main()
