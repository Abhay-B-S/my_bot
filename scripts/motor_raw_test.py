#!/usr/bin/env python3
"""Hold a steady raw motor power for a while, to test the motors, driver and wiring WITHOUT ROS.

Run it ON THE PI, with the robot controller (launch_robot) stopped, wheels off the floor:
  python3 motor_raw_test.py [left_pwm right_pwm seconds]
  python3 motor_raw_test.py 150 150 15      # both wheels, power 150 of 255, for 15 s
  python3 motor_raw_test.py 150 0 15        # left wheel only
  python3 motor_raw_test.py 0 150 15        # right wheel only
  python3 motor_raw_test.py -150 -150 10    # both backward
While it runs you can measure with a multimeter. It prints the wheel counters before and after.
The firmware stops the motors 2 s after the last command, so this re-sends the command every 0.5 s.
"""
import sys
import time

import serial

PORT = '/dev/arduino'
BAUD = 57600


def ask(ser, text):
    ser.reset_input_buffer()
    ser.write((text + '\r').encode())
    time.sleep(0.15)
    return ser.readline().decode(errors='ignore').strip()


def main():
    args = sys.argv[1:]
    left = int(args[0]) if len(args) > 0 else 150
    right = int(args[1]) if len(args) > 1 else 150
    secs = float(args[2]) if len(args) > 2 else 10.0
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(2.5)                                  # opening the port resets the Arduino
    print('encoders before:', ask(ser, 'e'))
    print('sending o %d %d for %.0f s ... (Ctrl-C stops)' % (left, right, secs))
    end = time.time() + secs
    try:
        while time.time() < end:
            ser.write(('o %d %d\r' % (left, right)).encode())
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        ser.write(b'o 0 0\r')
        time.sleep(0.3)
    print('encoders after: ', ask(ser, 'e'))
    ser.close()


if __name__ == '__main__':
    main()
