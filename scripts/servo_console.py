#!/usr/bin/env python3
"""Type an angle, the servo moves. An interactive console for the servo Nano (run it ON THE PI).

  python3 src/my_bot/scripts/servo_console.py                  # uses /dev/servo_nano
  python3 src/my_bot/scripts/servo_console.py /dev/ttyUSB1     # another port

At the prompt (servo 0 = lid on D9, servo 1 = lock on D10, servo 2 = D11):
  90            servo 0 to 90 degrees
  1 120         servo 1 to 120 degrees
  g 1           ask which angle servo 1 was last sent
  d 1           let servo 1 go limp (stop holding)
  v 25          slower moves: 25 ms per degree (default 15; 3 = fast, 60 = very slow)
  sweep 0       move servo 0 slowly 20 -> 160 -> 20
  q             quit
Do NOT run it while cargo_controller.py is running (both would use the same port).
"""
import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/servo_nano'


def ask(ser, text, wait=0.25):
    ser.reset_input_buffer()
    ser.write((text + '\n').encode())
    end = time.time() + 6.0
    time.sleep(wait)
    while time.time() < end:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            return line
    return '(no answer)'


def main():
    try:
        ser = serial.Serial(PORT, 57600, timeout=1.0)
    except Exception as exc:
        sys.exit('Cannot open %s: %s' % (PORT, exc))
    time.sleep(2.5)                                    # the Nano restarts when the port opens
    ser.reset_input_buffer()
    print('connected to %s: %s' % (PORT, ask(ser, '?')))
    print(__doc__.split('At the prompt')[1].split('Do NOT')[0].strip('\n'))
    while True:
        try:
            line = input('servo> ').strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        words = line.split()
        if words[0] in ('q', 'quit', 'exit'):
            break
        try:
            if words[0] == 'sweep':
                n = int(words[1]) if len(words) > 1 else 0
                for a in (20, 160, 20):
                    print('  servo %d to %d -> %s' % (n, a, ask(ser, 's %d %d' % (n, a))))
            elif words[0] == 'v':
                print('  ->', ask(ser, 'v %d' % int(words[1])))
            elif words[0] in ('g', 'd'):
                print('  ->', ask(ser, '%s %d' % (words[0], int(words[1]))))
            elif len(words) == 1:
                print('  servo 0 to %d -> %s' % (int(words[0]), ask(ser, 's 0 %d' % int(words[0]))))
            else:
                print('  servo %d to %d -> %s' % (int(words[0]), int(words[1]), ask(ser, 's %d %d' % (int(words[0]), int(words[1])))))
        except (ValueError, IndexError):
            print('  type an angle (90), or: <servo> <angle>, v <ms>, g <servo>, d <servo>, sweep <servo>, q')
    ser.close()


if __name__ == '__main__':
    main()
