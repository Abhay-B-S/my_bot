#!/usr/bin/env python3
"""Motor load benchmark: how fast do the wheels turn at a fixed power, with and without weight?

Run ON THE PI, with the robot controller (launch_robot) STOPPED, the robot ON THE FLOOR with about 1.5 m of
clear space ahead and behind, and your hand on the power switch. Each run repeats the same steps and appends
its numbers to ~/load_tests.csv, so different loads can be compared.

  python3 motor_load_test.py noload            # first run: no extra weight (the baseline)
  python3 motor_load_test.py 2kg               # later runs: same label as you like
  python3 motor_load_test.py --compare         # table of all runs, speed relative to the first run per step

Steps (about 2 s each, then a pause): forward at three powers, backward, and spinning in place both ways
(the hardest case for a 4-wheel robot, because the tyres must scrub sideways).
A wheel that turns far slower than in the straight steps during a spin is flagged STALLED.
The firmware stops the motors 2 s after the last command; this script re-sends the command every 0.4 s.
"""
import csv
import os
import sys
import time

import serial

PORT = '/dev/arduino'
BAUD = 57600
CSV = os.path.expanduser('~/load_tests.csv')
STEP_SECONDS = 1.6
REST_SECONDS = 1.8
# name, left power, right power  (power is 0..255, negative = backward)
STEPS = [
    ('forward_120', 120, 120),
    ('forward_170', 170, 170),
    ('forward_220', 220, 220),
    ('backward_170', -170, -170),
    ('backward_220', -220, -220),
    ('spin_left_170', -170, 170),
    ('spin_right_170', 170, -170),
    ('spin_left_230', -230, 230),
    ('spin_right_230', 230, -230),
]


def ask(ser, text):
    ser.reset_input_buffer()
    ser.write((text + '\r').encode())
    time.sleep(0.12)
    return ser.readline().decode(errors='ignore').strip()


def counters(ser):
    parts = ask(ser, 'e').split()
    return int(parts[0]), int(parts[1])


def run():
    label = sys.argv[1]
    ser = serial.Serial(PORT, BAUD, timeout=0.5)
    time.sleep(2.5)                                   # opening the port resets the Arduino
    rows = []
    print('%-16s %10s %10s %8s   %s' % ('step', 'left c/s', 'right c/s', 'R/L', 'note'))
    try:
        for name, left, right in STEPS:
            l0, r0 = counters(ser)
            t0 = time.time()
            while time.time() - t0 < STEP_SECONDS:
                ser.write(('o %d %d\r' % (left, right)).encode())
                time.sleep(0.4)
            elapsed = time.time() - t0
            ser.write(b'o 0 0\r')
            time.sleep(0.4)
            l1, r1 = counters(ser)
            lc, rc = (l1 - l0) / elapsed, (r1 - r0) / elapsed
            rows.append((name, lc, rc))
            ratio = (rc / lc) if abs(lc) > 20 else float('nan')
            print('%-16s %10.0f %10.0f %8.2f' % (name, lc, rc, ratio), flush=True)
            time.sleep(REST_SECONDS)
    except KeyboardInterrupt:
        print('stopped by you')
    finally:
        try:
            ser.write(b'o 0 0\r')
            ser.close()
        except Exception:
            pass
    # stall check on the spins, relative to the straight drive at the same wheel
    ref = {n: (abs(l), abs(r)) for n, l, r in rows}
    if 'forward_170' in ref:
        fl, fr = ref['forward_170']
        for n, l, r in rows:
            if n.startswith('spin') and (abs(l) < 0.15 * fl or abs(r) < 0.15 * fr):
                print('!! %s: a wheel barely turned (STALLED): the motors cannot spin the robot in place with this load' % n)
    new = not os.path.exists(CSV)
    with open(CSV, 'a', newline='') as f:
        w = csv.writer(f)
        if new:
            w.writerow(['time', 'label', 'step', 'left_cps', 'right_cps'])
        for n, l, r in rows:
            w.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), label, n, round(l), round(r)])
    print('saved to', CSV)


def compare():
    if not os.path.exists(CSV):
        sys.exit('no runs recorded yet')
    runs, order = {}, []
    for r in csv.DictReader(open(CSV)):
        key = r['label'] + ' ' + r['time'][11:16]
        if key not in runs:
            runs[key] = {}
            order.append(key)
        runs[key][r['step']] = (float(r['left_cps']), float(r['right_cps']))
    base = runs[order[0]]
    print('speed of each run relative to the first run (%s): 100%% = same speed as the baseline' % order[0])
    print('%-16s' % 'step' + ''.join('%22s' % k for k in order))
    for name, _, _ in STEPS:
        cells = []
        for k in order:
            if name in runs[k] and name in base:
                sp = (abs(runs[k][name][0]) + abs(runs[k][name][1])) / max(1.0, abs(base[name][0]) + abs(base[name][1]))
                cells.append('%21.0f%%' % (100 * sp))
            else:
                cells.append('%22s' % '-')
        print('%-16s' % name + ''.join(cells))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
    elif sys.argv[1] == '--compare':
        compare()
    else:
        run()
