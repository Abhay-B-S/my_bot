#!/usr/bin/env python3
"""Check recorded spots against a saved map, before the robot drives to them.

Usage:  python3 src/my_bot/scripts/check_locations.py maps/practice_room.yaml src/my_bot/config/locations_real.yaml

Prints, for every spot: is it on free floor, how far is the nearest wall or obstacle, and whether the
robot (kept 0.30 m from walls) can find a route from every spot to every other spot.
Rule of thumb: clearance >= 0.45 m is good, 0.35 to 0.45 m is a bit close, below 0.35 m should be recorded again.
"""
import math
import sys
from collections import deque

import numpy as np
import yaml
from PIL import Image


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    map_yaml, loc_yaml = sys.argv[1], sys.argv[2]
    import os
    m = yaml.safe_load(open(map_yaml))
    pgm = m['image'] if os.path.isabs(m['image']) else os.path.join(os.path.dirname(os.path.abspath(map_yaml)), m['image'])
    img = np.array(Image.open(pgm))
    h, w = img.shape
    res = m['resolution']
    ox, oy = m['origin'][:2]
    occ = img < 90
    free = img > 230
    ys, xs = np.nonzero(occ)
    wx, wy = ox + xs * res, oy + (h - 1 - ys) * res
    loc = yaml.safe_load(open(loc_yaml))['locations']

    def cell(p):
        return int(round((p['x'] - ox) / res)), h - 1 - int(round((p['y'] - oy) / res))

    print('spot        x       y     yaw  floor   clearance  verdict')
    for n, p in loc.items():
        c, r = cell(p)
        on_floor = 0 <= r < h and 0 <= c < w and free[r, c]
        d = float(np.min(np.hypot(wx - p['x'], wy - p['y'])))
        verdict = ('good' if d >= 0.45 else 'a bit close' if d >= 0.35 else 'TOO CLOSE, record again')
        if not on_floor:
            verdict = 'NOT ON FREE FLOOR, record again'
        print('%-8s %7.2f %7.2f %7.2f  %-6s  %.2f m     %s' % (n, p['x'], p['y'], p['yaw'], 'free' if on_floor else 'NO', d, verdict))
    names = list(loc)
    if len(names) > 1:
        close = min((math.hypot(loc[a]['x'] - loc[b]['x'], loc[a]['y'] - loc[b]['y']), a, b)
                    for i, a in enumerate(names) for b in names[i + 1:])
        print('closest pair: %s and %s, %.2f m%s' % (close[1], close[2], close[0], '' if close[0] >= 1.0 else '  (under 1 m, consider moving one)'))

    def grow(mask, radius_m):
        r = int(math.ceil(radius_m / res))
        out = mask.copy()
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    out |= np.roll(np.roll(mask, dy, 0), dx, 1)
        return out

    passable = free & ~grow(occ, 0.30)
    bad = []
    for a in names:
        sx, sy = cell(loc[a])
        if not passable[sy, sx]:
            bad.append('%s itself is closer than 0.30 m to a wall' % a)
            continue
        seen = np.zeros((h, w), bool)
        q = deque([(sx, sy)])
        seen[sy, sx] = True
        while q:
            x, y = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = x + dx, y + dy
                if 0 <= nx < w and 0 <= ny < h and passable[ny, nx] and not seen[ny, nx]:
                    seen[ny, nx] = True
                    q.append((nx, ny))
        for b in names:
            c, r = cell(loc[b])
            if b != a and passable[r, c] and not seen[r, c]:
                bad.append('no route %s -> %s (narrow gap?)' % (a, b))
    print('routes with a 0.30 m margin: ' + ('every spot reaches every other spot' if not bad else ''))
    for line in bad:
        print('  PROBLEM:', line)


if __name__ == '__main__':
    main()
