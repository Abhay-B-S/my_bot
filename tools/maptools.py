"""maptools.py loops MAP.yaml | spots MAP.yaml SPOTS.yaml | route MAP.yaml SPOTS.yaml A B
loops: obstacle islands the robot (0.20 m clearance = padded footprint) can drive all the way round
spots: distance of each spot to the nearest wall; route: shortest drivable route length A->B"""
import sys, yaml, heapq, math
from collections import deque
import numpy as np
from PIL import Image
mp = yaml.safe_load(open(sys.argv[2])); im = np.array(Image.open(mp['image'])); H, W = im.shape
res = mp['resolution']; ox, oy = mp['origin'][:2]
occ = im < 100; free = im > 250
def inflate(r):
    out = occ.copy(); ys, xs = np.where(occ)
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if dx * dx + dy * dy <= r * r:
                out[np.clip(ys + dy, 0, H - 1), np.clip(xs + dx, 0, W - 1)] = True
    return free & ~out
def label(mask):
    lab = np.zeros(mask.shape, int); n = 0
    for y0, x0 in zip(*np.where(mask)):
        if lab[y0, x0]: continue
        n += 1; q = deque([(y0, x0)]); lab[y0, x0] = n
        while q:
            y, x = q.popleft()
            for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                yy, xx = y + a, x + b
                if 0 <= yy < H and 0 <= xx < W and mask[yy, xx] and not lab[yy, xx]:
                    lab[yy, xx] = n; q.append((yy, xx))
    return lab, n
def cell(p): return int(round(H - 1 - (p['y'] - oy) / res)), int(round((p['x'] - ox) / res))
cmd = sys.argv[1]
if cmd == 'loops':
    for r in (4, 5, 6):
        ok = inflate(r); lab, n = label(ok); sizes = [(lab == i).sum() for i in range(1, n + 1)]; big = 1 + int(np.argmax(sizes))
        lab2, n2 = label(lab != big); found = []
        for i in range(1, n2 + 1):
            ys, xs = np.where(lab2 == i)
            if ys.min() == 0 or xs.min() == 0 or ys.max() == H - 1 or xs.max() == W - 1 or len(ys) < 30: continue
            found.append('island at x %.1f..%.1f y %.1f..%.1f (%.1f x %.1f m)' % (ox + xs.min() * res, ox + xs.max() * res,
                         oy + (H - 1 - ys.max()) * res, oy + (H - 1 - ys.min()) * res, (xs.max() - xs.min()) * res, (ys.max() - ys.min()) * res))
        print('clearance %.2f m: %d loop(s): %s' % (r * res, len(found), '; '.join(found) or '-'))
else:
    spots = yaml.safe_load(open(sys.argv[3]))['locations']
    if cmd == 'spots':
        ys, xs = np.where(occ)
        for name, p in spots.items():
            cy, cx = cell(p); d = np.sqrt((xs - cx) ** 2 + (ys - cy) ** 2).min() * res
            print('%-8s x %5.2f y %5.2f  nearest wall %.2f m  %s' % (name, p['x'], p['y'], d, 'OK' if d >= 0.4 else 'TOO CLOSE'))
    else:
        ok = inflate(4); s, g = cell(spots[sys.argv[4]]), cell(spots[sys.argv[5]]); ok[s] = ok[g] = True
        dist = {s: 0}; pq = [(0, s)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == g: break
            if d > dist[u]: continue
            for dy in (-1, 0, 1):
                for dx in (-1, 0, 1):
                    v = (u[0] + dy, u[1] + dx)
                    if (dy or dx) and 0 <= v[0] < H and 0 <= v[1] < W and ok[v]:
                        nd = d + math.hypot(dy, dx)
                        if nd < dist.get(v, 1e18): dist[v] = nd; heapq.heappush(pq, (nd, v))
        print('%s -> %s: %s' % (sys.argv[4], sys.argv[5], '%.1f m' % (dist[g] * res) if g in dist else 'NO ROUTE'))
