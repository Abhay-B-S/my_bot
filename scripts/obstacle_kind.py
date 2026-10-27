#!/usr/bin/env python3
"""Person or object? Classifies the obstacle that blocks the route from ~1.5-4 s of 2D lidar scans (YDLIDAR X2:
7 scans/s, 0.84 deg, 22 cm above the floor = shin height). Used by path_guard.py while it waits.

Rules from the research notes (research_notes/Prior art and evaluation plan/person_object_and_explanations.md;
Arras et al. 2007, Leigh et al. 2015, Bellotto & Hu 2005). With few points per leg the decision rests mostly on MOTION
and on leg PAIRS, and there is an explicit "unsure" that keeps the old, safe behaviour:
  segments  split the obstacle's points where neighbours are more than 13 cm apart
  leg       a segment 4-20 cm wide (2+ points, 3+ closer than 1.5 m), not a straight line
  leg pair  two legs 10-50 cm apart (one step), or one 20-45 cm segment (both legs merged)
  PERSON    the obstacle moved more than 15 cm, or a leg pair in >= 60 % of the scans while it sways (> 2 cm)
  OBJECT    a flat face wider than 40 cm, or no leg pair in >= 80 % of the scans and perfectly still
  UNSURE    anything else (a person standing very still, a coat, chair legs ...)
Calling an object a person only costs a longer wait; calling a person an object makes the robot drive round someone
who would have stepped aside (still safe), so borderline cases go to UNSURE or PERSON.

The thresholds are starting values (engineering guesses in the research), to be tuned on the robot's own scans.
"""
import math

import numpy as np

JUMP = 0.13                  # m between neighbouring points that splits two segments
LEG_W = (0.04, 0.20)         # m: width of one leg seen at shin height
PAIR_D = (0.10, 0.50)        # m between the centres of two legs (one step)
MERGED_W = (0.20, 0.45)      # m: both legs in one segment
FLAT_W, FLAT_RES = 0.40, 0.01  # m: a wide straight face (box, door, bin)
MOVED = 0.15                 # m the obstacle's centre moved = someone walking
SWAY = 0.02                  # m (std): legs / body moving a little
STILL_C, STILL_W = 0.015, 0.01  # m (std): perfectly still


def segments(pts):
    """Split points (Nx2, in scan order) into segments at gaps larger than JUMP."""
    if len(pts) == 0:
        return []
    gaps = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1])) > JUMP
    return [s for s in np.split(pts, np.nonzero(gaps)[0] + 1) if len(s) >= 2]


def line_residual(seg):
    """RMS distance (m) of the points from their best straight line."""
    c = seg - seg.mean(axis=0)
    if len(seg) < 3:
        return 0.0
    _, sv, _ = np.linalg.svd(c, full_matrices=False)
    return float(sv[-1] / math.sqrt(len(seg)))


def scan_features(pts, laser_xy):
    """Shape of the obstacle in one scan: (centre, width, leg pair seen, flat face seen)."""
    segs = segments(pts)
    rng = float(np.hypot(*(pts.mean(axis=0) - laser_xy))) if len(pts) else 9.0
    legs, pair, flat = [], False, False
    for s in segs:
        w = float(np.hypot(*(s[-1] - s[0])))
        if w > FLAT_W and line_residual(s) < FLAT_RES:
            flat = True
        need = 3 if rng < 1.5 else 2
        if LEG_W[0] <= w <= LEG_W[1] and len(s) >= need and (len(s) < 4 or line_residual(s) > 0.003):
            legs.append(s.mean(axis=0))
        if MERGED_W[0] <= w <= MERGED_W[1] and len(s) >= 4 and line_residual(s) > 0.008:
            pair = True                                # both legs merged into one curved segment
    for i in range(len(legs)):
        for j in range(i + 1, len(legs)):
            if PAIR_D[0] <= float(np.hypot(*(legs[i] - legs[j]))) <= PAIR_D[1]:
                pair = True
    width = float(np.hypot(*(pts[-1] - pts[0]))) if len(pts) > 1 else 0.0
    return pts.mean(axis=0) if len(pts) else None, width, pair, flat


def classify(history, laser_xy, min_scans=8):
    """history: list of Nx2 point arrays (one per scan, scan order) of the blocking obstacle.
    Returns ('person' | 'object' | 'unsure', reason)."""
    feats = [scan_features(p, laser_xy) for p in history if len(p) >= 2]
    if len(feats) < min_scans:
        return 'unsure', 'too few scans of it (%d)' % len(feats)
    centres = np.array([f[0] for f in feats])
    widths = np.array([f[1] for f in feats])
    moved = float(np.hypot(*(centres - centres[0]).T).max())
    c_std = float(np.sqrt(centres.var(axis=0).sum()))
    w_std = float(widths.std())
    pair_rate = float(np.mean([f[2] for f in feats]))
    flat_rate = float(np.mean([f[3] for f in feats]))
    info = 'moved %.0f cm, sway %.1f/%.1f cm, legs in %.0f %% of scans' % (100 * moved, 100 * c_std, 100 * w_std,
                                                                         100 * pair_rate)
    if moved > MOVED:
        return 'person', 'it moves (%s)' % info
    if pair_rate >= 0.6 and (c_std > SWAY or w_std > SWAY):
        return 'person', 'two legs that sway (%s)' % info
    if flat_rate >= 0.5:
        return 'object', 'a flat face wider than %.0f cm (%s)' % (100 * FLAT_W, info)
    if pair_rate < 0.2 and c_std < STILL_C and w_std < STILL_W:
        return 'object', 'perfectly still, no legs (%s)' % info
    return 'unsure', info
