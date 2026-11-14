"""Complete-ring analysis: for each offset distance d from the robot footprint, is the whole contour covered?"""
import math
from camsim import *

x0, x1, y0, y1 = FOOT


def contour(d, n=720):
    """n points on the rounded-rectangle offset of the footprint at distance d"""
    W, Hh = x1 - x0, y1 - y0
    per = 2 * (W + Hh) + 2 * math.pi * d
    pts = []
    for k in range(n):
        s = per * k / n
        segs = [('L', W), ('A', math.pi / 2 * d), ('L', Hh), ('A', math.pi / 2 * d),
                ('L', W), ('A', math.pi / 2 * d), ('L', Hh), ('A', math.pi / 2 * d)]
        # start at rear-right corner going +x along the right side
        corners = [(x0, y0, -90), (x1, y0, 0), (x1, y1, 90), (x0, y1, 180)]
        ci = 0
        for i, (t, L) in enumerate(segs):
            if s <= L or i == len(segs) - 1:
                cx, cy, ang = corners[i // 2]
                if t == 'L':
                    a = math.radians(ang)
                    dirx, diry = math.cos(a + math.pi / 2), math.sin(a + math.pi / 2)
                    pts.append((cx + dirx * s + d * math.cos(a), cy + diry * s + d * math.sin(a), 0.0))
                else:
                    cxx, cyy, _ = corners[(i // 2 + 1) % 4]
                    a = math.radians(ang) + (s / d if d > 0 else 0)
                    pts.append((cxx + d * math.cos(a), cyy + d * math.sin(a), 0.0))
                break
            s -= L
    return pts


def ring_profile(cams, ds, z=0.0, n=720):
    out = []
    for d in ds:
        pts = [(p[0], p[1], z) for p in contour(d, n)]
        mono = st = 0
        for q in pts:
            b = max(c.sees(q) for c in cams)
            mono += b >= 1
            st += b == 2
        out.append((d, mono / n, st / n))
    return out


def interval(prof, idx):
    """largest run of d where coverage == 100%"""
    best, cur = None, None
    for row in prof:
        if row[idx] >= 0.999:
            cur = (cur[0], row[0]) if cur else (row[0], row[0])
            if not best or cur[1] - cur[0] > best[1] - best[0]:
                best = cur
        else:
            cur = None
    return best
