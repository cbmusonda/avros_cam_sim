"""Lane-line detectability model for the 4x ZED X layouts (IGVC AutoNav).

For a 3 in (0.0762 m) white line on flat ground, compute its apparent thickness in pixels in each camera that sees the
point with valid stereo depth (rectified pinhole, 1920x1200), and score IGVC-relevant zones vs. mount height / pitch.

Run:  python3 lines.py            (full sweep, writes results/lines_sweep.txt)
      python3 lines.py check      (projection sanity checks only)
"""
import math, sys, itertools
from multiprocessing import Pool
from camsim import Cam, LENS, OCC, LIDAR, lidar_shadow, D2R, inside_box
from ring import ring_profile, interval

W_PX, H_PX = 1920, 1200
LINE_W = 0.0762          # 3 in
POT_D = 0.61             # 2 ft pothole
BASE = 0.12
FOOT = (-0.1778, 0.7271, -0.41, 0.41)


def fpx(cam):
    return (W_PX / 2) / cam.th            # rectified focal length in px (2.2 mm ~733, 4 mm ~1333)


def project(cam, q, eye):
    x, y, z = cam.local(q, eye)
    f = fpx(cam)
    return (-f * y / x, -f * z / x), x


def thickness(cam, P, d, w=LINE_W):
    """apparent thickness (px) of a ground line through P with unit ground direction d, in the left image.
    = perpendicular distance between the two projected line edges (not the cross-segment length)."""
    eye = cam.eyes[0]
    n = (-d[1], d[0], 0.0)
    a, _ = project(cam, (P[0] - n[0] * w / 2, P[1] - n[1] * w / 2, P[2]), eye)
    b, _ = project(cam, (P[0] + n[0] * w / 2, P[1] + n[1] * w / 2, P[2]), eye)
    e = 0.01
    c0, _ = project(cam, P, eye)
    c1, _ = project(cam, (P[0] + d[0] * e, P[1] + d[1] * e, P[2]), eye)
    tx, ty = c1[0] - c0[0], c1[1] - c0[1]
    tn = math.hypot(tx, ty)
    dx, dy = b[0] - a[0], b[1] - a[1]
    if tn < 1e-9:
        return math.hypot(dx, dy)
    return abs(tx * dy - ty * dx) / tn


def seg_len_px(cam, P, d, L=1.0):
    eye = cam.eyes[0]
    a, _ = project(cam, (P[0] - d[0] * L / 2, P[1] - d[1] * L / 2, P[2]), eye)
    b, _ = project(cam, (P[0] + d[0] * L / 2, P[1] + d[1] * L / 2, P[2]), eye)
    return math.hypot(b[0] - a[0], b[1] - a[1])


def depth_err(cam, P, sd):
    z = cam.local(P, cam.eyes[0])[0]
    return z * z * sd / (fpx(cam) * BASE)


def grazing(cam, P):
    o = cam.eyes[0]
    h = o[2] - P[2]
    r = math.hypot(P[0] - o[0], P[1] - o[1])
    return math.degrees(math.atan2(h, r))


def best_view(cams, P, d):
    """(thickness px, cam) of the best stereo-valid camera for a line at P with direction d; (0, None) if none."""
    best, bc = 0.0, None
    for c in cams:
        if c.sees(P) == 2:
            t = thickness(c, P, d)
            if t > best:
                best, bc = t, c
    return best, bc


# ───────── zones (base_link: x fwd, y left) ─────────
def frange(a, b, s):
    n = int(round((b - a) / s))
    return [a + i * s for i in range(n + 1)]


def zones():
    Z = {}
    Z['Z1'] = [((x, y, 0.0), (1.0, 0.0)) for y in (1.52, -1.52, 0.9, -0.9, 2.2, -2.2, 3.0, -3.0) for x in frange(-1, 2, 0.1)]
    Z['Z2'] = [((x, y, 0.0), (1.0, 0.0)) for y in (1.52, -1.52) for x in frange(1, 8, 0.1)]
    z3 = []
    for s in (1, -1):                       # left / right turn, robot on lane centre, curve starts at x = 1.0
        for R in (1.52, 4.57):              # inner (5 ft) and outer line, lane 10 ft wide
            cx, cy = 1.0, s * 3.05
            for k in range(31):
                a = -math.pi / 2 + (math.pi / 2) * k / 30   # sweep a quarter turn ahead of the robot
                px, py = cx + R * math.cos(a), cy + s * R * math.sin(a)
                # tangent direction
                tx, ty = -math.sin(a), s * math.cos(a)
                z3.append(((px, py, 0.0), (tx, ty)))
    Z['Z3'] = z3
    z4 = []
    for x in frange(-4.5, 5.0, 0.25):
        for y in frange(-4.5, 4.5, 0.25):
            dx = max(FOOT[0] - x, 0, x - FOOT[1]); dy = max(FOOT[2] - y, 0, y - FOOT[3])
            if 0.5 <= math.hypot(dx, dy) <= 4.0:
                z4.append((x, y, 0.0))
    Z['Z4'] = z4
    return Z


ZONES = zones()
SCALE_HALF, SCALE_THIRD = 0.5, 1 / 3     # 960x600, 640x400 (~640x384)


def line_ok(t):
    return t >= 5 and t * SCALE_HALF >= 3


def pothole_ok(cams, P):
    """centre stereo-valid and the projected disc's minor axis >= 10 px at full res (5 px at 960x600)"""
    best = 0.0
    for c in cams:
        if c.sees(P) != 2:
            continue
        # minor axis ~ min over directions of projected diameter
        m = min(thickness(c, P, (math.cos(a), math.sin(a)), POT_D) for a in (0, math.pi / 4, math.pi / 2, 3 * math.pi / 4))
        best = max(best, m)
    return best >= 10


def zone_scores(cams):
    out = {}
    for k in ('Z1', 'Z2', 'Z3'):
        pts = ZONES[k]
        ok = sum(1 for P, d in pts if line_ok(best_view(cams, P, (d[0], d[1]))[0]))
        out[k] = ok / len(pts)
    out['Z4'] = sum(1 for P in ZONES['Z4'] if pothole_ok(cams, P)) / len(ZONES['Z4'])
    return out


def ring(cams, fine=False):
    ds = frange(0, 7, 0.1 if fine else 0.2)
    p = ring_profile(cams, ds, 0.0, 360 if fine else 180)
    return interval(p, 2)


def farthest_ahead(cams, y=1.52, perp=False, crit=line_ok):
    """farthest x (contiguous run starting from x where the line first becomes OK) for the line at +y and -y (min of both)"""
    res = []
    for yy in (y, -y):
        run_end, started = None, False
        for x in frange(0.8, 16, 0.1):
            d = (0.0, 1.0) if perp else (1.0, 0.0)
            ok = crit(best_view(cams, (x, yy, 0.0), d)[0])
            if ok:
                started, run_end = True, x
            elif started:
                break
        res.append(run_end or 0.0)
    return min(res)


# ───────── layouts ─────────
RC = 0.12
XM = LIDAR[0]


def crown(H, pf, pr, ps, yw=90.0, xm=XM):
    return [Cam('front', '2.2', xm + RC, 0, H, 0, pf), Cam('rear', '2.2', xm - RC, 0, H, 180, pr),
            Cam('left', '4', xm, RC, H, yw, ps), Cam('right', '4', xm, -RC, H, -yw, ps)]


def body(H, pf, pr, ps, yw=90.0, xs=0.30):
    return [Cam('front', '2.2', 0.71, 0, H, 0, pf), Cam('rear', '2.2', -0.20, 0, H, 180, pr),
            Cam('left', '4', xs, 0.36, H, yw, ps), Cam('right', '4', xs, -0.36, H, -yw, ps)]


LAYOUT = {'crown': crown, 'body': body}
PF_MAX = 39.3 - 5.0     # front top edge >= +5 deg above horizon


def objective(kind, H, v):
    cams = LAYOUT[kind](H, *v)
    if any(inside_box(c.p, 0.01) for c in cams):
        return -1.0, None
    zs = zone_scores(cams)
    iv = ring(cams)
    rterm = 0.0 if not iv else min(iv[1], 5.0) / 5.0 - max(0.0, iv[0] - 0.6) / 2
    S = 0.25 * zs['Z1'] + 0.30 * zs['Z2'] + 0.15 * zs['Z3'] + 0.15 * zs['Z4'] + 0.15 * rterm
    return S, (zs, iv)


def _obj(args):
    kind, H, v = args
    return objective(kind, H, v)[0], v


def optimise(kind, H, P):
    # coarse grid (sides yaw 90, plus xs for body), then coordinate refinement
    grid = []
    for pf in frange(10, 34, 4):
        for pr in frange(15, 39, 6):
            for ps in frange(10, 50, 5):
                if kind == 'body':
                    for yw in (75, 90, 105):
                        grid.append((kind, H, (pf, pr, ps, yw)))
                else:
                    grid.append((kind, H, (pf, pr, ps)))
    res = P.map(_obj, grid)
    best_s, best_v = max(res, key=lambda r: r[0])
    best_v = list(best_v)
    lo = [5, 5, 5, 60][:len(best_v)]
    hi = [PF_MAX, 39, 55, 120][:len(best_v)]
    step = [2.0, 3.0, 2.5, 7.5][:len(best_v)]
    for _ in range(4):
        cands = []
        for i in range(len(best_v)):
            for sgn in (-1, 1):
                v = list(best_v)
                v[i] = min(max(v[i] + sgn * step[i], lo[i]), hi[i])
                cands.append((kind, H, tuple(v)))
        r = P.map(_obj, cands)
        s, v = max(r, key=lambda t: t[0])
        if s > best_s:
            best_s, best_v = s, list(v)
        else:
            step = [x / 2 for x in step]
    return best_s, best_v


def report(kind, H, v):
    cams = LAYOUT[kind](H, *v)
    S, (zs, _) = objective(kind, H, v)
    iv = ring(cams, fine=True)
    far5 = farthest_ahead(cams)
    far_q = farthest_ahead(cams, crit=lambda t: t * SCALE_THIRD >= 3)
    far_perp = farthest_ahead(cams, perp=True)
    front, left = cams[0], cams[2]
    de = []
    for d in (2, 4, 6):
        P = (FOOT[1] + d, 0.0, 0.0)
        de.append((depth_err(front, P, 0.1), depth_err(front, P, 0.25)) if front.sees(P) == 2 else (None, None))
    deS = []
    for d in (2, 4):
        P = (left.p[0], FOOT[3] + d, 0.0)
        deS.append((depth_err(left, P, 0.1), depth_err(left, P, 0.25)) if left.sees(P) == 2 else (None, None))
    # widths / grazing on the 10 ft lane line
    abeam = (0.3, 1.52, 0.0)
    t_ab, c_ab = best_view(cams, abeam, (1, 0))
    g_ab = grazing(c_ab, abeam) if c_ab else None
    w4 = best_view(cams, (4.0, 1.52, 0.0), (1, 0))
    w8 = best_view(cams, (8.0, 1.52, 0.0), (1, 0))
    p4 = best_view(cams, (4.0, 1.52, 0.0), (0, 1))
    dash4 = seg_len_px(w4[1], (4.0, 1.52, 0.0), (1, 0)) if w4[1] else 0
    g4 = grazing(front, (4.0, 1.52, 0.0)); g8 = grazing(front, (8.0, 1.52, 0.0))
    sh = lidar_shadow(cams, 180) if kind == 'crown' and H < 1.0 else 0.0
    return dict(kind=kind, H=H, v=v, S=S, zs=zs, ring=iv, far5=far5, farq=far_q, far_perp=far_perp, de=de, deS=deS,
                t_ab=t_ab, cam_ab=c_ab.name if c_ab else '-', g_ab=g_ab, w4=w4[0], w8=w8[0], p4=p4[0], dash4=dash4,
                g4=g4, g8=g8, lidar=sh)


def sanity():
    lines = []
    for lens in ('2.2', '4'):
        for h in (0.5, 1.0, 1.5):
            c = Cam('t', lens, 0, 0, h, 0, 90)
            P = (0.0, 0.0, 0.0)
            # eye is offset 6 cm sideways; put P under the left eye
            P = (c.eyes[0][0], c.eyes[0][1], 0.0)
            t = thickness(c, P, (1, 0))
            lines.append(f'down-looking {lens} mm h={h}: model {t:.2f} px, f*w/h = {fpx(c) * LINE_W / h:.2f} px (f={fpx(c):.0f})')
    # oblique: front camera level at h=1, line across at d: thickness = f*w*h/(d^2+h^2)*... check vs numeric
    c = Cam('t', '2.2', 0, 0, 1.0, 0, 0)
    P = (4.0, c.eyes[0][1], 0.0)
    t = thickness(c, P, (0, 1))
    # analytic: v = f*h/x  -> dv/dx = f*h/x^2
    lines.append(f'level cam h=1, perpendicular line at x=4: model {t:.3f} px, analytic f*h*w/x^2 = {fpx(c) * 1.0 * LINE_W / 16:.3f} px')
    t2 = thickness(c, (4.0, 1.0 + c.eyes[0][1], 0.0), (1, 0))
    lines.append(f'level cam h=1, parallel line at x=4,y=1: model {t2:.3f} px (lateral width ~ f*w/x = {fpx(c) * LINE_W / 4:.3f} px, reduced by obliquity)')
    return lines


def fmt_row(r):
    zs = r['zs']
    iv = r['ring']
    de = ' '.join('-' if a is None else f'{a*100:.1f}-{b*100:.0f}' for a, b in r['de'])
    deS = ' '.join('-' if a is None else f'{a*100:.1f}-{b*100:.0f}' for a, b in r['deS'])
    v = '/'.join(f'{x:.0f}' for x in r['v'])
    ring_s = f'{iv[0]:.1f}-{iv[1]:.1f}' if iv else 'open'
    return (f"{r['kind']:5s} {r['H']:4.2f}  {v:15s} {zs['Z1']*100:5.0f} {zs['Z2']*100:5.0f} {zs['Z3']*100:5.0f} {zs['Z4']*100:5.0f}  "
            f"{ring_s:8s} {r['far5']:5.1f} {r['farq']:5.1f} {r['far_perp']:5.1f}  "
            f"{r['t_ab']:5.1f}({r['cam_ab'][:1]}) {r['g_ab'] or 0:4.0f}°  {r['w4']:4.1f} {r['p4']:4.1f} {r['w8']:4.1f}  {r['dash4']:5.0f}  "
            f"{r['g4']:4.1f}° {r['g8']:4.1f}°  {de:22s} {deS:14s} {r['lidar']*100:4.1f}")


HDR = ("kind  H     pitch F/R/S[/yaw] Z1%   Z2%   Z3%   Z4%   ring(m)  ahd5 ahd3q perp   abeam px   graz  4m   4m⊥  8m    dash4  graz4 graz8  depthErr F 2/4/6m (cm) side 2/4m(cm) lidar%\n"
       "                                                                         (x,m)  (x,m) (x,m)                    ∥    ⊥    ∥     1m px")

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'check':
        print('\n'.join(sanity()))
        sys.exit()
    out = ['# Lane-line detectability sweep (lines.py)', '', '## Sanity checks'] + sanity() + ['']
    rows = []
    with Pool(16) as P:
        for kind, Hs in (('crown', (0.8, 1.0, 1.15, 1.3, 1.45, 1.6, 1.75)), ('body', (0.45, 0.6, 0.7))):
            for H in Hs:
                s, v = optimise(kind, H, P)
                r = report(kind, H, v)
                rows.append(r)
                print(fmt_row(r), flush=True)
        # also report the currently committed preset (H=1.5, 30/37/37) for reference
        ref = report('crown', 1.5, [30, 37, 37])
        rows.append(ref)
        print('preset:', fmt_row(ref), flush=True)
    out += ['## Sweep (pitches optimised per height; preset row = committed 1.5 m 30/37/37)', HDR] + [fmt_row(r) for r in rows]
    out += ['', 'Columns: Z1 alongside lines (y=±0.9..3.0, x -1..2), Z2 ahead (y=±1.52, x 1..8), Z3 5-ft-radius curve lines,',
            'Z4 2-ft potholes 0.5-4 m. Line OK = stereo-valid AND >=5 px at 1920x1200 AND >=3 px at 960x600 (i.e. >=6 px full res).',
            'Pothole OK = stereo-valid centre and minor axis >=10 px. ring = full 360 stereo ground ring (m off footprint).',
            'ahd5 = farthest x ahead (contiguous) where both 10-ft-lane lines pass the line test; ahd3q = >=3 px at 640x400;',
            'perp = same for a line across the path (stop line / dash end). abeam px = best thickness of the lane line beside the robot (x=0.3)',
            '(F/L/R camera), graz = ray-to-ground angle there. 4m/8m = thickness of lane line at x=4/8; ⊥ = a crossing line at x=4.',
            'dash4 = length in px of a 1 m dash at x=4. depthErr = z^2*sd/(f*B), sd 0.1-0.25 px. lidar% = beams newly blocked by crown.']
    open('results/lines_sweep.txt', 'w').write('\n'.join(out) + '\n')
