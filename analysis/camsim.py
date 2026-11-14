"""Coverage model for 4x ZED X on the AVROS robot (geometry copied from avros-cam-sim/main.js).
World frame = ROS base_link on ground: x fwd, y left, z up. Camera rpy = URDF (pitch + = down)."""
import math

D2R = math.pi / 180
XS = 0.5556
BOXES = [  # centre, size
    ((0.3143, 0, XS - 0.2191), (0.7430, 0.6795, 0.3747)),   # chassis
    ((0.1143, 0, XS + 0.0413), (0.5842, 0.5715, 0.1461)),   # sensor platform
    ((0.3143, 0.3653, XS - 0.4763), (0.8255, 0.0889, 0.1397)),
    ((0.3143, -0.3653, XS - 0.4763), (0.8255, 0.0889, 0.1397)),
]
LIDAR = (0.089, 0.0, XS + 0.159)          # optical centre
LIDAR_BOX = ((0.089, 0, LIDAR[2] - 0.03618 + 0.037), (0.087, 0.087, 0.074))
OCC = [tuple((c[i] - s[i] / 2 for i in range(3))) + tuple((c[i] + s[i] / 2 for i in range(3)))
       for c, s in BOXES + [LIDAR_BOX]]
FOOT = (-0.1778, 0.7271, -0.41, 0.41)      # x0, x1, y0, y1 of the robot footprint

SENSOR_W, SENSOR_H = 5.76, 3.6
def rect(f):
    return 2 * math.atan(SENSOR_W / 2 / f) / D2R, 2 * math.atan(SENSOR_H / 2 / f) / D2R
# rectified (what depth is computed on). Stereolabs support page: 2.2 mm -> 105 x 78 rectified.
LENS = {
    '2.2': dict(h=rect(2.2)[0], v=rect(2.2)[1], dmin=0.3, dmax=12.0),
    '4':   dict(h=rect(4.0)[0], v=rect(4.0)[1], dmin=1.0, dmax=20.0),
}
BASELINE = 0.12
CAM_BOX = (0.031, 0.165, 0.036)  # depth, width, height (used for the lidar-shadow check)


def rot(yaw, pitch, roll=0.0):
    cy, sy = math.cos(yaw * D2R), math.sin(yaw * D2R)
    cp, sp = math.cos(pitch * D2R), math.sin(pitch * D2R)
    cr, sr = math.cos(roll * D2R), math.sin(roll * D2R)
    # R = Rz Ry Rx ; columns = fwd, left, up
    R = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
         [-sp, cp * sr, cp * cr]]
    return R


def seg_blocked(o, p):
    """segment o->p hits an occluder (excluding the endpoints)."""
    d = (p[0] - o[0], p[1] - o[1], p[2] - o[2])
    for b in OCC:
        t0, t1 = 1e-4, 1 - 1e-4
        ok = True
        for i in range(3):
            if abs(d[i]) < 1e-12:
                if o[i] < b[i] or o[i] > b[i + 3]:
                    ok = False
                    break
                continue
            a = (b[i] - o[i]) / d[i]
            c = (b[i + 3] - o[i]) / d[i]
            if a > c:
                a, c = c, a
            if a > t0: t0 = a
            if c < t1: t1 = c
            if t0 > t1:
                ok = False
                break
        if ok:
            return True
    return False


def inside_box(p, pad=0.0):
    return any(b[0] - pad <= p[0] <= b[3] + pad and b[1] - pad <= p[1] <= b[4] + pad and b[2] - pad <= p[2] <= b[5] + pad for b in OCC)


class Cam:
    def __init__(self, name, lens, x, y, z, yaw, pitch, roll=0.0):
        self.name, self.lens = name, lens
        self.p = (x, y, z)
        self.yaw, self.pitch, self.roll = yaw, pitch, roll
        L = LENS[lens]
        self.th, self.tv = math.tan(L['h'] / 2 * D2R), math.tan(L['v'] / 2 * D2R)
        self.dmin, self.dmax = L['dmin'], L['dmax']
        R = rot(yaw, pitch, roll)
        self.f = (R[0][0], R[1][0], R[2][0])
        self.l = (R[0][1], R[1][1], R[2][1])
        self.u = (R[0][2], R[1][2], R[2][2])
        b = BASELINE / 2
        self.eyes = [tuple(self.p[i] + s * b * self.l[i] for i in range(3)) for s in (1, -1)]

    def local(self, q, o=None):
        o = o or self.p
        d = (q[0] - o[0], q[1] - o[1], q[2] - o[2])
        return (d[0] * self.f[0] + d[1] * self.f[1] + d[2] * self.f[2],
                d[0] * self.l[0] + d[1] * self.l[1] + d[2] * self.l[2],
                d[0] * self.u[0] + d[1] * self.u[1] + d[2] * self.u[2])

    def in_frustum(self, q, o):
        x, y, z = self.local(q, o)
        return x > 1e-6 and abs(y) <= self.th * x and abs(z) <= self.tv * x

    def sees(self, q):
        """returns 0 = no, 1 = image only (left eye), 2 = stereo depth valid"""
        le, re = self.eyes
        if not self.in_frustum(q, le) or seg_blocked(le, q):
            return 0
        x = self.local(q, le)[0]
        if not self.in_frustum(q, re) or x < self.dmin or x > self.dmax or seg_blocked(re, q):
            return 1
        return 2


def foot_dist(x, y):
    x0, x1, y0, y1 = FOOT
    dx = max(x0 - x, 0, x - x1)
    dy = max(y0 - y, 0, y - y1)
    return math.hypot(dx, dy)


FOOT_C = ((FOOT[0] + FOOT[1]) / 2, 0.0)


def make_grid(step=0.1, R=5.0, z=0.0):
    pts = []
    n = int((R + 1.0) / step) + 1
    for i in range(-n, n + 1):
        for j in range(-n, n + 1):
            x, y = FOOT_C[0] + i * step, j * step
            d = foot_dist(x, y)
            if 0 < d <= R:
                az = math.degrees(math.atan2(y, x - FOOT_C[0]))
                pts.append((x, y, z, d, az))
    return pts


def evaluate(cams, grid, bands=((0, 0.5), (0.5, 1.5), (1.5, 3), (3, 5))):
    """coverage stats on a grid of points."""
    nb = len(bands)
    tot = [0] * nb
    mono = [0] * nb
    stereo = [0] * nb
    multi = [0] * nb
    # gap analysis per 2-degree azimuth bin (about footprint centre): farthest uncovered point
    gap_mono = [0.0] * 180
    gap_st = [0.0] * 180
    for (x, y, z, d, az) in grid:
        q = (x, y, z)
        best = 0
        cnt = 0
        for c in cams:
            s = c.sees(q)
            if s:
                cnt += 1
            if s > best:
                best = s
        k = int((az + 180) // 2) % 180
        if best == 0 and d > gap_mono[k]:
            gap_mono[k] = d
        if best < 2 and d > gap_st[k]:
            gap_st[k] = d
        for bi, (a, b) in enumerate(bands):
            if a < d <= b:
                tot[bi] += 1
                mono[bi] += best >= 1
                stereo[bi] += best == 2
                multi[bi] += cnt >= 2
                break
    return dict(
        mono=[m / t for m, t in zip(mono, tot)],
        stereo=[s / t for s, t in zip(stereo, tot)],
        overlap=[m / t for m, t in zip(multi, tot)],
        gap_mono=max(gap_mono), gap_stereo=max(gap_st),
        gap_mono_bins=gap_mono, gap_st_bins=gap_st,
    )


def lidar_shadow(cams, n_az=720):
    """fraction of lidar beams (64 ch x n_az) newly blocked by camera bodies (beyond what the car already blocks).
    Camera bodies approximated as axis-aligned boxes sized by their rotated extents."""
    occ_extra = []
    for c in cams:
        # half extents of rotated body box (conservative AABB)
        hx = hy = hz = 0
        for axis, half in ((c.f, CAM_BOX[0] / 2), (c.l, CAM_BOX[1] / 2), (c.u, CAM_BOX[2] / 2)):
            hx += abs(axis[0]) * half; hy += abs(axis[1]) * half; hz += abs(axis[2]) * half
        # body sits behind the optical centre
        cx, cy, cz = (c.p[i] - c.f[i] * CAM_BOX[0] / 2 for i in range(3))
        occ_extra.append((cx - hx, cy - hy, cz - hz, cx + hx, cy + hy, cz + hz))
    o = LIDAR
    blocked_new = 0
    total = 0
    for ch in range(64):
        el = (21.2 - 42.4 * ch / 63) * D2R
        ce, se = math.cos(el), math.sin(el)
        for j in range(n_az):
            a = 2 * math.pi * j / n_az
            d = (ce * math.cos(a), ce * math.sin(a), se)
            t_car = ray_t(o, d, OCC[:-1], 0.05)
            t_cam = ray_t(o, d, occ_extra, 0.0)
            total += 1
            if t_cam < t_car and t_cam < 30:
                blocked_new += 1
    return blocked_new / total


def ray_t(o, d, boxes, tmin):
    best = math.inf
    for b in boxes:
        t0, t1 = tmin, math.inf
        ok = True
        for i in range(3):
            if abs(d[i]) < 1e-12:
                if o[i] < b[i] or o[i] > b[i + 3]:
                    ok = False; break
                continue
            a = (b[i] - o[i]) / d[i]; c = (b[i + 3] - o[i]) / d[i]
            if a > c: a, c = c, a
            t0 = max(t0, a); t1 = min(t1, c)
            if t0 > t1: ok = False; break
        if ok and t0 < best:
            best = t0
    if d[2] < 0:
        best = min(best, -o[2] / d[2])
    return best


def fmt(r):
    f = lambda v: ' '.join(f'{x*100:5.1f}' for x in v)
    return (f"mono% [{f(r['mono'])}]  stereo% [{f(r['stereo'])}]  overlap% [{f(r['overlap'])}]  "
            f"gap-close mono {r['gap_mono']:.2f} m, stereo {r['gap_stereo']:.2f} m")
