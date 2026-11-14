import math, random, sys, json
from multiprocessing import Pool
from camsim import *

G0 = make_grid(0.12, 6.0, 0.0)
G5 = make_grid(0.2, 5.0, 0.5)
RC = 0.12          # crown radius: optical centres 12 cm from the mast axis (ZED X bodies 166 mm wide don't collide)
MODE = sys.argv[1]  # crown | hybrid
H = float(sys.argv[2])

# v = pf, pr, ps, yaws, xm
LO = [5, 5, 5, 60, -0.10]
HI = [34, 39, 45, 120, 0.30]
if MODE == 'hybrid':   # front at bumper, extra param = front height
    LO += [0.30]; HI += [0.60]


def build(v):
    pf, pr, ps, yw, xm = v[:5]
    if MODE == 'crown':
        front = Cam('front', '2.2', xm + RC, 0, H, 0, pf)
    else:
        front = Cam('front', '2.2', 0.71, 0, v[5], 0, pf)
    return [front, Cam('rear', '2.2', xm - RC, 0, H, 180, pr),
            Cam('left', '4', xm, RC, H, yw, ps), Cam('right', '4', xm, -RC, H, -yw, ps)]


def seam_overlap(cams, rad=2.5):
    """angular overlap (deg) on a ground ring of radius rad about the footprint centre, per neighbouring pair (stereo-valid)"""
    cov = []
    for k in range(720):
        a = math.radians(k / 2)
        q = (FOOT_C[0] + rad * math.cos(a), rad * math.sin(a), 0.0)
        cov.append([c.sees(q) == 2 for c in cams])
    pairs = [(0, 2), (2, 1), (1, 3), (3, 0)]
    res = [sum(1 for c in cov if c[i] and c[j]) / 2 for i, j in pairs]
    nocov = sum(1 for c in cov if not any(c)) / 2
    return min(res), res, nocov


def score(v):
    cams = build(v)
    for c in cams:
        if inside_box(c.p, 0.015):
            return -999, None
    r = evaluate(cams, G0, bands=((0, 0.5), (0.5, 1.5), (1.5, 3), (3, 6)))
    r5 = evaluate(cams, G5)
    mo, ov, nocov = seam_overlap(cams)
    st = r['stereo']
    S = 10 * st[0] + 30 * st[1] + 30 * st[2] + 20 * st[3] + 10 * sum(r5['stereo'][1:3]) / 2
    S -= 6 * r['gap_stereo'] + 2 * r['gap_mono']
    S -= 3 * max(0, 5 - mo)   # want >= 5 deg seam overlap at 2.5 m (extrinsic calibration / stitching)
    return S, (r, r5, ov, nocov)


def local(v, seed):
    random.seed(seed)
    best, _ = score(v)
    span = [b - a for a, b in zip(LO, HI)]
    scale = 0.2
    while scale > 0.004:
        imp = False
        for _ in range(30):
            c = [min(max(x + random.gauss(0, scale * s), a), b) for x, s, a, b in zip(v, span, LO, HI)]
            sc, _ = score(c)
            if sc > best:
                best, v, imp = sc, c, True
        if not imp:
            scale *= 0.5
    return best, v


if __name__ == '__main__':
    random.seed(7)
    cands = [[random.uniform(a, b) for a, b in zip(LO, HI)] for _ in range(300)]
    with Pool(16) as P:
        sc = P.map(score, cands)
        top = sorted(zip([s[0] for s in sc], cands), reverse=True)[:8]
        res = P.starmap(local, [(v, i) for i, (_, v) in enumerate(top)])
    res.sort(reverse=True)
    s, v = res[0]
    S, (r, r5, ov, nocov) = score(v)
    print(f'{MODE} H={H:.2f} score {S:6.2f} v=' + str([round(x, 3) for x in v]))
    print('   ground ', fmt(r))
    print('   z=0.5  ', fmt(r5))
    print(f'   seam overlap @2.5m (F-L, L-Rear, Rear-R, R-F) {ov} deg, uncovered {nocov} deg')
    json.dump(dict(mode=MODE, H=H, v=v, S=S), open(f'r2_{MODE}_{H:.2f}.json', 'w'))
