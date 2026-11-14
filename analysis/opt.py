import math, random, sys, json
from multiprocessing import Pool
from camsim import *

GSTEP = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
G0 = make_grid(GSTEP, 5.0, 0.0)
G5 = make_grid(GSTEP * 1.5, 5.0, 0.5)
FAM = sys.argv[1]

# params: zf, pf, zr, pr, xs, ys, zs, yaws, ps
if FAM == 'body':
    LO = [0.20, 0, 0.20, 0, -0.15, 0.36, 0.20, 30, 0]
    HI = [0.60, 30, 0.64, 30, 0.72, 0.36, 0.62, 150, 50]
elif FAM == 'free':
    LO = [0.20, 0, 0.20, 0, -0.20, 0.0, 0.20, 30, 0]
    HI = [1.50, 30, 1.50, 30, 0.72, 0.45, 1.50, 150, 60]
elif FAM == 'mast':   # all four on one crown above the lidar's upper beam cone
    LO = [0.95, 0, 0.95, 0, -0.10, 0.06, 0.95, 30, 0]
    HI = [1.50, 30, 1.50, 30, 0.30, 0.12, 1.50, 150, 60]

def build(v):
    zf, pf, zr, pr, xs, ys, zs, yaws, ps = v
    if FAM == 'mast':
        xm = 0.089   # crown centred over the lidar
        zc = (zf + zr + zs) / 3
        return [Cam('front', '2.2', xm + 0.10, 0, zc, 0, pf), Cam('rear', '2.2', xm - 0.10, 0, zc, 180, pr),
                Cam('left', '4', xs, ys, zc, yaws, ps), Cam('right', '4', xs, -ys, zc, -yaws, ps)]
    xf = 0.71 if zf <= 0.62 else 0.72
    return [Cam('front', '2.2', xf, 0, zf, 0, pf), Cam('rear', '2.2', -0.20, 0, zr, 180, pr),
            Cam('left', '4', xs, ys, zs, yaws, ps), Cam('right', '4', xs, -ys, zs, -yaws, ps)]

W = [1, 2, 2, 1]
def score(v, detail=False):
    cams = build(v)
    for c in cams:
        if inside_box(c.p, 0.015):
            return -99, None
    r = evaluate(cams, G0)
    r5 = evaluate(cams, G5)
    s = sum(w * (0.7 * st + 0.3 * m) for w, st, m in zip(W, r['stereo'], r['mono'])) / sum(W)
    s5 = sum(w * (0.7 * st + 0.3 * m) for w, st, m in zip(W, r5['stereo'], r5['mono'])) / sum(W)
    gap = r['gap_mono']
    S = 100 * (0.75 * s + 0.25 * s5) - 4 * gap
    sh = 0.0
    if FAM == 'free':
        sh = lidar_shadow(cams, 180)
        S -= 300 * sh
    return S, (r, r5, sh)

def rnd():
    return [random.uniform(a, b) for a, b in zip(LO, HI)]

def clip(v):
    return [min(max(x, a), b) for x, a, b in zip(v, LO, HI)]

def local(v, seed):
    random.seed(seed)
    best, _ = score(v)
    span = [b - a for a, b in zip(LO, HI)]
    scale = 0.15
    while scale > 0.005:
        improved = False
        for _ in range(40):
            c = clip([x + random.gauss(0, scale * s) for x, s in zip(v, span)])
            sc, _ = score(c)
            if sc > best:
                best, v, improved = sc, c, True
        if not improved:
            scale *= 0.5
    return best, v

if __name__ == '__main__':
    random.seed(1)
    with Pool(16) as P:
        cands = [rnd() for _ in range(800)]
        sc = P.map(score, cands)
        ranked = sorted(zip([s[0] for s in sc], cands), reverse=True)[:16]
        res = P.starmap(local, [(v, i) for i, (_, v) in enumerate(ranked)])
    res.sort(reverse=True)
    out = []
    for s, v in res[:5]:
        S, (r, r5, sh) = score(v)
        print(f'{S:6.2f}', [round(x, 3) for x in v], f'lidar+{sh*100:.2f}%')
        print('   ground ', fmt(r)); print('   z=0.5  ', fmt(r5))
        out.append(v)
    json.dump(out, open(f'best_{FAM}.json', 'w'))
