import math, random, sys, json
from multiprocessing import Pool
from ring import *

RC = 0.12
H = float(sys.argv[1])
DS = [round(0.1 * i, 2) for i in range(0, 91)]
# v = pf, pr, ps, yaws, xm
LO = [10, 10, 10, 75, -0.05]
HI = [33, 39, 45, 105, 0.30]


def build(v, H=H):
    pf, pr, ps, yw, xm = v
    return [Cam('front', '2.2', xm + RC, 0, H, 0, pf), Cam('rear', '2.2', xm - RC, 0, H, 180, pr),
            Cam('left', '4', xm, RC, H, yw, ps), Cam('right', '4', xm, -RC, H, -yw, ps)]


def seam(cams, rad=2.0):
    cov = []
    for k in range(360):
        a = math.radians(k)
        q = (FOOT_C[0] + rad * math.cos(a), rad * math.sin(a), 0.0)
        cov.append([c.sees(q) == 2 for c in cams])
    return [sum(1 for c in cov if c[i] and c[j]) for i, j in [(0, 2), (2, 1), (1, 3), (3, 0)]]


def score(v):
    cams = build(v)
    p = ring_profile(cams, DS, 0.0, 360)
    iv = interval(p, 2)
    if not iv:
        return -100 + sum(r[2] for r in p), None
    rin, rout = iv
    sm = seam(cams)
    # front-sector weighting: the ring must hold to rout; mean stereo coverage 0-9 m as tiebreak
    S = rout - 3 * max(0, rin - 0.5) + 0.5 * sum(r[2] for r in p) / len(p) - 0.3 * max(0, 5 - min(sm[0], sm[3])) - 0.1 * max(0, 3 - min(sm[1], sm[2]))
    return S, (iv, sm)


def local(v, seed):
    random.seed(seed)
    best, _ = score(v)
    span = [b - a for a, b in zip(LO, HI)]
    scale = 0.2
    while scale > 0.01:
        imp = False
        for _ in range(16):
            c = [min(max(x + random.gauss(0, scale * s), a), b) for x, s, a, b in zip(v, span, LO, HI)]
            sc, _ = score(c)
            if sc > best:
                best, v, imp = sc, c, True
        if not imp:
            scale *= 0.5
    return best, v


if __name__ == '__main__':
    random.seed(11)
    cands = [[random.uniform(a, b) for a, b in zip(LO, HI)] for _ in range(96)]
    with Pool(16) as P:
        sc = P.map(score, cands)
        top = sorted(zip([s[0] for s in sc], cands), reverse=True)[:16]
        res = P.starmap(local, [(v, i) for i, (_, v) in enumerate(top)])
    res.sort(reverse=True)
    s, v = res[0]
    S, info = score(v)
    print(f'H={H:.2f} S={S:.3f} v=' + str([round(x, 2) for x in v]) + f' stereo full ring {info[0]} seams@2m(F-L,L-Re,Re-R,R-F) {info[1]} deg')
    json.dump(dict(H=H, v=v), open(f'r3_{H:.2f}.json', 'w'))
