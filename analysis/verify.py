import math
from multiprocessing import Pool
from ring import *
from camsim import lidar_shadow
RC=0.12
def crown(H, pf, pr, ps, xm, yw=90, dpf=0, dpr=0, dps=0):
    return [Cam('front','2.2',xm+RC,0,H,0,pf+dpf), Cam('rear','2.2',xm-RC,0,H,180,pr+dpr), Cam('left','4',xm,RC,H,yw,ps+dps), Cam('right','4',xm,-RC,H,-yw,ps+dps)]
DS=[round(0.05*i,2) for i in range(0,161)]
def where_gap(cams, d):
    pts = contour(d, 720)
    bad = [p for p in pts if max(c.sees(p) for c in cams) < 2]
    if not bad: return '-'
    az = sorted(round(math.degrees(math.atan2(p[1], p[0]-FOOT_C[0]))) for p in bad)
    return f'{az[0]}..{az[-1]}deg ({len(bad)/2:.0f} deg of contour)'
def run(a):
    name, cams = a
    p = ring_profile(cams, DS, 0.0, 720)
    iv = interval(p, 2); ivm = interval(p, 1)
    g = where_gap(cams, round(iv[1]+0.1,2)) if iv else '?'
    cov = sum(r[2] for r in p if 0.5<=r[0]<=6)/len([r for r in p if 0.5<=r[0]<=6])
    return f'{name:44s} mono ring {ivm}  stereo ring {iv}  mean stereo 0.5-6m {cov*100:.1f}%  first gap beyond ring: {g}'
cfgs = []
for H in (1.30, 1.45, 1.50, 1.60):
    for xm in (0.09, 0.19):
        cfgs.append((f'H={H} xm={xm} pf30 pr37 ps37', crown(H,30,37,37,xm)))
base = (1.50, 30, 37, 37, 0.19)
for k, d in [('dpf',3),('dpf',-3),('dpr',-5),('dps',3),('dps',-3)]:
    cfgs.append((f'H=1.5 xm=0.19 sensitivity {k}{d:+d}', crown(*base, **{k:d})))
cfgs.append(('H=1.5 xm=0.19 sides yaw 85 (fwd bias)', crown(*base, yw=85)))
cfgs.append(('H=1.5 xm=0.19 pitch all 25', crown(1.5,25,25,25,0.19)))
with Pool(16) as P:
    for line in P.map(run, cfgs): print(line)
print('lidar beams newly blocked by crown bodies:', lidar_shadow(crown(*base), 720))
