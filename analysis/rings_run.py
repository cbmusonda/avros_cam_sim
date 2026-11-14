import json, glob, sys
from multiprocessing import Pool
from ring import *
RC=0.12
def crown(H, pf, pr, ps, yw, xm):
    return [Cam('front','2.2',xm+RC,0,H,0,pf), Cam('rear','2.2',xm-RC,0,H,180,pr), Cam('left','4',xm,RC,H,yw,ps), Cam('right','4',xm,-RC,H,-yw,ps)]
cfgs = {
 'current+rear(0.6m,15deg)': [Cam('front','2.2',0.70,0,0.4476,0,15), Cam('left','4',0.098,0.30,0.6126,90,0), Cam('right','4',0.098,-0.30,0.6126,-90,0), Cam('rear','2.2',-0.20,0,0.60,180,15)],
}
b = json.load(open('best_body.json'))[0]
zf,pf,zr,pr,xs,ys,zs,yw,ps = b
cfgs['best body-mount'] = [Cam('front','2.2',0.71,0,zf,0,pf), Cam('rear','2.2',-0.20,0,zr,180,pr), Cam('left','4',xs,ys,zs,yw,ps), Cam('right','4',xs,-ys,zs,-yw,ps)]
for f in sorted(glob.glob('r2_crown_*.json')):
    d = json.load(open(f)); cfgs[f"crown H={d['H']:.2f} opt"] = crown(d['H'], *d['v'])
ds = [round(0.1*i,2) for i in range(0, 101)]
def run(item):
    name, cams = item
    out = [name]
    for z in (0.0, 0.5, 0.9):
        p = ring_profile(cams, ds, z, 360)
        out.append((z, interval(p,1), interval(p,2), min(r[1] for r in p if 1<=r[0]<=6), min(r[2] for r in p if 1<=r[0]<=6)))
    return out
with Pool(16) as P:
    for o in P.map(run, list(cfgs.items())):
        print(o[0])
        for z, im, ist, mm, ms in o[1:]:
            print(f'   z={z:.1f}  full-ring mono {im}  stereo {ist}   worst ring cov 1-6m: mono {mm*100:.1f}% stereo {ms*100:.1f}%')
