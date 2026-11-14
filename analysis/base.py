import time
from camsim import *
g = make_grid(0.1)
print(len(g), 'pts', {k: (round(v['h'],1), round(v['v'],1)) for k,v in LENS.items()})
cur = [Cam('front','2.2',0.70,0,0.4476,0,15), Cam('left','4',0.098,0.30,0.6126,90,0), Cam('right','4',0.098,-0.30,0.6126,-90,0),
       Cam('rear','2.2',-0.20,0,0.60,180,15)]
t=time.time(); r=evaluate(cur,g); print('current+rear', fmt(r), f'{time.time()-t:.1f}s'); print('lidar new-blocked', lidar_shadow(cur,360))
