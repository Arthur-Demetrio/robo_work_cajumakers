from pathlib import Path
import json,sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robo_rl_v2.env import rotation,wrap,RoboEnv
p=Path(__file__).parent;data=json.loads((p/'episodes_2000384.json').read_text());mask=RoboEnv().mask;out=[]
for row in data:
 if not row['collision']:continue
 i=row['seed']-(2100000000 if row['group']=='viewer' else 2300000000+row['level']*10000)
 d=np.load(p/f"2000384_{row['group']}_{i}.npz");pose=d['trajectory'][-1,:3];res=float(d['resolution']);origin=d['origin'];cells=d['cells'];base=d['base_cells'];yy,xx=np.nonzero(cells!=0)
 centers=(np.c_[xx,yy]+.5)*res@rotation(origin[2]).T+origin[:2]
 local=(centers-pose[:2])@rotation(pose[2]);delta=np.maximum(abs(local-[.08,0])-[.285,.315],0);dist=np.linalg.norm(delta,axis=1);k=int(dist.argmin());x,y=xx[k],yy[k]
 category='caixa_adicionada' if cells[y,x]!=base[y,x] else 'desconhecido' if base[y,x]==-1 else 'parede_base'
 sensor=pose[:2]+rotation(pose[2])@np.array([.33,.005]);point=centers[k]-sensor;angle=wrap(np.arctan2(point[1],point[0])-pose[2]);beam=int(np.round((angle+np.pi)/(2*np.pi)*72))%72
 out.append(dict(seed=row['seed'],group=row['group'],nearest_obstacle=category,nearest_center_polygon_distance=float(dist[k]),blocked_lidar_sector=bool(not mask[beam]),pose_error=row['final_est_error']))
(p/'collision_analysis.json').write_text(json.dumps(out,indent=2))
from collections import Counter
print(Counter(r['nearest_obstacle'] for r in out if r['group']!='viewer'));print('blind',sum(r['blocked_lidar_sector'] for r in out if r['group']!='viewer'))
