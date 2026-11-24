"""Diagnostico de registro offline. Nao altera pose nem SLAM."""
from pathlib import Path
import numpy as np,json,math
from scipy.ndimage import distance_transform_edt,map_coordinates
b=Path(__file__).parent;sc=np.load(b/'scans.npz');z=np.load(b/'snapshot.npz');d=json.loads((b/'conectividade.json').read_text());mt=d['costmap_metadata'];res=mt['res'];orig=np.array([mt['ox'],mt['oy']]);base=np.array(d['pose_atual']);pts=sc['scan_nav_map'];p=pts-base;p=p[np.linalg.norm(p,axis=1)<3.5]
mp=z['mapa'];dist=distance_transform_edt(mp<65)*res

def nota(dx,dy,a):
 c,s=math.cos(a),math.sin(a);q=p@np.array([[c,s],[-s,c]])+base+np.array([dx,dy]);cell=(q-orig)/res-.5
 ds=map_coordinates(dist,[cell[:,1],cell[:,0]],order=1,mode='constant',cval=.5)
 # aparar 20% evita que um objeto movido determine sozinho o registro.
 clipped=np.minimum(ds,.35);v=np.sort(clipped)[:int(.8*len(clipped))]
 return float(v.mean()),float((ds<.05).mean()),float(np.median(ds))
initial=nota(0,0,0);best=(initial[0],0,0,0,initial)
for a in np.radians(np.arange(-12,12.1,1.5)):
 for dx in np.arange(-.7,.701,.05):
  for dy in np.arange(-.4,.401,.05):
   v=nota(dx,dy,a)
   if v[0]<best[0]:best=(v[0],float(dx),float(dy),float(a),v)
r=dict(pontos=len(p),atual=dict(media_aparada=initial[0],fracao_ate5cm=initial[1],mediana=initial[2]),melhor_busca=dict(dx=best[1],dy=best[2],dyaw_graus=math.degrees(best[3]),media_aparada=best[4][0],fracao_ate5cm=best[4][1],mediana=best[4][2]),limite='Busca offline entre endpoints LiDAR e ocupacao. Sem raycasting/IMU/observabilidade, nao e autorizacao de relocalizacao.')
(b/'registro_offline.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
