from pathlib import Path
import heapq,json,math
import numpy as np
from scipy.ndimage import distance_transform_edt,label,find_objects
b=Path(__file__).parent;z=np.load(b/'snapshot.npz');cm=z['costmap'];mp=z['mapa'];d=json.loads((b/'conectividade.json').read_text());mt=d['costmap_metadata'];res=mt['res'];ox,oy=mt['ox'],mt['oy'];s=tuple(d['costmap_sem_unknown']['inicio_cel']);g=tuple(d['costmap_sem_unknown']['goal_cel']);h,w=cm.shape
known=(mp>=0)&(mp<65);extra=(cm==254)&known
from_static=distance_transform_edt(mp<65)*res
from_lethal=distance_transform_edt(cm!=254)*res
lc,n=label(extra,np.ones((3,3)));clusters=[]
for i,sl in enumerate(find_objects(lc),1):
 yy,xx=np.where(lc==i);clusters.append(dict(n=len(xx),x=[float(ox+xx.min()*res),float(ox+xx.max()*res)],y=[float(oy+yy.min()*res),float(oy+yy.max()*res)],dist_estatico_max=float(from_static[yy,xx].max())))
# Caminho diagnostico apenas. Minimizacao de celulas bloqueadas, sem autorizar trajetoria.
clearance=distance_transform_edt(known)*res;livre=known&(clearance>.215)
D={s:0};prev={};q=[(0,s)];done=set()
while q:
 c,p=heapq.heappop(q)
 if p in done:continue
 done.add(p)
 if p==g:break
 x,y=p
 for dx,dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
  v=(x+dx,y+dy)
  if not (0<=v[0]<w and 0<=v[1]<h) or not livre[v[1],v[0]]:continue
  nc=c+math.hypot(dx,dy)+(10000 if cm[v[1],v[0]]>=253 else 0)
  if nc<D.get(v,float('inf')):D[v]=nc;prev[v]=p;heapq.heappush(q,(nc,v))
path=[];p=g
if p in D:
 while p!=s:path.append(p);p=prev[p]
 path.append(s);path.reverse()
bloq=[]
for x,y in path:
 if cm[y,x]>=253:
  dist=(extra.astype(float))
  ey,ex=np.where(extra);idx=np.argmin((ex-x)**2+(ey-y)**2)
  bloq.append(dict(x=float(ox+(x+.5)*res),y=float(oy+(y+.5)*res),custo=int(cm[y,x]),dist_obstaculo=float(from_lethal[y,x]),extra_mais_proximo=[float(ox+(ex[idx]+.5)*res),float(oy+(ey[idx]+.5)*res)]))
r=dict(extra_count=int(extra.sum()),clusters=clusters,custo253_dist_lethal_max=float(from_lethal[cm==253].max()),caminho_diagnostico_existe=bool(path),celulas_bloqueadas_no_melhor_caminho=bloq)
(b/'grade_diagnostico.json').write_text(json.dumps(r,indent=2));np.save(b/'caminho_diagnostico.npy',path);print(json.dumps(r,indent=2))
