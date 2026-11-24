import sys,json,importlib.util
from pathlib import Path
import numpy as np,cv2
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robo_rl_v2.arena2016v2 import Arena2016V2Env
from scipy.ndimage import label
OUT=Path(__file__).parent;e=Arena2016V2Env();e.grid=e.physical_grid;cfg=e.config;physical=e.physical_grid.cells!=0;res=e.grid.resolution if hasattr(e,'grid') else e.physical_grid.resolution
spec=importlib.util.spec_from_file_location('review',e.layout_path.parent/'build_review.py');review=importlib.util.module_from_spec(spec);spec.loader.exec_module(review)
old=json.loads((e.layout_path.parent/'review/service_regions.json').read_text());unresolved=[r['object'] for r in old if not r['accessible']]
# Sample service fronts more densely. Keep source objects and all heights unchanged.
results=[]
for name in unresolved:
 obj=next(o for o in cfg['objects'] if o['id']==name);pts=np.array(obj['geometry']['polygon']);center=pts.mean(0);valid=[];reasons={}
 faces=[(pts[k],pts[(k+1)%len(pts)]) for k in range(len(pts))]
 if obj['kind']=='circular_object':
  faces=[]
  for angle in np.linspace(0,2*np.pi,36,endpoint=False):
   n=np.array([np.cos(angle),np.sin(angle)]);t=np.array([-n[1],n[0]])
   boundary=center+n*np.max((pts-center)@n)
   faces.append((boundary-.4*t,boundary+.4*t))
 for face,(a,b) in enumerate(faces):
  edge=b-a;length=np.linalg.norm(edge)
  if length<.25:continue
  tangent=edge/length;normal=np.array([-edge[1],edge[0]])/length
  if normal@(a-center)<0:normal=-normal
  yaw=float(np.arctan2(-normal[1],-normal[0]));mask_free=review.cspace(physical,yaw,res);labels,_=label(mask_free);sizes=np.bincount(labels.ravel());sizes[0]=0;main=sizes.argmax()
  for shift in np.linspace(-.30,.30,13):
   midpoint=(a+b)/2+tangent*shift
   # Width is fixed 0.80 m. Depth 0.72 m contains actual robot length with margin.
   pos=midpoint+normal*(.365+.09)
   approach=np.array([midpoint+tangent*.4+normal*.03,midpoint-tangent*.4+normal*.03,midpoint-tangent*.4+normal*.75,midpoint+tangent*.4+normal*.75])
   reason='ok'
   if (approach<0).any() or (approach>e.size).any():reason='outside_floor'
   elif e.collision(np.r_[pos,yaw],margin=.01):reason='robot_footprint_collision'
   else:
    mask=np.zeros_like(physical,dtype=np.uint8);cv2.fillPoly(mask,[np.round(approach/res).astype(np.int32)],1)
    if np.any(physical[mask.astype(bool)]):reason='approach_rectangle_blocked'
    else:
     x,y=np.floor(pos/res).astype(int)
     if labels[y,x]!=main:reason='not_connected_at_fixed_heading'
   reasons[reason]=reasons.get(reason,0)+1
   if reason=='ok':valid.append(dict(face=face,shift_m=float(shift),pose=[*pos.tolist(),yaw],approach_polygon=approach.tolist(),width_m=.8,depth_m=.72))
 results.append(dict(object=name,candidates=valid,counts=reasons));print(name,len(valid),reasons,flush=True)
(OUT/'service_diagnostics.json').write_text(json.dumps(dict(geometry_changed=False,heights_changed=False,candidates=results),indent=2))
