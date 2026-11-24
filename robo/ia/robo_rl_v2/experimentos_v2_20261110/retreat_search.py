from pathlib import Path
import json,sys,importlib.util
import numpy as np,cv2
from scipy.ndimage import label
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robo_rl_v2.arena2016v2 import Arena2016V2Env
P=Path(__file__).parent;e=Arena2016V2Env();e.grid=e.physical_grid;physical=e.grid.cells!=0;res=e.grid.resolution
spec=importlib.util.spec_from_file_location('review',e.layout_path.parent/'build_review.py');r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
rows=[]
for name in ['P02_base_cobertura_esquerda','P03_fundo_esquerdo']:
 obj=next(o for o in e.config['objects'] if o['id']==name);pts=np.array(obj['geometry']['polygon']);center=pts.mean(0);valid=[]
 for face in range(len(pts)):
  a,b=pts[face],pts[(face+1)%len(pts)];edge=b-a;length=np.linalg.norm(edge)
  if length<.25:continue
  tangent=edge/length;normal=np.array([-edge[1],edge[0]])/length
  if normal@(a-center)<0:normal=-normal
  yaw=float(np.arctan2(-normal[1],-normal[0]));free=r.cspace(physical,yaw,res)&(e.grid.clearance>.4);labs,_=label(free);sizes=np.bincount(labs.ravel());sizes[0]=0;main=sizes.argmax()
  for shift in np.linspace(-.3,.3,13):
   midpoint=(a+b)/2+tangent*shift
   for distance in [.454,.55,.65]:
    pos=midpoint+normal*distance;x,y=np.floor(pos/res).astype(int)
    if not(0<=x<physical.shape[1] and 0<=y<physical.shape[0]) or labs[y,x]!=main or e.collision(np.r_[pos,yaw],margin=.01):continue
    for start in [.03,.13,.23]:
     if not (distance-.365-.01>=start and distance+.205+.01<=start+.72):continue
     approach=np.array([midpoint+tangent*.4+normal*start,midpoint-tangent*.4+normal*start,midpoint-tangent*.4+normal*(start+.72),midpoint+tangent*.4+normal*(start+.72)])
     if (approach<0).any() or (approach>e.size).any():continue
     mask=np.zeros_like(physical,dtype=np.uint8);cv2.fillPoly(mask,[np.round(approach/res).astype(np.int32)],1)
     if physical[mask.astype(bool)].any():continue
     valid.append(dict(face=face,shift_m=float(shift),pose=[*pos.tolist(),yaw],approach_polygon=approach.tolist(),front_gap_m=distance-.365,manipulation_reach_validated=False))
 rows.append(dict(object=name,candidates=valid));print(name,len(valid),flush=True)
(P/'retreat_candidates.json').write_text(json.dumps(rows,indent=2))
