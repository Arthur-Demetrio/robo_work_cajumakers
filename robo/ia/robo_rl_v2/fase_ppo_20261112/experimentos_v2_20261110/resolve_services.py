from pathlib import Path
import sys,json,importlib.util
import numpy as np,cv2,yaml
from scipy.ndimage import label,distance_transform_edt
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robo_rl_v2.arena2016v2 import Arena2016V2Env
P=Path(__file__).parent;e=Arena2016V2Env();e.grid=e.physical_grid;cfg=e.config;res=e.grid.resolution;physical=e.grid.cells!=0
spec=importlib.util.spec_from_file_location('review',e.layout_path.parent/'build_review.py');r=importlib.util.module_from_spec(spec);spec.loader.exec_module(r)
services=json.loads((e.layout_path.parent/'review/service_regions.json').read_text());diagnostics=json.loads((P/'service_diagnostics.json').read_text())
# A 4x finer distance field avoids the old full-cell-diagonal over-padding.
factor=4;fine_res=res/factor
fine_blocked=np.repeat(np.repeat(physical,factor,0),factor,1)
fine_distance=distance_transform_edt(np.pad(~fine_blocked,1))[1:-1,1:-1]*fine_res-fine_res*np.sqrt(2)/2
fine_labels,_=label(fine_distance>.4);counts=np.bincount(fine_labels.ravel());counts[0]=0;fine_main=counts.argmax()
width_free=(fine_distance>.4)[2::4,2::4]
updates=[]
for row in diagnostics['candidates']:
 viable=[]
 for candidate in row['candidates']:
  pose=np.array(candidate['pose']);x,y=np.floor(pose[:2]/res).astype(int)
  free=r.cspace(physical,pose[2],res)&width_free
  labs,_=label(free);sizes=np.bincount(labs.ravel());sizes[0]=0
  if labs[y,x] and labs[y,x]==sizes.argmax():viable.append(candidate)
 if viable:
  chosen=max(viable,key=lambda c:e.grid.distance(np.array(c['pose'][:2])))
  replacement=dict(object=row['object'],front_status='ESTIMATED_DENSE_SEARCH_NOT_OFFICIAL',accessible=True,pose=chosen['pose'],approach_polygon=chosen['approach_polygon'],approach_width_m=.8,approach_depth_m=.72,exact_footprint_clear=True,geometric_corridor_width_lower_bound_m=.8,method='Fixed-heading configuration-space path intersected with 4x-refined conservative clearance>0.40 m; sampled largest component. Discretized validation, not continuous SE(2) certification.')
  idx=next(k for k,s in enumerate(services) if s['object']==row['object']);services[idx]=replacement;updates.append(row['object'])
# Verify all previous and new approaches against exact SAT and the actual raster.
for s in services:
 if not s['accessible']:continue
 assert not e.collision(np.array(s['pose']),margin=.01)
 mask=np.zeros_like(physical,dtype=np.uint8);cv2.fillPoly(mask,[np.round(np.array(s['approach_polygon'])/res).astype(np.int32)],1)
 assert not physical[mask.astype(bool)].any()
cfg['service_regions']=services;cfg['status']='service_candidate_geometry_and_heights_unchanged'
(P/'layout_service_candidate.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False,allow_unicode=True))
(P/'resolved_service_regions.json').write_text(json.dumps(services,indent=2));summary=dict(resolved=updates,accessible=sum(s['accessible'] for s in services),total=len(services),unresolved=[s['object'] for s in services if not s['accessible']],source_objects_unchanged=True,heights_unchanged=True)
(P/'service_resolution.json').write_text(json.dumps(summary,indent=2))
fig,ax=r.plt.subplots(figsize=(9,12));r.plot_objects(ax,cfg,services);ax.set_title('Aproximações reavaliadas — posições dos móveis preservadas');fig.tight_layout();fig.savefig(P/'service_regions_resolved.png',dpi=160);r.plt.close(fig)
print(json.dumps(summary),flush=True)
