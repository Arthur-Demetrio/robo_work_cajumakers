from pathlib import Path
import sys,json
import numpy as np
from envs import FastV2Env
RUN=Path(sys.argv[1]);rows=[]
for suite,maximum,base in [('full',12.,3300000000),('gate',2.,3301000000)]:
 e=FastV2Env(maximum=maximum)
 for i in range(100):
  e.reset(seed=base+i);route=e.planner.plan(e.pose[:2],e.goal);length=float(np.linalg.norm(np.diff(route,axis=0),axis=1).sum())
  rows.append(dict(suite=suite,seed=base+i,start=e.pose.tolist(),goal=e.goal.tolist(),euclidean=float(np.linalg.norm(e.goal-e.pose[:2])),grid_path_length=length,friction=float(e.friction),bias=e.bias.tolist(),slip=float(e.slip)))
 sizes=np.bincount(e.grid.components.ravel());sizes[0]=0
 coverage=dict(safe_components=e.grid.n_components,largest_safe_cells=int(sizes.max()),all_safe_cells=int(sizes.sum()),sampled_fraction_of_safe_centers=float(sizes.max()/sizes.sum()),largest_safe_center_area_m2=float(sizes.max()*e.grid.resolution**2),note='The disk-based sampler excludes physically navigable oriented-footprint passages; evaluation does not cover all service regions.')
(RUN/'scenario_manifest.json').write_text(json.dumps(dict(coverage=coverage,scenarios=rows),indent=2));print(coverage)
