from pathlib import Path
import json
import numpy as np
from full_sampler import FullFootprintEnv
P=Path(__file__).parent;e=FullFootprintEnv();counts=np.bincount(e.physical_grid.components.ravel());counts[0]=0;old_largest=counts.argmax();outside=0;components=set();starts=[]
for i in range(500):
 e.reset(seed=3400000000+i)
 assert not e.collision(e.pose,margin=.01)
 assert not e.collision(e.goal_pose,margin=.01)
 ix,iy=e.grid.indices(e.pose[:2]);component=int(e.grid.components[iy,ix]);components.add(component)
 outside+=int(component!=old_largest);starts.append(e.pose.tolist())
 if i<20:
  initial=e.pose.copy();obs=e.observation();again,_=e.reset(seed=3400000000+i);np.testing.assert_array_equal(initial,e.pose);np.testing.assert_array_equal(obs,again)
result=dict(passed=True,resets=500,collision_queries=1000,starts_outside_old_largest_component=outside,old_component_labels_reached=sorted(components),main_state_count=int(e.accum[-1]),coverage_method='Largest connected component of discretized full-footprint SE(2) graph; conservative swept rotations',guidance_compatibility='The old disk planner is deliberately rejected; no learning was run with the new sampler')
(P/'sampler_validation.json').write_text(json.dumps(result,indent=2));np.save(P/'sampler_starts.npy',starts);print(result)
