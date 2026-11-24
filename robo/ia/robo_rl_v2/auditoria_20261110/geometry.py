import sys,json
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robo_rl_v2.env import RoboEnv
from robo_rl_v2.maps import Grid
out=Path(__file__).parent
original=Grid.add_box
records=[]
def traced(self,center,size):
 c=self.local(center);lo=np.floor((c-np.array(size)/2)/self.resolution).astype(int);hi=np.ceil((c+np.array(size)/2)/self.resolution).astype(int)
 clipped=bool((lo<0).any() or (hi>np.array(self.cells.shape[::-1])).any())
 area=self.cells[max(0,lo[1]):min(self.cells.shape[0],hi[1]),max(0,lo[0]):min(self.cells.shape[1],hi[0])]
 records.append(dict(level=current_level,clipped=clipped,overlap_blocked=bool(np.any(area!=0)),overlap_unknown=bool(np.any(area==-1))))
 return original(self,center,size)
Grid.add_box=traced
scenario_original=RoboEnv._scenario
accepted=[]
def scenario_traced(self):
 begin=len(records)
 result=scenario_original(self)
 if result is not None:accepted.extend(records[begin:])
 return result
RoboEnv._scenario=scenario_traced
samples=[]
for current_level in range(1,6):
 env=RoboEnv(level=current_level,split='eval')
 for i in range(100):
  env.reset(seed=2400000000+current_level*10000+i)
  samples.append(dict(level=current_level,initial_collision=env.collision(),goal_collision_any=any(env.collision(np.r_[env.goal,a]) for a in np.linspace(-np.pi,np.pi,16,endpoint=False)),start_clearance=env.grid.distance(env.pose[:2]),goal_clearance=env.grid.distance(env.goal)))
(out/'geometry.json').write_text(json.dumps(dict(box_attempts=records,accepted_box_attempts=accepted,scenarios=samples),indent=2))
print('geometry complete',len(records),flush=True)
