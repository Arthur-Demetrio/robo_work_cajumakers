import sys,time,json
from pathlib import Path
import numpy as np
from envs import FastV2Env
from native import NativeV2Env
p=Path(__file__).parent;actions=np.random.default_rng(87).uniform(-1,1,(150,3));a=FastV2Env();b=NativeV2Env();n=0
for seed in range(30):
 x,_=a.reset(seed=seed);y,_=b.reset(seed=seed);np.testing.assert_array_equal(x,y)
 for action in actions:
  x=a.step(action);y=b.step(action);np.testing.assert_array_equal(x[0],y[0]);assert x[1:4]==y[1:4];assert x[4]==y[4];np.testing.assert_array_equal(a.pose,b.pose);n+=1
  if x[2] or x[3]:break
# At exact grid boundaries, both sides and all headings must preserve occupancy behavior.
for pos in [[0,0],[.025,.025],[8.264518214512025,12.09991888267675],[4.2,6.25]]:
 for yaw in np.arange(16)*2*np.pi/16:
  a.pose=np.r_[pos,yaw];b.pose=a.pose.copy();a._np_random=np.random.default_rng(1);b._np_random=np.random.default_rng(1)
  np.testing.assert_array_equal(a.lidar()[0],b.lidar()[0])
rates=[]
for kind in [FastV2Env,NativeV2Env]:
 e=kind();e.reset(seed=22);start=time.perf_counter()
 for i in range(1000):
  _,_,t,tr,_=e.step([0,0,0])
  if t or tr:e.reset(seed=22+i)
 rates.append(dict(kind=kind.__name__,steps_s=1000/(time.perf_counter()-start)))
result=dict(passed=True,equivalent_transitions=n,boundary_cases=64,step_rates=rates)
(p/'native_validation.json').write_text(json.dumps(result,indent=2));print(result)
