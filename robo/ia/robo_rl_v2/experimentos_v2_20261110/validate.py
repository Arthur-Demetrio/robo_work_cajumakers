import sys,json,time,cProfile,pstats,hashlib
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robo_rl_v2.arena2016v2 import Arena2016V2Env
from envs import FastV2Env
OUT=Path(sys.argv[1]);OUT.mkdir(parents=True,exist_ok=False)
old=Arena2016V2Env();fast=FastV2Env();checked=0;actions=np.random.default_rng(148).uniform(-1,1,(100,3))
for seed in range(20):
 a,ia=old.reset(seed=seed);b,ib=fast.reset(seed=seed);np.testing.assert_array_equal(a,b)
 for action in actions:
  x=old.step(action);y=fast.step(action)
  np.testing.assert_array_equal(x[0],y[0]);assert x[1:4]==y[1:4];assert x[4]==y[4]
  np.testing.assert_array_equal(old.pose,fast.pose);checked+=1
  if x[2] or x[3]:break
speed=[]
for kind in [Arena2016V2Env,FastV2Env]:
 e=kind();e.reset(seed=123);t=time.perf_counter()
 for i in range(1000):
  _,_,done,tr,_=e.step([0,0,0])
  if done or tr:e.reset(seed=123+i)
 speed.append(dict(kind=kind.__name__,steps_s=1000/(time.perf_counter()-t)))
profile=cProfile.Profile();e=FastV2Env();e.reset(seed=123);profile.enable()
for _ in range(300):e.step([0,0,0])
profile.disable()
with (OUT/'profile.txt').open('w') as fp:pstats.Stats(profile,stream=fp).sort_stats('cumtime').print_stats(20)
(OUT/'optimization.json').write_text(json.dumps(dict(equivalent_transitions=checked,identical_observations_rewards_states=True,speed=speed),indent=2));print(checked,speed,flush=True)
