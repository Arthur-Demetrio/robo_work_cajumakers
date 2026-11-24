import os
os.environ.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
import sys,json,copy,types,hashlib,math
from pathlib import Path
import numpy as np,torch
R=Path(__file__).resolve().parents[1];O=Path(__file__).parent
sys.path[:0]=[str(R),str(R/'experimentos_v2_20261110')]
from train_gated import StageEnv
from robo_rl_v2.arena2016v2 import Arena2016V2Env
from stable_baselines3 import PPO
torch.set_num_threads(1)
p=R/'runs/treino_continuo_20261111/checkpoints/ppo_000006025000.zip';m=PPO.load(p,device='cpu');a=StageEnv();b=Arena2016V2Env();results={'checkpoint':str(p),'physics':{k:getattr(a,k).tolist() if isinstance(getattr(a,k),np.ndarray) else getattr(a,k) for k in ['dt','max_steps','beams','limits','reverse_limit','accel','decel','max_xy','max_range','lidar_offset']},'physical_grid_equal':np.array_equal(a.physical_grid.cells,b.physical_grid.cells),'laser_grid_equal':np.array_equal(a.laser_grid.cells,b.laser_grid.cells),'layout_objects_equal':a.config['objects']==b.config['objects'],'shape_observation':a.observation_space.shape,'shape_action':a.action_space.shape}
means=[];sat=[];scenarios=[];max_obs=0.;max_reward=0.;max_pose=0.
for i in range(100):
 obs,_=a.reset(seed=3700000000+i);scenarios.append([*a.pose,*a.goal,np.linalg.norm(a.goal-a.pose[:2]),a.friction])
 with torch.no_grad():d=m.policy.get_distribution(torch.tensor(obs[None]));mu=d.distribution.mean.numpy()[0];sd=d.distribution.stddev.numpy()[0]
 means.append(mu.tolist());sat.append([1-(math.erf((1-u)/v/math.sqrt(2))-math.erf((-1-u)/v/math.sqrt(2)))/2 for u,v in zip(mu,sd)])
 if i<10:
  c=copy.deepcopy(a);c.lidar=types.MethodType(Arena2016V2Env.lidar,c)
  gen=np.random.default_rng(i)
  for j in range(20):
   action=gen.uniform(-1,1,3);x,r,t,tr,_=a.step(action);y,s,u,ur,_=c.step(action)
   max_obs=max(max_obs,float(np.max(np.abs(x-y))));max_reward=max(max_reward,abs(r-s));max_pose=max(max_pose,float(np.max(np.abs(a.pose-c.pose))))
   if t or tr:break
results.update(saturation_by_axis=np.mean(sat,axis=0).tolist(),mean_action_abs=np.mean(np.abs(means),axis=0).tolist(),deterministic_clip_by_axis=np.mean(np.abs(means)>=np.ones(3),axis=0).tolist(),native_python_max_obs_error=max_obs,native_python_max_reward_error=max_reward,native_python_max_pose_error=max_pose,unique_starts=len(np.unique(np.array(scenarios)[:,:3],axis=0)),sample_distance_range=[float(np.min(np.array(scenarios)[:,5])),float(np.max(np.array(scenarios)[:,5]))])
results['hashes']={str(p.relative_to(R)):hashlib.sha256(p.read_bytes()).hexdigest() for p in [R/'robo_rl_v2/env.py',R/'experimentos_v2_20261110/train_gated.py',R/'experimentos_v2_20261110/full_sampler.py',R/'experimentos_v2_20261110/native.py',p]}
(O/'probe.json').write_text(json.dumps(results,indent=2));print(json.dumps(results,indent=2))
a.close();b.close()
