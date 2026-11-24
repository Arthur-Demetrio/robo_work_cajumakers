import os
os.environ['OMP_NUM_THREADS']='1'
os.environ['MKL_NUM_THREADS']='1'
import sys,json,hashlib,csv,time
from pathlib import Path
import numpy as np,torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from stable_baselines3 import PPO
from robo_rl_v2.env import RoboEnv,RADIUS,rotation,wrap
from robo_rl_v2.maps import Grid,ROOT
OUT=Path(__file__).resolve().parent
RUN=ROOT/'runs/principal'
torch.set_num_threads(1)
torch.set_num_interop_threads(1)
def manifest():
 return {str(p.relative_to(ROOT)):dict(size=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for folder in ['runs/principal','robo_rl_v2','assets'] for p in (ROOT/folder).rglob('*') if p.is_file() and '__pycache__' not in str(p)}
if not (OUT/'preservacao_antes.json').exists(): (OUT/'preservacao_antes.json').write_text(json.dumps(manifest(),indent=2))
models=[('300000',RUN/'checkpoints/ppo_000000300000.zip'),('1100000',RUN/'checkpoints/ppo_000001100000.zip'),('best_1175000',RUN/'best.zip'),('2000384',RUN/'checkpoints/ppo_000002000384.zip')]
for label,path in models:
 if len(sys.argv)>1 and label!=sys.argv[1]:continue
 if (OUT/f'episodes_{label}.json').exists():continue
 model=PPO.load(path,device='cpu');rows=[]
 groups=[(f'level_{l}',l,[2_300_000_000+l*10000+i for i in range(24)]) for l in range(1,6)]
 if label=='2000384':groups.append(('viewer',4,list(range(2_100_000_000,2_100_000_008))))
 for group,level,seeds in groups:
  envs=[RoboEnv(level=level,split='viewer_eval' if group=='viewer' else 'eval') for _ in seeds]
  obs=[];states=[]
  for env,seed in zip(envs,seeds):
   ob,_=env.reset(seed=seed);obs.append(ob)
   states.append(dict(group=group,level=level,seed=seed,reward=0.,initial_distance=float(np.linalg.norm(env.goal-env.pose[:2])),initial_collision=env.collision(),min_clearance=999.,max_est_error=0.,max_vx=0.,trajectory=[],actions=[],fingerprint=hashlib.sha256(env.grid.cells.tobytes()+env.pose.tobytes()+env.goal.tobytes()).hexdigest()))
  active=list(range(len(seeds)));obs=np.array(obs)
  while active:
   actions,_=model.predict(obs,deterministic=True);next_obs=[];next_active=[]
   for k,i in enumerate(active):
    env=envs[i];s=states[i];s['trajectory'].append(np.r_[env.pose,env.estimate,env.velocity].tolist());s['actions'].append(actions[k].tolist())
    ob,r,t,tr,info=env.step(actions[k]);s['reward']+=r
    s['min_clearance']=min(s['min_clearance'],env.grid.distance(env.pose[:2])-RADIUS)
    s['max_est_error']=max(s['max_est_error'],float(np.linalg.norm(env.pose[:2]-env.estimate[:2])))
    s['max_vx']=max(s['max_vx'],float(abs(env.velocity[0])))
    if t or tr:
     s.update(info);s['timeout']=tr;s['final_est_error']=float(np.linalg.norm(env.pose[:2]-env.estimate[:2]));s['estimated_goal_distance']=float(np.linalg.norm(env.goal-env.estimate[:2]));s['yaw_error']=float(abs(wrap(env.pose[2]-env.estimate[2])))
     name=f'{label}_{group}_{i}'
     np.savez_compressed(OUT/f'{name}.npz',trajectory=s.pop('trajectory'),actions=s.pop('actions'),cells=env.grid.cells,base_cells=env.grid.base_cells,goal=env.goal,resolution=env.grid.resolution,origin=env.grid.origin)
     rows.append(s)
    else: next_active.append(i);next_obs.append(ob)
   active=next_active;obs=np.array(next_obs)
  print(label,group,'success',sum(r['success'] for r in rows if r['group']==group),'/',len(seeds),flush=True)
  (OUT/f'partial_{label}.json').write_text(json.dumps(rows,indent=2))
 (OUT/f'episodes_{label}.json').write_text(json.dumps(rows,indent=2))
after=manifest();before=json.loads((OUT/'preservacao_antes.json').read_text());(OUT/'preservacao_depois.json').write_text(json.dumps(after,indent=2));(OUT/'preservacao_resultado.json').write_text(json.dumps(dict(identical=before==after,changed=[k for k in before if before[k]!=after.get(k)]),indent=2))
