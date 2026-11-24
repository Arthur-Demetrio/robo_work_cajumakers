import os
os.environ.update(OMP_NUM_THREADS='1',MKL_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
import sys,time,json,argparse,hashlib,functools,signal,csv
from pathlib import Path
import numpy as np,torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv,SubprocVecEnv
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor
from envs import FastV2Env,make_env
ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'runs/treino_2016_transferido_20261110_085342/final.zip'
torch.set_num_threads(1)
class Recorder(BaseCallback):
 def __init__(self,out):super().__init__();self.out=out;self.stop=False;self.rows=[];self.last=0
 def _on_step(self):
  for info,done in zip(self.locals['infos'],self.locals['dones']):
   if done:self.rows.append(dict(total_steps=self.num_timesteps,**{k:info[k] for k in ['success','collision','elapsed','distance']}))
  if self.num_timesteps-self.last>=8192:
   self.model.save(self.out/f'checkpoint_{self.num_timesteps:08d}.zip');self.last=self.num_timesteps
   (self.out/'episodes.json').write_text(json.dumps(self.rows))
   print('steps',self.num_timesteps,flush=True)
  return not self.stop and not (self.out/'stop.request').exists()
def vec(n,maximum,guidance,native=False):
 factory=make_env
 if native:
  from native import NativeV2Env
  factory=NativeV2Env
 factories=[functools.partial(factory,maximum=maximum,guidance=guidance) for _ in range(n)]
 return DummyVecEnv(factories) if n==1 else SubprocVecEnv(factories,start_method='fork')
def train(args,bench=False):
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False);n=args.envs
 env=vec(n,12. if bench else 2.,args.guidance,args.native)
 model=PPO.load(SOURCE,env=env,device='cpu',n_steps=512//n)
 model.set_random_seed(args.seed)
 config=dict(native=args.native,mode='benchmark' if bench else 'pilot',n_envs=n,steps_requested=args.steps,maximum_goal_distance_m=12. if bench else 2.,guidance=args.guidance,seed=args.seed,source=str(SOURCE),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),source_steps=int(model.num_timesteps),rollout_total_transitions=model.n_steps*n,n_steps_per_env=model.n_steps,stage='fixed_short_goals_no_clock_progression',advance_gate='>=90% success and zero observed collisions in 100 independent short-goal scenarios')
 (out/'config.json').write_text(json.dumps(config,indent=2));recorder=Recorder(out)
 signal.signal(signal.SIGTERM,lambda *_:setattr(recorder,'stop',True));signal.signal(signal.SIGINT,lambda *_:setattr(recorder,'stop',True))
 start=time.perf_counter();model.learn(total_timesteps=args.steps,reset_num_timesteps=True,callback=recorder);elapsed=time.perf_counter()-start
 model.save(out/'final.zip');(out/'episodes.json').write_text(json.dumps(recorder.rows))
 result=dict(transitions=model.num_timesteps,wall_s=elapsed,transitions_s=model.num_timesteps/elapsed,stopped=recorder.stop or (out/'stop.request').exists())
 (out/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result),flush=True);env.close()
def evaluate(args):
 out=Path(args.output);out.mkdir(parents=True,exist_ok=False);rows=[]
 planner=args.oracle or args.planner_estimated
 model=PPO.load(args.model,device='cpu') if not planner else None
 # Equal episode assignment independent of parallelism; reset each worker explicitly.
 from native import NativeV2Env
 EvalEnv=NativeV2Env
 if args.full_sampler:
  if args.guidance or planner:raise ValueError('Disk planner incompatible with full footprint sampler')
  from full_sampler import FullFootprintEnv
  EvalEnv=FullFootprintEnv
 envs=[EvalEnv(maximum=args.maximum,guidance=args.guidance) for _ in range(args.envs)]
 for offset in range(0,args.episodes,args.envs):
  active=[];obs=[];states={}
  for i,env in enumerate(envs):
   j=offset+i
   if j>=args.episodes:break
   seed=args.seed+j;ob,_=env.reset(seed=seed);active.append(i);obs.append(ob)
   states[i]=dict(seed=seed,initial_distance=float(np.linalg.norm(env.goal-env.pose[:2])),reward=0.,fingerprint=hashlib.sha256(env.grid.cells.tobytes()+env.pose.tobytes()+env.goal.tobytes()).hexdigest(),max_est_error=0.)
   if planner:env.route=env.planner.plan(env.pose[:2],env.goal)
  while active:
   if args.truth_pose:
    for i in active:envs[i].estimate=envs[i].pose.copy()
    obs=[envs[i].observation() for i in active]
   if not planner:actions,_=model.predict(np.array(obs),deterministic=True)
   follow=[];next_obs=[]
   for k,i in enumerate(active):
    e=envs[i];s=states[i]
    if planner:
     position=e.estimate if args.planner_estimated else e.pose
     target=e.planner.waypoint(e.route,position[:2],e.goal);local=(target-position[:2])@__import__('envs').rotation(position[2])
     angle=np.arctan2(local[1],local[0]);cmd=np.array([np.clip(local[0]*.7,-.16,.2),np.clip(local[1]*.7,-.08,.08),np.clip(angle*.5,-.16,.16)])
     action=cmd/e.limits
     if cmd[0]<0:action[0]=cmd[0]/e.reverse_limit
    else:action=actions[k]
    ob,r,t,tr,info=e.step(action);s['reward']+=r;s['max_est_error']=max(s['max_est_error'],float(np.linalg.norm(e.pose[:2]-e.estimate[:2])))
    if t or tr:
     s.update(info);s['timeout']=tr;rows.append(s)
     if len(rows)<=3:
      np.savez_compressed(out/f'example_{s["seed"]}.npz',path=e.path,goal=e.goal,cells=e.grid.cells,resolution=e.grid.resolution)
    else:follow.append(i);next_obs.append(ob)
   active=follow;obs=next_obs
  (out/'partial.json').write_text(json.dumps(rows));print('evaluated',len(rows),flush=True)
 rows=sorted(rows,key=lambda r:r['seed']);(out/'episodes.json').write_text(json.dumps(rows,indent=2))
 summary={k:float(np.mean([r[k] for r in rows])) for k in ['success','collision','timeout','elapsed','distance','reward','max_est_error']};summary.update(n=len(rows),guidance=args.guidance,oracle=args.oracle,maximum=args.maximum,implementation='NativeV2Env_equivalence_validated',planner_estimated=args.planner_estimated,truth_pose_input=args.truth_pose)
 (out/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary),flush=True)
 for e in envs:e.close()
def main():
 p=argparse.ArgumentParser();p.add_argument('mode',choices=['benchmark','pilot','evaluate']);p.add_argument('--output',required=True);p.add_argument('--envs',type=int,default=1,choices=[1,2,4,8]);p.add_argument('--steps',type=int,default=32768);p.add_argument('--seed',type=int,default=42);p.add_argument('--guidance',action='store_true');p.add_argument('--model',default=str(SOURCE));p.add_argument('--episodes',type=int,default=100);p.add_argument('--maximum',type=float,default=12.);p.add_argument('--oracle',action='store_true');p.add_argument('--native',action='store_true');p.add_argument('--planner-estimated',action='store_true');p.add_argument('--truth-pose',action='store_true');p.add_argument('--full-sampler',action='store_true');a=p.parse_args()
 if a.mode!='evaluate' and (a.oracle or a.planner_estimated or a.truth_pose or a.full_sampler):p.error('Flags de diagnóstico e amostrador completo são somente para avaliação')
 if a.oracle and a.planner_estimated:p.error('Escolha referência com pose verdadeira ou estimada')
 if a.steps<=0 or a.steps%512:p.error('steps deve ser múltiplo positivo de 512')
 if a.mode=='evaluate':evaluate(a)
 else:train(a,a.mode=='benchmark')
if __name__=='__main__':main()
