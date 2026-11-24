"""Independent v2 run: native lidar, full-footprint sampling, performance-gated stages."""
import os
os.environ.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
import sys,json,hashlib,time,argparse,functools,signal,csv
from pathlib import Path
from collections import deque
import numpy as np,torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from full_sampler import FullFootprintEnv
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3.common.callbacks import BaseCallback
CODE=Path(__file__).parent;ROOT=CODE.parent
SOURCE=ROOT/'runs/treino_2016_transferido_20261110_085342/final.zip'
STAGES=[dict(maximum=2.,line_of_sight=True),dict(maximum=2.,line_of_sight=False),dict(maximum=4.,line_of_sight=False),dict(maximum=12.,line_of_sight=False)]
torch.set_num_threads(1)
def atomic(path,data):
 tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2));os.replace(tmp,path)
class StageEnv(FullFootprintEnv):
 def __init__(self,stage=0):
  self.stage=stage
  super().__init__(layout=CODE/'layout_service_candidate.yaml',maximum=STAGES[stage]['maximum'])
 def set_stage(self,stage):self.stage=stage;self.maximum=STAGES[stage]['maximum']
 def reset(self,*,seed=None,options=None):
  obs,info=super().reset(seed=seed,options=options)
  if STAGES[self.stage]['line_of_sight']:
   for _ in range(10000):
    a=self.state()
    if self.grid.distance(a[:2])<=.52:continue
    b=self.state();distance=np.linalg.norm(b[:2]-a[:2])
    if not .8<distance<self.maximum:continue
    samples=np.linspace(a[:2],b[:2],max(2,int(distance/self.grid.resolution*2)+1))
    ij=self.grid.indices(samples);x,y=ij[:,0],ij[:,1]
    if not np.all(self.grid.clearance[y,x]>.52):continue
    self.pose=a;self.estimate=a.copy();self.goal=b[:2];self.goal_pose=b.copy();self.path=[a.copy()];self.window=deque([a[:2].copy()],maxlen=32);self.scan=self.lidar();obs=self.observation();break
   else:raise RuntimeError('Não foi possível amostrar percurso inicial livre')
  info['stage']=self.stage;return obs,info
 def step(self,action):
  obs,reward,t,tr,info=super().step(action);info['stage']=self.stage;return obs,reward,t,tr,info

def evaluate(model,stage,episodes):
 env=SubprocVecEnv([functools.partial(StageEnv,stage=stage) for _ in range(2)],start_method='fork')
 rows=[];active=[None,None];obs=np.zeros((2,152),np.float32);next_index=0
 def assign(i):
  nonlocal next_index
  if next_index>=episodes:active[i]=None;return
  seed=3500000000+stage*10000+next_index;next_index+=1;active[i]=seed
  ob,_=env.env_method('reset',seed=seed,indices=i)[0];obs[i]=ob
 for i in range(2):assign(i)
 try:
  while any(s is not None for s in active):
   actions,_=model.predict(obs,deterministic=True);new,_,done,infos=env.step(actions);obs[:]=new
   for i in range(2):
    if done[i] and active[i] is not None:
     info=infos[i];rows.append(dict(seed=active[i],**{k:info[k] for k in ['success','collision','elapsed','distance','stage']}));assign(i)
 finally:env.close()
 return rows
class Supervision(BaseCallback):
 def __init__(self,out,args):
  super().__init__();self.out,self.args=out,args;self.stage=0;self.stop=False;self.stale=0;self.best=-float('inf');self.last_save=0;self.last_eval=0
  self.fp=(out/'episodes.csv').open('x',buffering=1);self.writer=csv.DictWriter(self.fp,fieldnames=['steps','stage','success','collision','elapsed','distance']);self.writer.writeheader()
 def save(self):
  steps=self.model.num_timesteps;path=self.out/'checkpoints'/f'ppo_{steps:012d}.zip'
  if not path.exists():
   tmp=path.with_name(f'pending_{steps:012d}.zip');self.model.save(tmp);os.replace(tmp,path)
  atomic(self.out/'latest.json',dict(model=str(path),steps=steps));return path
 def _on_step(self):
  for done,info in zip(self.locals['dones'],self.locals['infos']):
   if done:self.writer.writerow(dict(steps=self.num_timesteps,**{k:info[k] for k in ['stage','success','collision','elapsed','distance']}))
  if self.num_timesteps%512==0:atomic(self.out/'live.json',dict(steps=self.num_timesteps,stage=self.stage,stale_evaluations=self.stale))
  if self.num_timesteps-self.last_save>=self.args.checkpoint_every:self.save();self.last_save=self.num_timesteps
  if self.num_timesteps-self.last_eval>=self.args.eval_every and not self.stop:
   rows=evaluate(self.model,self.stage,self.args.eval_episodes);success=np.mean([r['success'] for r in rows]);collisions=sum(r['collision'] for r in rows);score=float(success-collisions/len(rows));passed=success>=.9 and collisions==0
   self.save();record=dict(steps=self.num_timesteps,stage=self.stage,n=len(rows),success=float(success),collisions=collisions,gate_passed=bool(passed),episodes=rows)
   folder=self.out/'evaluations';folder.mkdir(exist_ok=True);atomic(folder/f'{self.num_timesteps:012d}.json',record);print('EVALUATION',json.dumps({k:v for k,v in record.items() if k!='episodes'}),flush=True)
   if score>self.best+.005:self.best=score;self.stale=0
   else:self.stale+=1
   if passed and self.stage<len(STAGES)-1:
    self.stage+=1;self.training_env.env_method('set_stage',self.stage);self.stale=0;self.best=-float('inf');print('STAGE',self.stage,flush=True)
   elif self.stale>=self.args.patience:self.stop=True;print('STOP: no improvement in fixed-seed evaluations',flush=True)
   self.last_eval=self.num_timesteps
  if (self.out/'stop.request').exists():self.stop=True
  return not self.stop

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--steps',type=int,default=2000000);p.add_argument('--seed',type=int,default=42);p.add_argument('--checkpoint-every',type=int,default=10000);p.add_argument('--eval-every',type=int,default=25000);p.add_argument('--eval-episodes',type=int,default=100);p.add_argument('--patience',type=int,default=4);a=p.parse_args()
 if min(a.steps,a.checkpoint_every,a.eval_every,a.eval_episodes,a.patience)<=0:p.error('Valores devem ser positivos')
 out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False);(out/'checkpoints').mkdir()
 cfg=dict(vars(a),variant='arena_2016_v2',native=True,n_envs=2,n_steps_per_env=256,rollout_total=512,source=str(SOURCE),source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),prior_cumulative_steps=2347094,layout_sha256=hashlib.sha256((CODE/'layout_service_candidate.yaml').read_bytes()).hexdigest(),stages=STAGES,gate=dict(success_rate=.9,observed_collisions=0),localization_sensor_correction_implemented=False,imitation_learning_implemented=False)
 atomic(out/'config.json',cfg);(out/'layout.yaml').write_bytes((CODE/'layout_service_candidate.yaml').read_bytes())
 env=SubprocVecEnv([StageEnv,StageEnv],start_method='fork');model=PPO.load(SOURCE,env=env,device='cpu',n_steps=256);model.set_random_seed(a.seed);model.num_timesteps=0
 callback=Supervision(out,a);callback.model=model;callback.save()
 signal.signal(signal.SIGTERM,lambda *_:setattr(callback,'stop',True));signal.signal(signal.SIGINT,lambda *_:setattr(callback,'stop',True))
 atomic(out/'process.json',dict(pid=os.getpid(),started=time.time(),control='Create stop.request to save and stop'))
 start=time.monotonic()
 try:
  model.learn(total_timesteps=a.steps,reset_num_timesteps=True,callback=callback);callback.save();model.save(out/'final.zip')
  atomic(out/'status.json',dict(steps=model.num_timesteps,cumulative_steps=2347094+model.num_timesteps,stage=callback.stage,stopped=callback.stop,completed=model.num_timesteps>=a.steps,wall_s=time.monotonic()-start))
 finally:callback.fp.close();env.close();(out/'process.json').unlink(missing_ok=True)
if __name__=='__main__':main()
