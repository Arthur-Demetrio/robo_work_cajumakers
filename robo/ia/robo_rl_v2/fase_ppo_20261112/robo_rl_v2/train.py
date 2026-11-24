"""Standalone PPO. No network, ROS, subprocess API calls, or Codex dependency."""
import argparse,csv,fcntl,json,os,random,signal,time
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure
from stable_baselines3.common.monitor import Monitor
from .env import RoboEnv
from .evaluate import evaluate
from .events import ScalarEvents


def atomic_json(path,data):
    temp=path.with_suffix('.tmp');temp.write_text(json.dumps(data,indent=2));os.replace(temp,path)

def save_checkpoint(model, run):
    checkpoint=run/'checkpoints'/f'ppo_{model.num_timesteps:012d}.zip'
    checkpoint.parent.mkdir(parents=True,exist_ok=True)
    temporary=checkpoint.with_name('pending.zip')
    model.save(temporary);os.replace(temporary,checkpoint)
    atomic_json(run/'latest.json',dict(model=str(checkpoint.resolve()),steps=model.num_timesteps))
    return checkpoint

class Supervision(BaseCallback):
    def __init__(self,run,args,env):
        super().__init__();self.run,self.args,self.env=run,args,env;self.stop=False
        self.events=ScalarEvents(run/'tensorboard');self.last_save=0;self.last_eval=0
        best_path=run/'best.json';self.best=json.loads(best_path.read_text())['score'] if best_path.exists() else -float('inf')
        self.csv=(run/'episodes.csv').open('a',buffering=1)
        self.writer=None
    def _on_training_start(self):
        self.last_save=self.model.num_timesteps;self.last_eval=self.model.num_timesteps//self.args.eval_every*self.args.eval_every
    def _on_step(self):
        # Single env counter survives curriculum restarts and checkpoint continuation.
        self.env.unwrapped.transitions=self.num_timesteps
        for info,done in zip(self.locals['infos'],self.locals['dones']):
            if done:
                row={k:info[k] for k in ['level','success','collision','elapsed','path_length','stopped_time','rotations','oscillations','recoveries']}
                row['steps_total']=self.num_timesteps
                if self.writer is None:
                    self.writer=csv.DictWriter(self.csv,fieldnames=list(row))
                    if self.csv.tell()==0:self.writer.writeheader()
                self.writer.writerow(row)
                for key,value in row.items():self.events.scalar('episode/'+key,value,self.num_timesteps)
        if self.num_timesteps-self.last_save>=self.args.checkpoint_every:
            save_checkpoint(self.model,self.run);self.last_save=self.num_timesteps
        if self.num_timesteps-self.last_eval>=self.args.eval_every and not self.stop:
            summary=evaluate(self.model,self.run/'evaluations'/str(self.num_timesteps),self.args.eval_episodes,self.args.max_steps)
            score=float(np.mean([x['success']-x['collision'] for x in summary.values()]))
            for scenario,metrics in summary.items():
                for key,value in metrics.items():
                    if value is not None:self.events.scalar(f'eval/{scenario}/{key}',value,self.num_timesteps)
            if score>self.best:
                self.best=score
                best=self.run/'best.zip';tmp=self.run/'best_pending.zip';self.model.save(tmp);os.replace(tmp,best)
                atomic_json(self.run/'best.json',dict(score=score,steps=self.num_timesteps))
            self.last_eval=self.num_timesteps
        return not self.stop
    def _on_rollout_end(self):
        for key,value in self.model.logger.name_to_value.items():
            if isinstance(value,(float,int,np.floating)):self.events.scalar(key,value,self.num_timesteps)
    def close(self):self.csv.close();self.events.close()


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--run',default='runs/principal');p.add_argument('--steps',type=int,default=None,help='Meta total acumulada; não passos adicionais')
    p.add_argument('--resume',action='store_true');p.add_argument('--seed',type=int,default=42)
    p.add_argument('--threads',type=int,default=2);p.add_argument('--eval-every',type=int,default=25000)
    p.add_argument('--checkpoint-every',type=int,default=10000);p.add_argument('--eval-episodes',type=int,default=3)
    p.add_argument('--max-steps',type=int,default=1500);p.add_argument('--n-steps',type=int,default=512)
    a=p.parse_args()
    if a.steps is None:
        config_path=Path(a.run).resolve()/'config.json'
        a.steps=json.loads(config_path.read_text())['steps'] if a.resume and config_path.exists() else 2_000_000
    if min(a.steps,a.threads,a.eval_every,a.checkpoint_every,a.eval_episodes,a.max_steps)<=0 or a.n_steps<64 or a.n_steps%64:
        p.error('Valores positivos; n-steps deve ser múltiplo de 64')
    run=Path(a.run).resolve();run.mkdir(parents=True,exist_ok=True)
    lock=(run/'train.lock').open('w')
    try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:raise SystemExit('Já há um treino neste diretório')
    torch.set_num_threads(a.threads);torch.set_num_interop_threads(1)
    torch.use_deterministic_algorithms(True)
    prior=run/'config.json'
    boundaries=[int(a.steps*f) for f in (.15,.35,.55,.8)]
    if a.resume:
        if not (run/'latest.json').exists():raise SystemExit('Nenhum checkpoint para continuar')
        old=json.loads(prior.read_text())
        # Preserve task/curriculum/hyperparameters from initial run.
        for name in ('seed','max_steps','n_steps'):setattr(a,name,old[name])
        boundaries=old['curriculum_boundaries']
    elif (run/'latest.json').exists():raise SystemExit('Diretório já treinado; use continuar_treino.sh')
    else:atomic_json(prior,dict(vars(a),curriculum_boundaries=boundaries))
    env=Monitor(RoboEnv(max_steps=a.max_steps),str(run/f'monitor_{time.time_ns()}.csv'))
    env.unwrapped.set_curriculum(boundaries)
    if a.resume:
        state=json.loads((run/'latest.json').read_text())
        model=PPO.load(state['model'],env=env,device='cpu')
        env.unwrapped.transitions=model.num_timesteps
        # Resume weights + optimizer + schedule counters; start fresh episode and RNG stream.
        model.set_random_seed(a.seed+model.num_timesteps)
    else:
        model=PPO('MlpPolicy',env,seed=a.seed,device='cpu',n_steps=a.n_steps,batch_size=64,
                  n_epochs=10,learning_rate=3e-4,gamma=.995,gae_lambda=.95,ent_coef=.01,
                  policy_kwargs=dict(net_arch=dict(pi=[128,128],vf=[128,128])),verbose=1)
    model.set_logger(configure(str(run/f'optimizer_{time.time_ns()}'),['stdout','csv']))
    callback=Supervision(run,a,env)
    def stop(signum,frame):
        callback.stop=True
        print('Parada solicitada; salvando no próximo passo seguro.',flush=True)
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    atomic_json(run/'process.json',dict(pid=os.getpid(),started=time.time(),start_ticks=Path(f'/proc/{os.getpid()}/stat').read_text().split()[21]))
    try:
        remaining=max(0,a.steps-model.num_timesteps)
        if remaining:model.learn(total_timesteps=remaining,reset_num_timesteps=False,callback=callback)
        save_checkpoint(model,run)
        atomic_json(run/'status.json',dict(steps=model.num_timesteps,stopped=callback.stop,completed=model.num_timesteps>=a.steps))
    finally:
        callback.close();env.close();(run/'process.json').unlink(missing_ok=True)
        fcntl.flock(lock,fcntl.LOCK_UN);lock.close()

if __name__=='__main__':main()
