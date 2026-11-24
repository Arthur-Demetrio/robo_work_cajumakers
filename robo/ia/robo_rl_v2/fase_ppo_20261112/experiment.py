"""Isolated, paired PPO pilot; never writes to the original run."""
import os
os.environ.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',MPLBACKEND='Agg')
import sys,json,time,hashlib,random,resource,signal,copy,math
from pathlib import Path
import numpy as np
import torch
ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT),str(ROOT/'experimentos_v2_20261110')]
from train_gated import StageEnv,STAGES
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.buffers import RolloutBuffer
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.logger import configure

torch.set_num_threads(1)
torch.use_deterministic_algorithms(True)
ARMS={'controle':.01,'entropia_zero':0.,'variancia_angular':.01}
SEEDS=[4201,4202,4203]
BUDGET=250000
EVAL_EVERY=25000

def atomic(path,data):
    tmp=path.with_suffix('.tmp');tmp.write_text(json.dumps(data,indent=2,allow_nan=False));os.replace(tmp,path)
def clean(x):
    if isinstance(x,dict):return {k:clean(v) for k,v in x.items()}
    if isinstance(x,(list,tuple,np.ndarray)):return [clean(v) for v in x]
    if isinstance(x,(float,np.floating)):return float(x) if np.isfinite(x) else None
    if isinstance(x,np.integer):return int(x)
    if isinstance(x,np.bool_):return bool(x)
    return x

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def memory():
    data={k:int(v.split()[0])*1024 for k,v in (line.split(':',1) for line in Path('/proc/meminfo').read_text().splitlines())}
    status={k:v for k,v in (line.split(':',1) for line in Path('/proc/self/status').read_text().splitlines())}
    return dict(available_bytes=data['MemAvailable'],rss_bytes=int(status['VmRSS'].split()[0])*1024,load1=os.getloadavg()[0],time=time.time())
def resource_gate(out):
    while True:
        m=memory()
        with (out/'resources.jsonl').open('a') as f:f.write(json.dumps(m)+'\n')
        if m['rss_bytes']>2*1024**3:raise RuntimeError('Experimental RSS exceeded 2 GiB')
        if m['available_bytes']>=3*1024**3 and m['load1']<6.5:return
        atomic(out/'resource_wait.json',m)
        time.sleep(10)
        if (ROOT/'stop.request').exists():raise InterruptedError('User stop request')

class AngularProjection:
    """Projected Adam: constrain the parameter itself AFTER every optimizer step.
    Raw samples/log_probs always refer to the same Gaussian. Adam moments retained.
    """
    def __init__(self,policy):
        self.policy=policy;self.count=0;self.steps=0;self.project()
        self.handle=policy.optimizer.register_step_post_hook(self.after_step)
    def project(self):
        with torch.no_grad():
            p=self.policy.log_std
            old=float(p[2]);p[2].clamp_(-3.,0.)
            self.count+=int(old!=float(p[2]))
    def after_step(self,*_):self.steps+=1;self.project()
    def state(self):
        state=self.policy.optimizer.state.get(self.policy.log_std,{})
        return dict(projections=self.count,optimizer_steps=self.steps,adam_angular_m= float(state.get('exp_avg',torch.zeros(3))[2]),adam_angular_v=float(state.get('exp_avg_sq',torch.zeros(3))[2]))

class OptimizerAudit:
    def __init__(self,policy):
        self.policy=policy;self.metrics={};self.calls=0
        self.pre=policy.optimizer.register_step_pre_hook(self.before)
        self.post=policy.optimizer.register_step_post_hook(self.after)
    def before(self,*_):
        for group,prefixes in {'actor':('action_net','mlp_extractor.policy_net'),'critic':('value_net','mlp_extractor.value_net'),'log_std':('log_std',)}.items():
            grads=[p.grad.detach().square().sum() for name,p in self.policy.named_parameters() if name.startswith(prefixes) and p.grad is not None]
            self.metrics['audit/grad_norm_'+group]=float(torch.stack(grads).sum().sqrt()) if grads else 0.
        self.previous={k:p.detach().clone() for k,p in self.policy.named_parameters()}
    def after(self,*_):
        self.calls+=1
        if not all(torch.isfinite(p).all() for p in self.policy.parameters()):raise FloatingPointError('nonfinite parameters after Adam')
        changes=[(p.detach()-self.previous[k]).square().sum() for k,p in self.policy.named_parameters()]
        self.metrics['audit/parameter_delta_norm']=float(torch.stack(changes).sum().sqrt())
        state=self.policy.optimizer.state.get(self.policy.log_std,{})
        self.metrics['audit/adam_angular_m']=float(state.get('exp_avg',torch.zeros(3))[2])
        self.metrics['audit/adam_angular_v']=float(state.get('exp_avg_sq',torch.zeros(3))[2])

class Telemetry(BaseCallback):
    def __init__(self,out):super().__init__();self.out=out;self.stop=False;self.returns=np.zeros(2);self.sat=np.zeros(3);self.count=0;self.raw2=np.zeros(3);self.rawsum=np.zeros(3)
    def _on_step(self):
        actions=self.locals['actions'];self.sat+=(np.abs(actions)>=1).sum(0);self.count+=len(actions);self.raw2+=(actions**2).sum(0);self.rawsum+=actions.sum(0)
        self.returns+=self.locals['rewards']
        for i,(done,info) in enumerate(zip(self.locals['dones'],self.locals['infos'])):
            if done:
                row=dict(steps=int(self.num_timesteps),return_=float(self.returns[i]),**{k:info[k] for k in ['success','collision','distance','stage','elapsed','path_length','rotations','oscillations']})
                with (self.out/'episodes.jsonl').open('a') as f:f.write(json.dumps(clean(row))+'\n')
                self.returns[i]=0
        return not self.stop and not (ROOT/'stop.request').exists()
    def metrics(self):
        n=max(1,self.count)
        return dict(saturation=(self.sat/n).tolist(),raw_action_variance=(self.raw2/n-(self.rawsum/n)**2).tolist())

def evaluate(model,stage,seeds,stochastic=False,full=False):
    # Preserve the entire training RNG stream, including stochastic evaluation.
    rng=(random.getstate(),np.random.get_state(),torch.get_rng_state())
    env=StageEnv(stage=3 if full else stage)
    rows=[]
    try:
        for seed in seeds:
            torch.manual_seed(seed)
            obs,_=env.reset(seed=seed);total=0.;saturation=np.zeros(3);n=0
            initial=float(np.linalg.norm(env.goal-env.pose[:2]));pose=env.pose.copy().tolist();goal=env.goal.copy().tolist()
            while True:
                with torch.no_grad():
                    d=model.policy.get_distribution(torch.as_tensor(obs[None],device=model.device))
                    raw=d.get_actions(deterministic=not stochastic).cpu().numpy()[0]
                saturation+=np.abs(raw)>=1;n+=1
                obs,r,t,tr,info=env.step(np.clip(raw,-1,1));total+=r
                if t or tr:break
            rows.append(dict(seed=seed,return_=total,timeout=bool(tr),initial_distance=initial,initial_pose=pose,goal=goal,saturation=(saturation/n).tolist(),localization_error=float(np.linalg.norm(env.pose[:2]-env.estimate[:2])),**{k:info[k] for k in ['success','collision','distance','stage','elapsed','path_length','rotations','oscillations']}))
    finally:
        env.close();random.setstate(rng[0]);np.random.set_state(rng[1]);torch.set_rng_state(rng[2])
    return rows

def summary(rows):
    return {k:float(np.mean([r[k] for r in rows])) for k in ['success','collision','return_','timeout','distance','localization_error']}

def assessment(model,out,new_steps,stage,telemetry,projection,baseline=False,final=False):
    resource_gate(out)
    # 100 fixed gate scenarios. Holdout never selects checkpoints or curriculum.
    groups={'fixed':evaluate(model,0,range(3500000000,3500000100)),
            'holdout':evaluate(model,0,range(3600000000,3600000100)),
            'stochastic':evaluate(model,0,range(3700000000,3700000030),True)}
    if stage:groups['gate']=evaluate(model,stage,range(3500000000+stage*10000,3500000100+stage*10000))
    if final:
        groups['unseen']=evaluate(model,0,range(3800000000,3800000300))
        groups['full_arena']=evaluate(model,3,range(3900000000,3900000100),full=True)
    metrics={k:summary(v) for k,v in groups.items()}
    std=model.policy.log_std.detach().exp().cpu().numpy()
    record=clean(dict(new_steps=new_steps,steps=model.num_timesteps,stage=stage,metrics=metrics,episodes=groups,std=std,angular_variance=float(std[2]**2),telemetry=telemetry.metrics(),optimizer=getattr(model,"pilot_metrics",{}),projection=projection.state() if projection else None))
    atomic(out/'evaluations'/f'{new_steps:06d}.json',record)
    print(json.dumps(dict(out=out.name,new_steps=new_steps,metrics=metrics,std=std.tolist())),flush=True)
    return record

def train(arm,seed,budget=BUDGET):
    out=ROOT/'runs'/f'{arm}_s{seed}';out.mkdir(parents=True,exist_ok=False);(out/'evaluations').mkdir();(out/'checkpoints').mkdir()
    env=DummyVecEnv([StageEnv,StageEnv]);model=PPO.load(ROOT/'origem.zip',env=env,device='cpu')
    initial=model.num_timesteps
    assert model.n_steps==256 and model.n_envs==2 and model.batch_size==64
    model.ent_coef=ARMS[arm];model.set_random_seed(seed)
    model.set_logger(configure(str(out/'optimizer'),['csv']))
    initial_weights=copy.deepcopy(model.policy.state_dict())
    atomic(out/'config.json',dict(arm=arm,seed=seed,budget=budget,source_sha256=digest(ROOT/'origem.zip'),initial_steps=initial,ent_coef=model.ent_coef,angular_log_std_bounds=[-3,0] if arm=='variancia_angular' else None,optimizer_restored=True,stage_initial=0,n_steps_nominal=256,n_envs=2,evaluation_every=EVAL_EVERY))
    projection=AngularProjection(model.policy) if arm=='variancia_angular' else None
    audit=OptimizerAudit(model.policy)
    telemetry=Telemetry(out);stage=0;reason=None;history=[];best=-float('inf');start=time.monotonic()
    signal.signal(signal.SIGTERM,lambda *_:setattr(telemetry,'stop',True))
    signal.signal(signal.SIGINT,lambda *_:setattr(telemetry,'stop',True))
    _,cb=model._setup_learn(budget,telemetry,reset_num_timesteps=False,progress_bar=False)
    cb.on_training_start(locals(),globals())
    try:
        history.append(assessment(model,out,0,stage,telemetry,projection,baseline=True))
        while model.num_timesteps-initial<budget:
            resource_gate(out)
            new=model.num_timesteps-initial
            next_eval=min((new//EVAL_EVERY+1)*EVAL_EVERY,budget)
            # Shorten only the rollout crossing an evaluation/budget boundary.
            # All arms use the identical schedule: no discarded transitions/overshoot.
            n=min(256,(next_eval-new)//2)
            model.rollout_buffer=RolloutBuffer(n,model.observation_space,model.action_space,device=model.device,gae_lambda=model.gae_lambda,gamma=model.gamma,n_envs=2)
            if not model.collect_rollouts(env,cb,model.rollout_buffer,n_rollout_steps=n):reason='requested_stop';break
            model._update_current_progress_remaining(model.num_timesteps,initial+budget)
            model.train();model.pilot_metrics=dict(model.logger.name_to_value,**audit.metrics);model.logger.dump(model.num_timesteps)
            if not all(torch.isfinite(p).all() for p in model.policy.parameters()):reason='nonfinite_parameters';break
            if model.num_timesteps-initial==next_eval:
                new=next_eval
                # Snapshots are always AFTER optimization; checkpoint supervisor paired.
                model.save(out/'checkpoints'/f'{new:06d}.zip')
                atomic(out/'checkpoints'/f'{new:06d}.json',dict(new_steps=new,stage=stage,projection=projection.state() if projection else None))
                rec=assessment(model,out,new,stage,telemetry,projection,final=new==budget);history.append(rec)
                fixed=rec['metrics']['fixed'];score=fixed['success']-fixed['collision']
                if score>best:best=score;model.save(out/'best_fixed.zip')
                gate=rec['metrics'].get('gate',fixed)
                if gate['success']>=.9 and gate['collision']==0 and stage<3:stage+=1;env.env_method('set_stage',stage)
                last=history[-3:]
                if len(last)==3 and all(r['std'][2]>10 for r in last):reason='angular_std_above_10_three_evaluations'
                if len(last)==3 and all((r['optimizer'].get('train/approx_kl') or 0)>.2 for r in last):reason='KL_above_0.2_three_evaluations'
                if len(history)>=4 and all(r['metrics']['fixed']['success']==0 and r['metrics']['fixed']['collision']>=.98 and r['telemetry']['saturation'][2]>=.98 for r in last):reason='zero_success_collision_and_saturation_three_evaluations'
                atomic(out/'live.json',dict(new_steps=new,stage=stage,reason=reason))
                import report
                report.generate()
                if reason:break
        model.save(out/'final.zip')
        delta={}
        for group in ['action_net','mlp_extractor.policy_net','value_net','mlp_extractor.value_net','log_std']:
            values=[(v-initial_weights[k]).float().square().flatten() for k,v in model.policy.state_dict().items() if k.startswith(group)]
            delta[group]=float(torch.cat(values).mean().sqrt())
        atomic(out/'status.json',dict(completed=model.num_timesteps-initial==budget,reason=reason,new_steps=model.num_timesteps-initial,wall_s=time.monotonic()-start,parameter_rms_delta=delta,stage=stage))
    except BaseException as exc:
        model.save(out/'interrupted.zip');atomic(out/'status.json',dict(completed=False,reason=repr(exc),new_steps=model.num_timesteps-initial));raise
    finally:
        cb.on_training_end();env.close();audit.pre.remove();audit.post.remove()
        if projection:projection.handle.remove()
    import report
    report.generate()

def main():
    validation=json.loads((ROOT/'validation.json').read_text())
    assert validation['passed'] and validation['experiment_sha256']==digest(Path(__file__)), 'Run automated tests before training'
    os.nice(15);os.sched_setaffinity(0,[max(os.sched_getaffinity(0))])
    atomic(ROOT/'process.json',dict(pid=os.getpid(),started=time.time(),affinity=list(os.sched_getaffinity(0)),priority=os.getpriority(os.PRIO_PROCESS,0)))
    try:
        # Interleave arms within each seed to reduce temporal resource confounding.
        for seed in SEEDS:
            for arm in ARMS:
                if (ROOT/'stop.request').exists():return
                train(arm,seed)
        import preservation
        verified=preservation.verify()
        atomic(ROOT/'complete.json',dict(finished=time.time(),preservation=verified))
    finally:
        import report
        report.generate()
        (ROOT/'process.json').unlink(missing_ok=True)
if __name__=='__main__':main()
