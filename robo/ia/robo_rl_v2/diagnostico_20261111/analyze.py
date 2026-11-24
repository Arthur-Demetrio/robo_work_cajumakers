import os
os.environ.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',MPLBACKEND='Agg')
import sys,json,hashlib, csv
from pathlib import Path
import numpy as np,torch,matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1];OUT=Path(__file__).parent;RUN=ROOT/'runs/treino_continuo_20261111'
sys.path[:0]=[str(ROOT),str(ROOT/'experimentos_v2_20261110')]
from train_gated import StageEnv
from native import NativeV2Env
from stable_baselines3 import PPO
torch.set_num_threads(1)
# Pin a read-only snapshot; only independent analysis artifacts are written.
latest=json.loads((RUN/'latest.json').read_text());limit=latest['steps']
evals=[json.loads(p.read_text()) for p in sorted((RUN/'evaluations').glob('*.json')) if int(p.stem)<=limit]
def writecsv(path,rows):
 if not rows:return
 with path.open('w') as fp:
  w=csv.DictWriter(fp,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def readcsv(path):
 with path.open() as fp:rows=list(csv.DictReader(fp))
 for r in rows:
  for k,v in r.items():
   try:r[k]=float(v) if v else float('nan')
   except ValueError:r[k]={'True':1.,'False':0.}.get(v,v)
 return rows
e=[{k:v for k,v in r.items() if k!='episodes'} for r in evals];writecsv(OUT/'evaluations_snapshot.csv',e)
p=[r for r in readcsv(RUN/'optimizer/progress.csv') if r['time/total_timesteps']<=limit];writecsv(OUT/'optimizer_snapshot.csv',p)
t=[r for r in readcsv(RUN/'episodes.csv') if r['steps']<=limit];writecsv(OUT/'episodes_snapshot.csv',t)
def mean(rows,k):return float(np.mean([r[k] for r in rows]))
summary={'snapshot':latest,'eval_count':len(e),'episodes':len(t),'windows':{}}
for name,span in [('500k',500000),('1M',1000000)]:
 end=e[-1]['steps'];groups={}
 for label,lo,hi in [('previous',end-2*span,end-span),('recent',end-span,end)]:
  z=[r for r in e if lo<r['steps']<=hi];q=[r for r in t if lo<r['steps']<=hi];o=[r for r in p if lo<r['time/total_timesteps']<=hi]
  groups[label]={'evals':len(z),'success':mean(z,'success'),'collision':float(np.mean([r['collisions']/r['n'] for r in z])),'train_success':mean(q,'success'),'train_collision':mean(q,'collision'),'train_distance':mean(q,'distance'),'optimizer_median':{k:float(np.nanmedian([r[k] for r in o])) for k in p[0] if k.startswith('train/')}}
 summary['windows'][name]=groups
fig,ax=plt.subplots(3,1,figsize=(12,10),sharex=True)
x=[r['steps'] for r in e]
ax[0].plot(x,[r['success'] for r in e],label='sucesso avaliação');ax[0].plot(x,[r['collisions']/r['n'] for r in e],label='colisão avaliação');ax[0].axhline(.9,color='green',ls='--',label='gate sucesso');ax[0].legend()
b=[]
for key in sorted(set(int(r['steps']//100000)*100000 for r in t)):
 q=[r for r in t if int(r['steps']//100000)*100000==key];b.append(dict(steps=key,success=mean(q,'success'),collision=mean(q,'collision'),distance=mean(q,'distance')))
for k in ['success','collision']:ax[1].plot([r['steps'] for r in b],[r[k] for r in b],label=k+' treino estocástico')
ax[1].legend();ax[2].plot(x,[r['success']-r['collisions']/r['n'] for r in e],label='score de seleção (não retorno)');ax[2].plot([r['steps'] for r in b],[r['distance'] for r in b],label='distância final treino (m)');ax[2].legend();ax[2].set_xlabel('Passos');fig.tight_layout();fig.savefig(OUT/'aprendizado.png');plt.close(fig)
fig,axs=plt.subplots(3,2,figsize=(12,10),sharex=True)
for a,k in zip(axs.flat,['std','approx_kl','entropy_loss','value_loss','explained_variance','clip_fraction']):
 values=np.array([r['train/'+k] for r in p]);a.plot([r['time/total_timesteps'] for r in p],values,alpha=.4);a.set_title(k)
 if k in ['std','value_loss']:a.set_yscale('log')
fig.tight_layout();fig.savefig(OUT/'ppo.png');plt.close(fig)
paths=sorted((RUN/'checkpoints').glob('ppo_*.zip'));chosen=[]
for target in [125000,limit-1000000,limit-500000,limit]:chosen.append(min(paths,key=lambda x:abs(int(x.stem.split('_')[1])-target)))
rows=[];weights=[];reference=None
for path in chosen:
 model=PPO.load(path,device='cpu');model.policy.set_training_mode(False)
 params=torch.cat([v.detach().flatten() for k,v in model.policy.state_dict().items() if k!='log_std']);arr=params.numpy()
 weights.append({'steps':model.num_timesteps,'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'log_std':model.policy.log_std.detach().tolist(),'ent_coef':model.ent_coef,'n_updates':model._n_updates,'n_epochs':model.n_epochs,'target_kl':model.target_kl,'learning_rate':model.learning_rate,'parameter_rms':float(np.sqrt(np.mean(arr**2))),'rms_delta_initial':float(np.sqrt(np.mean((arr-reference)**2))) if reference is not None else 0})
 if reference is None:reference=arr.copy()
 for mode in ['stage0_fixed','stage0_holdout','arena_full']:
  env=StageEnv() if mode!='arena_full' else NativeV2Env()
  for i in range(30):
   seed=(3500000000 if mode=='stage0_fixed' else 3600000000)+i
   obs,_=env.reset(seed=seed);initial=float(np.linalg.norm(env.goal-env.pose[:2]));total=0.;sat=[];progress=0.
   for j in range(env.max_steps):
    with torch.no_grad():dist=model.policy.get_distribution(torch.as_tensor(obs[None],device=model.device));mu=dist.distribution.mean.cpu().numpy()[0];sd=dist.distribution.stddev.cpu().numpy()[0]
    sat.append(float(np.mean(1-(torch.distributions.Normal(torch.tensor(mu),torch.tensor(sd)).cdf(torch.tensor(1.))-torch.distributions.Normal(torch.tensor(mu),torch.tensor(sd)).cdf(torch.tensor(-1.))).numpy())))
    action,_=model.predict(obs,deterministic=True);obs,r,done,tr,info=env.step(action);total+=r
    if done or tr:break
   rows.append(dict(checkpoint=model.num_timesteps,mode=mode,seed=seed,reward=total,initial_distance=initial,action_saturation_probability=np.mean(sat),timeout=tr,**info))
  env.close();writecsv(OUT/'paired_episodes.csv',rows);print(model.num_timesteps,mode,flush=True)
summary['weights']=weights
aggregates=[]
for checkpoint in sorted(set(r['checkpoint'] for r in rows)):
 for mode in ['stage0_fixed','stage0_holdout','arena_full']:
  q=[r for r in rows if r['checkpoint']==checkpoint and r['mode']==mode]
  aggregates.append(dict(checkpoint=checkpoint,mode=mode,n=len(q),**{k:mean(q,k) for k in ['success','collision','reward','distance','elapsed','action_saturation_probability']},reward_std=float(np.std([r['reward'] for r in q],ddof=1))))
writecsv(OUT/'paired_summary.csv',aggregates)
# Native rays versus Python implementation on exactly the same internal state.
from robo_rl_v2.arena2016v2 import Arena2016V2Env
a=StageEnv();a.reset(seed=3500000000);rng=a.np_random.bit_generator.state
native=a.lidar();a.np_random.bit_generator.state=rng;python=Arena2016V2Env.lidar(a)
summary['native_python_max_abs']=float(np.max(np.abs(native[0]-python[0])))
summary['sampling']={'angles':len(a.state_angles),'states':int(a.accum[-1]),'physical_grid_shape':a.physical_grid.cells.shape,'layout_loaded':str(a.layout_path)}
samples=[]
for i in range(100):
 a.reset(seed=3700000000+i);samples.append([*a.pose,*a.goal,np.linalg.norm(a.goal-a.pose[:2]),a.friction])
np.savetxt(OUT/'sampled_scenarios.csv',samples,delimiter=',',header='x,y,yaw,goal_x,goal_y,distance,friction')
samples=np.array(samples);summary['sampling'].update(unique_start=len(np.unique(samples[:,:3],axis=0)),distance_min=float(samples[:,5].min()),distance_max=float(samples[:,5].max()))
a.close()
fig,axs=plt.subplots(1,3,figsize=(14,4))
for mode in ['stage0_fixed','stage0_holdout','arena_full']:
 g=[r for r in aggregates if r['mode']==mode];x=[r['checkpoint'] for r in g]
 for ax,k in zip(axs,['success','collision','reward']):ax.plot(x,[r[k] for r in g],marker='o',label=mode)
for ax,title in zip(axs,['Sucesso (30 episódios)','Colisões (30 episódios)','Retorno médio']):ax.set_title(title);ax.set_xlabel('Passos');ax.legend(fontsize=7)
fig.tight_layout();fig.savefig(OUT/'checkpoints.png');plt.close(fig)
(OUT/'summary.json').write_text(json.dumps(summary,indent=2,default=lambda x:int(x) if isinstance(x,np.integer) else list(x)))
print(json.dumps(summary,indent=2),flush=True)
