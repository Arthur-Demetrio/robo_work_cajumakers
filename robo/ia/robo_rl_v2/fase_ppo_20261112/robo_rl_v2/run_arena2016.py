"""Separate training/evaluation entry point; refuses to overwrite any output directory."""
import argparse,csv,json,hashlib,time,signal,os
from pathlib import Path
import numpy as np,torch
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.callbacks import BaseCallback
from .arena2016 import Arena2016Env,LAYOUT,read_layout
from .visual import draw,plt

class Checkpoints(BaseCallback):
    def __init__(self,folder,interval,base):super().__init__();self.folder,self.interval,self.base=folder,interval,base;self.last=0;self.stop=False
    def _on_step(self):
        self.base.transitions=self.num_timesteps
        if self.num_timesteps-self.last>=self.interval:
            self.model.save(self.folder/f'ppo_{self.num_timesteps:012d}.zip');self.last=self.num_timesteps
        return not (self.stop or (self.folder.parent/'stop.request').exists())

def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('mode',choices=['train','evaluate']);p.add_argument('--variant',default='arena_2016_limpa',choices=list(read_layout()['variants']));p.add_argument('--output',required=True);p.add_argument('--model');p.add_argument('--init-model',help='Checkpoint inicial para novo treino; preserva pesos e otimizador');p.add_argument('--steps',type=int,default=2000000);p.add_argument('--episodes',type=int,default=100);p.add_argument('--max-steps',type=int,default=1500);p.add_argument('--seed',type=int,default=3000000000);p.add_argument('--progression',action='store_true');p.add_argument('--noise',choices=['on','off','config'],default='config');p.add_argument('--boxes',type=int);p.add_argument('--checkpoint-every',type=int,default=10000);p.add_argument('--n-steps',type=int,default=512)
    a=p.parse_args(argv)
    if min(a.steps,a.episodes,a.max_steps,a.checkpoint_every)<=0 or a.n_steps<64 or a.n_steps%64:p.error('Parâmetros inválidos; n-steps deve ser múltiplo de 64')
    if a.boxes is not None and a.boxes<0:p.error('boxes deve ser >=0')
    if a.mode=='evaluate' and not a.model:p.error('evaluate requer --model')
    if a.mode=='evaluate' and a.progression:p.error('Avalie uma variante isolada sem --progression')
    if a.mode=='train' and a.model:p.error('Novo treino usa saída independente; --model é somente para avaliação')
    if a.mode=='evaluate' and a.init_model:p.error('--init-model é somente para treino')
    if a.init_model and not Path(a.init_model).is_file():p.error('Checkpoint inicial não encontrado')
    folder=Path(a.output).resolve();folder.mkdir(parents=True,exist_ok=False)
    torch.set_num_threads(1)
    env=Arena2016Env(variant=a.variant,noise=None if a.noise=='config' else a.noise=='on',boxes=a.boxes,progression=a.progression,max_steps=a.max_steps,split='train' if a.mode=='train' else 'eval')
    config=dict(vars(a),layout_sha256=hashlib.sha256(LAYOUT.read_bytes()).hexdigest(),settings=env.settings,created=time.time())
    (folder/'config.json').write_text(json.dumps(config,indent=2));(folder/'layout.yaml').write_bytes(LAYOUT.read_bytes())
    if a.mode=='train':
        if a.progression:env.set_curriculum([int(a.steps*f) for f in [.15,.35,.55,.8]])
        monitored=Monitor(env,str(folder/'monitor.csv'),info_keywords=('success','collision','level'))
        if a.init_model:
            source=Path(a.init_model).resolve()
            model=PPO.load(source,env=monitored,device='cpu')
            config.update(initial_model=str(source),initial_steps=int(model.num_timesteps),initial_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),step_count='additional_steps_from_zero')
            model.set_random_seed(a.seed)
            (folder/'config.json').write_text(json.dumps(config,indent=2))
        else:
            model=PPO('MlpPolicy',monitored,seed=a.seed,device='cpu',n_steps=a.n_steps,batch_size=64,n_epochs=10,gamma=.995,gae_lambda=.95,ent_coef=.01,learning_rate=3e-4,policy_kwargs=dict(net_arch=dict(pi=[128,128],vf=[128,128])),verbose=1)
        (folder/'checkpoints').mkdir()
        callback=Checkpoints(folder/'checkpoints',a.checkpoint_every,env)
        def request_stop(signum,frame):callback.stop=True
        signal.signal(signal.SIGTERM,request_stop);signal.signal(signal.SIGINT,request_stop)
        (folder/'process.json').write_text(json.dumps(dict(pid=os.getpid(),started=time.time(),control='Create stop.request for safe stop and final save')))
        try:
            model.learn(total_timesteps=a.steps,callback=callback,reset_num_timesteps=True)
            model.save(folder/'final.zip')
            (folder/'status.json').write_text(json.dumps(dict(steps=model.num_timesteps,initial_steps=config.get('initial_steps',0),cumulative_steps=config.get('initial_steps',0)+model.num_timesteps,completed=model.num_timesteps>=a.steps,stopped=callback.stop or (folder/'stop.request').exists())))
        finally:
            monitored.close();(folder/'process.json').unlink(missing_ok=True)
    else:
        model=PPO.load(a.model,device='cpu');rows=[]
        for i in range(a.episodes):
            obs,reset_info=env.reset(seed=a.seed+i);reward=0.
            for _ in range(a.max_steps):
                action,_=model.predict(obs,deterministic=True);obs,r,t,tr,info=env.step(action);reward+=r
                if t or tr:break
            rows.append(dict(seed=a.seed+i,variant=a.variant,reward=reward,boxes=reset_info['boxes'],noise=reset_info['noise'],timeout=tr,**info))
            if i<3:
                fig=draw(env,folder/f'episode_{i}.png');plt.close(fig)
                np.savez_compressed(folder/f'episode_{i}.npz',path=env.path,goal=env.goal,cells=env.grid.cells,origin=env.grid.origin,resolution=env.grid.resolution)
            print(f'{i+1}/{a.episodes} success={info["success"]} collision={info["collision"]}',flush=True)
        with (folder/'episodes.csv').open('w') as fp:
            w=csv.DictWriter(fp,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        summary={k:float(np.mean([r[k] for r in rows])) for k in ['success','collision','timeout','elapsed','distance','reward']};summary['episodes']=len(rows);(folder/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2));env.close()
if __name__=='__main__':main()
