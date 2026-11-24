import csv,json
from pathlib import Path
import numpy as np
from .env import RoboEnv
from .maps import ROOT
from .visual import draw,plt

METRICS=['success','collision','elapsed','path_length','stopped_time','rotations','oscillations','recoveries','distance']

def evaluate(model, folder, episodes=3, max_steps=1500):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    rows=[]; summaries={}
    scenarios=[(f'level_{n}',n,None) for n in range(1,6)]
    scenarios.append(('unseen_backup_map',4,ROOT/'assets/maps/mapa_atual.yaml'))
    for name,level,map_path in scenarios:
        env=RoboEnv(level=level,split='eval',map_path=map_path,max_steps=max_steps)
        group=[]
        for j in range(episodes):
            obs,_=env.reset(seed=2_000_000_000+level*10000+j)
            for _ in range(max_steps):
                action,_=model.predict(obs,deterministic=True)
                obs,_,term,trunc,info=env.step(action)
                if term or trunc: break
            row=dict(scenario=name,seed=2_000_000_000+level*10000+j,**{k:info[k] for k in METRICS})
            rows.append(row);group.append(row)
            if j==0:
                fig=draw(env,folder/f'{name}.png');plt.close(fig)
                np.savez_compressed(folder/f'{name}_episode.npz',path=np.array(env.path),goal=env.goal,cells=env.grid.cells,
                                    resolution=env.grid.resolution,origin=env.grid.origin,base_cells=env.grid.base_cells)
        summaries[name]={k:float(np.mean([r[k] for r in group])) for k in METRICS}
        successes=[r['elapsed'] for r in group if r['success']]
        summaries[name]['completion_time_success_only']=float(np.mean(successes)) if successes else None
    with (folder/'episodes.csv').open('w') as fp:
        writer=csv.DictWriter(fp,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (folder/'summary.json').write_text(json.dumps(summaries,indent=2))
    labels=list(summaries)
    fig,ax=plt.subplots(figsize=(10,4));x=np.arange(len(labels))
    ax.bar(x-.2,[summaries[k]['success'] for k in labels],.4,label='Sucesso')
    ax.bar(x+.2,[summaries[k]['collision'] for k in labels],.4,label='Colisão')
    ax.set_xticks(x,labels,rotation=20);ax.set_ylim(0,1);ax.legend();fig.tight_layout();fig.savefig(folder/'rates.png');plt.close(fig)
    return summaries

if __name__=='__main__':
    import argparse,torch
    from stable_baselines3 import PPO
    p=argparse.ArgumentParser();p.add_argument('--model',required=True);p.add_argument('--output',default='runs/evaluation')
    p.add_argument('--episodes',type=int,default=10);p.add_argument('--max-steps',type=int,default=1500)
    a=p.parse_args();torch.set_num_threads(1)
    print(json.dumps(evaluate(PPO.load(a.model,device='cpu'),a.output,a.episodes,a.max_steps),indent=2))
