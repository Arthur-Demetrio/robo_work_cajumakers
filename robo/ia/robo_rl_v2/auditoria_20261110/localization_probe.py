import sys,json,os
from pathlib import Path
import numpy as np,torch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from stable_baselines3 import PPO
from robo_rl_v2.env import RoboEnv
p=Path(__file__).parent;torch.set_num_threads(1)
m=PPO.load(p.parent/'runs/principal/checkpoints/ppo_000002000384.zip',device='cpu')
rows=[]
for seed in list(range(2300040000,2300040024))+list(range(2100000000,2100000008)):
 e=RoboEnv(level=4,split='eval');ob,_=e.reset(seed=seed);total=0
 for _ in range(1500):
  e.estimate=e.pose.copy();ob=e.observation()
  a,_=m.predict(ob,deterministic=True);ob,r,t,tr,info=e.step(a);total+=r
  if t or tr:break
 rows.append(dict(seed=seed,reward=total,**info));print(seed,info['success'],info['collision'],flush=True)
(p/'localization_probe.json').write_text(json.dumps(rows,indent=2))
