import os,sys,time,subprocess,json
from pathlib import Path
CODE=Path(__file__).parent;ROOT=CODE.parent;RUN=Path(sys.argv[1])
while not (RUN/'complete.json').exists():time.sleep(1)
results=[]
for n in [1,2,4]:
 out=RUN/f'native_benchmark_{n}'
 with (RUN/f'native_benchmark_{n}.log').open('x') as fp:
  subprocess.run([sys.executable,'-u',str(CODE/'run.py'),'benchmark','--native','--envs',str(n),'--steps','4096','--output',str(out)],cwd=ROOT,stdout=fp,stderr=fp,check=True)
 r=json.loads((out/'result.json').read_text());r['n_envs']=n;results.append(r);print(n,r,flush=True)
(RUN/'native_benchmarks.json').write_text(json.dumps(results,indent=2))
import torch
from stable_baselines3 import PPO
torch.set_num_threads(1);checks=[]
for n in [1,2,4]:
 a=PPO.load(RUN/f'benchmark_{n}/final.zip',device='cpu');b=PPO.load(RUN/f'native_benchmark_{n}/final.zip',device='cpu')
 x=a.policy.state_dict();y=b.policy.state_dict();equal=all(torch.equal(x[k],y[k]) for k in x)
 checks.append(dict(n_envs=n,identical_policy_weights=equal));assert equal,checks[-1]
(RUN/'native_end_to_end_equivalence.json').write_text(json.dumps(checks,indent=2));print('all equivalent',flush=True)
