import csv,json
from pathlib import Path
import numpy as np
O=Path(__file__).parent;R=O.parent/'runs/treino_continuo_20261111';e=list(csv.DictReader((O/'evaluations_snapshot.csv').open()));end=int(e[-1]['steps']);rng=np.random.default_rng(42);out={}
def boot(delta):
 d=np.array(delta);z=d[rng.integers(len(d),size=(10000,len(d)))].mean(1);return [float(np.quantile(z,.025)),float(np.quantile(z,.975))]
for span in [500000,1000000]:
 batches=[]
 for lo,hi in [(end-2*span,end-span),(end-span,end)]:
  zz=[json.loads((R/'evaluations'/f'{int(r["steps"]):012d}.json').read_text()) for r in e if lo<int(r['steps'])<=hi];seeds=sorted(v['seed'] for v in zz[0]['episodes']);batches.append({k:np.array([np.mean([next(v[k] for v in z['episodes'] if v['seed']==seed) for z in zz]) for seed in seeds]) for k in ['success','collision']})
 out[str(span)]={k:dict(delta=float(np.mean(batches[1][k]-batches[0][k])),scenario_bootstrap_95ci=boot(batches[1][k]-batches[0][k])) for k in ['success','collision']}
if (O/'paired_summary.csv').exists():
 r=list(csv.DictReader((O/'paired_episodes.csv').open()));steps=sorted(set(int(v['checkpoint']) for v in r));out['paired_changes']={}
 for mode in ['stage0_fixed','stage0_holdout','arena_full']:
  for start in steps[:-1]:
   a=sorted([v for v in r if int(v['checkpoint'])==start and v['mode']==mode],key=lambda v:int(v['seed']));b=sorted([v for v in r if int(v['checkpoint'])==steps[-1] and v['mode']==mode],key=lambda v:int(v['seed']));result={}
   for k in ['success','collision','reward']:
    def val(v):return float({'True':1,'False':0}.get(v[k],v[k]))
    d=[val(y)-val(x) for x,y in zip(a,b)];result[k]=dict(delta=float(np.mean(d)),scenario_bootstrap_95ci=boot(d))
   out['paired_changes'][f'{mode}_{start}_to_{steps[-1]}']=result
(O/'statistics.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
