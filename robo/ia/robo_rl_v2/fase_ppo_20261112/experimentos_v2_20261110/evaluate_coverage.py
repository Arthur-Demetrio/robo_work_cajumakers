import sys,os,subprocess,json
from pathlib import Path
import numpy as np
CODE=Path(__file__).parent;ROOT=CODE.parent;RUN=Path(sys.argv[1]);env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
for name,model in [('reference',ROOT/'runs/treino_2016_transferido_20261110_085342/final.zip'),('baseline',RUN/'pilot_baseline/final.zip')]:
 folder=RUN/f'eval_coverage_{name}';folder.mkdir(exist_ok=False);children=[];logs=[]
 for rank in range(4):
  fp=(folder/f'part_{rank}.log').open('x');logs.append(fp)
  children.append(subprocess.Popen([sys.executable,'-u',str(CODE/'run.py'),'evaluate','--full-sampler','--model',str(model),'--episodes','25','--envs','1','--seed',str(3401000000+rank*25),'--output',str(folder/f'part_{rank}')],cwd=ROOT,env=env,stdout=fp,stderr=fp))
 codes=[c.wait() for c in children]
 for f in logs:f.close()
 assert not any(codes),codes
 rows=[]
 for rank in range(4):rows+=json.loads((folder/f'part_{rank}/episodes.json').read_text())
 rows.sort(key=lambda r:r['seed']);(folder/'episodes.json').write_text(json.dumps(rows,indent=2));summary={k:float(np.mean([r[k] for r in rows])) for k in ['success','collision','timeout','elapsed','distance','reward','max_est_error']};summary.update(n=100,sampler='full_footprint_SE2')
 (folder/'summary.json').write_text(json.dumps(summary,indent=2));print(name,summary,flush=True)
