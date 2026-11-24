import sys,os,time,subprocess,json
from pathlib import Path
import numpy as np
CODE=Path(__file__).parent;ROOT=CODE.parent;RUN=Path(sys.argv[1]);env=os.environ.copy();env.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
while not (RUN/'native_end_to_end_equivalence.json').exists():time.sleep(1)
for name,flag in [('planner_estimated','--planner-estimated'),('policy_true_pose','--truth-pose')]:
 folder=RUN/f'eval_full_{name}';folder.mkdir(exist_ok=False);children=[];fps=[]
 for rank in range(4):
  fp=(folder/f'part_{rank}.log').open('x');fps.append(fp)
  child=subprocess.Popen([sys.executable,'-u',str(CODE/'run.py'),'evaluate',flag,'--episodes','25','--envs','1','--seed',str(3300000000+rank*25),'--output',str(folder/f'part_{rank}')],cwd=ROOT,env=env,stdout=fp,stderr=fp);children.append(child)
 codes=[c.wait() for c in children]
 for fp in fps:fp.close()
 assert not any(codes),(folder,codes)
 rows=[]
 for rank in range(4):rows+=json.loads((folder/f'part_{rank}/episodes.json').read_text())
 rows.sort(key=lambda r:r['seed']);(folder/'episodes.json').write_text(json.dumps(rows,indent=2))
 summary={k:float(np.mean([r[k] for r in rows])) for k in ['success','collision','timeout','elapsed','distance','reward','max_est_error']};summary['n']=len(rows)
 (folder/'summary.json').write_text(json.dumps(summary,indent=2));print(name,summary,flush=True)
(RUN/'diagnostics_complete.json').write_text(json.dumps(dict(complete=True)))
