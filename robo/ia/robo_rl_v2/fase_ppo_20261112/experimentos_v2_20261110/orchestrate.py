import os,sys,subprocess,time,json,hashlib
from pathlib import Path
from datetime import datetime
ROOT=Path(__file__).resolve().parents[1];CODE=Path(__file__).parent;PYTHON=ROOT/'.venv/bin/python'
def main():
 out=ROOT/'runs'/('desenvolvimento_v2_'+datetime.now().strftime('%Y%m%d_%H%M%S'));out.mkdir(exist_ok=False)
 print(str(out),flush=True)
 env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
 protected=list((ROOT/'assets/arenas/2016').rglob('*.yaml'))+list((ROOT/'assets/arenas/2016').rglob('*.pgm'))+list((ROOT/'assets/arenas/arena_2016_v2').rglob('*.yaml'))+list((ROOT/'runs/principal/checkpoints').glob('*.zip'))+[ROOT/'runs/treino_2016_transferido_20261110_085342/final.zip']
 before={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in protected};(out/'preservation_before.json').write_text(json.dumps(before,indent=2))
 def run(args,log):
  with (out/log).open('x') as fp:
   child=subprocess.Popen([str(PYTHON),'-u',*map(str,args)],cwd=ROOT,env=env,stdout=fp,stderr=fp)
   status=child.wait()
  if status:raise RuntimeError(f'Processo falhou: {log}, exit {status}')
 run([CODE/'validate.py',out/'validation'],'validation.log');print('equivalence passed',flush=True)
 benchmarks=[]
 for n in [1,2,4]:
  run([CODE/'run.py','benchmark','--envs',n,'--steps',4096,'--output',out/f'benchmark_{n}'],f'benchmark_{n}.log')
  result=json.loads((out/f'benchmark_{n}/result.json').read_text());result['n_envs']=n;benchmarks.append(result);print('benchmark',n,result,flush=True)
 (out/'benchmarks.json').write_text(json.dumps(benchmarks,indent=2));chosen=max(benchmarks,key=lambda x:x['transitions_s'])['n_envs']
 # Prefer fewer processes when performance is within 5% of the best.
 best=max(r['transitions_s'] for r in benchmarks);chosen=min(r['n_envs'] for r in benchmarks if r['transitions_s']>=.95*best)
 (out/'selection.json').write_text(json.dumps(dict(selected_envs=chosen,criterion='minimum processes within 5% of best measured throughput'),indent=2));print('selected',chosen,flush=True)
 for name,guidance in [('baseline',False),('waypoints',True)]:
  args=[CODE/'run.py','pilot','--envs',chosen,'--steps',32768,'--output',out/f'pilot_{name}']
  if guidance:args+=['--guidance']
  run(args,f'pilot_{name}.log');print('pilot done',name,flush=True)
 # A frozen reference and two equal-budget candidates. Every seed and scene is shared.
 source=ROOT/'runs/treino_2016_transferido_20261110_085342/final.zip'
 subjects=[('reference',source,False,False),('baseline',out/'pilot_baseline/final.zip',False,False),('waypoints',out/'pilot_waypoints/final.zip',True,False),('planner_oracle',None,False,True)]
 for suite,maximum,base in [('full',12.,3300000000),('gate',2.,3301000000)]:
  for name,model,guidance,oracle in subjects:
   if oracle and suite=='gate':continue
   folder=out/f'eval_{suite}_{name}';folder.mkdir()
   children=[];streams=[]
   for rank in range(4):
    args=[str(PYTHON),'-u',str(CODE/'run.py'),'evaluate','--envs','1','--episodes','25','--seed',str(base+rank*25),'--maximum',str(maximum),'--output',str(folder/f'part_{rank}')]
    if guidance:args+=['--guidance']
    if oracle:args+=['--oracle']
    else:args+=['--model',str(model)]
    fp=(folder/f'part_{rank}.log').open('x');streams.append(fp);children.append(subprocess.Popen(args,cwd=ROOT,env=env,stdout=fp,stderr=fp))
   codes=[child.wait() for child in children]
   for fp in streams:fp.close()
   if any(codes):raise RuntimeError(f'Evaluation failed: {folder} {codes}')
   rows=[]
   for rank in range(4):rows+=json.loads((folder/f'part_{rank}/episodes.json').read_text())
   rows.sort(key=lambda r:r['seed']);(folder/'episodes.json').write_text(json.dumps(rows,indent=2))
   import numpy as np
   summary={k:float(np.mean([r[k] for r in rows])) for k in ['success','collision','timeout','elapsed','distance','reward','max_est_error']};summary['n']=len(rows);summary['gate_passed']=summary['success']>=.9 and summary['collision']==0
   (folder/'summary.json').write_text(json.dumps(summary,indent=2));print('evaluation',suite,name,summary,flush=True)
 after={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in before};assert before==after
 (out/'preservation_result.json').write_text(json.dumps(dict(unchanged=True,files=len(before)),indent=2));(out/'complete.json').write_text(json.dumps(dict(completed=True,run=str(out)),indent=2))
 print('COMPLETE',out,flush=True)
if __name__=='__main__':main()
