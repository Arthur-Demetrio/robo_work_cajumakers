"""Retoma o currículo nativo em uma nova pasta, independente do terminal."""
import argparse,os,signal,subprocess,time,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent
p=argparse.ArgumentParser()
p.add_argument('--resume-from',type=Path,default=ROOT/'runs/treino_v2_nativo_gated_20261110_102312/checkpoints/ppo_000000125000.zip')
p.add_argument('--output',type=Path,default=ROOT/'runs'/time.strftime('treino_continuo_%Y%m%d_%H%M%S'))
a=p.parse_args();out=a.output.resolve();out.parent.mkdir(parents=True,exist_ok=True)
if out.exists():raise SystemExit('Use uma pasta nova para preservar os modelos existentes')
log=out.with_suffix('.terminal.log')
command=[str(ROOT/'.venv/bin/python'),'-u',str(ROOT/'experimentos_v2_20261110/train_gated.py'),'--output',str(out),'--resume-from',str(a.resume_from.resolve()),'--steps','0','--checkpoint-every','25000']
def limits():
 signal.signal(signal.SIGHUP,signal.SIG_IGN)
 os.sched_setaffinity(0,sorted(os.sched_getaffinity(0))[:2]);os.nice(10)
env=os.environ.copy();env.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1',MPLBACKEND='Agg')
with log.open('xb') as fp:
 child=subprocess.Popen(command,cwd=ROOT,stdin=subprocess.DEVNULL,stdout=fp,stderr=fp,start_new_session=True,preexec_fn=limits,env=env)
print(json.dumps(dict(pid=child.pid,output=str(out),log=str(log),command=command),indent=2))
