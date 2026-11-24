"""Read-only graphical replay of pilot checkpoints on the unchanged StageEnv."""
import os
os.environ.update(OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1',MKL_NUM_THREADS='1')
import argparse,json,sys,zipfile
from pathlib import Path
from types import SimpleNamespace
ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT),str(ROOT/'experimentos_v2_20261110')]
from train_gated import StageEnv
from robo_rl_v2.viewer import Viewer
from robo_rl_v2.viewer2016v2 import V2Viewer
from robo_rl_v2.viewer_session import EvaluationSession,Checkpoint
import torch
import tkinter as tk

def latest(run):
    initial=json.loads((run/'config.json').read_text())['initial_steps']
    for p in sorted((run/'checkpoints').glob('*.zip'),reverse=True):
        if not p.stem.isdigit():continue
        try:
            before=p.stat()
            with zipfile.ZipFile(p) as z:
                if z.testzip() is not None:continue
            after=p.stat()
            if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):continue
            return Checkpoint(p.resolve(),initial+int(p.stem),after.st_mtime_ns,after.st_size)
        except (OSError,zipfile.BadZipFile):continue
    raise FileNotFoundError('Aguardando checkpoint completo do piloto')

class Session(EvaluationSession):
    def __init__(self):
        super().__init__(seed=3500000000,scenarios=8)
        self.env.close();self.env=StageEnv(stage=0)

class PilotViewer(V2Viewer):
    def __init__(self,root,args):
        self.checkpoint_reader=latest
        Viewer.__init__(self,root,args,session=Session())
        root.title('Robo-WORK • Piloto PPO • robô em movimento • '+args.run.name)
        root.lift();root.after(500,root.lift)
    def update_metrics(self):Viewer.update_metrics(self)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run',type=Path,default=ROOT/'runs/controle_s4201');a=parser.parse_args()
    os.nice(15)
    cpus=sorted(os.sched_getaffinity(0));os.sched_setaffinity(0,[cpus[-2] if len(cpus)>1 else cpus[0]])
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    args=SimpleNamespace(run=a.run.resolve(),speed=1.,max_steps=1500,seed=3500000000,scenarios=8,cpus=1,cpu_budget=.2,fps=15,poll_seconds=5)
    root=tk.Tk();viewer=PilotViewer(root,args);root.mainloop()
