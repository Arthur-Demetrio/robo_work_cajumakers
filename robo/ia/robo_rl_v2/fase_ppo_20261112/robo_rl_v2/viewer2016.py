"""Read-only checkpoint viewer for the independent 2016 training pipeline."""
import argparse,json,re,zipfile
from pathlib import Path
from .maps import ROOT
from .viewer_session import Checkpoint,EvaluationSession
from .arena2016 import Arena2016Env
from .viewer import Viewer,limit_resources

def latest_2016_checkpoint(run):
    run=Path(run).resolve()
    candidates=[]
    for path in (run/'checkpoints').glob('ppo_*.zip'):
        match=re.fullmatch(r'ppo_(\d{12})\.zip',path.name)
        if match and path.resolve().is_relative_to(run/'checkpoints'):
            candidates.append((int(match[1]),path))
    for steps,path in sorted(candidates,reverse=True):
        try:
            before=path.stat()
            with zipfile.ZipFile(path) as archive:
                if archive.testzip() is not None:continue
            after=path.stat()
            if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):continue
            return Checkpoint(path.resolve(),steps,after.st_mtime_ns,after.st_size)
        except (OSError,zipfile.BadZipFile,EOFError):continue
    raise FileNotFoundError('Aguardando primeiro checkpoint íntegro da arena 2016')

class ArenaEvaluationSession(EvaluationSession):
    def __init__(self,run,max_steps=1500,seed=2100000000,scenarios=8):
        super().__init__(max_steps,seed,scenarios)
        self.env.close();self.config=json.loads((Path(run)/'config.json').read_text())
        cfg=self.config
        self.env=Arena2016Env(variant=cfg['variant'],noise=None if cfg.get('noise','config')=='config' else cfg['noise']=='on',boxes=cfg.get('boxes'),progression=cfg.get('progression',False),max_steps=max_steps,split='viewer_eval',layout=Path(run)/'layout.yaml')
        if cfg.get('progression'):self.env.set_curriculum([int(cfg['steps']*f) for f in [.15,.35,.55,.8]])
    def restart(self):
        if self.checkpoint is not None:self.env.transitions=self.checkpoint.steps
        super().restart()

class ArenaViewer(Viewer):
    def __init__(self,root,args):
        self.checkpoint_reader=latest_2016_checkpoint
        super().__init__(root,args,session=ArenaEvaluationSession(args.run,args.max_steps,args.seed,args.scenarios))
        root.title('Robo-WORK • Arena 2016 • avaliação do treino transferido')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run',type=Path,default=ROOT/'runs/treino_2016_transferido_20261110_085342')
    p.add_argument('--speed',type=float,default=1.);p.add_argument('--seed',type=int,default=2100000000);p.add_argument('--scenarios',type=int,default=8);p.add_argument('--max-steps',type=int,default=1500)
    a=p.parse_args()
    if not .1<=a.speed<=16 or a.scenarios<1 or a.max_steps<1 or a.seed<0:p.error('Parâmetros inválidos')
    a.run=a.run.expanduser().resolve();a.cpus=1;a.cpu_budget=.2;a.fps=15;a.poll_seconds=5
    limit_resources(a.cpus)
    import torch,tkinter as tk
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    try:root=tk.Tk()
    except tk.TclError as exc:raise SystemExit(f'Sem acesso à sessão gráfica: {exc}. Execute ./visualizar_2016.sh no terminal gráfico.')
    ArenaViewer(root,a);root.mainloop()
if __name__=='__main__':main()
