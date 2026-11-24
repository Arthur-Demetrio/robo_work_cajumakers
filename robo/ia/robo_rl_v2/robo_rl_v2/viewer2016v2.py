"""V2 visualization with all visual furniture, independent collision and LiDAR layers."""
import argparse
from pathlib import Path
import numpy as np
from .maps import ROOT
from .arena2016v2 import Arena2016V2Env
from .viewer_session import EvaluationSession
from .viewer2016 import latest_2016_checkpoint
from .viewer import Viewer,limit_resources
class V2Session(EvaluationSession):
    def __init__(self,max_steps=1500,seed=2100000000,scenarios=8):
        super().__init__(max_steps,seed,scenarios);self.env.close();self.env=Arena2016V2Env(max_steps=max_steps,split='viewer_eval')
class V2Viewer(Viewer):
    def __init__(self,root,args):
        self.checkpoint_reader=latest_2016_checkpoint
        super().__init__(root,args,session=V2Session(args.max_steps,args.seed,args.scenarios))
        root.title('Robo-WORK • Arena 2016 v2 • camadas separadas')
    def draw_map(self):
        super().draw_map();canvas=self.canvas;env=self.session.env;canvas.delete('caption')
        # Furniture remains visible even when its top surface does not block the laser.
        for obj in sorted(env.config['objects'],key=lambda o:o['z_range_m'][0]):
            geom=obj['geometry'];color=obj['color']
            if 'polygon' in geom:
                options=dict(fill=color,outline='#343a40',width=1,tags='map')
                if obj['kind']=='elevated_shelf':options['stipple']='gray50'
                canvas.create_polygon(*self.world_to_screen(geom['polygon']).ravel(),**options)
            elif 'polyline' in geom:
                canvas.create_line(*self.world_to_screen(geom['polyline']).ravel(),fill=color,width=max(2,geom['width_m']*self.scale),tags='map')
            else:
                point=self.world_to_screen(geom['center']);r=max(2,geom['radius_m']*self.scale)
                canvas.create_oval(point[0]-r,point[1]-r,point[0]+r,point[1]+r,fill=color,outline='',tags='map')
        canvas.create_text(18,15,anchor='nw',text='Arena 2016 v2 • reconstrução aproximada • alturas estimadas',fill='#183346',tags='caption')
    def update_metrics(self):
        super().update_metrics()
        self.status_var.set(self.status_var.get()+' | Avaliação na v2; o checkpoint foi treinado na arena anterior. Mesas/coberturas visuais não equivalem à ocupação do LiDAR.')
def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,default=ROOT/'runs/treino_2016_transferido_20261110_085342')
    parser.add_argument('--speed',type=float,default=1.);parser.add_argument('--seed',type=int,default=2100000000);parser.add_argument('--scenarios',type=int,default=8);parser.add_argument('--max-steps',type=int,default=1500)
    args=parser.parse_args()
    if not .1<=args.speed<=16 or args.max_steps<1 or args.scenarios<1 or args.seed<0:parser.error('Parâmetros inválidos')
    args.run=args.run.expanduser().resolve();args.cpus=1;args.cpu_budget=.2;args.fps=15;args.poll_seconds=5;limit_resources(1)
    import torch,tkinter as tk
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    try:root=tk.Tk()
    except tk.TclError as exc:raise SystemExit(f'Sem acesso ao display: {exc}. Execute ./visualizar_2016_v2.sh no terminal gráfico.')
    V2Viewer(root,args);root.mainloop()
if __name__=='__main__':main()
