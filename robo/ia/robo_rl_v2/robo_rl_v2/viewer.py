"""Desktop checkpoint evaluation viewer. Training is accessed only through read operations."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import time


def limit_resources(cpus):
    available = sorted(os.sched_getaffinity(0))
    # Training launcher chooses first CPUs; prefer the other end of the available set.
    chosen = available[-min(cpus, len(available)):]
    os.sched_setaffinity(0, chosen)
    try: os.nice(15)
    except PermissionError: pass
    return chosen


class Viewer:
    def __init__(self, root, args, session=None):
        import tkinter as tk
        from tkinter import ttk
        from .viewer_session import EvaluationSession
        self.tk, self.ttk, self.root, self.args = tk, ttk, root, args
        self.session = session if session is not None else EvaluationSession(args.max_steps, args.seed, args.scenarios)
        self.loader = ThreadPoolExecutor(max_workers=1, thread_name_prefix='checkpoint-read-only')
        self.future = None
        self.loading_checkpoint = None
        self.pending = None
        self.status = 'Aguardando primeiro checkpoint completo…'
        self.paused = False
        self.closed = False
        self.next_poll = 0.
        self.end_until = None
        self.map_dirty = True
        self.last_size = None
        self.scale = 1.
        root.title('Robo-WORK • AVALIAÇÃO de checkpoints • treino independente')
        root.geometry('1240x860'); root.minsize(950, 650)
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.columnconfigure(0, weight=1); root.rowconfigure(2, weight=1)
        tk.Label(root, text='AVALIAÇÃO DO CHECKPOINT — NÃO É UM EPISÓDIO DO TREINAMENTO',
                 bg='#153d61', fg='white', font=('Sans', 13, 'bold'), pady=9).grid(row=0, column=0, columnspan=2, sticky='ew')
        controls = ttk.Frame(root, padding=8); controls.grid(row=1, column=0, columnspan=2, sticky='ew')
        self.pause_button = ttk.Button(controls, text='Pausar [Espaço]', command=self.toggle_pause)
        self.pause_button.pack(side='left', padx=4)
        ttk.Button(controls, text='Reiniciar [R]', command=self.restart).pack(side='left', padx=4)
        ttk.Button(controls, text='Próximo cenário [N]', command=self.next_episode).pack(side='left', padx=4)
        ttk.Button(controls, text='Novo checkpoint [C]', command=self.apply_pending).pack(side='left', padx=4)
        ttk.Label(controls, text='Velocidade desejada:').pack(side='left', padx=(16,4))
        self.speed = tk.StringVar(value=str(args.speed))
        speed = ttk.Combobox(controls, textvariable=self.speed, values=['0.25','0.5','1','2','4','8','16'], state='readonly', width=5)
        speed.pack(side='left'); ttk.Label(controls, text='× (limitada pelo orçamento de CPU)').pack(side='left', padx=4)
        self.canvas = tk.Canvas(root, bg='#d6dfe7', highlightthickness=0)
        self.canvas.grid(row=2, column=0, sticky='nsew')
        self.canvas.bind('<Configure>', lambda event: self.invalidate_map())
        panel = ttk.Frame(root); panel.grid(row=2, column=1, sticky='ns')
        sidebar = tk.Canvas(panel, width=350, highlightthickness=0)
        sidebar.pack(side='left', fill='both', expand=True)
        scrollbar = ttk.Scrollbar(panel, orient='vertical', command=sidebar.yview)
        scrollbar.pack(side='right', fill='y'); sidebar.configure(yscrollcommand=scrollbar.set)
        side = ttk.Frame(sidebar, padding=12)
        sidebar.create_window((0,0), window=side, anchor='nw')
        side.bind('<Configure>', lambda event: sidebar.configure(scrollregion=sidebar.bbox('all')))
        self.metrics = tk.StringVar()
        ttk.Label(side, textvariable=self.metrics, justify='left', font=('Monospace',10)).pack(anchor='nw')
        ttk.Separator(side, orient='horizontal').pack(fill='x', pady=12)
        self.decisions = tk.StringVar()
        ttk.Label(side, textvariable=self.decisions, justify='left', font=('Monospace',10)).pack(anchor='nw')
        ttk.Label(side, text='Legenda\nPreto: ocupado no mapa\nCinza: desconhecido (bloqueado)\nMarrom: caixas sintéticas\nLaranja: footprint completo\nVerde: raios válidos do LiDAR\nAzul: trajetória física\nCiano: translação comandada\nMagenta: rotação comandada', justify='left').pack(anchor='nw', pady=14)
        self.history = tk.StringVar()
        ttk.Label(side, textvariable=self.history, justify='left', font=('Monospace',9)).pack(anchor='nw')
        ttk.Label(side, text='Somente inferência em CPU.\nPausar/reiniciar/fechar afeta apenas\na avaliação desta janela.\n\nO cenário acompanha a configuração\ndo visualizador selecionado.', justify='left', foreground='#536575').pack(anchor='nw', pady=14)
        self.status_var = tk.StringVar()
        ttk.Label(root, textvariable=self.status_var, padding=8, wraplength=1160).grid(row=3, column=0, columnspan=2, sticky='ew')
        root.bind('<space>', lambda event: self.toggle_pause())
        root.bind('r', lambda event: self.restart()); root.bind('n', lambda event: self.next_episode())
        root.bind('c', lambda event: self.apply_pending())
        self.last_paint = 0.
        self.frames = 0
        self.tick()

    def invalidate_map(self): self.map_dirty = True

    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_button.configure(text='Continuar [Espaço]' if self.paused else 'Pausar [Espaço]')

    def restart(self):
        self.session.restart(); self.end_until = None; self.map_dirty = True

    def next_episode(self):
        self.session.next_episode(); self.end_until = None; self.map_dirty = True

    def apply_pending(self):
        if self.pending is None: return
        checkpoint,model=self.pending
        self.session.activate(checkpoint,model);self.pending=None
        self.status='Checkpoint completo aplicado; nova avaliação com pesos congelados.'
        self.end_until=None;self.map_dirty=True

    def poll_checkpoint(self, now):
        from .viewer_session import latest_checkpoint, load_policy
        if self.future is not None and self.future.done():
            try:
                model = self.future.result()
                self.pending = self.loading_checkpoint,model
                self.status = f'Novo checkpoint ({self.loading_checkpoint.steps:,}) pronto; será aplicado ao terminar o episódio. [C] aplica agora e descarta o parcial.'
            except Exception as exc:
                self.status = f'Leitura adiada; mantendo avaliação anterior: {exc}'
            self.future = None
        if self.pending is not None and (self.session.model is None or
                (self.session.done and self.end_until is not None and now>=self.end_until)):
            self.apply_pending()
        if now < self.next_poll or self.future is not None: return
        self.next_poll = now + self.args.poll_seconds
        try:
            checkpoint = getattr(self, 'checkpoint_reader', latest_checkpoint)(self.args.run)
            if checkpoint != self.session.checkpoint and (self.pending is None or checkpoint != self.pending[0]):
                self.loading_checkpoint = checkpoint
                self.status = f'Carregando cópia de leitura: {checkpoint.steps:,} passos…'
                self.future = self.loader.submit(load_policy, checkpoint)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.status = f'Aguardando manifest/checkpoint completo: {exc}'

    def world_to_screen(self, points):
        import numpy as np
        p = np.asarray(points)
        result = p.copy().astype(float)
        result[...,0] = self.offset[0] + (p[...,0]-self.bounds[0])*self.scale
        result[...,1] = self.offset[1] + (self.bounds[3]-p[...,1])*self.scale
        return result

    def draw_map(self):
        import numpy as np
        from PIL import Image, ImageTk
        env = self.session.env; grid = env.grid
        h,w = grid.cells.shape; r = grid.resolution
        corners = grid.world([[0,0],[w*r,0],[0,h*r],[w*r,h*r]])
        xmin,ymin = corners.min(axis=0); xmax,ymax = corners.max(axis=0)
        self.bounds = xmin,ymin,xmax,ymax
        width,height = max(100,self.canvas.winfo_width()),max(100,self.canvas.winfo_height())
        self.scale = min((width-36)/(xmax-xmin),(height-36)/(ymax-ymin))
        pw,ph = max(1,round((xmax-xmin)*self.scale)),max(1,round((ymax-ymin)*self.scale))
        self.offset = ((width-pw)/2,(height-ph)/2)
        rgb = np.full((*grid.cells.shape,3),255,np.uint8)
        rgb[grid.base_cells==-1] = [128,128,128]; rgb[grid.base_cells==1] = [20,25,29]
        rgb[grid.cells!=grid.base_cells] = [189,111,38]
        source = Image.fromarray(rgb[::-1])
        c,s = np.cos(grid.origin[2]),np.sin(grid.origin[2]); ox,oy = grid.origin[:2]
        inv = 1/(self.scale*r)
        affine = (c*inv,-s*inv,(c*(xmin-ox)+s*(ymax-oy))/r,
                  s*inv,c*inv,h+(s*(xmin-ox)-c*(ymax-oy))/r)
        image = source.transform((pw,ph), Image.Transform.AFFINE, affine,
                                 resample=Image.Resampling.NEAREST, fillcolor=(128,128,128))
        self.map_image = ImageTk.PhotoImage(image)
        self.canvas.delete('all')
        self.canvas.create_image(*self.offset, image=self.map_image, anchor='nw', tags='map')
        self.canvas.create_text(18,15,anchor='nw',text=(f'{env.variant} • reconstrução aproximada • coordenadas em metros' if hasattr(env,'variant') else 'Arena real importada + caixas sintéticas • coordenadas físicas em metros'),fill='#183346',tags='caption')
        self.map_dirty = False

    def draw_episode(self):
        import numpy as np
        from .env import rotation
        env = self.session.env
        if self.map_dirty: self.draw_map()
        canvas = self.canvas; canvas.delete('dynamic')
        xy = self.world_to_screen
        if len(env.path)>1:
            path = xy(np.array(env.path)[:,:2]); canvas.create_line(*path.ravel(),fill='#246ec4',width=2,tags='dynamic')
        start=xy(env.path[0][:2]); canvas.create_oval(*(start-4),*(start+4),fill='#7553a4',outline='',tags='dynamic')
        goal=xy(env.goal); radius=.2*self.scale
        canvas.create_oval(goal[0]-radius,goal[1]-radius,goal[0]+radius,goal[1]+radius,outline='#04876a',width=2,tags='dynamic')
        canvas.create_text(*goal,text='★',font=('Sans',20),fill='#04876a',tags='dynamic')
        origin = env.pose[:2]+rotation(env.pose[2])@env.lidar_offset
        origin_px=xy(origin)
        distances,mask=env.scan
        angles=env.angles+env.pose[2]
        ends=origin+np.column_stack([np.cos(angles),np.sin(angles)])*distances[:,None]*env.max_range
        for end in xy(ends[mask.astype(bool)]):
            canvas.create_line(*origin_px,*end,fill='#75ac87',width=1,tags='dynamic')
        canvas.create_polygon(*xy(env.corners()).ravel(),fill='#f6a341',outline='#965218',width=2,tags='dynamic')
        canvas.create_oval(*(origin_px-3),*(origin_px+3),fill='red',outline='',tags='dynamic')
        base=xy(env.pose[:2]); front=xy(env.pose[:2]+rotation(env.pose[2])@np.array([.4,0]))
        canvas.create_line(*base,*front,fill='#59320c',width=2,arrow='last',tags='dynamic')
        decision = self.session.decision_velocity()
        end=xy(env.pose[:2]+rotation(env.pose[2])@decision[:2]*4)
        if np.linalg.norm(decision[:2])>.002:
            canvas.create_line(*base,*end,fill='#00a8ce',width=4,arrow='last',tags='dynamic')
        if abs(decision[2])>.005:
            radius=max(18,.55*self.scale)
            canvas.create_arc(base[0]-radius,base[1]-radius,base[0]+radius,base[1]+radius,
                              start=15,extent=220 if decision[2]>0 else -220,style='arc',outline='#bd36bd',width=3,tags='dynamic')
            canvas.create_text(base[0],base[1]-radius-10,text='↺' if decision[2]>0 else '↻',fill='#bd36bd',font=('Sans',17),tags='dynamic')
        if self.session.done:
            result='CHEGOU AO OBJETIVO' if self.session.info['success'] else 'COLISÃO' if self.session.info['collision'] else 'LIMITE DE TEMPO'
            canvas.create_text(canvas.winfo_width()/2,52,text='AVALIAÇÃO: '+result,fill='#9a2424',font=('Sans',16,'bold'),tags='dynamic')

    def update_metrics(self):
        s=self.session
        if s.model is None:
            self.metrics.set('Sem checkpoint carregado.\nO treinamento segue independente.');self.status_var.set(self.status);return
        summary=s.summary();rate='—' if summary['success_rate'] is None else f"{summary['success_rate']:.1%}"
        arena_label=f'{s.env.variant} (nível {s.env.level})' if hasattr(s.env,'variant') else 'arena real (nível 4)'
        self.metrics.set(f'CHECKPOINT: {s.checkpoint.steps:,} passos\n\nAVALIAÇÃO — {arena_label}\nCenário: {s.episode_index%s.scenarios+1}/{s.scenarios}\nSemente: {s.episode_seed}\nPassos do episódio: {s.env.steps}\nTempo simulado: {s.env.steps*s.env.dt:.2f} s\nRecompensa do passo: {s.reward:+.4f}\nRecompensa acumulada: {s.total_reward:+.3f}\n\nCenários concluídos: {summary["episodes"]}/{s.scenarios}\nColisões: {summary["collisions"]}\nTaxa de sucesso: {rate}\n(este checkpoint; sementes únicas)\n\nReprodução: {"PAUSADA" if self.paused else "EM EXECUÇÃO"}')
        v=s.decision_velocity();a=s.action
        self.decisions.set(f'ÚLTIMA DECISÃO DA POLÍTICA\nNormalizada (vx, vy, wz):\n({a[0]:+.3f}, {a[1]:+.3f}, {a[2]:+.3f})\n\nComando solicitado, frame base_link:\nvx {v[0]:+.3f} m/s\nvy {v[1]:+.3f} m/s\nwz {v[2]:+.3f} rad/s\nLiDAR: {int(s.env.scan[1].sum())}/{s.env.beams} raios válidos\nPose real só para visualizar:\nx {s.env.pose[0]:+.2f}, y {s.env.pose[1]:+.2f} m')
        history=['VERSÕES ANTERIORES (avaliação)']
        for row in s.history:
            success='—' if row['success_rate'] is None else f"{row['success_rate']:.0%}"
            history.append(f"{row['steps']:>9,}: sucesso {success}, N={row['episodes']}")
        self.history.set('\n'.join(history))
        self.status_var.set(f'{self.status} | {self.args.run} | 1 thread, nice 15, CPUs {self.args.cpus}; orçamento de reprodução {self.args.cpu_budget:.0%} de um CPU.')

    def tick(self):
        if self.closed: return
        started=time.monotonic()
        try:
            self.poll_checkpoint(started)
            if self.session.model is not None:
                if not self.paused:
                    if self.session.done:
                        if self.end_until is None: self.end_until=started+1.5
                        elif started>=self.end_until:self.next_episode()
                    else:self.session.step()
                if started-self.last_paint>=1/self.args.fps or self.map_dirty:
                    self.draw_episode();self.last_paint=started
                self.update_metrics();self.frames+=1
            else:self.update_metrics()
        except Exception as exc:
            self.status=f'Avaliação pausada por erro: {exc}';self.paused=True
            self.pause_button.configure(text='Continuar [Espaço]');self.update_metrics()
        elapsed=time.monotonic()-started
        # Bound evaluation work duty; user acceleration never disables CPU pacing.
        interval=self.session.env.dt/float(self.speed.get())
        rest=max(.01, interval-elapsed, elapsed*(1-self.args.cpu_budget)/self.args.cpu_budget)
        if self.paused or self.session.model is None:rest=max(rest,.15)
        self.root.after(max(1,int(rest*1000)),self.tick)

    def close(self):
        self.closed=True
        self.loader.shutdown(wait=False,cancel_futures=True)
        self.root.destroy()


def main():
    p=argparse.ArgumentParser(description='Janela 2D de AVALIAÇÃO; não controla o treino.')
    p.add_argument('--run',default='runs/principal',type=Path)
    p.add_argument('--cpus',type=int,default=1)
    p.add_argument('--cpu-budget',type=float,default=.2,help='Fração de um CPU para os ticks de reprodução')
    p.add_argument('--speed',type=float,default=1)
    p.add_argument('--fps',type=int,default=15)
    p.add_argument('--poll-seconds',type=float,default=5)
    p.add_argument('--max-steps',type=int,default=1500)
    p.add_argument('--seed',type=int,default=2_100_000_000)
    p.add_argument('--scenarios',type=int,default=8)
    p.add_argument('--smoke-seconds',type=float,default=0,help=argparse.SUPPRESS)
    a=p.parse_args()
    if a.cpus<1 or not 0<a.cpu_budget<=1 or not .1<=a.speed<=16 or not 1<=a.fps<=30 or a.poll_seconds<1 or a.max_steps<1 or a.scenarios<1 or a.seed<0:
        p.error('Limites inválidos: CPUs>=1, budget em (0,1], speed .1..16, fps 1..30, poll>=1, passos/cenários positivos')
    a.run=a.run.expanduser().resolve()
    limit_resources(a.cpus)
    import torch
    torch.set_num_threads(1);torch.set_num_interop_threads(1)
    import tkinter as tk
    try:root=tk.Tk()
    except tk.TclError as exc:
        raise SystemExit(f'Não foi possível abrir a janela: {exc}\nExecute ./visualizar_treino.sh em um terminal da sessão gráfica Linux.')
    viewer=Viewer(root,a)
    if a.smoke_seconds:
        def finish_smoke():
            if viewer.session.model is None or viewer.frames==0:
                print('GUI_SMOKE_FAILED:',viewer.status,flush=True);viewer.close();raise SystemExit(1)
            print(f'GUI_SMOKE_OK checkpoint={viewer.session.checkpoint.steps} frames={viewer.frames} episode_steps={viewer.session.env.steps}',flush=True)
            viewer.close()
        root.after(int(a.smoke_seconds*1000),finish_smoke)
    root.mainloop()

if __name__=='__main__':main()
