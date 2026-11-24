"""Replay an evaluation episode as standalone animated GIF."""
import argparse
from types import SimpleNamespace
import numpy as np
from matplotlib.animation import FuncAnimation,PillowWriter
from matplotlib.patches import Polygon
from .env import FOOTPRINT,rotation
from .maps import Grid
from .visual import draw,plt

def replay(episode,output,stride=4):
    data=np.load(episode);path=data['path'];grid=Grid(data['cells'],float(data['resolution']),data['origin'])
    if 'base_cells' in data:grid.base_cells=data['base_cells']
    env=SimpleNamespace(grid=grid,path=path[:1],goal=data['goal'],pose=path[0],level='replay',lidar_offset=np.array([.33,.005]))
    env.corners=lambda: FOOTPRINT@rotation(env.pose[2]).T+env.pose[:2]
    fig=draw(env);ax=fig.axes[0];line=ax.lines[0];poly=next(p for p in ax.patches if isinstance(p,Polygon))
    frames=list(range(0,len(path),stride))
    if frames[-1]!=len(path)-1:frames.append(len(path)-1)
    def update(i):
        env.pose=path[i];line.set_data(path[:i+1,0],path[:i+1,1]);poly.set_xy(env.corners());ax.set_title(f'Episódio simulado: t={i*.16:.2f} s')
        # Last scatter collection is LiDAR marker.
        ax.collections[-1].set_offsets((env.pose[:2]+rotation(env.pose[2])@env.lidar_offset)[None,:])
        return line,poly
    animation=FuncAnimation(fig,update,frames=frames,interval=160*stride,blit=False)
    animation.save(output,writer=PillowWriter(fps=6.25/stride));plt.close(fig);print(output)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('episode');p.add_argument('--output',default='episode.gif');p.add_argument('--stride',type=int,default=4)
    a=p.parse_args()
    if a.stride<1:p.error('stride deve ser positivo')
    replay(a.episode,a.output,a.stride)
