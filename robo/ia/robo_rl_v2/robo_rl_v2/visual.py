from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Patch
from matplotlib.transforms import Affine2D

def draw(env, output=None):
    grid=env.grid
    fig,ax=plt.subplots(figsize=(9,7))
    transform=Affine2D().rotate(grid.origin[2]).translate(*grid.origin[:2])+ax.transData
    colors=np.where(grid.base_cells==-1,.5,np.where(grid.base_cells==1,0.,1.))
    h,w=grid.cells.shape
    ax.imshow(colors,origin='lower',extent=[0,w*grid.resolution,0,h*grid.resolution],
              cmap='gray',vmin=0,vmax=1,transform=transform,interpolation='nearest')
    added=np.ma.masked_where(grid.cells==grid.base_cells,np.ones_like(grid.cells))
    if np.any(grid.cells!=grid.base_cells):
        from matplotlib.colors import ListedColormap
        ax.imshow(added,origin='lower',extent=[0,w*grid.resolution,0,h*grid.resolution],
                  cmap=ListedColormap(['#bd6f26']),vmin=0,vmax=1,transform=transform,interpolation='nearest')
    bounds=grid.world(np.array([[0,0],[w*grid.resolution,0],[0,h*grid.resolution],[w*grid.resolution,h*grid.resolution]]))
    ax.set_xlim(bounds[:,0].min(),bounds[:,0].max());ax.set_ylim(bounds[:,1].min(),bounds[:,1].max())
    path=np.array(env.path)
    ax.plot(path[:,0],path[:,1],color='royalblue',label='Trajetória física')
    ax.scatter(*path[0,:2],c='purple',label='Início')
    ax.scatter(*env.goal,c='green',marker='*',s=120,label='Destino')
    ax.add_patch(Polygon(env.corners(),color='orange',alpha=.7))
    lidar=env.pose[:2]+__import__('robo_rl_v2.env',fromlist=['rotation']).rotation(env.pose[2])@env.lidar_offset
    ax.scatter(*lidar,c='red',s=12,label='LiDAR')
    ax.set_aspect('equal');ax.set_xlabel('map X (m)');ax.set_ylabel('map Y (m)')
    ax.set_title(f'Nível {env.level} | {Path(grid.source).name} | cinza = desconhecido')
    handles,labels=ax.get_legend_handles_labels()
    if np.any(grid.cells!=grid.base_cells):handles.append(Patch(color='#bd6f26'));labels.append('Caixas sintéticas adicionais')
    ax.legend(handles,labels);fig.tight_layout()
    if output:
        Path(output).parent.mkdir(parents=True,exist_ok=True);fig.savefig(output,dpi=130)
    return fig

def render_array(env):
    fig=draw(env);fig.canvas.draw();arr=np.asarray(fig.canvas.buffer_rgba())[...,:3].copy();plt.close(fig);return arr
