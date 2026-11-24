"""Independent builder/environment for the approximate RoboCup@Work 2016 arena."""
import argparse,json
from pathlib import Path
import numpy as np,yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt
from .maps import Grid,ROOT
from .env import RoboEnv,RADIUS
LAYOUT=ROOT/'assets/arenas/2016/layout.yaml'
def read_layout(path=LAYOUT):return yaml.safe_load(Path(path).read_text())
def shape_mask(shape,xx,yy,res):
    if 'rect' in shape:
        x,y,w,h=shape['rect'];return (abs(xx-x)<=w/2+res/2-1e-10)&(abs(yy-y)<=h/2+res/2-1e-10)
    x,y,r=shape['circle'];return np.hypot(xx-x,yy-y)<=r+res/np.sqrt(2)
def rasterize(config):
    res=config['resolution_m'];w,h=config['size_m'];yy,xx=np.mgrid[0:round(h/res),0:round(w/res)];xx=(xx+.5)*res;yy=(yy+.5)*res
    b=config['boundary_m'];masks={'boundary':(xx<b)|(xx>w-b)|(yy<b)|(yy>h-b)}
    for f in config['fixtures']:
        mask=np.zeros_like(xx,dtype=bool)
        for s in f['shapes']:mask|=shape_mask(s,xx,yy,res)
        masks[f['id']]=mask
    return Grid(np.logical_or.reduce(list(masks.values())).astype(np.int8),res,source='arena_2016'),masks
def validate(config,grid,masks):
    res=grid.resolution;ids=list(masks);gaps=[]
    for i,a in enumerate(ids):
        d=distance_transform_edt(~masks[a])*res
        for b in ids[i+1:]:gaps.append(dict(a=a,b=b,lower_bound_m=max(0.,float(d[masks[b]].min()-res*np.sqrt(2)))))
    bad=[x for x in gaps if x['lower_bound_m']+1e-9<config['minimum_passage_m']]
    internal={}
    yy,xx=np.mgrid[:grid.cells.shape[0],:grid.cells.shape[1]];xx=(xx+.5)*res;yy=(yy+.5)*res
    for area in config.get('passage_areas',[]):
        x,y,w,h=area['rect'];internal[area['id']]=[w,h]
        mask=shape_mask(area,xx,yy,res)
        if min(w,h)<config['minimum_passage_m'] or np.any(grid.cells[mask]):
            raise ValueError('Passagem interna inválida: '+area['id'])
    report=dict(minimum_passage_m=config['minimum_passage_m'],min_pair_gap_m=min(x['lower_bound_m'] for x in gaps),violations=bad,internal_clearances_m=internal,conservative_disk_components=grid.n_components,safe_cells=int(np.sum(grid.components>0)),pairs=gaps)
    if bad:raise ValueError('Passagens inferiores ao mínimo: '+json.dumps(bad))
    return report
def build(output=None):
    config=read_layout();grid,masks=rasterize(config);report=validate(config,grid,masks)
    output=Path(output or LAYOUT.parent/'generated');output.mkdir(parents=True,exist_ok=True)
    from .visual import plt
    from matplotlib.patches import Rectangle,Circle
    for variant in config['variants']:
        image=output/f'{variant}.pgm';Image.fromarray(np.where(grid.cells[::-1],0,254).astype(np.uint8)).save(image)
        meta=dict(image=image.name,mode='trinary',resolution=grid.resolution,origin=[0.,0.,0.],negate=0,occupied_thresh=.65,free_thresh=.196)
        (output/f'{variant}.yaml').write_text(yaml.safe_dump(meta,sort_keys=False))
        fig,ax=plt.subplots(figsize=(12,10));ax.imshow(np.where(grid.cells,0.,1.),origin='lower',extent=[0,12,0,10],cmap='gray',vmin=0,vmax=1)
        colors=dict(wall='#596777',workstation='#9aa8b5',canopy='#9aa8b5',red_table='#a34128',circular='#a8afb7',platform='#fafafa')
        for f in config['fixtures']:
            for s in f['shapes']:
                color='#fafafa' if s.get('white') else colors[f['kind']]
                if 'rect' in s:
                    x,y,w,h=s['rect'];patch=Rectangle((x-w/2,y-h/2),w,h,facecolor=color,edgecolor='#334155',linewidth=.7)
                else:x,y,r=s['circle'];patch=Circle((x,y),r,facecolor=color,edgecolor='#334155')
                ax.add_patch(patch)
            center=np.mean([s.get('rect',s.get('circle'))[:2] for s in f['shapes']],axis=0);ax.text(*center,f['id'].replace('_','\n'),ha='center',va='center',fontsize=6,color='#102030')
        ax.set(xlim=(0,12),ylim=(0,10),xlabel='x (m)',ylabel='y (m)',title=variant+'\nReconstrução aproximada — referência visual RoboCup@Work 2016');ax.set_xticks(np.arange(13));ax.set_yticks(np.arange(11));ax.grid(alpha=.15);ax.set_aspect('equal');fig.tight_layout();fig.savefig(output/f'{variant}.png',dpi=160);plt.close(fig)
    (output/'validation.json').write_text(json.dumps(report,indent=2));return report
class Arena2016Env(RoboEnv):
    def __init__(self,variant='arena_2016_limpa',layout=LAYOUT,noise=None,boxes=None,progression=False,**kwargs):
        self.layout=read_layout(layout)
        if variant not in self.layout['variants']:raise ValueError('Variante desconhecida')
        self.variant=variant;self.settings=dict(self.layout['variants'][variant])
        if noise is not None:self.settings['noise']=noise
        if boxes is not None:
            if boxes<0:raise ValueError('Caixas deve ser >=0')
            self.settings['mobile_boxes']=boxes
        self.progression=progression;self._base,self._masks=rasterize(self.layout);validate(self.layout,self._base,self._masks);self.boxes_placed=[]
        kwargs.pop('level',None);super().__init__(level=4,**kwargs)
    def _scenario(self):
        rng=self.np_random;grid=Grid(self._base.cells.copy(),self._base.resolution,source=self.variant);self.boxes_placed=[];count=self.settings.get('mobile_boxes',0)
        if self.progression:count=0 if self.level<=2 else min(count,self.level-2)
        for _ in range(count):
            distance=distance_transform_edt(~grid.blocked)*grid.resolution-grid.resolution*np.sqrt(2)
            old_components=grid.components.copy()
            placed=False
            for attempt in range(500):
                size=rng.uniform(*self.settings.get('box_size_m',[.25,.45]),size=2);center=rng.uniform([0,0],self.layout['size_m']);low=np.floor((center-size/2)/grid.resolution).astype(int);high=np.ceil((center+size/2)/grid.resolution).astype(int)
                if (low<0).any() or (high>np.array(grid.cells.shape[::-1])).any():continue
                region=grid.cells[low[1]:high[1],low[0]:high[0]]
                if not region.size or np.any(region):continue
                if distance[low[1]:high[1],low[0]:high[0]].min()<self.layout['minimum_passage_m']:continue
                old=grid.cells.copy();grid.add_box(center,size);grid.refresh()
                split=False
                for component in range(1,self._base.n_components+1):
                    surviving=np.unique(grid.components[(old_components==component)&(grid.components>0)])
                    if len(surviving)!=1:split=True;break
                if split:grid.cells=old;grid.refresh();continue
                self.boxes_placed.append(dict(center=center.tolist(),size=size.tolist(),attempts=attempt+1));placed=True;break
            if not placed:return None
        start,goal=grid.sample_pair(rng,maximum=3. if self.progression and self.level==1 else 12.);jitter=self.settings.get('pose_jitter_m',0.)
        for _ in range(100):
            points=np.array([start,goal])+rng.uniform(-jitter,jitter,(2,2))
            if all(grid.distance(p)>RADIUS+.03 for p in points):
                si,gi=grid.indices(points);a=grid.components[si[1],si[0]];b=grid.components[gi[1],gi[0]]
                if a>0 and a==b and np.linalg.norm(points[1]-points[0])>.8:return grid,*points
        return None
    def reset(self,*,seed=None,options=None):
        self.noise_enabled=False
        _,info=super().reset(seed=seed,options=options);rng=self.np_random;noisy=self.settings.get('noise',False)
        if self.progression:noisy=noisy and self.level==5
        self.friction=rng.uniform(*self.settings.get('friction_range',[.9,1.])) if noisy else 1.;self.slip=rng.uniform(*self.settings.get('slip_range',[.02,.06])) if noisy else 0.;self.sensor_noise=self.settings.get('sensor_noise_m',.015) if noisy else 0.;self.bias=rng.normal(0,self.settings.get('odom_bias_std',[.003,.003,.002])) if noisy else np.zeros(3)
        from collections import deque
        limits=self.settings.get('delay_ticks',[1,2]);self.delay_ticks=int(rng.integers(limits[0],limits[1]+1)) if noisy else 1;self.queue=deque([np.zeros(3) for _ in range(self.delay_ticks)]);self.noise_enabled=noisy;self.scan=self.lidar();info.update(arena=self.variant,boxes=len(self.boxes_placed),noise=noisy);return self.observation(),info
    def lidar(self):
        level=self.level
        self.level=4  # Use configured dropout rather than the inherited level-5 default.
        try:scan,valid=super().lidar()
        finally:self.level=level
        probability=self.settings.get('scan_dropout_probability',0.) if getattr(self,'noise_enabled',False) else 0.
        if probability:
            keep=self.np_random.random(self.beams)>=probability
            valid=valid*keep;scan=scan*keep
        return scan.astype(np.float32),valid.astype(np.float32)

    def step(self, action):
        rng=self.np_random
        self._np_random=_StepRandom(rng,self.settings,self.noise_enabled)
        try:return super().step(action)
        finally:self._np_random=rng

class _StepRandom:
    def __init__(self, rng, settings, enabled):self.rng,self.settings,self.enabled=rng,settings,enabled
    def __getattr__(self,name):return getattr(self.rng,name)
    def normal(self,loc=0.,scale=1.,size=None):
        if np.ndim(scale)==1 and len(scale)==3:
            scale=np.asarray(self.settings.get('physical_noise_std',[.001,.002,.001])) if self.enabled else np.zeros(3)
        elif not self.enabled:scale=np.zeros_like(scale)
        return self.rng.normal(loc,scale,size)
    def random(self,size=None):
        return self.rng.random(size) if self.enabled else np.ones(size) if size is not None else 1.

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--build',action='store_true');p.add_argument('--output');a=p.parse_args()
    if a.build:print(json.dumps(build(a.output),indent=2))
