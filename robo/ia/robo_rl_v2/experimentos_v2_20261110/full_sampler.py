from pathlib import Path
from collections import deque
import numpy as np
from native import NativeV2Env
class FullFootprintEnv(NativeV2Env):
 def __init__(self,**kwargs):
  if kwargs.get('guidance',False):raise ValueError('Disk planner is incompatible with full-footprint narrow-passage sampling')
  super().__init__(**kwargs)
  data=np.load(Path(__file__).with_name('sampler_states.npz'));self.state_masks=data['main'];self.state_angles=data['angles']
  self.indices=[np.flatnonzero(m).astype(np.int32) for m in self.state_masks];self.accum=np.cumsum([len(i) for i in self.indices])
 def state(self):
  pick=int(self.np_random.integers(self.accum[-1]));k=int(np.searchsorted(self.accum,pick,side='right'));start=0 if k==0 else self.accum[k-1]
  index=int(self.indices[k][pick-start]);y,x=divmod(index,self.physical_grid.cells.shape[1]);pos=self.physical_grid.world((np.array([x,y])+.5)*self.physical_grid.resolution)
  return np.r_[pos,self.state_angles[k]]
 def reset(self,*,seed=None,options=None):
  _,info=super().reset(seed=seed,options=options)
  a=self.state()
  for _ in range(2000):
   b=self.state()
   if .8<np.linalg.norm(a[:2]-b[:2])<self.maximum:break
  else:raise RuntimeError('No connected state pair')
  self.pose=a;self.estimate=a.copy();self.goal=b[:2];self.goal_pose=b.copy();self.path=[a.copy()];self.window=deque([a[:2].copy()],maxlen=32)
  self.scan=self.lidar();info['sampler']='full_footprint_connected_SE2';return self.observation(),info
