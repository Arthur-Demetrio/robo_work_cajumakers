"""Separate v2 environment: physical and laser occupancy are never conflated."""
from pathlib import Path
import numpy as np,yaml
from .maps import Grid,ROOT
from .env import RoboEnv,rotation
LAYOUT=ROOT/'assets/arenas/arena_2016_v2/layout_review.yaml'
class Arena2016V2Env(RoboEnv):
    def __init__(self,layout=LAYOUT,**kwargs):
        self.layout_path=Path(layout).resolve();self.config=yaml.safe_load(self.layout_path.read_text())
        self.variant='arena_2016_v2';self.size=np.array(self.config['floor']['size_m'])
        self.physical_grid=Grid.load(self.layout_path.parent/'review/colisao_fisica.yaml')
        self.laser_grid=Grid.load(self.layout_path.parent/'review/lidar.yaml')
        kwargs.pop('level',None);super().__init__(level=4,**kwargs)
    def _scenario(self):
        grid=Grid(self.physical_grid.cells.copy(),self.physical_grid.resolution,source=self.variant)
        start,goal=grid.sample_pair(self.np_random,maximum=12.)
        return grid,start,goal
    def collision(self,pose=None,margin=0.):
        corners=self.corners(pose)
        if (corners-margin<0).any() or (corners+margin>self.size).any():return True
        return super().collision(pose,margin)
    def laser_hits(self,points):
        points=np.asarray(points);ij=self.laser_grid.indices(points);x,y=ij[...,0],ij[...,1]
        valid=(x>=0)&(y>=0)&(x<self.laser_grid.cells.shape[1])&(y<self.laser_grid.cells.shape[0])&(points[...,0]<self.size[0])&(points[...,1]<self.size[1])
        return valid&self.laser_grid.blocked[np.clip(y,0,self.laser_grid.cells.shape[0]-1),np.clip(x,0,self.laser_grid.cells.shape[1]-1)]
    def lidar(self):
        origin=self.pose[:2]+rotation(self.pose[2])@self.lidar_offset
        angles=self.angles+self.pose[2];dirs=np.c_[np.cos(angles),np.sin(angles)]
        distances=np.arange(0,self.max_range+self.laser_grid.resolution/2,self.laser_grid.resolution/2)
        hits=self.laser_hits(origin+dirs[:,None,:]*distances[None,:,None])
        ranges=np.where(hits.any(1),distances[hits.argmax(1)],self.max_range)
        ranges+=self.np_random.normal(0,self.sensor_noise,self.beams)
        return np.where(self.mask,np.clip(ranges,0,self.max_range)/self.max_range,0).astype(np.float32),self.mask.astype(np.float32)
