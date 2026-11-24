"""Isolated candidates: allocation optimization and map/estimated-localization waypoints."""
import sys,heapq,math
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from robo_rl_v2.arena2016v2 import Arena2016V2Env
from robo_rl_v2.env import rotation

class FastV2Env(Arena2016V2Env):
    def __init__(self,maximum=12.,guidance=False,**kwargs):
        self.maximum=maximum;self.guidance=guidance;self.route=None
        super().__init__(**kwargs)
        self._ranges=np.arange(0,self.max_range+self.laser_grid.resolution/2,self.laser_grid.resolution/2)
        sizes=np.bincount(self.physical_grid.components.ravel());sizes[0]=0
        self._candidates=np.argwhere(self.physical_grid.components==sizes.argmax())
        self.planner=SafePlanner(self.physical_grid)
    def _scenario(self):
        grid=self.physical_grid;rng=self.np_random
        for _ in range(2000):
            a,b=self._candidates[rng.integers(len(self._candidates),size=2)]
            start,goal=grid.world((np.array([a[::-1],b[::-1]])+.5)*grid.resolution)
            if .8<np.linalg.norm(goal-start)<self.maximum:return grid,start,goal
        raise RuntimeError('Não foi possível amostrar destino conectado')
    def laser_hits(self,points):
        points=np.asarray(points);ij=np.floor(points/self.laser_grid.resolution).astype(int);x,y=ij[...,0],ij[...,1]
        valid=(x>=0)&(y>=0)&(x<self.laser_grid.cells.shape[1])&(y<self.laser_grid.cells.shape[0])&(points[...,0]<self.size[0])&(points[...,1]<self.size[1])
        # Flattened indexing cuts allocation but retains outside-as-no-return semantics.
        return valid&self.laser_grid.blocked[np.clip(y,0,self.laser_grid.cells.shape[0]-1),np.clip(x,0,self.laser_grid.cells.shape[1]-1)]
    def lidar(self):
        origin=self.pose[:2]+rotation(self.pose[2])@self.lidar_offset;angles=self.angles+self.pose[2];dirs=np.c_[np.cos(angles),np.sin(angles)]
        distances=self._ranges
        hits=self.laser_hits(origin+dirs[:,None,:]*distances[None,:,None])
        ranges=np.where(hits.any(1),distances[hits.argmax(1)],self.max_range)
        ranges+=self.np_random.normal(0,self.sensor_noise,self.beams)
        return np.where(self.mask,np.clip(ranges,0,self.max_range)/self.max_range,0).astype(np.float32),self.mask.astype(np.float32)
    def reset(self,*,seed=None,options=None):
        self.route=None
        obs,info=super().reset(seed=seed,options=options)
        if self.guidance:self.route=self.planner.plan(self.estimate[:2],self.goal)
        return self.observation(),info
    def observation(self):
        obs=super().observation()
        if self.guidance and self.route is not None:
            target=self.planner.waypoint(self.route,self.estimate[:2],self.goal)
            obs[144:146]=np.clip(rotation(self.estimate[2]).T@(target-self.estimate[:2])/10,-1,1)
        return obs

class SafePlanner:
    """A* on the known static map. Never accesses true pose when supplying policy input."""
    def __init__(self,grid):
        self.grid=grid;self.safe=grid.clearance>.52;self.h,self.w=self.safe.shape
        self.points=np.argwhere(self.safe)
    def cell(self,point):
        x,y=self.grid.indices(point)
        if 0<=x<self.w and 0<=y<self.h and self.safe[y,x]:return int(x),int(y)
        # Projection uses only map and supplied estimated pose, not simulator truth.
        candidates=self.points;local=self.grid.local(point)/self.grid.resolution-.5
        k=np.argmin(np.sum((candidates[:,::-1]-local)**2,axis=1));y,x=candidates[k];return int(x),int(y)
    def plan(self,start,goal):
        a=self.cell(start);b=self.cell(goal)
        todo=[(abs(a[0]-b[0])+abs(a[1]-b[1]),0,a)];cost={a:0};came={};visited=set()
        while todo:
            _,g,node=heapq.heappop(todo)
            if node in visited:continue
            if node==b:
                route=[node]
                while node!=a:node=came[node];route.append(node)
                route=route[::-1]
                return self.grid.world((np.asarray(route)+.5)*self.grid.resolution)
            visited.add(node);x,y=node
            for nxt in [(x-1,y),(x+1,y),(x,y-1),(x,y+1)]:
                xx,yy=nxt
                if not(0<=xx<self.w and 0<=yy<self.h and self.safe[yy,xx]):continue
                ng=g+1
                if ng<cost.get(nxt,math.inf):
                    cost[nxt]=ng;came[nxt]=node;heapq.heappush(todo,(ng+abs(xx-b[0])+abs(yy-b[1]),ng,nxt))
        raise RuntimeError('Sem caminho A* no mapa conservador')
    def waypoint(self,route,position,goal,lookahead=.55):
        if np.linalg.norm(goal-position)<lookahead:return goal
        idx=int(np.argmin(np.sum((route-position)**2,axis=1)))
        end=min(len(route)-1,idx+round(lookahead/self.grid.resolution))
        # Stay on a locally visible safe part of the polyline; no corner cutting.
        target=route[idx]
        for j in range(idx,end+1):
            p=route[j];t=np.linspace(0,1,max(2,int(np.linalg.norm(p-position)/self.grid.resolution*2)+1))
            samples=position+(p-position)*t[:,None];ij=self.grid.indices(samples);x,y=ij[:,0],ij[:,1]
            if (x<0).any() or (y<0).any() or (x>=self.w).any() or (y>=self.h).any():break
            if not self.safe[y,x].all():break
            target=p
        return target

def make_env(maximum=12.,guidance=False,max_steps=1500):
    return FastV2Env(maximum=maximum,guidance=guidance,max_steps=max_steps)
