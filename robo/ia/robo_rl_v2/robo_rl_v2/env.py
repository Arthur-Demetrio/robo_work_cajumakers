"""Gymnasium mecanum: política recebe sensores estimados; reward usa somente verdade física."""
from collections import deque
from pathlib import Path
import math
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import yaml
from .maps import Grid, ROOT

FOOTPRINT = np.array([[.365,.315],[.365,-.315],[-.205,-.315],[-.205,.315]])
RADIUS = float(np.linalg.norm(FOOTPRINT, axis=1).max())

def rotation(yaw):
    c,s = np.cos(yaw),np.sin(yaw)
    return np.array([[c,-s],[s,c]])

def wrap(x): return (x+np.pi)%(2*np.pi)-np.pi

class RoboEnv(gym.Env):
    metadata = {'render_modes':['rgb_array'], 'render_fps':6}
    dt = .16  # 6.25 Hz; 4 command/dynamics ticks per scan
    beams = 72
    max_range = 4.
    lidar_offset = np.array([.33,.005])

    def __init__(self, level=1, max_steps=1500, render_mode=None, split='train', map_path=None):
        super().__init__()
        if level not in range(1,6): raise ValueError('level deve ser 1..5')
        self.level, self.max_steps, self.render_mode, self.split = level,max_steps,render_mode,split
        self.map_path = Path(map_path or ROOT/'assets/maps/arena_04-11_0917.yaml')
        config = yaml.safe_load((ROOT/'assets/config/nav2_params.yaml').read_text())
        controller = config['controller_server']['ros__parameters']['FollowPath']
        # Conservative controller limits; reverse from velocity smoother (DWB disallows reverse).
        smoother = config['velocity_smoother']['ros__parameters']
        self.limits = np.array([controller['max_vel_x'],controller['max_vel_y'],controller['max_vel_theta']])
        self.reverse_limit = abs(smoother['min_velocity'][0])
        self.accel = np.array([controller['acc_lim_x'],controller['acc_lim_y'],controller['acc_lim_theta']])
        self.decel = np.abs([controller['decel_lim_x'],controller['decel_lim_y'],controller['decel_lim_theta']])
        self.max_xy = controller['max_speed_xy']
        self.action_space = spaces.Box(-1.,1.,(3,),dtype=np.float32)
        # scan + mask + local estimated goal(2) + command(3) + estimated odom rate(3)
        self.observation_space = spaces.Box(-1.,1.,(self.beams*2+8,),dtype=np.float32)
        self.angles = np.linspace(-np.pi,np.pi,self.beams,endpoint=False)
        sectors = yaml.safe_load((ROOT/'assets/config/lidar.yaml').read_text())['scan_filter']['ros__parameters']['blocked_intervals_deg']
        self.mask = np.ones(self.beams,dtype=bool)
        for interval in sectors.split(','):
            lo,hi = map(float,interval.split(':'))
            self.mask &= ~((np.rad2deg(self.angles)>=lo)&(np.rad2deg(self.angles)<=hi))
        self.done = True
        self.curriculum = None
        self.transitions = 0

    def set_curriculum(self, boundaries):
        self.curriculum = list(boundaries)

    def corners(self, pose=None):
        pose = self.pose if pose is None else pose
        return FOOTPRINT @ rotation(pose[2]).T + pose[:2]

    def collision(self, pose=None, margin=0.):
        """Exact polygon/cell SAT; positive margin conservatively bounds swept volume."""
        pose = self.pose if pose is None else np.asarray(pose)
        poly = self.grid.local(self.corners(pose))
        res = self.grid.resolution
        low = np.floor((poly.min(axis=0)-margin)/res).astype(int)
        high = np.floor((poly.max(axis=0)+margin)/res).astype(int)
        height,width = self.grid.cells.shape
        if (low<0).any() or high[0]>=width or high[1]>=height: return True
        yy,xx = np.nonzero(self.grid.blocked[low[1]:high[1]+1,low[0]:high[0]+1])
        if not len(xx): return False
        centers = (np.column_stack([xx+low[0],yy+low[1]])+.5)*res
        yaw = pose[2]-self.grid.origin[2]
        axes = np.array([[1.,0.],[0.,1.],[np.cos(yaw),np.sin(yaw)],[-np.sin(yaw),np.cos(yaw)]])
        projection = poly @ axes.T
        cp = centers @ axes.T
        half = res*.5*np.abs(axes).sum(axis=1)
        overlap = (cp+half >= projection.min(axis=0)-margin)&(cp-half <= projection.max(axis=0)+margin)
        return bool(np.any(overlap.all(axis=1)))

    def _scenario(self):
        rng = self.np_random
        if self.level==4:
            grid = Grid.load(self.map_path)
        elif self.level==5:
            # Train uses synthetic rooms, unseen uses procedural internal walls too.
            grid = Grid.synthetic(rng, variant=self.split!='train')
        else: grid = Grid.synthetic(rng)
        start,goal = grid.sample_pair(rng, maximum=3. if self.level==1 else 7.)
        if self.level==2:
            center = start+(goal-start)*rng.uniform(.35,.65)
            if np.linalg.norm(goal-start)<2.: return None
            grid.add_box(center,rng.uniform([.3,.3],[.65,.65]))
        elif self.level>=3:
            for _ in range(int(rng.integers(2,7))):
                # Uniform map-frame positions also produce boxes near walls.
                center = grid.world(rng.uniform([0,0],np.array(grid.cells.shape[::-1])*grid.resolution))
                if min(np.linalg.norm(center-start),np.linalg.norm(center-goal))>.95:
                    grid.add_box(center,rng.uniform([.25,.25],[.9,.9]))
        grid.refresh()
        # Conservative disk connectivity guarantees a feasible sampled translational route.
        si,gi = grid.indices(np.array([start,goal]))
        a,b = grid.components[si[1],si[0]],grid.components[gi[1],gi[0]]
        if a==0 or a!=b: return None
        return grid,start,goal

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        if self.curriculum:
            self.level = min(5,1+sum(self.transitions>=x for x in self.curriculum))
        result = None
        for _ in range(200):
            result = self._scenario()
            if result is not None: break
        if result is None: raise RuntimeError('Cenários rejeitados: sem caminho seguro')
        self.grid,start,self.goal = result
        self.pose = np.r_[start,self.np_random.uniform(-np.pi,np.pi)]
        self.estimate = self.pose.copy()
        self.velocity = np.zeros(3)
        self.odom_rate = np.zeros(3)
        self.command = np.zeros(3)
        self.previous = np.zeros(3)
        self.steps=0; self.done=False
        self.path=[self.pose.copy()]
        self.metrics=dict(path_length=0., stopped_time=0., rotations=0., oscillations=0, recoveries=0)
        self.stuck=False; self.window=deque([self.pose[:2].copy()],maxlen=32)
        noisy = self.level==5
        self.friction = self.np_random.uniform(.65,1.1) if noisy else self.np_random.uniform(.9,1.)
        self.slip = self.np_random.uniform(.05,.25) if noisy else .03
        self.sensor_noise = .025 if noisy else .005
        self.bias = self.np_random.normal(0,[.012,.012,.008] if noisy else [.003,.003,.002])
        self.delay_ticks = int(self.np_random.integers(1,6)) if noisy else 1
        self.queue = deque([np.zeros(3) for _ in range(self.delay_ticks)])
        self.scan = self.lidar()
        return self.observation(), {'level':self.level}

    def lidar(self):
        origin = self.pose[:2]+rotation(self.pose[2])@self.lidar_offset
        angles = self.angles+self.pose[2]
        dirs = np.column_stack([np.cos(angles),np.sin(angles)])
        # Sampling <= half a cell; ray occupancy includes unknown and outside.
        distances = np.arange(0,self.max_range+self.grid.resolution/2,self.grid.resolution/2)
        samples = origin+dirs[:,None,:]*distances[None,:,None]
        hits = self.grid.hits(samples)
        first = hits.argmax(axis=1)
        ranges = np.where(hits.any(axis=1),distances[first],self.max_range)
        ranges += self.np_random.normal(0,self.sensor_noise,self.beams)
        valid = self.mask.copy()
        if self.level==5: valid &= self.np_random.random(self.beams)>.02
        return np.where(valid,np.clip(ranges,0,self.max_range)/self.max_range,0).astype(np.float32), valid.astype(np.float32)

    def observation(self):
        # Estimated pose integrates delayed commands, never wheel encoders or true velocity.
        local_goal = rotation(self.estimate[2]).T@(self.goal-self.estimate[:2])
        return np.r_[self.scan[0],self.scan[1],np.clip(local_goal/10,-1,1),
                     self.command,np.clip(self.odom_rate/self.limits,-1,1)].astype(np.float32)

    def step(self, action):
        if self.done: raise RuntimeError('Chame reset antes de step')
        action = np.asarray(action,dtype=float)
        if action.shape!=(3,) or not np.isfinite(action).all(): raise ValueError('Ação inválida')
        self.command = np.clip(action,-1,1)
        target = self.command*self.limits
        if target[0]<0: target[0]=self.command[0]*self.reverse_limit
        norm=np.linalg.norm(target[:2])
        if norm>self.max_xy: target[:2]*=self.max_xy/norm
        before = self.pose.copy(); old_d = np.linalg.norm(self.goal-before[:2])
        collided=False; h=self.dt/4
        for _ in range(4):
            self.queue.append(target.copy()); delayed=self.queue.popleft()
            limits=np.where(np.abs(delayed)>np.abs(self.velocity),self.accel,self.decel)
            self.velocity += np.clip(delayed-self.velocity,-limits*h,limits*h)
            physical=self.velocity.copy()*self.friction
            physical[:2] *= [1-self.slip*.3,1-self.slip]
            physical += self.np_random.normal(0,[.001,.002,.001] if self.level<5 else [.004,.008,.004])
            # For every point in this interval, displacement <= translation + radius*angle.
            bound=h*(np.linalg.norm(physical[:2])+RADIUS*abs(physical[2]))+1e-9
            if self.collision(self.pose,margin=bound): collided=True; break
            self.pose[:2] += rotation(self.pose[2]+physical[2]*h/2)@physical[:2]*h
            self.pose[2]=wrap(self.pose[2]+physical[2]*h)
            self.odom_rate = delayed+self.bias
            self.estimate[:2] += rotation(self.estimate[2])@self.odom_rate[:2]*h
            self.estimate[2]=wrap(self.estimate[2]+self.odom_rate[2]*h)
            self.metrics['path_length'] += float(np.linalg.norm(physical[:2])*h)
            self.metrics['rotations'] += float(abs(physical[2])*h)
        displacement=np.linalg.norm(self.pose[:2]-before[:2])
        if displacement/self.dt<.015: self.metrics['stopped_time']+=self.dt
        reversal=bool(np.any((self.command*self.previous<-.1)))
        self.metrics['oscillations']+=int(reversal)
        self.previous=self.command.copy()
        self.window.append(self.pose[:2].copy())
        stuck=len(self.window)==32 and np.linalg.norm(self.window[-1]-self.window[0])<.08
        if self.stuck and not stuck: self.metrics['recoveries']+=1
        self.stuck=stuck
        self.steps+=1; self.transitions+=1; self.path.append(self.pose.copy())
        distance=float(np.linalg.norm(self.goal-self.pose[:2]))
        success=bool(distance<.2 and not collided)
        # Truth-only shaping; no odometry-derived progress reward.
        clearance=max(0,self.grid.distance(self.pose[:2])-RADIUS)
        progress=float(old_d-distance)
        reward=15*progress-.015-.025*max(0,1-clearance/.25)
        reward-=.01*displacement+.03*abs(wrap(self.pose[2]-before[2]))+.01*int(reversal)+.025*int(stuck)
        if displacement<.001: reward-=.02
        if success: reward+=100
        if collided: reward-=100
        terminated=bool(success or collided); truncated=bool(self.steps>=self.max_steps and not terminated)
        self.done=terminated or truncated
        self.scan=self.lidar()
        info=dict(success=success,collision=collided,level=self.level,steps=self.steps,
                  elapsed=self.steps*self.dt,distance=distance,**self.metrics)
        return self.observation(),float(reward),terminated,truncated,info

    def render(self):
        from .visual import render_array
        return render_array(self)
