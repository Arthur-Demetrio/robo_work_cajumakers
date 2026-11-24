#!/usr/bin/env python3
"""Collect real Encosto-core demonstrations in a static virtual-wall docking task.

This calls the frozen, ROS-independent Encosto and RastreioFaceWS code. It does
not start ROS, write to the robot workspace, or publish motor commands.
"""
import argparse, hashlib, json, math, sys, time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
CODE=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(CODE)]
from full_sampler import FullFootprintEnv
from envs import FastV2Env
from robo_rl_v2.teacher_legacy.adapter import load_core, points_from_scan, command_label, DemonstrationWriter

FACE_X=6.39
FACE_Y=3.00
FACE_WIDTH=.92
WALL_CENTER=(FACE_X+.05,FACE_Y)
WALL_SIZE=(.10,FACE_WIDTH)
GOAL=(FACE_X-(.365+.08),FACE_Y)

class DockTeacherEnv(FullFootprintEnv):
    # The aligner control loop runs at 20 Hz. Keep its task-specific dynamics at
    # that rate; the simulated laser below remains at the robot's ~6 Hz scan rate.
    dt=.05
    def __init__(self,**kwargs):
        super().__init__(layout=CODE/'layout_service_candidate.yaml',maximum=12.,**kwargs)
        # This is a task fixture, isolated to this new environment instance.
        self.laser_grid.add_box(WALL_CENTER,WALL_SIZE);self.laser_grid.refresh()
        self.core=load_core();self.face={'x':FACE_X,'y':FACE_Y,'yaw':0.,'largura_medida':FACE_WIDTH}
        self.teacher=None;self.tracker=None;self.task_terminal=False
        self.sim_clock=0.;self._last_lidar_time=-float('inf');self._cached_lidar=None;self._last_teacher_scan=-float('inf')
        self.last_med=None;self.last_points=[]
    def lidar(self):
        if getattr(self,'_cached_lidar',None) is not None and self.sim_clock-self._last_lidar_time<.16-1e-9:
            return self._cached_lidar
        scan=super().lidar();self._cached_lidar=(scan[0].copy(),scan[1].copy());self._last_lidar_time=self.sim_clock
        return self._cached_lidar
    def _scenario(self):
        grid=type(self.physical_grid)(self.physical_grid.cells.copy(),self.physical_grid.resolution,
                                     self.physical_grid.origin.copy(),self.physical_grid.source)
        grid.add_box(WALL_CENTER,WALL_SIZE);grid.refresh()
        a,b=grid.sample_pair(self.np_random,minimum=.8,maximum=3.)
        return grid,a,b
    def task_reset(self,seed):
        # Skip FullFootprintEnv's global pose sampler: this task samples valid poses
        # against the newly inserted fixture using the same exact polygon collision.
        self.sim_clock=0.;self._last_lidar_time=-float('inf');self._cached_lidar=None;self._last_teacher_scan=-float('inf')
        obs,info=FastV2Env.reset(self,seed=seed)
        fx=FACE_X
        found=None
        for _ in range(300):
            x=fx-float(self.np_random.uniform(.84,1.12))
            y=FACE_Y+float(self.np_random.uniform(-.12,.12))
            yaw=float(self.np_random.uniform(-math.radians(12),math.radians(12)))
            candidate=np.array([x,y,yaw])
            if not self.collision(candidate):found=candidate;break
        if found is None:raise RuntimeError('Não encontrei pose inicial livre para docking')
        self.pose=found;self.estimate=found.copy();self.goal=np.array(GOAL);self.goal_pose=np.array([*GOAL,0.])
        self.velocity=np.zeros(3);self.odom_rate=np.zeros(3);self.command=np.zeros(3);self.previous=np.zeros(3)
        self.steps=0;self.done=False;self.path=[self.pose.copy()];self.window.clear();self.window.append(self.pose[:2].copy())
        self.metrics=dict(path_length=0.,stopped_time=0.,rotations=0.,oscillations=0,recoveries=0)
        self.stuck=False;self.queue.clear();self.queue.extend(np.zeros(3) for _ in range(self.delay_ticks))
        self._cached_lidar=None;self._last_lidar_time=-float('inf');self._last_teacher_scan=-float('inf')
        self.scan=self.lidar();self.teacher=self.core.Encosto(log=lambda *_:None);self.tracker=self.core.RastreioFaceWS(self.face)
        self.last_med=None;self.last_points=[]
        self.task_terminal=False
        return self.observation(),dict(level='dock_teacher',fixture='virtual_wall',start=self.pose.copy(),goal=self.goal.copy())
    def teacher_action(self,obs,t):
        pose=tuple(float(v) for v in self.estimate)
        if self._last_lidar_time>self._last_teacher_scan+1e-9:
            ranges=np.asarray(obs[:self.beams],float)*self.max_range
            valid=np.asarray(obs[self.beams:2*self.beams]>.5,bool)
            points=[tuple(p) for p in points_from_scan(ranges,self.angles,valid,lidar_xy=tuple(self.lidar_offset))]
            self.last_med=self.tracker.medir(points,self._last_lidar_time,pose,pose)
            self.last_points=points
            self.teacher.nova_medicao(self._last_lidar_time,self.last_med,pose)
            self.teacher.obstaculo=self.core.obstaculo_antes_da_face(points,self.last_med,self.teacher.pc)
            self._last_teacher_scan=self._last_lidar_time
        command=self.teacher.passo(t,pose)
        label=command_label(command,limits=tuple(self.limits),reverse=self.reverse_limit,max_xy=self.max_xy)
        return command,label,self.last_med,self.last_points
    def step_teacher(self,action):
        """Approximate measured motor dead-zone/gain in this skill-only fixture.

        The aligned controller writes cmd_vel in SI units; the robot bridge produces
        substantially larger motion at low PWM. Map that request to measured motion,
        then run the same collision/dynamics code at the resulting physical velocity.
        This is local to this experiment; the general PPO environment is untouched.
        """
        raw=np.asarray(action,dtype=float);requested=raw*self.limits
        physical=requested.copy()
        if abs(requested[0])>=.012:
            physical[0]=math.copysign(.078+.75*abs(requested[0]),requested[0])
        else:
            physical[0]=0.
        if abs(requested[1])>=.015:
            physical[1]=math.copysign(.09,requested[1])
        else:
            physical[1]=0.
        if abs(requested[2])>=.015:
            # PWM rotation tests: 0.02 rad/s request -> about 0.19 rad/s measured.
            physical[2]=math.copysign(.19+.45*max(0.,abs(requested[2])-.02),requested[2])
        else:
            physical[2]=0.
        nominal=self.limits.copy();nominal_xy=self.max_xy
        effective=np.array([.30,.12,.40])
        sim_action=physical/effective
        self.limits=effective;self.max_xy=max(nominal_xy,.32)
        self.sim_clock+=self.dt
        try:
            obs,reward,terminated,truncated,info=super().step(sim_action)
        finally:
            self.limits=nominal;self.max_xy=nominal_xy
        # Observation remains in the policy's command frame; odometry rate is
        # re-normalized under the nominal limits expected at deployment.
        self.command=np.clip(raw,-1,1);self.previous=self.command.copy()
        obs=self.observation()
        return obs,reward,terminated,truncated,info

def main():
    p=argparse.ArgumentParser();p.add_argument('--episodes',type=int,default=120);p.add_argument('--seed',type=int,default=71000);p.add_argument('--output-dir',required=True);a=p.parse_args()
    if a.episodes<30:p.error('Use pelo menos 30 episódios independentes para uma coleta inicial')
    out=Path(a.output_dir).resolve();out.mkdir(parents=True,exist_ok=False)
    env=DockTeacherEnv(max_steps=1300);rows=[];nsteps=0
    data=out/'dock_demonstrations.jsonl'
    with DemonstrationWriter(data) as writer:
        for ep in range(a.episodes):
            seed=a.seed+ep;obs,reset_info=env.task_reset(seed);ep_rows=0;collision=False;success=False;reason='max_steps';t=0.
            for step in range(env.max_steps):
                command,label,med,points=env.teacher_action(obs,t)
                state=str(env.teacher.estado)
                writer.append(obs,command,'dock',state,t,episode_id=f'dock-{seed}',
                    seed=seed,teacher_reason=str(env.teacher.motivo),measurement_valid=med is not None,
                    face_distance=None if med is None else float(med['dist']),
                    face_center=None if med is None or med.get('centro') is None else float(med['centro']),
                    face_angle=None if med is None else float(med['ang']))
                ep_rows+=1;nsteps+=1
                obs,reward,terminated,truncated,info=env.step_teacher(np.asarray(label['action'],np.float32))
                t+=env.dt
                if info.get('collision'):
                    collision=True;reason='collision';break
                if env.teacher.estado=='fim':success=True;reason='teacher_success';break
                if env.teacher.estado=='abortar':reason='teacher_abort:'+str(env.teacher.motivo);break
                # RoboEnv's generic 20 cm waypoint success is not docking completion.
                if terminated and info.get('success'):
                    env.done=False
            rows.append(dict(seed=seed,steps=ep_rows,success=success,collision=collision,reason=reason,
                             distance=float(np.linalg.norm(env.goal-env.pose[:2])),
                             final_pose=env.pose.tolist(),teacher_state=env.teacher.estado,
                             teacher_reason=env.teacher.motivo))
            if (ep+1)%10==0:print(f'episodes {ep+1}/{a.episodes} success={sum(r["success"] for r in rows)} collisions={sum(r["collision"] for r in rows)} labels={nsteps}',flush=True)
    env.close()
    manifest={'task':'docking_to_virtual_wall_using_actual_frozen_Encosto_and_RastreioFaceWS','episodes':rows,'n_episodes':len(rows),'transitions':nsteps,'successes':sum(r['success'] for r in rows),'collisions':sum(r['collision'] for r in rows),'success_rate':float(np.mean([r['success'] for r in rows])),'collision_rate':float(np.mean([r['collision'] for r in rows])),'dataset':data.name,'dataset_sha256':hashlib.sha256(data.read_bytes()).hexdigest(),'sim_dt_s':env.dt,'teacher_loop_hz':20,'simulated_scan_update_period_s':.20,'teacher_source':'robo_rl_v2.teacher_legacy.snapshot; source SHA-verified by adapter','actuator_approximation':{'forward_mps':'sign(cmd)*(0.078+0.75*abs(cmd)) for abs(cmd)>=0.012; below threshold zero','lateral_mps':'0.09 for abs(cmd)>=0.015; below threshold zero','yaw_radps':'0.19 at 0.02 rad/s request for abs(cmd)>=0.015; linear interpolation above; below threshold zero','status':'existing measurements/comments, not recalibrated during this run'},'fixture':{'type':'virtual wall, axis-aligned','center_xy':WALL_CENTER,'size_xy_m':WALL_SIZE,'face_xyyaw':[FACE_X,FACE_Y,0.],'width_m':FACE_WIDTH,'goal_xy':GOAL,'assumptions':['task fixture, not official arena object','sensor extrinsics use simulator lidar_offset']},'uses_real_nav2_behavior_tree':False,'uses_actual_encosto_core':True,'uses_actual_face_tracking_core':True,'robot_or_ros_started':False}
    (out/'collection.json').write_text(json.dumps(manifest,indent=2));print(json.dumps({k:v for k,v in manifest.items() if k!='episodes'},indent=2))
if __name__=='__main__':main()
