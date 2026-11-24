#!/usr/bin/env python3
"""Paired-seed evaluation of the frozen Encosto teacher and a BC docking actor."""
import argparse,json,math,sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];CODE=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(CODE)]
from collect_dock_teacher import DockTeacherEnv
from robo_rl_v2.teacher_legacy.bc_policy import BehaviorCloningPolicy

def rollout(env,seed,policy=None,max_steps=1300):
    obs,_=env.task_reset(seed);stable=0;success=False;collision=False;reason='timeout';t=0.
    for k in range(max_steps):
        if policy is None:
            cmd,label,med,points=env.teacher_action(obs,t);act=np.asarray(label['action'],np.float32)
        else:
            act,_=policy.predict(obs,'dock')
        obs,_,terminated,truncated,info=env.step_teacher(act);t+=env.dt
        if info.get('collision'):
            collision=True;reason='collision';break
        if policy is None and env.teacher.estado in ('fim','abortar'):
            success=env.teacher.estado=='fim';reason='teacher_'+env.teacher.estado+(':'+env.teacher.motivo if env.teacher.estado=='abortar' else '');break
        d=float(np.linalg.norm(env.goal-env.pose[:2]));yaw=abs(math.atan2(math.sin(env.pose[2]),math.cos(env.pose[2])))
        stopped=float(np.linalg.norm(env.velocity))<.02
        stable=stable+1 if d<.03 and yaw<math.radians(2.) and stopped else 0
        if policy is not None and stable>=3:success=True;reason='pose_gap_yaw_stopped';break
        if terminated and info.get('success'):env.done=False
    return dict(seed=seed,success=success,collision=collision,reason=reason,steps=k+1,
        distance_m=float(np.linalg.norm(env.goal-env.pose[:2])),yaw_error_deg=abs(math.degrees(env.pose[2])),
        speed_mps=float(np.linalg.norm(env.velocity[:2])),elapsed_s=(k+1)*env.dt)

def main():
    p=argparse.ArgumentParser();p.add_argument('--checkpoint',required=True);p.add_argument('--episodes',type=int,default=100);p.add_argument('--seed',type=int,default=82000);p.add_argument('--output-dir',required=True);a=p.parse_args()
    out=Path(a.output_dir).resolve();out.mkdir(parents=True,exist_ok=False);policy=BehaviorCloningPolicy(a.checkpoint)
    env=DockTeacherEnv(max_steps=1300);teacher=[];candidate=[]
    for i in range(a.episodes):
        seed=a.seed+i;teacher.append(rollout(env,seed,None));candidate.append(rollout(env,seed,policy))
        if (i+1)%10==0:print(f'evaluated {i+1}/{a.episodes}',flush=True)
    env.close()
    def summary(rows):
        return {'n':len(rows),'successes':sum(r['success'] for r in rows),'success_rate':float(np.mean([r['success'] for r in rows])),
            'collisions':sum(r['collision'] for r in rows),'collision_rate':float(np.mean([r['collision'] for r in rows])),
            'mean_final_distance_m':float(np.mean([r['distance_m'] for r in rows])),
            'mean_yaw_error_deg':float(np.mean([r['yaw_error_deg'] for r in rows])),
            'mean_steps':float(np.mean([r['steps'] for r in rows]))}
    report={'task':'isolated virtual-wall docking','teacher':summary(teacher),'candidate_bc':summary(candidate),
        'paired_seeds':True,'seed_start':a.seed,'episodes':a.episodes,'teacher_uses_actual_encosto_and_face_tracker':True,
        'actuator_model':'same approximate measured motor map as collection','limitations':['not a test of full navigation or Nav2 behavior tree','virtual wall is not an official arena station','actuator map comes from existing robot calibration notes and requires physical confirmation'],
        'teacher_episodes':teacher,'candidate_episodes':candidate}
    (out/'results.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('teacher_episodes','candidate_episodes')},indent=2))
if __name__=='__main__':main()
