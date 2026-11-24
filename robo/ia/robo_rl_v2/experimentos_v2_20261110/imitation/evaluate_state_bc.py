#!/usr/bin/env python3
"""Paired test: pure frozen Encosto vs BC low-level actor under the same FSM/safety guards."""
import argparse,json,sys,hashlib
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[2];CODE=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(CODE)]
from collect_dock_teacher import DockTeacherEnv
from robo_rl_v2.teacher_legacy.state_bc_policy import StateConditionedDockPolicy

def rollout(env,seed,policy=None,max_steps=1300):
 obs,_=env.task_reset(seed);t=0.;fallback=0;success=False;collision=False;reason='timeout'
 for k in range(max_steps):
  teacher_cmd,label,med,points=env.teacher_action(obs,t)
  state=env.teacher.estado
  if state=='fim':success=True;reason='encosto_fim';break
  if state=='abortar':reason='encosto_abort:'+env.teacher.motivo;break
  if policy is None:act=np.asarray(label['action'],np.float32)
  elif state in policy.states:act=policy.predict(obs,state)
  else:fallback+=1;act=np.asarray(label['action'],np.float32)
  obs,_,terminated,truncated,info=env.step_teacher(act);t+=env.dt
  if info.get('collision'):collision=True;reason='collision';break
  if terminated and info.get('success'):env.done=False
 return dict(seed=seed,success=success,collision=collision,reason=reason,steps=k+1,
  final_distance_m=float(np.linalg.norm(env.goal-env.pose[:2])),yaw_error_deg=abs(float(np.rad2deg(env.pose[2]))),
  fallback_steps=fallback,final_pose=env.pose.tolist())

def summarize(rows):return {'n':len(rows),'successes':sum(r['success'] for r in rows),'success_rate':float(np.mean([r['success'] for r in rows])),
 'collisions':sum(r['collision'] for r in rows),'collision_rate':float(np.mean([r['collision'] for r in rows])),
 'mean_final_distance_m':float(np.mean([r['final_distance_m'] for r in rows])),'mean_yaw_error_deg':float(np.mean([r['yaw_error_deg'] for r in rows])),
 'mean_steps':float(np.mean([r['steps'] for r in rows])),'fallback_steps':sum(r['fallback_steps'] for r in rows)}
def main():
 p=argparse.ArgumentParser();p.add_argument('--checkpoint',required=True);p.add_argument('--episodes',type=int,default=100);p.add_argument('--seed',type=int,default=83000);p.add_argument('--output-dir',required=True);a=p.parse_args()
 out=Path(a.output_dir).resolve();out.mkdir(parents=True,exist_ok=False);policy=StateConditionedDockPolicy(a.checkpoint);env=DockTeacherEnv(max_steps=1300);teacher=[];hybrid=[]
 for i in range(a.episodes):
  seed=a.seed+i;teacher.append(rollout(env,seed));hybrid.append(rollout(env,seed,policy))
  if (i+1)%10==0:print(f'evaluated {i+1}/{a.episodes}',flush=True)
 env.close();report={'task':'isolated virtual-wall docking','teacher':summarize(teacher),'hybrid_state_conditioned_bc':summarize(hybrid),
  'paired_seeds':True,'seed_start':a.seed,'episodes':a.episodes,'actor_checkpoint_sha256':hashlib.sha256(Path(a.checkpoint).read_bytes()).hexdigest(),
  'hybrid_definition':'actual Encosto FSM and face tracker retain state transitions and abort guards; BC supplies low-level cmd_vel only for states represented in training; teacher command is fallback for unseen states',
  'not_a_full_nav2_bt_test':True,'teacher_episodes':teacher,'hybrid_episodes':hybrid}
 (out/'results.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('teacher_episodes','hybrid_episodes')},indent=2))
if __name__=='__main__':main()
