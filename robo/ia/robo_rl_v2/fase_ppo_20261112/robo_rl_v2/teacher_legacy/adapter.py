"""Actual pure alignment core, isolated from ROS. No training/motor side effects."""
import ast,json,math,hashlib,types,sys
from pathlib import Path
import numpy as np
HERE=Path(__file__).parent
_CORE=None

def load_core():
 global _CORE
 if _CORE is not None:return _CORE
 manifest=json.loads((HERE/'provenance.json').read_text())
 for record in manifest['files'].values():
  path=HERE/'snapshot'/record['snapshot']
  if hashlib.sha256(path.read_bytes()).hexdigest()!=record['sha256']:raise ValueError(f'Snapshot alterado: {path.name}')
 path=HERE/'snapshot/alinhador_passagem.py';tree=ast.parse(path.read_text())
 for node in ast.walk(tree):
  if isinstance(node,ast.ImportFrom) and node.module in ('face_ws','giro_frontal'):
   node.module='robo_rl_v2.teacher_legacy.snapshot.'+node.module
 module=types.ModuleType('robo_rl_v2.teacher_legacy.frozen_core');module.__file__=str(path);module.__package__='robo_rl_v2.teacher_legacy'
 sys.modules[module.__name__]=module
 exec(compile(ast.fix_missing_locations(tree),str(path),'exec'),module.__dict__)
 _CORE=module;return module

def points_from_scan(ranges_m,angles_rad,valid,lidar_xy=(.33,.005),lidar_yaw=0.):
 ranges=np.asarray(ranges_m,float);angles=np.asarray(angles_rad,float);valid=np.asarray(valid,bool)
 if ranges.ndim!=1 or ranges.shape!=angles.shape or valid.shape!=ranges.shape:raise ValueError('Shapes de scan incompatíveis')
 keep=valid&np.isfinite(ranges)&np.isfinite(angles)&(ranges>0)
 return np.c_[np.cos(angles[keep]+lidar_yaw)*ranges[keep]+lidar_xy[0],np.sin(angles[keep]+lidar_yaw)*ranges[keep]+lidar_xy[1]].tolist()

def command_label(command,limits=(.2,.08,.16),reverse=.16,max_xy=.2):
 command=np.asarray(command,float)
 if command.shape!=(3,) or not np.isfinite(command).all():raise ValueError('Comando inválido')
 scales=np.asarray(limits,float).copy()
 if command[0]<0:scales[0]=reverse
 action=command/scales
 representable=bool(np.max(np.abs(action))<=1+1e-9 and np.linalg.norm(command[:2])<=max_xy+1e-9)
 # Unrepresentable commands are recorded, never silently clipped into training targets.
 return dict(command_mps_rps=command.tolist(),action=action.tolist(),representable=representable)

class DemonstrationWriter:
 def __init__(self,path):self.fp=Path(path).open('x');self.count=0
 def append(self,observation,command,phase,state,timestamp,**measured_context):
  obs=np.asarray(observation,float)
  if obs.shape!=(152,) or not np.isfinite(obs).all():raise ValueError('Observação deve ter 152 valores finitos')
  if not math.isfinite(timestamp):raise ValueError('Timestamp inválido')
  record=dict(schema=1,observation=obs.tolist(),teacher=command_label(command),phase=str(phase),state=str(state),timestamp=float(timestamp),context=measured_context)
  self.fp.write(json.dumps(record,allow_nan=False)+'\n');self.fp.flush();self.count+=1;return record
 def close(self):self.fp.close()
 def __enter__(self):return self
 def __exit__(self,*_):self.close()
