"""Inference for docking behavior cloned with an explicit Encosto FSM state."""
from pathlib import Path
import sys
import numpy as np
import torch
_HERE=Path(__file__).resolve();_TRAINER=_HERE.parents[2]/'experimentos_v2_20261110'/'imitation'
if str(_TRAINER) not in sys.path:sys.path.insert(0,str(_TRAINER))
from train_state_bc import StateActor
from train_bc import PHASES,OBS
class StateConditionedDockPolicy:
 def __init__(self,checkpoint):
  self.path=Path(checkpoint).resolve();d=torch.load(self.path,map_location='cpu',weights_only=True)
  if tuple(d.get('phases',()))!=PHASES or d.get('observation_size')!=OBS:raise ValueError('Checkpoint BC incompatível')
  self.states=tuple(d['states']);self.actor=StateActor(len(self.states)).eval();self.actor.load_state_dict(d['state_dict'],strict=True)
 @torch.no_grad()
 def predict(self,observation,state,phase='dock'):
  if phase not in PHASES:raise ValueError(f'Fase inválida: {phase}')
  if state not in self.states:raise KeyError(f'Estado não treinado: {state}')
  x=np.asarray(observation,np.float32)
  if x.shape!=(OBS,) or not np.isfinite(x).all():raise ValueError(f'Observação deve conter {OBS} valores finitos')
  ph=np.zeros((1,len(PHASES)),np.float32);ph[0,PHASES.index(phase)]=1
  st=np.zeros((1,len(self.states)),np.float32);st[0,self.states.index(state)]=1
  return self.actor(torch.from_numpy(x[None]),torch.from_numpy(ph),torch.from_numpy(st))[0].numpy().astype(np.float32)
