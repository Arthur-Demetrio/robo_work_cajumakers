#!/usr/bin/env python3
"""Train a docking skill actor conditioned on the actual Encosto FSM state."""
import argparse,hashlib,json,math,random
from pathlib import Path
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader,TensorDataset
from train_bc import PHASES,OBS,split_groups
class StateActor(nn.Module):
 def __init__(self,nstates):
  super().__init__();self.net=nn.Sequential(nn.Linear(OBS+len(PHASES)+nstates,256),nn.Tanh(),nn.Linear(256,256),nn.Tanh(),nn.Linear(256,3),nn.Tanh())
 def forward(self,obs,phase,state):return self.net(torch.cat((obs,phase,state),dim=-1))
def main():
 p=argparse.ArgumentParser();p.add_argument('--dataset',required=True);p.add_argument('--episode-manifest',required=True);p.add_argument('--output',required=True);p.add_argument('--epochs',type=int,default=100);p.add_argument('--seed',type=int,default=84);a=p.parse_args()
 src=Path(a.dataset).resolve();man=json.loads(Path(a.episode_manifest).read_text());allowed={f"dock-{r['seed']}" for r in man['episodes'] if r.get('success')}
 kept=[];states=set()
 for line_no,line in enumerate(src.read_text().splitlines(),1):
  r=json.loads(line);ctx=r.get('context',{});ep=ctx.get('episode_id')
  if ep not in allowed:continue
  if r.get('schema')!=1 or r.get('phase') not in PHASES:raise ValueError(f'linha/schema/fase inválidos: {line_no}')
  obs=np.asarray(r.get('observation'),np.float32);target=r.get('teacher',{});act=np.asarray(target.get('action'),np.float32)
  if obs.shape!=(OBS,) or act.shape!=(3,) or not np.isfinite(obs).all() or not np.isfinite(act).all():raise ValueError(f'observação/ação inválida linha {line_no}')
  if not target.get('representable',False) or np.max(np.abs(act))>1.000001:continue
  st=str(r.get('state',''))
  if not st:raise ValueError(f'estado do alinhador ausente linha {line_no}')
  states.add(st);kept.append((obs,act,r['phase'],str(ep),st))
 if len(kept)<1000:raise ValueError(f'poucos rótulos de episódios bem-sucedidos: {len(kept)}')
 states=tuple(sorted(states))
 tr,va=split_groups(kept,a.seed)
 def tensors(data):
  x=np.stack([r[0] for r in data]);y=np.stack([r[1] for r in data]);ph=np.zeros((len(data),len(PHASES)),np.float32);st=np.zeros((len(data),len(states)),np.float32)
  for i,r in enumerate(data):ph[i,PHASES.index(r[2])]=1.;st[i,states.index(r[4])]=1.
  return TensorDataset(torch.from_numpy(x),torch.from_numpy(ph),torch.from_numpy(st),torch.from_numpy(y))
 out=Path(a.output).resolve();out.mkdir(parents=True,exist_ok=False);torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed);torch.set_num_threads(1)
 train=DataLoader(tensors(tr),batch_size=256,shuffle=True);valid=DataLoader(tensors(va),batch_size=256);model=StateActor(len(states));opt=torch.optim.AdamW(model.parameters(),lr=3e-4,weight_decay=1e-5);lossfn=nn.MSELoss();best=1e9;best_epoch=0;hist=[]
 for epoch in range(1,a.epochs+1):
  model.train();tl=[]
  for x,ph,st,y in train:opt.zero_grad(set_to_none=True);loss=lossfn(model(x,ph,st),y);loss.backward();nn.utils.clip_grad_norm_(model.parameters(),1.);opt.step();tl.append(float(loss))
  model.eval();vl=[];sq=0.;n=0
  with torch.no_grad():
   for x,ph,st,y in valid:
    pred=model(x,ph,st);vl.append(float(lossfn(pred,y)));sq+=float(((pred-y)**2).sum());n+=y.numel()
  rec={'epoch':epoch,'train_mse':float(np.mean(tl)),'validation_mse':float(np.mean(vl)),'validation_rmse':math.sqrt(sq/n)};hist.append(rec)
  if rec['validation_mse']<best:
   best=rec['validation_mse'];best_epoch=epoch;torch.save({'state_dict':model.state_dict(),'phases':PHASES,'observation_size':OBS,'states':states,'seed':a.seed},out/'actor.pt')
 rep={'candidate':'state_conditioned_hybrid_action_clone','dataset_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'successful_episodes_only':True,'episodes_used':len({r[3] for r in kept}),'train_rows':len(tr),'validation_rows':len(va),'state_vocab':states,'phase':'dock','best_epoch':best_epoch,'best_validation_mse':best,'best_validation_rmse':math.sqrt(best),'history':hist}
 (out/'report.json').write_text(json.dumps(rep,indent=2));print(json.dumps({k:v for k,v in rep.items() if k!='history'},indent=2))
if __name__=='__main__':main()
