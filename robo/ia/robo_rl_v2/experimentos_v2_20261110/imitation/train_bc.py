#!/usr/bin/env python3
"""Train a phase-conditioned behavioral-cloning candidate from teacher JSONL.

This is deliberately separate from PPO. It never edits a source model and refuses
an existing output directory. Dataset rows use teacher_legacy.DemonstrationWriter.
"""
from __future__ import annotations
import argparse, hashlib, json, math, random
from pathlib import Path
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset

PHASES = ("navigate", "prepare", "traverse", "dock", "recover")
OBS = 152
class Actor(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(nn.Linear(OBS + len(PHASES), 256), nn.Tanh(),
            nn.Linear(256, 256), nn.Tanh(), nn.Linear(256, 3), nn.Tanh())
    def forward(self, obs, phase):
        return self.net(torch.cat((obs, phase), dim=-1))

def read_dataset(path: Path, allowed_episodes=None):
    rows=[]
    for line_no,line in enumerate(path.read_text().splitlines(),1):
        try:r=json.loads(line)
        except Exception as e:raise ValueError(f"JSON inválido linha {line_no}: {e}")
        if r.get('schema') != 1: raise ValueError(f"schema não suportado linha {line_no}")
        phase=r.get('phase')
        if phase not in PHASES: raise ValueError(f"fase desconhecida linha {line_no}: {phase!r}")
        obs=np.asarray(r.get('observation'),np.float32)
        teacher=r.get('teacher',{})
        action=np.asarray(teacher.get('action'),np.float32)
        if obs.shape!=(OBS,) or action.shape!=(3,) or not np.isfinite(obs).all() or not np.isfinite(action).all():
            raise ValueError(f"dimensão/valor inválido linha {line_no}")
        if not teacher.get('representable',False) or np.max(np.abs(action))>1.000001:
            continue
        episode=r.get('context',{}).get('episode_id')
        if episode is None: raise ValueError(f"faltou context.episode_id linha {line_no}; split por amostra vazaria episódios")
        if allowed_episodes is not None and str(episode) not in allowed_episodes: continue
        if not math.isfinite(float(r.get('timestamp',float('nan')))): raise ValueError(f"timestamp inválido linha {line_no}")
        rows.append((obs,action,phase,str(episode)))
    if len(rows)<1000: raise ValueError(f"dataset tem {len(rows)} rótulos representáveis; mínimo operacional: 1000")
    return rows

def split_groups(rows, seed):
    groups=sorted({r[3] for r in rows});random.Random(seed).shuffle(groups)
    nval=max(1,round(len(groups)*.2));val=set(groups[:nval])
    tr=[r for r in rows if r[3] not in val];va=[r for r in rows if r[3] in val]
    if len(tr)<100 or len(va)<50: raise ValueError("split por episódio insuficiente; forneça mais episódios independentes")
    missing=set(r[2] for r in rows)-set(r[2] for r in va)
    if missing: raise ValueError(f"validação sem fases: {sorted(missing)}; grave episódios independentes de cada fase")
    return tr,va

def tensors(rows):
    x=np.stack([r[0] for r in rows]); y=np.stack([r[1] for r in rows]); p=np.zeros((len(rows),len(PHASES)),np.float32)
    for i,r in enumerate(rows):p[i,PHASES.index(r[2])]=1
    return TensorDataset(torch.from_numpy(x),torch.from_numpy(p),torch.from_numpy(y))

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--dataset',required=True);ap.add_argument('--output',required=True)
    ap.add_argument('--episode-manifest',help='collection.json; use junto de --success-only')
    ap.add_argument('--success-only',action='store_true',help='treine apenas episódios marcados como sucesso no manifesto')
    ap.add_argument('--epochs',type=int,default=80);ap.add_argument('--batch-size',type=int,default=256);ap.add_argument('--seed',type=int,default=42)
    a=ap.parse_args();src=Path(a.dataset).resolve();out=Path(a.output).resolve()
    if not src.is_file():ap.error(f"dataset não encontrado: {src}")
    allowed=None
    if a.success_only:
        if not a.episode_manifest:ap.error('--success-only exige --episode-manifest')
        manifest=json.loads(Path(a.episode_manifest).read_text())
        allowed={f"dock-{r['seed']}" for r in manifest['episodes'] if r.get('success')}
        if not allowed:ap.error('manifesto não tem episódios bem-sucedidos')
    out.mkdir(parents=True,exist_ok=False)
    torch.manual_seed(a.seed);np.random.seed(a.seed);random.seed(a.seed);torch.set_num_threads(1)
    rows=read_dataset(src,allowed_episodes=allowed);tr,va=split_groups(rows,a.seed)
    train=DataLoader(tensors(tr),batch_size=a.batch_size,shuffle=True)
    valid=DataLoader(tensors(va),batch_size=a.batch_size,shuffle=False)
    model=Actor();opt=torch.optim.AdamW(model.parameters(),lr=3e-4,weight_decay=1e-5);loss_fn=nn.MSELoss()
    best=float('inf');best_epoch=0;history=[]
    for epoch in range(1,a.epochs+1):
        model.train();tl=[]
        for x,p,y in train:
            opt.zero_grad(set_to_none=True);loss=loss_fn(model(x,p),y);loss.backward();nn.utils.clip_grad_norm_(model.parameters(),1.0);opt.step();tl.append(loss.item())
        model.eval();vl=[];sq=0.;n=0
        with torch.no_grad():
            for x,p,y in valid:
                pred=model(x,p);vl.append(loss_fn(pred,y).item());sq+=float(((pred-y)**2).sum());n+=y.numel()
        record={'epoch':epoch,'train_mse':float(np.mean(tl)),'validation_mse':float(np.mean(vl)),'validation_rmse':math.sqrt(sq/n)};history.append(record)
        if record['validation_mse']<best:
            best=record['validation_mse'];best_epoch=epoch
            torch.save({'state_dict':model.state_dict(),'phases':PHASES,'observation_size':OBS,'seed':a.seed},out/'actor.pt')
    digest=hashlib.sha256(src.read_bytes()).hexdigest()
    report={'candidate':'phase_conditioned_behavior_cloning','dataset':str(src),'dataset_sha256':digest,'success_only':a.success_only,'episode_manifest':a.episode_manifest,'rows_total':len(rows),'rows_train':len(tr),'rows_validation':len(va),'episodes_total':len({r[3] for r in rows}),'phase_counts':{p:sum(r[2]==p for r in rows) for p in PHASES},'best_epoch':best_epoch,'best_validation_mse':best,'best_validation_rmse':math.sqrt(best),'checkpoint':'actor.pt','history':history}
    (out/'report.json').write_text(json.dumps(report,indent=2))
    print(json.dumps({k:v for k,v in report.items() if k!='history'},indent=2))
if __name__=='__main__':main()
