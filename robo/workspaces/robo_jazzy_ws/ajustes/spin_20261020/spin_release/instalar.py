#!/usr/bin/env python3
"""Instala alteracoes com hashes, backup e restauracao. Execute com Nav2 parado."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import re
import tempfile
from datetime import datetime
import yaml

HERE=Path(__file__).resolve().parent
MANIFEST=json.loads((HERE/'manifest.json').read_text())
def digest(b): return hashlib.sha256(b).hexdigest()
def atomic(path, content):
    # Resolve symlinks to retain colcon --symlink-install links.
    path=path.resolve()
    fd,name=tempfile.mkstemp(dir=path.parent)
    try:
        os.fchmod(fd,path.stat().st_mode & 0o777)
        with os.fdopen(fd,'wb') as f:
            f.write(content); f.flush(); os.fsync(f.fileno())
        os.replace(name,path)
    finally:
        if os.path.exists(name): os.unlink(name)

def write_batch(ws, changes, stage):
    if not changes:
        print('Ja aplicado; nenhum arquivo alterado.'); return
    backup=ws/'logs'/('spin_v2_'+stage+'_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    backup.mkdir(parents=True)
    entries=[]
    for rel,new in changes.items():
        old=(ws/'src'/rel).read_bytes()
        saved=backup/rel; saved.parent.mkdir(parents=True,exist_ok=True); saved.write_bytes(old)
        entries.append(dict(path=rel,after=digest(new)))
    (backup/'restore.json').write_text(json.dumps(entries,indent=2))
    done=[]
    try:
        for rel,new in changes.items():
            atomic(ws/'src'/rel,new); done.append(rel)
    except Exception:
        for rel in reversed(done): atomic(ws/'src'/rel,(backup/rel).read_bytes())
        raise
    print('APLICADO:',stage,'\nBackup:',backup)
    print('Para desfazer: python3 instalar.py --restore',backup)
    print('Compile os pacotes e reinicie Nav2. Nenhum comando de movimento enviado.')

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--workspace',type=Path,default=Path.home()/'robo_jazzy_ws')
g=p.add_mutually_exclusive_group(required=True)
g.add_argument('--spin',action='store_true'); g.add_argument('--inflacao',action='store_true')
g.add_argument('--restore',type=Path)
a=p.parse_args(); ws=a.workspace.resolve()
try:
    if a.restore:
        backup=a.restore.resolve(); entries=json.loads((backup/'restore.json').read_text())
        changes={}
        for e in entries:
            rel=e['path']
            if Path(rel).is_absolute() or '..' in Path(rel).parts: raise RuntimeError('Caminho invalido')
            current=(ws/'src'/rel).read_bytes()
            if digest(current)!=e['after']: raise RuntimeError('Arquivo mudou depois da instalacao: '+rel)
            changes[rel]=(backup/rel).read_bytes()
        write_batch(ws,changes,'restauracao')
    elif a.spin:
        changes={}
        for rel,item in MANIFEST.items():
            new=(HERE/'payload'/rel).read_bytes()
            if digest(new)!=item['new']: raise RuntimeError('Pacote alterado: '+rel)
            ast.parse(new.decode())
            current=(ws/'src'/rel).read_bytes()
            if digest(current)==item['new']: continue
            if digest(current)!=item['old']: raise RuntimeError('Arquivo diferente do pacote enviado: '+rel)
            changes[rel]=new
        # Do not silently overwrite BT/launch/config changes.
        import xml.etree.ElementTree as ET
        bt=ET.parse(ws/'src/robo_navigation/behavior_trees/navigate_to_pose_smooth.xml')
        escape=bt.findall(".//DriveOnHeading[@server_name='mecanum_escape']")
        if len(escape)!=1 or float(escape[0].get('dist_to_travel'))!=0.06:
            raise RuntimeError('Escape de 6 cm nao confirmado na Smooth')
        write_batch(ws,changes,'spin')
    else:
        rel='robo_navigation/config/nav2_params.yaml'; path=ws/'src'/rel
        old=path.read_text(); data=yaml.safe_load(old)
        import copy
        expected=copy.deepcopy(data)
        for key in ('local_costmap','global_costmap'):
            params=expected[key][key]['ros__parameters']['inflation_layer']
            if params['inflation_radius'] not in (0.55,0.50): raise RuntimeError('Inflacao inesperada: '+key)
            params['inflation_radius']=0.50
        if expected==data:
            print('Inflacao ja esta em 0.50 m.'); raise SystemExit(0)
        new,count=re.subn(r'(?m)^(\s*inflation_radius:)\s*0\.55\s*$',r'\1 0.50',old)
        if count!=2 or yaml.safe_load(new)!=expected: raise RuntimeError('YAML divergiu; nada alterado')
        write_batch(ws,{rel:new.encode()},'inflacao')
except (OSError,ValueError,KeyError,RuntimeError) as exc:
    raise SystemExit('ERRO: '+str(exc))
