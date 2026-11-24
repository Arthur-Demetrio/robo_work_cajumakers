#!/usr/bin/env python3
"""Restaura fontes preservando copia do estado removido. Recusa sobrescrever edicoes posteriores."""
import argparse,datetime,hashlib,json,shutil
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--workspace',type=Path,default=Path(__file__).resolve().parents[2]);ap.add_argument('--verificar',action='store_true');a=ap.parse_args()
w=a.workspace.resolve();rel=Path('entregas/correcao_partida_20261111');m=json.loads((w/rel/'rollback_manifest.json').read_text())
for f,digest in m['esperado'].items():
 p=w/f
 if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=digest:
  raise SystemExit('RECUSADO: arquivo alterado depois da refatoracao: '+f)
for f,digest in m['originais'].items():
 p=w/m['backup']/f
 if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:raise SystemExit('RECUSADO: backup divergente: '+f)
if a.verificar:print('Rollback conferido; nenhum arquivo alterado.');raise SystemExit(0)
saved=w/'backups'/('antes_rollback_partida_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S'));saved.mkdir(parents=True)
for f in m['esperado']:
 dst=saved/f;dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(w/f,dst)
for f in m['originais']:shutil.copy2(w/m['backup']/f,w/f)
for f in m['novos']:(w/f).unlink()
print('Rollback local concluido; estado removido preservado em',saved)
