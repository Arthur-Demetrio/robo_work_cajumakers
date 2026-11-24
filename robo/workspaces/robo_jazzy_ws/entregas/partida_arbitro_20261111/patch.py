#!/usr/bin/env python3
"""PARTIDA_ARBITRO_20261111: snapshot idempotente e rollback guardado; sem processos ROS."""
import argparse,ast,hashlib,json,subprocess
from pathlib import Path
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--desfazer',action='store_true');p.add_argument('--arquivo',type=Path)
a=p.parse_args();b=Path(__file__).resolve().parent;ops=[]
for item in json.loads((b/'manifest.json').read_text()):
 dest=(a.arquivo/Path(item['destino']).name) if a.arquivo else Path(item['destino'])
 antigo=(b/'backup'/item['backup']).read_bytes() if item['backup'] else None
 novo=(b/'aplicado'/item['snapshot']).read_bytes()
 if hashlib.sha256(novo).hexdigest()!=item['sha256']:raise SystemExit('Snapshot alterado')
 esperado,querido=(novo,antigo) if a.desfazer else (antigo,novo)
 atual=dest.read_bytes() if dest.exists() else None
 if atual==querido:continue
 if atual!=esperado:raise SystemExit('RECUSADO: edicao posterior em '+str(dest))
 if querido is not None:
  if dest.suffix=='.py':ast.parse(querido.decode())
  if dest.suffix=='.sh':subprocess.run(['bash','-n'],input=querido,check=True)
 ops.append((dest,querido))
for dest,conteudo in ops:
 if conteudo is None:dest.unlink()
 else:dest.parent.mkdir(parents=True,exist_ok=True);dest.write_bytes(conteudo)
print(('Rollback' if a.desfazer else 'Patch')+': '+str(len(ops))+' arquivos; nenhum processo ROS iniciado/encerrado')
