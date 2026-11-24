#!/usr/bin/env python3
"""SUBIDA_SUPERVISOR_20261111: aplicar/desfazer em arquivos ou --arquivo DIR copia."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--desfazer',action='store_true');p.add_argument('--arquivo',type=Path)
a=p.parse_args();b=Path(__file__).resolve().parent
ops=[]
for i in json.loads((b/'manifest.json').read_text()):
 dest=(a.arquivo/Path(i['destino']).name) if a.arquivo else Path(i['destino'])
 antigo=(b/'backup'/i['backup']).read_bytes();novo=(b/'aplicado'/i['backup']).read_bytes()
 assert hashlib.sha256(novo).hexdigest()==i['sha256']
 esperado,querido=(novo,antigo) if a.desfazer else (antigo,novo)
 atual=dest.read_bytes()
 if atual==querido:continue
 if atual!=esperado:raise SystemExit('RECUSADO: edicao posterior em '+str(dest))
 if dest.suffix=='.sh':subprocess.run(['bash','-n'],input=querido,check=True)
 ops.append((dest,querido))
for dest,conteudo in ops:dest.write_bytes(conteudo)
print(('Rollback' if a.desfazer else 'Patch')+': '+str(len(ops))+' arquivos; nenhum processo iniciado')
