#!/usr/bin/env python3
"""Patch SUPERVISOR_LOCALIZATION_20261111. Snapshot, idempotencia e rollback protegido.
--arquivo DIR testa numa arvore copia; nao inicia processos ROS.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import yaml

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[1]
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--desfazer',action='store_true')
p.add_argument('--arquivo',type=Path)
a=p.parse_args()
manifest=json.loads((BASE/'patch_manifest.json').read_text())
operacoes=[]
for item in manifest:
    destino=(a.arquivo/item['nome']) if a.arquivo else (Path('/home/marcos/subir_robo.sh') if item['nome']=='subir_robo.sh' else ROOT/item['nome'])
    atual=destino.read_bytes() if destino.exists() else None
    antigo=(BASE/'backup'/item['nome']).read_bytes() if item['existia'] else None
    novo=(BASE/'aplicado'/item['nome']).read_bytes()
    if hashlib.sha256(novo).hexdigest()!=item['sha256']:
        raise SystemExit('snapshot alterado: '+item['nome'])
    esperado,querido=(novo,antigo) if a.desfazer else (antigo,novo)
    if atual == querido:continue
    if atual != esperado:raise SystemExit('RECUSADO: edicao posterior ou baseline diferente: '+str(destino))
    if querido is not None:
        if destino.suffix=='.py':ast.parse(querido.decode())
        if destino.suffix in ('.yaml','.yml'):yaml.safe_load(querido)
    operacoes.append((destino,querido))
# Validar tudo antes de gravar qualquer arquivo.
for destino,conteudo in operacoes:
    if conteudo is None:destino.unlink()
    else:
        destino.parent.mkdir(parents=True,exist_ok=True)
        destino.write_bytes(conteudo)
print(('Rollback' if a.desfazer else 'Patch')+': '+str(len(operacoes))+' arquivos; nenhum processo iniciado')
print('Apos rollback, recompilar overlay isolado antes de usar seus perfis instalados.')
