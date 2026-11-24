#!/usr/bin/env python3
"""Replay offline: para cada /plan do bag, aplica as checagens (Nav2 e proposta) no costmap global gravado.
Uso: replay_caminhos.py <dados_extraidos> [saida.csv]"""
import sys, os, csv, bisect, math
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from comum import ler, hora, Poses
import checagem_caminho as cc

pasta = sys.argv[1]
cm = np.load(os.path.join(pasta, 'global_costmap_raw.npz'))
ct, cmeta, cdata = cm['t'], cm['meta'], cm['data']
po = Poses(pasta)
falhas = [d['t'] for d in ler(pasta, 'rosout.jsonl') if 'Smoothed path leads to a collision' in d['msg']]

def grade_em(t):
    i = max(0, bisect.bisect_right(ct, t) - 1)
    sx, sy = int(cmeta[i][0]), int(cmeta[i][1])
    return cdata[i][:sy, :sx], (sx, sy, float(cmeta[i][2]), float(cmeta[i][3]), float(cmeta[i][4]))

linhas = []
# 1) consistencia do modelo: caminhos suavizados aceitos nao devem colidir no modelo do smoother
inconsist = 0
for d in ler(pasta, 'plan_smoothed.jsonl'):
    g, meta = grade_em(d['t'])
    ok, i = cc.colisao_estilo_smoother(g, meta, d['pts'], [p[2] for p in d['pts']])
    inconsist += (not ok)
print('plan_smoothed aceitos pelo Nav2 e reprovados no modelo do smoother: %d de %d' %
      (inconsist, len(ler(pasta, 'plan_smoothed.jsonl'))))
# 2) caminhos originais
for d in ler(pasta, 'plan.jsonl'):
    pts = d['pts']
    if len(pts) < 2:
        continue
    g, meta = grade_em(d['t'])
    rb = po.em(d['t'])
    k = bisect.bisect_left(falhas, d['t'] - 0.02)
    suav_falhou = k < len(falhas) and falhas[k] - d['t'] < 0.2
    ipv, ipv_i = cc.is_path_valid_jazzy(g, meta, pts, rb[0], rb[1])
    full, full_i = cc.colisao_estilo_smoother(g, meta, pts, cc.yaw_tangente(pts, pts[-1][2]))
    prop, prop_i, motivo = cc.validar_original(g, meta, pts, rb[0], rb[1], rb[2], pts[-1][2])
    linhas.append(dict(hora=hora(d['t']), n=len(pts), robo=f'{rb[0]:.2f},{rb[1]:.2f}',
                       goal=f'{pts[-1][0]:.2f},{pts[-1][1]:.2f}', suavizacao_falhou=suav_falhou,
                       ispathvalid_jazzy=ipv, ipv_idx=ipv_i, original_sem_colisao_total=full, total_idx=full_i,
                       proposta_valida=prop, prop_idx=prop_i, motivo=motivo))
from collections import Counter
print('planos:', len(linhas))
print('suavizacao falhou:', sum(l['suavizacao_falhou'] for l in linhas))
for chave in ('ispathvalid_jazzy', 'original_sem_colisao_total', 'proposta_valida'):
    print(chave, Counter((l['suavizacao_falhou'], l[chave]) for l in linhas))
if len(sys.argv) > 2:
    with open(sys.argv[2], 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(linhas[0])); w.writeheader(); w.writerows(linhas)
for l in linhas:
    if l['suavizacao_falhou'] and not l['proposta_valida']:
        print('PROPOSTA REPROVA:', l['hora'], l['robo'], '->', l['goal'], l['motivo'])
