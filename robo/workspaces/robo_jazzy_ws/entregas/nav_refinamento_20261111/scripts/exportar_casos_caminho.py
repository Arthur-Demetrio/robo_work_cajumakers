#!/usr/bin/env python3
"""Exporta, para cada /plan cuja suavizacao falhou, o costmap global gravado + caminho + pose do robo,
no formato do avaliar_casos_caminho (C++). Uso: exportar_casos_caminho.py <dados> <pasta_casos>"""
import sys, os, bisect
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from comum import ler, hora, Poses
pasta, saida = sys.argv[1], sys.argv[2]
os.makedirs(saida, exist_ok=True)
cm = np.load(os.path.join(pasta, 'global_costmap_raw.npz'))
ct, cmeta, cdata = cm['t'], cm['meta'], cm['data']
po = Poses(pasta)
falhas = [d['t'] for d in ler(pasta, 'rosout.jsonl') if 'Smoothed path leads to a collision' in d['msg']]
idx = []
for n, d in enumerate(ler(pasta, 'plan.jsonl')):
    k = bisect.bisect_left(falhas, d['t'] - 0.02)
    if not (k < len(falhas) and falhas[k] - d['t'] < 0.2) or len(d['pts']) < 2:
        continue
    i = max(0, bisect.bisect_right(ct, d['t']) - 1)
    sx, sy = int(cmeta[i][0]), int(cmeta[i][1])
    rb = po.em(d['t'])
    nome = '%03d' % n
    with open(os.path.join(saida, nome + '.txt'), 'w') as f:
        f.write('%d %d %.6f %.6f %.6f\n' % (sx, sy, cmeta[i][2], cmeta[i][3], cmeta[i][4]))
        f.write('%.4f %.4f %.5f %.5f\n%d\n' % (rb[0], rb[1], rb[2], d['pts'][-1][2], len(d['pts'])))
        for p in d['pts']:
            f.write('%.4f %.4f %.5f\n' % tuple(p))
        f.write('0.30 0.25 0.10\n')
    np.ascontiguousarray(cdata[i][:sy, :sx]).tofile(os.path.join(saida, nome + '.bin'))
    idx.append((nome, hora(d['t']), '%.2f,%.2f' % (rb[0], rb[1]), '%.2f,%.2f' % tuple(d['pts'][-1][:2])))
with open(os.path.join(saida, 'indice.tsv'), 'w') as f:
    for l in idx:
        f.write('\t'.join(l) + '\n')
print(len(idx), 'casos')
