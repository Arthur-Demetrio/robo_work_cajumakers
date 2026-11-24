#!/usr/bin/env python3
"""Confere o modelo offline do checker do smoother contra o log: cada /plan_smoothed e casado com o log
'Smoothed path leads to a collision' (mesmo instante) e comparado com o veredito do modelo."""
import sys, os, bisect
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from comum import ler, hora
import checagem_caminho as cc
pasta = sys.argv[1]
cm = np.load(os.path.join(pasta, 'global_costmap_raw.npz'))
ct, cmeta, cdata = cm['t'], cm['meta'], cm['data']
falhas = [d['t'] for d in ler(pasta, 'rosout.jsonl') if 'Smoothed path leads to a collision' in d['msg']]
rec = [d['t'] for d in ler(pasta, 'rosout.jsonl') if 'Received a path to smooth' in d['msg']]
print('pedidos de suavizacao no log:', len(rec), ' falhas:', len(falhas))
from collections import Counter
cnt = Counter()
for d in ler(pasta, 'plan_smoothed.jsonl'):
    i = max(0, bisect.bisect_right(ct, d['t']) - 1)
    sx, sy = int(cmeta[i][0]), int(cmeta[i][1])
    g, meta = cdata[i][:sy, :sx], (sx, sy, float(cmeta[i][2]), float(cmeta[i][3]), float(cmeta[i][4]))
    ok, _ = cc.colisao_estilo_smoother(g, meta, d['pts'], [p[2] for p in d['pts']])
    k = bisect.bisect_left(falhas, d['t'] - 0.05)
    log_falhou = k < len(falhas) and abs(falhas[k] - d['t']) < 0.05
    cnt[('log_falhou' if log_falhou else 'log_ok', 'modelo_colide' if not ok else 'modelo_livre')] += 1
print(cnt)
