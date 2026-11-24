#!/usr/bin/env python3
"""Onde o smoother acusou colisao: perto do robo (inicio), do fim do caminho (goal) ou no meio.
Uso: python3 colisoes_smoother.py <dados_extraidos> [saida.csv]"""
import sys, re, math, csv, bisect
sys.path.insert(0, __import__('os').path.dirname(__file__))
from comum import ler, hora, Poses

pasta = sys.argv[1]
po = Poses(pasta)
planos = ler(pasta, 'plan.jsonl')
tp = [p['t'] for p in planos]
rx = re.compile(r'collision at x: ([-\d.]+), y: ([-\d.]+), theta: ([-\d.]+)')
linhas = []
for d in ler(pasta, 'rosout.jsonl'):
    m = rx.search(d['msg'])
    if not m:
        continue
    x, y, th = map(float, m.groups())
    rb = po.em(d['t'])
    i = bisect.bisect_right(tp, d['t'] + 0.05) - 1
    pl = planos[i]['pts'] if i >= 0 else []
    d_rob = math.hypot(x - rb[0], y - rb[1])
    d_fim = math.hypot(x - pl[-1][0], y - pl[-1][1]) if pl else float('nan')
    # posicao ao longo do plano
    s_tot, s_col, best = 0.0, 0.0, 1e9
    for k in range(1, len(pl)):
        s_tot += math.hypot(pl[k][0]-pl[k-1][0], pl[k][1]-pl[k-1][1])
        dd = math.hypot(pl[k][0]-x, pl[k][1]-y)
        if dd < best:
            best, s_col = dd, s_tot
    zona = 'inicio(robo)' if d_rob < 0.30 else ('fim(goal)' if d_fim < 0.45 else 'meio')
    linhas.append(dict(hora=hora(d['t']), x=x, y=y, theta=th, robo_x=round(rb[0], 3), robo_y=round(rb[1], 3),
                       dist_robo=round(d_rob, 3), dist_fim=round(d_fim, 3), s_colisao=round(s_col, 2),
                       comprimento=round(s_tot, 2), zona=zona))
from collections import Counter
print(Counter(l['zona'] for l in linhas), 'total', len(linhas))
if len(sys.argv) > 2:
    with open(sys.argv[2], 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(linhas[0].keys())); w.writeheader(); w.writerows(linhas)
for l in linhas:
    if l['zona'] == 'meio':
        print(l)
