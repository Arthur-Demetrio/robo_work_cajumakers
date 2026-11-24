#!/usr/bin/env python3
"""Para cada caminho original reprovado (Patch 01, criterio proposto): a celula LETHAL sob o footprint e
parede do mapa estatico ou obstaculo do obstacle_layer (dinamico)? Uso: origem_colisao.py <dados>"""
import sys, os, bisect, math, csv
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from comum import ler, hora, Poses
import checagem_caminho as cc
pasta = sys.argv[1]
def carregar(n):
    z = np.load(os.path.join(pasta, n)); return z['t'], z['meta'], z['data']
G, S, O = carregar('global_costmap_raw.npz'), carregar('global_static_raw.npz'), carregar('global_obstacle_raw.npz')
def em(L, t):
    i = max(0, bisect.bisect_right(L[0], t) - 1); m = L[1][i]; sx, sy = int(m[0]), int(m[1])
    return L[2][i][:sy, :sx], (sx, sy, float(m[2]), float(m[3]), float(m[4]))
po = Poses(pasta)
falhas = [d['t'] for d in ler(pasta, 'rosout.jsonl') if 'Smoothed path leads to a collision' in d['msg']]
from collections import Counter
cnt = Counter(); out = []
for d in ler(pasta, 'plan.jsonl'):
    pts = d['pts']
    k = bisect.bisect_left(falhas, d['t'] - 0.02)
    if not (k < len(falhas) and falhas[k] - d['t'] < 0.2) or len(pts) < 2:
        continue
    g, meta = em(G, d['t']); gs, ms = em(S, d['t']); go, mo = em(O, d['t'])
    rb = po.em(d['t'])
    ok, i, mot = cc.validar_original(g, meta, pts, rb[0], rb[1], rb[2], pts[-1][2])
    if ok:
        cnt['valido'] += 1; continue
    yaws = cc.yaw_tangente(pts, pts[-1][2])
    est = cc.custo_footprint(gs, ms, pts[i][0], pts[i][1], yaws[i], ignorar_desconhecido=True) >= 254
    obs = cc.custo_footprint(go, mo, pts[i][0], pts[i][1], yaws[i], ignorar_desconhecido=True) >= 254
    # folga: menor deslocamento lateral (ate 10 cm) que livra o footprint nessa pose
    livre_com = None
    for dl in (0.02, 0.04, 0.06, 0.08, 0.10):
        for sg in (1, -1):
            nx = pts[i][0] - sg*dl*math.sin(yaws[i]); ny = pts[i][1] + sg*dl*math.cos(yaws[i])
            if cc.custo_footprint(g, meta, nx, ny, yaws[i], ignorar_desconhecido=True) < 254 and livre_com is None:
                livre_com = dl*sg
    tipo = ('estatico+obstaculo' if est and obs else 'estatico' if est else 'obstaculo' if obs else 'so_composto')
    cnt[tipo] += 1
    s_fim = sum(math.hypot(pts[j][0]-pts[j-1][0], pts[j][1]-pts[j-1][1]) for j in range(i+1, len(pts)))
    out.append(dict(hora=hora(d['t']), x=pts[i][0], y=pts[i][1], yaw_deg=round(math.degrees(yaws[i])),
                    origem=tipo, dist_ao_goal=round(s_fim, 2), deslocamento_que_livra=livre_com, motivo=mot))
print(cnt)
for o in out: print(o)
with open(os.path.join(pasta, '..', 'origem_colisao_original.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(out[0])); w.writeheader(); w.writerows(out)
