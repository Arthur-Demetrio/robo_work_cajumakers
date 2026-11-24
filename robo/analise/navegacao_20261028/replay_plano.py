#!/usr/bin/env python3
"""Refaz a analise de passagens do alinhador (planejar_etapas) em cada /plan longo do bag.

Uso: python3 replay_plano.py bag_0.mcap      (AL=outro_alinhador.py para comparar versoes)
"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bagio import open_bag, deser
from _comum import hms, alinhador

al = alinhador()
ts, it = open_bag(sys.argv[1], topics=['/plan', '/map'])
mapa = None
def yaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
for top, t, d, typ in it():
    m = deser(ts, d, typ)
    if top == '/map':
        i = m.info
        mapa = al.Mapa(i.resolution, i.width, i.height, i.origin.position.x, i.origin.position.y, yaw(i.origin.orientation), list(m.data))
    elif len(m.poses) > 100 and mapa is not None:
        pts = [(p.pose.position.x, p.pose.position.y) for p in m.poses]
        et, info = al.planejar_etapas(mapa, pts, pts[-1])
        if isinstance(info, dict):
            print(hms(t), 'entrada (%.2f,%.2f) eixo %.0f graus, preparo a %.2f m, curva: %s' % (
                info['entrada'][0], info['entrada'][1], info['eixo_graus'], info['distancia_preparo'], 'sim' if info.get('curva') else 'nao'))
        else:
            print(hms(t), info)
