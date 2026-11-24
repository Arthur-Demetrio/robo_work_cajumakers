#!/usr/bin/env python3
"""Saida lateral: medida do alinhador (parede da frente/tras) x folga real do corpo, scan a scan.

Uso: python3 folgas_laterais.py saida.pkl 10:50:26 10:50:51 LADO   (LADO = 1 esquerda, -1 direita)
Colunas: angulo/desvio/paredes/frente de medir_corredor_lateral; folgas_lateral (frente, tras,
P = parede de verdade); centro pelas folgas e PARADA DE SEGURANCA como na TravessiaLateral.
"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _comum import carregar, ep, hms, pontos, alinhador

al = alinhador()
D = carregar(sys.argv[1])
t0, t1, lado = ep(D, sys.argv[2]), ep(D, sys.argv[3]), float(sys.argv[4])
tl = al.TravessiaLateral(lado, log=lambda m: None)
g = lambda v: '%5.1f' % (v * 100) if v is not None else '  -  '
print('hora        ang   desvio lados frente | folga fr  tras  P P | centro  seguranca')
for sc in D['scan']:
    t = sc[0]
    if not t0 <= t <= t1:
        continue
    pts = pontos(sc)
    m = al.medir_corredor_lateral(pts, lado, 0.82, 0.08)
    f = al.folgas_lateral(pts, lado)
    tl.nova_folga(t, *f)
    c = tl._centro_folga(t)
    med = ('%+5.1f %+6.1f  %d   %s' % (math.degrees(m['angulo']), 100 * m['desvio'], m['lados'], g(m['frente']))) if m else '   sem medida        '
    print(hms(t), med, '|', g(f[0]), g(f[1]), 'P' if f[2] else '.', 'P' if f[3] else '.', '|',
          ('%+5.1f' % (c * 100)) if c is not None else '  -  ', ' SEGURANCA' if tl._perigo(t) is not None else '')
