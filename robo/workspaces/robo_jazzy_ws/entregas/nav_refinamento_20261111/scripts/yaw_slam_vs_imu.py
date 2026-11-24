#!/usr/bin/env python3
"""Diferenca entre o yaw no mapa (map->base_link, SLAM) e o yaw da IMU (desenrolado), a cada 1 s.
Saltos > limiar em 1 s = correcao (ou erro) do SLAM. Uso: yaw_slam_vs_imu.py <dados> [limiar_graus] [csv]"""
import sys, os, math, bisect, csv
sys.path.insert(0, os.path.dirname(__file__))
from comum import Poses, ler, hora
P = sys.argv[1]; LIM = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
po = Poses(P); imu = ler(P, 'imu__yaw_deg.jsonl'); ti = [d['t'] for d in imu]
uw, ant, ac = [], None, 0.0
for d in imu:
    v = d['yaw_deg']; ac = v if ant is None else ac + (v - ant + 180) % 360 - 180; ant = v; uw.append(ac)
A = lambda a: (a + 180) % 360 - 180
L = []; t = po.t[0]
while t < po.t[-1]:
    x, y, yw = po.em(t)[:3]; i = min(bisect.bisect_left(ti, t), len(ti) - 1)
    L.append((t, x, y, math.degrees(yw), uw[i], A(math.degrees(yw) - uw[i]))); t += 1.0
base = L[0][5]
if len(sys.argv) > 3:
    with open(sys.argv[3], 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['hora', 'x', 'y', 'yaw_mapa', 'yaw_imu', 'mapa_menos_imu', 'desvio_do_inicio'])
        for r in L: w.writerow([hora(r[0])] + [round(v, 2) for v in r[1:]] + [round(A(r[5] - base), 1)])
grandes = 0
for k, (t, x, y, ym, yi, d) in enumerate(L):
    s = A(d - L[k-1][5]) if k else 0
    if abs(s) > LIM: grandes += 1
    if k % 30 == 0 or abs(s) > 12:
        print('%s (%5.2f,%5.2f) yaw_mapa %7.1f imu %7.1f  desvio %6.1f %s' % (hora(t), x, y, A(ym), A(yi), A(d - base),
              '<< salto %+.1f' % s if abs(s) > 12 else ''))
print('saltos > %.0f graus em 1 s: %d de %d s; desvio final %.1f graus; max |desvio| %.1f' % (
    LIM, grandes, len(L), A(L[-1][5] - base), max(abs(A(r[5] - base)) for r in L)))
