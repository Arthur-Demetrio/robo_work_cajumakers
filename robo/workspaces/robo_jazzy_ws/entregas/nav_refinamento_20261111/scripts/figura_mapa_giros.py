#!/usr/bin/env python3
"""Figura: mapa estatico (costmap global, camada estatica), trajetoria map->base_link, goals e giros >=45 graus.
Uso: figura_mapa_giros.py <dados> <TABELA_GIROS.csv> <LINHA_DO_TEMPO.csv> <saida.png>"""
import sys, os, csv, math, re
import numpy as np
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
sys.path.insert(0, os.path.dirname(__file__))
from comum import Poses
pasta, tg, tl, out = sys.argv[1:5]
z = np.load(os.path.join(pasta, 'global_static_raw.npz'))
mt = z['meta'][-1]; sx, sy, res, ox, oy = int(mt[0]), int(mt[1]), mt[2], mt[3], mt[4]
g = z['data'][-1][:sy, :sx].astype(float)
img = np.where(g == 254, 0.0, np.where(g == 255, 0.75, 1.0))
fig, ax = plt.subplots(figsize=(8, 9))
ax.imshow(img, cmap='gray', origin='lower', extent=[ox, ox + sx * res, oy, oy + sy * res], vmin=0, vmax=1)
po = Poses(pasta)
xs = [p[0] for p in po.p[::5]]; ys = [p[1] for p in po.p[::5]]
ax.plot(xs, ys, '-', color='#3b6fb6', lw=1.2, label='trajetoria (map)')
cores = {'alinhador VIRAR_CAMINHO (pre-giro)': '#d9822b', 'DWB (Nav2 controller)': '#c23b3b',
         'alinhador TRAJETO_CURTO (yaw final)': '#2e8b57'}
vistos = set()
for r in csv.DictReader(open(tg)):
    h = r['hora']
    import datetime
    t = datetime.datetime.strptime('2026-11-10 ' + h, '%Y-%m-%d %H:%M:%S.%f').timestamp()
    x, y, _ = po.em(t)[:3]
    c = cores.get(r['origem'], '#777777')
    lab = r['origem'] if r['origem'] not in vistos else None
    vistos.add(r['origem'])
    ax.plot(x, y, 'o', color=c, ms=9, label=lab)
    ax.annotate('%s %+d°' % (h[:8], int(float(r['delta_imu_graus']))), (x, y), fontsize=7, xytext=(5, 4),
                textcoords='offset points', color=c)
for r in csv.DictReader(open(tl)):
    m = re.search(r'Novo goal: x=([-\d.]+) y=([-\d.]+)', r['detalhe'])
    if m and r['evento'].startswith('GOAL_NOVO'):
        ax.plot(float(m.group(1)), float(m.group(2)), '*', color='k', ms=11)
        ax.annotate(r['hora'][:8], (float(m.group(1)), float(m.group(2))), fontsize=7, xytext=(-30, -10),
                    textcoords='offset points')
ax.plot([], [], '*', color='k', label='goal do operador')
ax.set_xlabel('x mapa (m)'); ax.set_ylabel('y mapa (m)')
ax.set_title('2nav 10/11 12:33: giros >= 45 graus por origem')
ax.legend(fontsize=7, loc='upper right'); ax.set_aspect('equal')
xs_all = xs + [0]; ax.set_xlim(min(xs) - 0.8, max(xs) + 0.8); ax.set_ylim(min(ys) - 0.8, max(ys) + 0.8)
fig.tight_layout(); fig.savefig(out, dpi=130)
print('ok', out)
