#!/usr/bin/env python3
"""ROTA_MAPA x obstaculo dinamico (Patch 04): recalcula a rota do mapa do alinhador (rota_no_mapa) no /map
gravado e confere, no obstacle_layer do costmap global gravado, se ha obstaculo do momento perto dela.
Uso: rota_mapa_vs_costmap.py <bag> <dados> <hora HH:MM:SS.ss> <goal_x> <goal_y>"""
import sys, os, math, bisect, datetime
import numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.expanduser('~/robo_jazzy_ws/src/robo_navigation/scripts'))
from comum import Poses, ler
import alinhador_passagem as al
bag, pasta, h, gx, gy = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), float(sys.argv[5])
t = datetime.datetime.strptime('2026-11-10 ' + h, '%Y-%m-%d %H:%M:%S.%f').timestamp()
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/map']))
G = get_message('nav_msgs/msg/OccupancyGrid'); m = None
while r.has_next():
    _, data, tt = r.read_next()
    if tt * 1e-9 > t: break
    m = deserialize_message(data, G)
q = m.info.origin.orientation
mapa = al.Mapa(m.info.resolution, m.info.width, m.info.height, m.info.origin.position.x, m.info.origin.position.y,
               math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)), list(m.data))
pm = Poses(pasta).em(t)
rota = al.rota_no_mapa(mapa, (pm[0], pm[1]), (gx, gy))
z = np.load(os.path.join(pasta, 'global_obstacle_raw.npz'))
i = max(0, bisect.bisect_right(z['t'], t) - 1); mt = z['meta'][i]
sx, sy, res, ox, oy = int(mt[0]), int(mt[1]), mt[2], mt[3], mt[4]
obs = z['data'][i][:sy, :sx]
js, iis = np.nonzero(obs == 254)
pts_obs = [(ox + (a + 0.5) * res, oy + (b + 0.5) * res) for b, a in zip(js, iis)]
din = [p for p in pts_obs if mapa.estado(*p) == 0 and not mapa.ocupado_firme(*p)]  # livre no mapa = do momento
def perto(rt, raio):
    return sorted({(round(o[0], 2), round(o[1], 2)) for o in din for p in rt if math.hypot(o[0]-p[0], o[1]-p[1]) < raio})
planos = ler(pasta, 'plan.jsonl'); k = bisect.bisect_right([p['t'] for p in planos], t) - 1
plano = [(p[0], p[1]) for p in planos[k]['pts']] if k >= 0 else []
def longe_da_parede(o, d=0.10):
    passo = mapa.res
    n = int(math.ceil(d / passo))
    for a in range(-n, n + 1):
        for b in range(-n, n + 1):
            if math.hypot(a, b) * passo <= d and mapa.estado(o[0] + a * passo, o[1] + b * passo) == 1:
                return False
    return True
novos = [o for o in din if longe_da_parede(o)]
def perto_n(rt, raio):
    return sorted({(round(o[0], 2), round(o[1], 2)) for o in novos for p in rt if math.hypot(o[0]-p[0], o[1]-p[1]) < raio})
print('robo (%.2f, %.2f) -> goal (%.2f, %.2f)' % (pm[0], pm[1], gx, gy))
print('rota do mapa: %.1f m; ultimo /plan: %.1f m' % (al._comp_rota(rota) if rota else -1, al._comp_rota(plano) if plano else -1))
print('celulas LETHAL do obstacle_layer livres no /map (obstaculo do momento): %d' % len(din))
print('  a < 0,35 m da rota do mapa: %d %s' % (len(perto(rota or [], 0.35)), perto(rota or [], 0.35)[:8]))
print('  a < 0,35 m do /plan:        %d' % len(perto(plano, 0.35)))
print('objeto novo (> 10 cm de parede do mapa): %d celulas; a < 0,35 m da rota do mapa: %d %s' % (
    len(novos), len(perto_n(rota or [], 0.35)), perto_n(rota or [], 0.35)[:6]))
