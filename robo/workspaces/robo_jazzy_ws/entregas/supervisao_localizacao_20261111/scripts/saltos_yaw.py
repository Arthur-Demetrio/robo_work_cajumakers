"""Analise offline (somente leitura) dos saltos de yaw map->odom do bag ensaio_goal_superior.
Para cada salto >3 graus: comando na janela, IMU (valor/estampa), odom->base, e ajuste do scan ao /map
antes e depois da correcao (distancia media endpoint->ocupado, sem raycasting)."""
import sys, math, json, bisect
from pathlib import Path
import numpy as np
from scipy import ndimage
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

BAG = Path(sys.argv[1]); OUT = Path(sys.argv[2])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=str(BAG), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
tipos = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
def st(s): return s.sec + s.nanosec * 1e-9
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def ang(a): return (a+math.pi) % (2*math.pi) - math.pi
mo, ob, imu, cmd, scans, mapas, static = [], [], [], [], [], [], []
recv_mo = []
while r.has_next():
    top, d, trec = r.read_next()
    if top not in ('/tf', '/tf_static', '/imu/data', '/cmd_vel_nav', '/scan_front', '/map'): continue
    m = deserialize_message(d, tipos[top])
    if top in ('/tf', '/tf_static'):
        for t in m.transforms:
            p, c = t.header.frame_id.strip('/'), t.child_frame_id.strip('/')
            v = (st(t.header.stamp), t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation))
            if top == '/tf_static': static.append((p, c, v))
            elif (p, c) == ('map', 'odom'): mo.append(v); recv_mo.append((trec*1e-9, v[0]))
            elif (p, c) == ('odom', 'base_link'): ob.append(v)
    elif top == '/imu/data': imu.append((st(m.header.stamp), yaw(m.orientation), m.angular_velocity.z, trec*1e-9))
    elif top == '/cmd_vel_nav':
        tw = m.twist if hasattr(m, 'twist') else m
        cmd.append((trec*1e-9, tw.linear.x, tw.linear.y, tw.angular.z))
    elif top == '/scan_front': scans.append((st(m.header.stamp), m, trec*1e-9))
    elif top == '/map': mapas.append((st(m.header.stamp), m))
for L in (mo, ob, imu, scans): L.sort(key=lambda x: x[0])
res = {}
# 1) publicadores/estampas de map->odom
dts = np.diff([v[0] for v in mo]); res['map_odom'] = dict(n=len(mo), dt_med=float(np.median(dts)), dt_min=float(dts.min()), dt_max=float(dts.max()),
    estampas_regressivas=int((dts < 0).sum()), estampas_repetidas=int((dts == 0).sum()),
    atraso_recepcao_med=float(np.median([a-b for a, b in recv_mo])))
dts = np.diff([v[0] for v in ob]); res['odom_base'] = dict(n=len(ob), dt_med=float(np.median(dts)), dt_max=float(dts.max()), regressivas=int((dts < 0).sum()))
res['tf_static'] = [(p, c, [round(x, 4) for x in v[1:]]) for p, c, v in static]
# atraso do scan: estampa vs recepcao
res['scan_atraso_recepcao_med'] = float(np.median([rc - s for s, _, rc in scans]))
res['imu_atraso_recepcao_med'] = float(np.median([i[3]-i[0] for i in imu]))
# IMU travada: sequencias de yaw identico com velocidade angular nao nula
it = [i[0] for i in imu]
def interp(L, t):
    ts = [x[0] for x in L]; i = bisect.bisect_left(ts, t); i = min(max(i, 1), len(L)-1)
    a, b = L[i-1], L[i]; f = 0 if b[0] == a[0] else (t-a[0])/(b[0]-a[0])
    return (a[1]+f*(b[1]-a[1]), a[2]+f*(b[2]-a[2]), a[3]+f*ang(b[3]-a[3]))
mot = [x[0] for x in mo]
def ultimo(L, ts, t):
    i = bisect.bisect_right(ts, t) - 1; return L[max(i, 0)]
# mapa estatico (ultimo antes do evento)
def campo(mp):
    g = np.array(mp.data, dtype=np.int16).reshape(mp.info.height, mp.info.width)
    occ = g >= 50
    dist = ndimage.distance_transform_edt(~occ) * mp.info.resolution
    return dist, mp.info
def ajuste(scan, pose, base_laser, dist, info):
    s = scan
    a = s.angle_min + np.arange(len(s.ranges))*s.angle_increment; rr = np.array(s.ranges)
    ok = np.isfinite(rr) & (rr > s.range_min) & (rr < min(s.range_max, 5.8))
    lx, ly, lth = base_laser
    xs = lx + rr[ok]*np.cos(a[ok]+lth); ys = ly + rr[ok]*np.sin(a[ok]+lth)
    x, y, th = pose
    mx = x + xs*math.cos(th) - ys*math.sin(th); my = y + xs*math.sin(th) + ys*math.cos(th)
    ox, oy = info.origin.position.x, info.origin.position.y
    i = ((my-oy)/info.resolution).astype(int); j = ((mx-ox)/info.resolution).astype(int)
    dentro = (i >= 0) & (i < dist.shape[0]) & (j >= 0) & (j < dist.shape[1])
    d = np.full(len(mx), 0.5); d[dentro] = np.minimum(dist[i[dentro], j[dentro]], 0.5)
    return float(np.mean(d)), float(np.mean(d < 0.05)), int(ok.sum())
laser = [v for p, c, v in static if c.startswith('laser') or 'lidar' in c]
base_laser = (laser[0][1], laser[0][2], laser[0][3]) if laser else (0.33, 0, 0)
res['base_laser_usado'] = base_laser
saltos = []
for a, b in zip(mo, mo[1:]):
    dy = math.degrees(abs(ang(b[3]-a[3])))
    if dy > 3: saltos.append((a, b, dy))
mts = [m[0] for m in mapas]
def compor(m_o, o_b):
    x, y, th = m_o
    return (x + o_b[0]*math.cos(th) - o_b[1]*math.sin(th), y + o_b[0]*math.sin(th) + o_b[1]*math.cos(th), ang(th + o_b[2]))
eventos = []
for a, b, dy in saltos:
    t = b[0]
    ev = dict(t=round(t, 4), dyaw_graus=round(dy, 3), dtrans_origem=round(math.hypot(b[1]-a[1], b[2]-a[2]), 4))
    jan = [c for c in cmd if abs(c[0]-t) <= 1.0]
    ev['cmd_wz_max_1s'] = round(max([abs(c[3]) for c in jan], default=0), 3)
    ev['cmd_v_max_1s'] = round(max([math.hypot(c[1], c[2]) for c in jan], default=0), 3)
    ji = [i for i in imu if abs(i[0]-t) <= 1.0]
    ev['imu_amostras_2s'] = len(ji)
    ev['imu_gz_max_2s'] = round(max([abs(i[2]) for i in ji], default=0), 3)
    ev['imu_yaw_var_2s_graus'] = round(math.degrees(max([i[1] for i in ji])-min([i[1] for i in ji])), 3) if ji else None
    ev['imu_maior_intervalo_2s'] = round(max(np.diff([i[0] for i in ji])), 3) if len(ji) > 1 else None
    obj = [o for o in ob if abs(o[0]-t) <= 1.0]
    ev['odom_yaw_var_2s_graus'] = round(math.degrees(max(o[3] for o in obj)-min(o[3] for o in obj)), 3) if obj else None
    # scan mais proximo antes e depois do salto
    sts = [s[0] for s in scans]; k = bisect.bisect_left(sts, t)
    mp = mapas[max(bisect.bisect_right(mts, t)-1, 0)][1]; dist, info = campo(mp)
    comp = []
    for s in scans[max(k-3, 0):k+3]:
        o_b = interp(ob, s[0])
        p_antes = compor(a[1:], o_b); p_depois = compor(b[1:], o_b)
        fa = ajuste(s[1], p_antes, base_laser, dist, info); fd = ajuste(s[1], p_depois, base_laser, dist, info)
        comp.append(dict(scan_t=round(s[0]-t, 3), antes_dist_m=round(fa[0], 4), antes_frac5cm=round(fa[1], 3), depois_dist_m=round(fd[0], 4), depois_frac5cm=round(fd[1], 3), pontos=fa[2]))
    ev['ajuste_scan_mapa'] = comp
    eventos.append(ev)
res['saltos'] = eventos
# ajuste geral ao longo do bag a cada 10 scans
geral = []
for s in scans[::10]:
    mp = mapas[max(bisect.bisect_right(mts, s[0])-1, 0)][1]
    if 'dist_cache' not in res: pass
    dist, info = campo(mp)
    p = compor(ultimo(mo, mot, s[0])[1:], interp(ob, s[0]))
    f = ajuste(s[1], p, base_laser, dist, info)
    geral.append((round(s[0], 2), round(f[0], 4), round(f[1], 3)))
res['ajuste_geral'] = dict(n=len(geral), dist_med=float(np.median([g[1] for g in geral])), frac5cm_med=float(np.median([g[2] for g in geral])), pior=sorted(geral, key=lambda g: -g[1])[:8])
OUT.write_text(json.dumps(res, indent=1, ensure_ascii=False)); print(json.dumps({k: v for k, v in res.items() if k not in ('saltos',)}, indent=1)[:3000])
for e in eventos: print({k: v for k, v in e.items() if k != 'ajuste_scan_mapa'}); [print('   ', c) for c in e['ajuste_scan_mapa']]
