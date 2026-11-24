"""Compara map->odom de replays (CSV) e o original do bag: saltos, inversoes (cacada) e ajuste scan x /map."""
import sys, math, json, bisect, csv
import numpy as np
from scipy import ndimage
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BAG = sys.argv[1]; csvs = sys.argv[2:]
r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=BAG, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
T = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
def st(s): return s.sec + s.nanosec*1e-9
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
def ang(a): return (a+math.pi) % (2*math.pi) - math.pi
mo, ob, scans, mapa = [], [], [], None
while r.has_next():
    top, d, _ = r.read_next()
    if top == '/tf':
        for x in deserialize_message(d, T[top]).transforms:
            v = (st(x.header.stamp), x.transform.translation.x, x.transform.translation.y, yaw(x.transform.rotation))
            if x.child_frame_id == 'odom': mo.append(v)
            elif x.child_frame_id == 'base_link': ob.append(v)
    elif top == '/scan_front': m = deserialize_message(d, T[top]); scans.append((st(m.header.stamp), m))
    elif top == '/map' and mapa is None: mapa = deserialize_message(d, T[top])
ob.sort(); obt = [o[0] for o in ob]
g = np.array(mapa.data, dtype=np.int16).reshape(mapa.info.height, mapa.info.width)
dist = ndimage.distance_transform_edt(~(g >= 50))*mapa.info.resolution; info = mapa.info
def interp(t):
    i = min(max(bisect.bisect_left(obt, t), 1), len(ob)-1); a, b = ob[i-1], ob[i]
    f = 0 if b[0] == a[0] else (t-a[0])/(b[0]-a[0]); return (a[1]+f*(b[1]-a[1]), a[2]+f*(b[2]-a[2]), a[3]+f*ang(b[3]-a[3]))
def ajuste(s, p):
    a = s.angle_min+np.arange(len(s.ranges))*s.angle_increment; rr = np.array(s.ranges)
    ok = np.isfinite(rr) & (rr > s.range_min) & (rr < 5.8)
    xs = .33+rr[ok]*np.cos(a[ok]); ys = .005+rr[ok]*np.sin(a[ok]); x, y, th = p
    mx = x+xs*math.cos(th)-ys*math.sin(th); my = y+xs*math.sin(th)+ys*math.cos(th)
    i = ((my-info.origin.position.y)/info.resolution).astype(int); j = ((mx-info.origin.position.x)/info.resolution).astype(int)
    den = (i >= 0) & (i < dist.shape[0]) & (j >= 0) & (j < dist.shape[1]); dd = np.full(len(mx), .5); dd[den] = np.minimum(dist[i[den], j[den]], .5)
    return dd.mean(), (dd < .05).mean()
def metricas(serie, nome):
    serie = [s for s in serie if s[0] > 0]; serie.sort(); ts = [s[0] for s in serie]
    passos = [(b[0], math.degrees(ang(b[3]-a[3]))) for a, b in zip(serie, serie[1:])]
    grandes = [p for p in passos if abs(p[1]) > 3]
    inv = sum(1 for a, b in zip(grandes, grandes[1:]) if a[1]*b[1] < 0 and b[0]-a[0] < 2.0)
    # variacao total do yaw map->odom (soma |passos| > 0.5 grau) = "trabalho" de correcao
    trabalho = sum(abs(p[1]) for p in passos if abs(p[1]) > .5)
    fits = []
    for s in scans[::5]:
        if s[0] < ts[0] or s[0] > ts[-1]: continue
        k = max(bisect.bisect_right(ts, s[0])-1, 0); m = serie[k]; o = interp(s[0])
        p = (m[1]+o[0]*math.cos(m[3])-o[1]*math.sin(m[3]), m[2]+o[0]*math.sin(m[3])+o[1]*math.cos(m[3]), m[3]+o[2])
        fits.append(ajuste(s[1], p))
    f = np.array(fits)
    return dict(perfil=nome, amostras=len(serie), saltos_3graus=len(grandes), maior_salto_graus=round(max([abs(p[1]) for p in passos], default=0), 2),
                inversoes_2s=inv, correcao_total_graus=round(trabalho, 1), ajuste_dist_med_m=round(float(np.median(f[:, 0])), 4),
                ajuste_frac5cm_med=round(float(np.median(f[:, 1])), 3), ajuste_dist_p90_m=round(float(np.percentile(f[:, 0], 90)), 4), scans=len(f))
res = [metricas(mo, 'original_bag')]
for c in csvs:
    rows = [tuple(map(float, l)) for l in list(csv.reader(open(c)))[1:] if len(l) == 4]
    rows = [r_ for r_ in rows if not (r_[1] == 0 and r_[2] == 0 and r_[3] == 0)]
    res.append(metricas(rows, c.split('/')[-1].replace('.csv', '')))
for x in res: print(x)
json.dump(res, open('replay_slam/metricas.json', 'w'), indent=1)
