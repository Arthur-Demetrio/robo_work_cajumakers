"""Para varios scans do bag: ajuste scan x /map na pose do SLAM e apos busca rigida (+-0,30 m, +-10 graus).
Separa: (a) erro de localizacao (melhora grande com deslocamento consistente) de (b) ambiente diferente do
mapa (pontos sem parede no mapa mesmo apos o melhor ajuste). Somente leitura."""
import sys, math, json, bisect
import numpy as np
from scipy import ndimage
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
BAG, OUT = sys.argv[1], sys.argv[2]
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
            (mo if x.child_frame_id == 'odom' else ob if x.child_frame_id == 'base_link' else []).append(v)
    elif top == '/scan_front': m = deserialize_message(d, T[top]); scans.append((st(m.header.stamp), m))
    elif top == '/map' and mapa is None: mapa = deserialize_message(d, T[top])
mo.sort(); ob.sort(); mot = [m[0] for m in mo]; obt = [o[0] for o in ob]
info = mapa.info; g = np.array(mapa.data, dtype=np.int16).reshape(info.height, info.width)
dist = ndimage.distance_transform_edt(~(g >= 50))*info.resolution
def pose(t):
    m = mo[max(bisect.bisect_right(mot, t)-1, 0)]; i = min(max(bisect.bisect_left(obt, t), 1), len(ob)-1); o = ob[i]
    return (m[1]+o[1]*math.cos(m[3])-o[2]*math.sin(m[3]), m[2]+o[1]*math.sin(m[3])+o[2]*math.cos(m[3]), ang(m[3]+o[3]))
def pts_base(s):
    a = s.angle_min+np.arange(len(s.ranges))*s.angle_increment; rr = np.array(s.ranges)
    ok = np.isfinite(rr) & (rr > s.range_min) & (rr < 4.0)
    return np.c_[.33+rr[ok]*np.cos(a[ok]), .005+rr[ok]*np.sin(a[ok])]
def d_map(P):
    i = ((P[:, 1]-info.origin.position.y)/info.resolution).astype(int); j = ((P[:, 0]-info.origin.position.x)/info.resolution).astype(int)
    den = (i >= 0) & (i < dist.shape[0]) & (j >= 0) & (j < dist.shape[1]); dd = np.full(len(P), .5); dd[den] = np.minimum(dist[i[den], j[den]], .5); return dd
def transf(B, p):
    c, s = math.cos(p[2]), math.sin(p[2]); return np.c_[p[0]+c*B[:, 0]-s*B[:, 1], p[1]+s*B[:, 0]+c*B[:, 1]]
def nota(B, p):
    dd = np.minimum(d_map(transf(B, p)), .3); return np.sort(dd)[:int(.8*len(dd))].mean()  # aparado 20%
res = []
for k in range(0, len(scans), 40):
    t, s = scans[k]; B = pts_base(s); p0 = pose(t)
    if len(B) < 40: continue
    best = (nota(B, p0), 0, 0, 0)
    for dth in np.radians(np.arange(-10, 10.1, 1.0)):
        for dx in np.arange(-.3, .301, .03):
            for dy in np.arange(-.3, .301, .03):
                v = nota(B, (p0[0]+dx, p0[1]+dy, p0[2]+dth))
                if v < best[0]: best = (v, dx, dy, dth)
    pb = (p0[0]+best[1], p0[1]+best[2], p0[2]+best[3])
    d0, d1 = d_map(transf(B, p0)), d_map(transf(B, pb))
    res.append(dict(t=round(t, 2), pose_slam=[round(v, 3) for v in p0], ajuste_dx=round(best[1], 2), ajuste_dy=round(best[2], 2), ajuste_dyaw_graus=round(math.degrees(best[3]), 1),
                    frac5_slam=round(float((d0 < .05).mean()), 3), frac5_melhor=round(float((d1 < .05).mean()), 3),
                    frac_sem_parede_15cm_melhor=round(float((d1 > .15).mean()), 3), pontos=len(B)))
    print(res[-1])
resumo = dict(scans=len(res), frac5_slam_med=float(np.median([x['frac5_slam'] for x in res])), frac5_melhor_med=float(np.median([x['frac5_melhor'] for x in res])),
              frac_sem_parede_15cm_med=float(np.median([x['frac_sem_parede_15cm_melhor'] for x in res])),
              desloc_med_m=float(np.median([math.hypot(x['ajuste_dx'], x['ajuste_dy']) for x in res])), dyaw_med_abs=float(np.median([abs(x['ajuste_dyaw_graus']) for x in res])))
print(resumo); json.dump(dict(resumo=resumo, scans=res), open(OUT, 'w'), indent=1)
