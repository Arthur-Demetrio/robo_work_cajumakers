"""Avanco REAL (LiDAR) no inicio das saidas laterais: paredes do corredor ficam na frente e atras do robo.
avanco = media de -(variacao da parede da frente) e -(variacao da parede de tras), em cm, contra o scan do inicio."""
import sys, json, math, re, statistics as st, bisect
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

LX, LY = 0.33, 0.005


def scans(bag):
    cru = sorted(Path(bag).glob('*.mcap'))
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(cru[0]), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    T = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=['/scan_front']))
    out = []
    while r.has_next():
        _, d, t = r.read_next()
        m = deserialize_message(d, T['/scan_front'])
        fr, tr = [], []
        a = m.angle_min
        for rr in m.ranges:
            if math.isfinite(rr) and m.range_min < rr < 2.0:
                x, y = LX + rr * math.cos(a), LY + rr * math.sin(a)
                if abs(y) < 0.22:
                    if 0.40 < x < 1.3:
                        fr.append(x)
                    elif -1.2 < x < -0.25:
                        tr.append(x)
            a += m.angle_increment
        out.append((t * 1e-9, st.median(fr) if len(fr) >= 3 else None, st.median(tr) if len(tr) >= 3 else None))
    return out


def avanco(S, ts, t0, t):
    i0, i = bisect.bisect_left(ts, t0), bisect.bisect_left(ts, t)
    if i0 >= len(S) or i >= len(S):
        return None
    a, b = S[i0], S[i]
    v = []
    if a[1] and b[1] and abs(b[1] - a[1]) < 0.25:
        v.append(-(b[1] - a[1]))
    if a[2] and b[2] and abs(b[2] - a[2]) < 0.25:
        v.append(-(b[2] - a[2]))
    return round(100 * sum(v) / len(v), 1) if v else None


ev = json.load(open(sys.argv[1]))
res = {}
for bag, lista in ev.items():
    if not isinstance(lista, list):
        continue
    sel = [e for e in lista if re.search(r'CURVA \d+:', ' '.join(m for a, m in e['log']))]
    if not sel:
        continue
    S = scans(bag)
    ts = [s[0] for s in S]
    for e in sel:
        lado = 'ESQ' if e['vy'] > 0 else 'DIR'
        r = dict(t=e['t'], lado=lado, vx_cmd=e['vx_onset'], vx_max=e['vx_max_cmd_vel_nav'],
                 a05=avanco(S, ts, e['t'] - 0.1, e['t'] + 0.5), a10=avanco(S, ts, e['t'] - 0.1, e['t'] + 1.0),
                 a15=avanco(S, ts, e['t'] - 0.1, e['t'] + 1.5), a30=avanco(S, ts, e['t'] - 0.1, e['t'] + 3.0))
        res.setdefault(Path(bag).parent.name[:8] + ' ' + lado, []).append(r)
for k, v in res.items():
    def med(c):
        x = [r[c] for r in v if r[c] is not None]
        return '%+5.1f (n%d, max %+5.1f)' % (st.median(x), len(x), max(x)) if x else '-'
    print('%-14s n=%2d vx_cmd med %+.3f | avanco real cm: 0,5 s %s | 1,0 s %s | 1,5 s %s | 3,0 s %s' % (
        k, len(v), st.median(r['vx_cmd'] for r in v), med('a05'), med('a10'), med('a15'), med('a30')))
json.dump(res, open(sys.argv[2], 'w'), indent=1)
