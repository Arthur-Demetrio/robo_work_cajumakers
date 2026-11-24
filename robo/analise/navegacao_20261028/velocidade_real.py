"""Velocidade real x comando, medida num bag ja convertido (pkl do analise.py).

Uso: python3 velocidade_real.py t24.pkl
Giro: rapidez do yaw da odom (IMU) quando o /cmd_vel final so tem giro.
Linear: deslocamento no mapa (map->odom->base_link) com comando linear estavel por 0,8 s.
"""
import bisect
import math
import pickle
import statistics as st
import sys

d = pickle.load(open(sys.argv[1], 'rb'))
mo, ob = d['mo'], d['ob']
mot = [r[0] for r in mo]
obt = [r[0] for r in ob]


def comp(a, b):
    x, y, th = a
    return (x + b[0] * math.cos(th) - b[1] * math.sin(th), y + b[0] * math.sin(th) + b[1] * math.cos(th), th + b[2])


def pose(t):
    i = bisect.bisect_right(mot, t) - 1
    j = bisect.bisect_right(obt, t) - 1
    return comp(mo[i][1:], ob[j][1:])


def opose(t):
    return ob[bisect.bisect_right(obt, t) - 1][1:]


cf = d['cmds']['/cmd_vel']
giro = {}
for c in cf:
    w = abs(c[3])
    if math.hypot(c[1], c[2]) > 0.01 or w < 0.015:
        continue
    dy = opose(c[0] + 0.25)[2] - opose(c[0] - 0.25)[2]
    giro.setdefault(round(w, 2), []).append(abs(math.atan2(math.sin(dy), math.cos(dy)) / 0.5))
print('GIRO PARADO (comando -> real)')
for k in sorted(giro):
    if len(giro[k]) > 5:
        m = st.median(giro[k])
        print('  cmd %.2f rad/s -> real %.2f rad/s (%.0f graus/s)  n=%d' % (k, m, math.degrees(m), len(giro[k])))
lin = {}
for i, c in enumerate(cf):
    v = math.hypot(c[1], c[2])
    if v < 0.015 or abs(c[3]) > 0.03:
        continue
    win = [x for x in cf[max(0, i - 12):i + 12] if abs(x[0] - c[0]) <= 0.4]
    if any(abs(math.hypot(x[1], x[2]) - v) > 0.012 or abs(x[3]) > 0.03 for x in win):
        continue
    a, b = pose(c[0] - 0.3), pose(c[0] + 0.3)
    oa, obb = opose(c[0] - 0.3), opose(c[0] + 0.3)
    lin.setdefault(round(v / 0.01) * 0.01, []).append((math.hypot(b[0] - a[0], b[1] - a[1]) / 0.6,
                                                       math.hypot(obb[0] - oa[0], obb[1] - oa[1]) / 0.6))
print('LINEAR (comando -> real no mapa / odom)')
for k in sorted(lin):
    if len(lin[k]) > 5:
        print('  cmd %.2f m/s -> real %.3f  odom %.3f  n=%d' % (
            k, st.median(r[0] for r in lin[k]), st.median(r[1] for r in lin[k]), len(lin[k])))
