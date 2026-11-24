"""Linha do tempo (a cada 0,5 s) de pose no mapa e comandos nav -> smoothed -> final.

Uso: python3 linha_comandos.py t25.pkl 11:00:27.5 11:01:14.5
"""
import bisect
import math
import pickle
import sys
from datetime import datetime

d = pickle.load(open(sys.argv[1], 'rb'))
mo, ob = d['mo'], d['ob']
mot = [r[0] for r in mo]
obt = [r[0] for r in ob]


def comp(a, b):
    x, y, th = a
    return (x + b[0] * math.cos(th) - b[1] * math.sin(th), y + b[0] * math.sin(th) + b[1] * math.cos(th), th + b[2])


def pose(t):
    return comp(mo[bisect.bisect_right(mot, t) - 1][1:], ob[bisect.bisect_right(obt, t) - 1][1:])


dia = datetime.fromtimestamp(mot[0]).strftime('%Y-%m-%d ')
ts = lambda s: datetime.strptime(dia + s, '%Y-%m-%d %H:%M:%S.%f').timestamp()
listas = [d['cmds'][k] for k in ('/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel')]
tempos = [[c[0] for c in l] for l in listas]


def ultimo(k, t):
    i = bisect.bisect_right(tempos[k], t) - 1
    return listas[k][i] if i >= 0 and t - listas[k][i][0] < 0.3 else None


fmt = lambda x: ('%+.2f %+.2f %+.2f' % x[1:4]) if x else '   ---          '
t, fim = ts(sys.argv[2]), ts(sys.argv[3])
print('hora     pose x y yaw      | nav vx vy wz        | smoothed            | final (/cmd_vel)')
while t < fim:
    p = pose(t)
    print(datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-5], 'pose %.2f %.2f %4.0f' % (p[0], p[1], math.degrees(p[2])),
          '| nav', fmt(ultimo(0, t)), '| smooth', fmt(ultimo(1, t)), '| final', fmt(ultimo(2, t)))
    t += 0.5
