#!/usr/bin/env python3
"""Comando do Nav2 (/cmd_vel_nav), comando final (/cmd_vel), pose no mapa, yaw da odom e da IMU.

Uso: python3 linha_tempo.py saida.pkl 10:50:50 10:51:15 [passo_s=0.25]
"""
import math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from _comum import carregar, ep, hms, em, pose_mapa

D = carregar(sys.argv[1])
t, t1 = ep(D, sys.argv[2]), ep(D, sys.argv[3])
dt = float(sys.argv[4]) if len(sys.argv) > 4 else 0.25
print('hora        nav(vx vy wz)        final(vx vy wz)      mapa x y yaw          odom yaw  imu yaw')
while t < t1:
    n, f, p, o, im = em(D, 'cmn', t), em(D, 'cmd', t), pose_mapa(D, t), em(D, 'odom', t), em(D, 'imu', t)
    print('%s  %5.3f %6.3f %6.3f | %5.3f %6.3f %6.3f | %6.3f %6.3f %7.1f | %7.1f %7.1f' % (
        hms(t), n[1], n[2], n[3], f[1], f[2], f[3], p[0], p[1], math.degrees(p[2]), math.degrees(o[3]), math.degrees(im[1])))
    t += dt
