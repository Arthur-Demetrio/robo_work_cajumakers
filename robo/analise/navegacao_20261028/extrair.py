#!/usr/bin/env python3
"""bag (.mcap) -> pkl com TF map/odom/base_link, comandos, /odom, IMU e /scan_front.

Uso: python3 extrair.py bag_0.mcap saida.pkl
"""
import math, os, pickle, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from bagio import open_bag, deser

ts, it = open_bag(sys.argv[1], topics=['/tf', '/cmd_vel', '/cmd_vel_smoothed', '/cmd_vel_nav', '/odom', '/imu/data', '/scan_front'])
D = {k: [] for k in ['mo', 'ob', 'cmd', 'cms', 'cmn', 'odom', 'imu', 'scan']}
def yaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
for top, t, d, typ in it():
    m = deser(ts, d, typ)
    if top == '/tf':
        for tr in m.transforms:
            r = (t, tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation))
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom': D['mo'].append(r)
            elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link': D['ob'].append(r)
    elif top.startswith('/cmd_vel'):
        tw = m.twist if hasattr(m, 'twist') else m
        D[{'/cmd_vel': 'cmd', '/cmd_vel_smoothed': 'cms', '/cmd_vel_nav': 'cmn'}[top]].append((t, tw.linear.x, tw.linear.y, tw.angular.z))
    elif top == '/odom':
        p = m.pose.pose
        D['odom'].append((t, p.position.x, p.position.y, yaw(p.orientation), m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z))
    elif top == '/imu/data':
        D['imu'].append((t, yaw(m.orientation), m.angular_velocity.z))
    elif top == '/scan_front':
        D['scan'].append((t, m.angle_min, m.angle_increment, list(m.ranges)))
pickle.dump(D, open(sys.argv[2], 'wb'))
print({k: len(v) for k, v in D.items()})
