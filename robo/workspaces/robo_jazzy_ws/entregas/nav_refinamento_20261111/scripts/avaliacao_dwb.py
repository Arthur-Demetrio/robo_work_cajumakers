#!/usr/bin/env python3
"""Pontuacao por critica do DWB (/evaluation) numa janela: melhor trajetoria e as alternativas so-giro.
Uso: avaliacao_dwb.py <bag> <t_ini> <t_fim> [passo_s]"""
import sys, rosbag2_py, datetime
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, a, b = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
passo = float(sys.argv[4]) if len(sys.argv) > 4 else 1.0
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/evaluation']))
E = get_message('dwb_msgs/msg/LocalPlanEvaluation')
ult = 0
while r.has_next():
    _, data, t = r.read_next(); t *= 1e-9
    if t < a or t > b or t - ult < passo: continue
    ult = t
    m = deserialize_message(data, E)
    print(datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-4], 'n=%d validas=%d best=%d' % (
        len(m.twists), sum(1 for tw in m.twists if tw.total >= 0), m.best_index))
    cand = [(i, tw) for i, tw in enumerate(m.twists) if abs(tw.traj.velocity.x) < 1e-6 and abs(tw.traj.velocity.y) < 1e-6]
    if 0 <= m.best_index < len(m.twists):
        cand = [(m.best_index, m.twists[m.best_index])] + [c for c in cand if c[0] != m.best_index]
    for i, tw in cand[:4]:
        v = tw.traj.velocity
        sc = ' '.join('%s=%.1f' % (s.name.split('/')[-1], s.raw_score * s.scale) for s in tw.scores if s.raw_score * s.scale != 0 or s.raw_score < 0)
        print('   %s vx=%.2f vy=%.2f wz=%+.2f total=%.1f | %s' % ('*' if i == m.best_index else ' ', v.x, v.y, v.theta, tw.total, sc))
