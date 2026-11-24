#!/usr/bin/env python3
"""Folgas pelo LiDAR (frame base_link, LiDAR em x=0,33) numa janela: frente (faixa |y|<=0,33 a frente do para-choque
x=0,375), e lados livres a frente (faixa x 0,375..1,2: maior |y| livre a esquerda e a direita). Com o comando da ponte.
Uso: folgas_lidar.py <bag> <dados> <ini HH:MM:SS> <fim HH:MM:SS> [topico_scan] [passo_s]"""
import sys, os, math, json, bisect, datetime
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, dados, hi, hf = sys.argv[1:5]; top = sys.argv[5] if len(sys.argv) > 5 else '/scan_nav'
passo = float(sys.argv[6]) if len(sys.argv) > 6 else 0.2
T = lambda h: datetime.datetime.strptime('2026-11-11 ' + h, '%Y-%m-%d %H:%M:%S').timestamp()
a, b = T(hi), T(hf)
hw = [json.loads(l) for l in open(os.path.join(dados, 'cmd_vel_hardware.jsonl'))]; th = [d['t'] for d in hw]
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=[top])); S = get_message('sensor_msgs/msg/LaserScan')
ult = 0
print('hora        frente(cm) livre_esq(cm) livre_dir(cm) | ponte vx vy wz')
while r.has_next():
    _, data, t = r.read_next(); t *= 1e-9
    if t < a or t > b or t - ult < passo: continue
    ult = t; m = deserialize_message(data, S)
    fr, esq, dir_ = 9.9, 9.9, 9.9
    pts = []
    for k, d in enumerate(m.ranges):
        if not math.isfinite(d) or d < m.range_min: continue
        ang = m.angle_min + k * m.angle_increment
        x, y = 0.33 + d * math.cos(ang), d * math.sin(ang)
        pts.append((x, y))
        if x > 0.375 and abs(y) <= 0.33: fr = min(fr, x - 0.375)
    # maior faixa livre lateral a frente: menor |y| ocupado em x 0,375..1,2, de cada lado
    for x, y in pts:
        if 0.375 < x < 1.2:
            if y > 0.33: esq = min(esq, y - 0.325)
            if y < -0.33: dir_ = min(dir_, -y - 0.325)
    i = bisect.bisect_right(th, t) - 1; c = hw[i] if i >= 0 and t - hw[i]['t'] < 0.3 else {'vx': 0, 'vy': 0, 'wz': 0}
    print('%s %8.1f %10.1f %12.1f | %+.3f %+.3f %+.3f' % (datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-4],
          100 * fr, 100 * esq, 100 * dir_, c['vx'], c['vy'], c['wz']))
