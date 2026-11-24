#!/usr/bin/env python3
"""Ultima pose dos caminhos recebidos pelo controller (/received_global_plan) e do /plan numa janela.
Uso: extrair_janela_plano.py <bag> <t_ini_epoch> <t_fim_epoch>"""
import sys, math, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, a, b = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
tops = ['/received_global_plan', '/plan', '/plan_smoothed']
r.set_filter(rosbag2_py.StorageFilter(topics=tops))
P = get_message('nav_msgs/msg/Path')
yaw = lambda q: math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
ult = {}
import datetime
while r.has_next():
    top, data, t = r.read_next(); t *= 1e-9
    if t < a or t > b: continue
    m = deserialize_message(data, P)
    if not m.poses: continue
    e = m.poses[-1].pose
    chave = (top, round(e.position.x, 2), round(e.position.y, 2), round(yaw(e.orientation)))
    if ult.get(top) != chave:
        ult[top] = chave
        print(datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-4], top, 'n=%d' % len(m.poses), 'fim (%.2f, %.2f) yaw %.0f' % chave[1:], 'frame', m.header.frame_id)
