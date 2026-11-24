#!/usr/bin/env python3
"""Banco de replay do SLAM (11/11): toca do bag /clock, /tf (SEM map->odom), /tf_static e o scan, e grava o map->odom
que o slam_toolbox do teste publicar. Antes de tocar, chama /slam_toolbox/deserialize_map na pose dada.
Uso (dominio isolado!): tocar_bag_slam.py <bag> <saida.csv> <mapa> <match_type> <x> <y> <yaw> [scan_topic]"""
import sys, time, csv
import rclpy, rosbag2_py
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import DeserializePoseGraph
import math
bag, saida, mapa, mt, x0, y0, th0 = sys.argv[1:8]
scan_top = sys.argv[8] if len(sys.argv) > 8 else '/scan_front'
rclpy.init()
n = Node('tocador_bag_slam', parameter_overrides=[])
pc = n.create_publisher(Clock, '/clock', 10)
ptf = n.create_publisher(TFMessage, '/tf', 100)
pst = n.create_publisher(TFMessage, '/tf_static', QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                                             reliability=ReliabilityPolicy.RELIABLE))
ps = n.create_publisher(LaserScan, scan_top, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE))
reg = []
def ao_tf(m):
    for tr in m.transforms:
        if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
            q = tr.transform.rotation
            reg.append((tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9, tr.transform.translation.x,
                        tr.transform.translation.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
n.create_subscription(TFMessage, '/tf', ao_tf, 100)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
tipos = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/tf_static', scan_top]))
C = {t: get_message(tipos[t]) for t in ['/tf', '/tf_static', scan_top]}
msgs = []
while r.has_next():
    top, data, t = r.read_next()
    msgs.append((top, deserialize_message(data, C[top]), t))
t_ini = msgs[0][2]
def clock(tns):
    c = Clock(); c.clock.sec = tns // 10**9; c.clock.nanosec = tns % 10**9; pc.publish(c)
# 1) estaticas + relogio + TF de odom do inicio, para o SLAM ter frames
for top, m, t in msgs:
    if top == '/tf_static': pst.publish(m)
for k in range(30):
    clock(t_ini); rclpy.spin_once(n, timeout_sec=0.1)
for top, m, t in msgs[:200]:
    if top == '/tf':
        m.transforms = [tr for tr in m.transforms if not (tr.header.frame_id == 'map' and tr.child_frame_id == 'odom')]
        if m.transforms: clock(t); ptf.publish(m)
# 2) carrega o mapa
cli = n.create_client(DeserializePoseGraph, '/slam_toolbox/deserialize_map')
if not cli.wait_for_service(timeout_sec=60): sys.exit('slam_toolbox sem deserialize_map')
rq = DeserializePoseGraph.Request(); rq.filename = mapa; rq.match_type = int(mt)
rq.initial_pose.x, rq.initial_pose.y, rq.initial_pose.theta = float(x0), float(y0), float(th0)
f = cli.call_async(rq)
while not f.done():
    clock(t_ini); rclpy.spin_once(n, timeout_sec=0.1)
print('mapa carregado; tocando', len(msgs), 'mensagens', flush=True)
time.sleep(2.0)
# 3) toca em tempo real
w0 = time.monotonic()
for top, m, t in msgs:
    alvo = w0 + (t - t_ini) * 1e-9
    while time.monotonic() < alvo:
        rclpy.spin_once(n, timeout_sec=max(0.0, min(0.005, alvo - time.monotonic())))
    clock(t)
    if top == '/tf':
        m.transforms = [tr for tr in m.transforms if not (tr.header.frame_id == 'map' and tr.child_frame_id == 'odom')]
        if m.transforms: ptf.publish(m)
    elif top == scan_top:
        ps.publish(m)
for _ in range(20): rclpy.spin_once(n, timeout_sec=0.1)
with open(saida, 'w', newline='') as fh:
    wr = csv.writer(fh); wr.writerow(['stamp', 'x', 'y', 'yaw'])
    for row in reg: wr.writerow(row)
print('map->odom gravados:', len(reg))
