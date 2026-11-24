#!/usr/bin/env python3
"""Extrai /global_costmap/costmap_raw (nav2_msgs/Costmap) para .npz. Uso: extrair_costmap.py <bag> <saida.npz> [topico]"""
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, saida = sys.argv[1], sys.argv[2]
top = sys.argv[3] if len(sys.argv) > 3 else '/global_costmap/costmap_raw'
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=[top]))
C = get_message('nav2_msgs/msg/Costmap')
ts, metas, datas = [], [], []
while r.has_next():
    _, data, t = r.read_next()
    m = deserialize_message(data, C)
    md = m.metadata
    ts.append(t*1e-9)
    metas.append((md.size_x, md.size_y, md.resolution, md.origin.position.x, md.origin.position.y))
    datas.append(np.frombuffer(bytes(m.data), dtype=np.uint8).reshape(md.size_y, md.size_x))
shapes = {d.shape for d in datas}
H = max(s[0] for s in shapes); W = max(s[1] for s in shapes)
arr = np.full((len(datas), H, W), 255, np.uint8)
for i, d in enumerate(datas):
    arr[i, :d.shape[0], :d.shape[1]] = d
np.savez_compressed(saida, t=np.array(ts), meta=np.array(metas), data=arr)
print(len(ts), shapes)
