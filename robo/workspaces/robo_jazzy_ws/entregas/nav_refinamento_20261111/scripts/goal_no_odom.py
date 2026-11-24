#!/usr/bin/env python3
"""Compara, no frame do controller (odom), o fim do /transformed_global_plan com a pose odom->base_link.
Uso: goal_no_odom.py <bag> <t_ini> <t_fim>"""
import sys, math, bisect, rosbag2_py, datetime
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, a, b = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
r.set_filter(rosbag2_py.StorageFilter(topics=['/transformed_global_plan', '/tf']))
P, T = get_message('nav_msgs/msg/Path'), get_message('tf2_msgs/msg/TFMessage')
yaw = lambda q: math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
ob = None; ultimo = 0
while r.has_next():
    top, data, t = r.read_next(); t *= 1e-9
    if t < a - 1 or t > b: continue
    if top == '/tf':
        for tr in deserialize_message(data, T).transforms:
            if tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link':
                ob = (tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation))
        continue
    m = deserialize_message(data, P)
    if not m.poses or ob is None or t - ultimo < 1.0: continue
    ultimo = t
    e = m.poses[-1].pose
    gy = yaw(e.orientation)
    d = math.hypot(e.position.x - ob[0], e.position.y - ob[1])
    print(datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-4], m.header.frame_id,
          'n=%3d fim_odom (%.2f,%.2f) yaw_fim %5.0f | robo_odom (%.2f,%.2f) yaw %5.0f | dist %.3f erro_yaw %5.0f' % (
          len(m.poses), e.position.x, e.position.y, gy, ob[0], ob[1], ob[2], d, (ob[2]-gy+180) % 360 - 180))
