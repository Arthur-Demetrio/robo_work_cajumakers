#!/usr/bin/env python3
"""Extrai do bag (somente leitura) os topicos usados na analise de 11/11 para JSONL.
Uso: python3 -I extrair_bag.py <pasta_do_bag> <pasta_saida>
"""
import sys, os, json, math
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICOS = ['/rosout', '/cmd_vel_nav', '/cmd_vel_ctrl', '/cmd_vel_smoothed', '/cmd_vel', '/cmd_vel_hardware',
           '/navigate_to_pose/_action/status', '/follow_path/_action/status', '/smooth_path/_action/status',
           '/compute_path_to_pose/_action/status', '/destravar/_action/status', '/destravar/estado',
           '/collision_monitor_state', '/goal_pose_alinhado', '/goal_pose', '/tf', '/imu/yaw_deg', '/plan',
           '/plan_smoothed', '/odom', '/evaluation']

def yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

def conv(top, m, t):
    if top == '/rosout':
        return dict(t=t, name=m.name, level=m.level, msg=m.msg, file=m.file, line=m.line)
    if top.startswith('/cmd_vel'):
        tw = m.twist
        return dict(t=t, vx=tw.linear.x, vy=tw.linear.y, wz=tw.angular.z)
    if top.endswith('/_action/status'):
        return dict(t=t, s=[(bytes(g.goal_info.goal_id.uuid).hex()[:8], g.status,
                             g.goal_info.stamp.sec + g.goal_info.stamp.nanosec*1e-9) for g in m.status_list])
    if top == '/destravar/estado':
        return dict(t=t, data=m.data)
    if top == '/collision_monitor_state':
        return dict(t=t, action=m.action_type, poly=m.polygon_name)
    if top in ('/goal_pose_alinhado', '/goal_pose'):
        p = m.pose
        return dict(t=t, x=p.position.x, y=p.position.y, yaw=yaw(p.orientation), frame=m.header.frame_id)
    if top == '/tf':
        out = []
        for tr in m.transforms:
            if (tr.header.frame_id, tr.child_frame_id) in (('map', 'odom'), ('odom', 'base_link')):
                tt = tr.transform
                out.append(dict(p=tr.header.frame_id, c=tr.child_frame_id, x=tt.translation.x, y=tt.translation.y,
                                yaw=yaw(tt.rotation), st=tr.header.stamp.sec + tr.header.stamp.nanosec*1e-9))
        return dict(t=t, tf=out) if out else None
    if top == '/imu/yaw_deg':
        return dict(t=t, yaw_deg=m.data)
    if top in ('/plan', '/plan_smoothed'):
        pts = [(round(p.pose.position.x, 3), round(p.pose.position.y, 3), round(yaw(p.pose.orientation), 3)) for p in m.poses]
        return dict(t=t, n=len(pts), pts=pts)
    if top == '/odom':
        return dict(t=t, x=m.pose.pose.position.x, y=m.pose.pose.position.y, yaw=yaw(m.pose.pose.orientation),
                    vx=m.twist.twist.linear.x, vy=m.twist.twist.linear.y, wz=m.twist.twist.angular.z)
    if top == '/evaluation':
        return dict(t=t, n=len(m.twists), best=m.best_index,
                    valid=sum(1 for tw in m.twists if tw.total >= 0))
    return None

def main(bag, saida):
    os.makedirs(saida, exist_ok=True)
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    tipos = {t.name: t.type for t in r.get_all_topics_and_types()}
    sel = [t for t in TOPICOS if t in tipos]
    r.set_filter(rosbag2_py.StorageFilter(topics=sel))
    cls = {t: get_message(tipos[t]) for t in sel}
    fs = {t: open(os.path.join(saida, t.strip('/').replace('/', '__') + '.jsonl'), 'w') for t in sel}
    while r.has_next():
        top, data, tns = r.read_next()
        d = conv(top, deserialize_message(data, cls[top]), tns*1e-9)
        if d is not None:
            fs[top].write(json.dumps(d) + '\n')
    for f in fs.values():
        f.close()
    print('ok', sel)

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
