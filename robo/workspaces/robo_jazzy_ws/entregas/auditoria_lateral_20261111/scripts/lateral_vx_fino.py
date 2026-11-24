"""Inicios de movimento lateral no /cmd_vel_nav: ha vx junto? Onde aparece (nav/smoothed/final) e o que o robo fez (TF map)."""
import sys, math, json, bisect
from pathlib import Path
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOP = ['/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel', '/rosout', '/tf']


def ler(bag):
    cru = sorted(Path(bag).glob('*.mcap'))
    if cru:   # ja descomprimido: abre o arquivo direto (o metadata aponta para o .zstd)
        r = rosbag2_py.SequentialReader()
        r.open(rosbag2_py.StorageOptions(uri=str(cru[0]), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    else:
        r = rosbag2_py.SequentialCompressionReader()
        r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    T = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOP if t in T]))
    while r.has_next():
        top, d, t = r.read_next()
        yield top, deserialize_message(d, T[top]), t * 1e-9


def tw(m):
    t = m.twist if hasattr(m, 'twist') else m
    return t.linear.x, t.linear.y, t.angular.z


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def analisar(bag):
    S = {k: [] for k in TOP[:3]}
    logs, pose = [], []
    mo = None
    for top, m, t in ler(bag):
        if top in S:
            S[top].append((t,) + tw(m))
        elif top == '/rosout':
            if m.name in ('alinhador_passagem', 'destravar', 'bt_navigator', 'controller_server'):
                logs.append((t, m.name, m.msg))
        else:
            for tr in m.transforms:
                q, p = tr.transform.rotation, tr.transform.translation
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    mo = (p.x, p.y, yaw(q))
                elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link' and mo:
                    c, s = math.cos(mo[2]), math.sin(mo[2])
                    pose.append((t, mo[0] + c * p.x - s * p.y, mo[1] + s * p.x + c * p.y, mo[2] + yaw(q)))
    nav = S['/cmd_vel_nav']
    tn = [x[0] for x in nav]
    tp = [x[0] for x in pose]
    tl = [x[0] for x in logs]
    ev, ult = [], -9
    for i, (t, vx, vy, wz) in enumerate(nav):
        if abs(vy) < 0.012 or abs(vy) < 2 * abs(vx) or t - ult < 3.0:
            continue
        j = bisect.bisect_left(tn, t - 0.6)
        if any(abs(x[2]) >= 0.01 for x in nav[j:i]):
            continue
        ult = t
        e = dict(t=round(t, 2), vy=round(vy, 3), vx_onset=round(vx, 3))
        for k in TOP[:3]:
            w = [x for x in S[k] if t - 0.2 <= x[0] <= t + 1.5]
            e['vx_max_' + k.strip('/')] = round(max([x[1] for x in w] or [0]), 3)
            e['vx_min_' + k.strip('/')] = round(min([x[1] for x in w] or [0]), 3)
        w = [x for x in nav if t <= x[0] <= t + 1.5]
        e['n_nav'] = len(w)
        e['n_nav_vx_fwd'] = sum(x[1] > 0.012 for x in w)
        e['n_nav_puro_lat'] = sum(abs(x[1]) < 0.004 and abs(x[2]) >= 0.03 for x in w)
        # antes: o que estava saindo ate 1 s antes (residuo do movimento anterior)
        wa = [x for x in S['/cmd_vel'] if t - 1.0 <= x[0] < t]
        e['vx_final_antes'] = round(max([x[1] for x in wa] or [0]), 3)
        k0 = bisect.bisect_left(tp, t)
        k1 = bisect.bisect_left(tp, t + 1.5)
        if k0 < len(pose) and k1 < len(pose):
            p0, p1 = pose[k0], pose[k1]
            dx, dy = p1[1] - p0[1], p1[2] - p0[2]
            c, s = math.cos(p0[3]), math.sin(p0[3])
            e['tf_frente_cm'] = round(100 * (c * dx + s * dy), 1)
            e['tf_lado_cm'] = round(100 * (-s * dx + c * dy), 1)
        a = bisect.bisect_right(tl, t + 0.05)
        ctx = [l for l in logs[max(0, a - 6):a] if l[1] == 'alinhador_passagem']
        e['log'] = [('%.2f' % (l[0] - t), l[2][:110]) for l in ctx[-3:]]
        ev.append(e)
    return ev


if __name__ == '__main__':
    out = {}
    for b in sys.argv[1:]:
        try:
            out[b] = analisar(b)
        except Exception as exc:
            out[b] = 'ERRO %s' % exc
        print(Path(b).parent.name, len(out[b]) if isinstance(out[b], list) else out[b], file=sys.stderr)
    json.dump(out, sys.stdout, indent=1, ensure_ascii=False)
