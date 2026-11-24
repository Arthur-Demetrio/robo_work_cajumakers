import sys, math, bisect, pickle
from collections import Counter
sys.path.insert(0, sys.argv[2])
from bagio import open_bag, deser
from datetime import datetime

BAG, OUT = sys.argv[1], sys.argv[3]
FP = (-0.205, 0.365, -0.315, 0.315)  # footprint físico (sem padding)
LASER = (0.33, 0.005)

ts, it = open_bag(BAG, topics=['/tf', '/scan_front', '/cmd_vel_nav', '/cmd_vel_smoothed', '/cmd_vel',
                                '/collision_monitor_state', '/evaluation', '/map', '/plan', '/plan_smoothed',
                                '/behavior_tree_log', '/local_costmap/costmap', '/local_costmap/costmap_updates'])
mo, ob = [], []
scans, cmds, cm, ev, bt = [], {'/cmd_vel_nav': [], '/cmd_vel_smoothed': [], '/cmd_vel': []}, [], [], []
lastmap = None
plans = []
lc = []
grid = None
for top, t, d, typ in it():
    m = deser(ts, d, typ)
    if top == '/tf':
        for tr in m.transforms:
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
            rec = (t, tr.transform.translation.x, tr.transform.translation.y, yaw)
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                mo.append(rec)
            elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link':
                ob.append(rec)
    elif top == '/scan_front':
        pts = []
        for i, r in enumerate(m.ranges):
            if math.isfinite(r) and m.range_min <= r <= m.range_max:
                a = m.angle_min + i * m.angle_increment
                pts.append((LASER[0] + r * math.cos(a), LASER[1] + r * math.sin(a)))
        scans.append((t, pts))
    elif top in cmds:
        tw = m.twist
        cmds[top].append((t, tw.linear.x, tw.linear.y, tw.angular.z))
    elif top == '/collision_monitor_state':
        cm.append((t, m.action_type))
    elif top == '/evaluation':
        legal = [x for x in m.twists if x.total >= 0]
        fal = Counter(x.scores[0].name if False else next((s.name for s in x.scores if s.raw_score < 0), '?') for x in m.twists if x.total < 0)
        best = m.twists[m.best_index] if legal and 0 <= m.best_index < len(m.twists) else None
        bv = (best.traj.velocity.x, best.traj.velocity.y, best.traj.velocity.theta) if best else None
        ev.append((t, len(m.twists), len(legal), fal, bv))
    elif top == '/map':
        lastmap = m
    elif top in ('/plan', '/plan_smoothed'):
        plans.append((t, top, [(p.pose.position.x, p.pose.position.y) for p in m.poses]))
    elif top == '/behavior_tree_log':
        for e in m.event_log:
            bt.append((e.timestamp.sec + e.timestamp.nanosec * 1e-9, e.node_name, e.previous_status, e.current_status))

def compose(a, b):
    x, y, th = a
    return (x + b[0] * math.cos(th) - b[1] * math.sin(th), y + b[0] * math.sin(th) + b[1] * math.cos(th), th + b[2])

mot = [r[0] for r in mo]; obt = [r[0] for r in ob]
def pose(t):
    i = bisect.bisect_right(mot, t) - 1; j = bisect.bisect_right(obt, t) - 1
    if i < 0 or j < 0:
        return None
    p = compose(mo[i][1:], ob[j][1:])
    return (p[0], p[1], math.atan2(math.sin(p[2]), math.cos(p[2])))

pickle.dump(dict(mo=mo, ob=ob, scans=scans, cmds=cmds, cm=cm, ev=ev, plans=plans, bt=bt,
                 map=(lastmap.info.resolution, lastmap.info.width, lastmap.info.height,
                      lastmap.info.origin.position.x, lastmap.info.origin.position.y, bytes(lastmap.data.astype('int8').tobytes())) if lastmap is not None else None),
            open(OUT, 'wb'))
print('ok', len(mo), len(ob), len(scans), len(ev), len(bt), len(plans))
