#!/usr/bin/env python3
"""Reconstrucao ciclo a ciclo do giro travado (2nav 12:37:10-42), so leitura do bag.
Para cada ciclo do controller (/cmd_vel_ctrl): pose odom->base_link, fim do caminho do FollowPath atual
convertido para odom de 2 jeitos (A: map->odom no carimbo da pose final, como o tf->transform do
nav_2d_utils::transformPose do Nav2 1.3.12; B: map->odom mais recente), o SimpleGoalChecker (stateful,
reset a cada caminho novo, como controller_server::setPlannerPath) nos 2 jeitos, wz da odom (velocidade que
o DWB usa para a janela), wz comandado, trajetorias validas do /evaluation e eventos do BT.
Uso: reconstruir_giro_travado.py <bag> <t_ini> <t_fim> <saida_dir> [xy_tol yaw_tol]"""
import sys, os, math, bisect, csv, datetime
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
bag, a, b, out = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), sys.argv[4]
XY = float(sys.argv[5]) if len(sys.argv) > 5 else 0.06
YAW = float(sys.argv[6]) if len(sys.argv) > 6 else 0.17
os.makedirs(out, exist_ok=True)
H = lambda t: datetime.datetime.fromtimestamp(t).strftime('%H:%M:%S.%f')[:-4]
yaw = lambda q: math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
ang = lambda x: math.atan2(math.sin(x), math.cos(x))
st = lambda s: s.sec + s.nanosec * 1e-9
tops = ['/tf', '/received_global_plan', '/follow_path/_action/status', '/cmd_vel_ctrl', '/odom', '/evaluation',
        '/behavior_tree_log', '/navigate_to_pose/_action/status']
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
tipos = {t.name: t.type for t in r.get_all_topics_and_types()}
r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in tops if t in tipos]))
C = {t: get_message(tipos[t]) for t in tops if t in tipos}
mo = []      # (stamp, x, y, yaw) map->odom pelo carimbo do header
ob = []      # (t_bag, stamp, x, y, yaw) odom->base_link
planos, ctrl, odom, ev, bt, fp = [], [], [], [], [], []
while r.has_next():
    top, data, t = r.read_next(); t *= 1e-9
    if t < a - 30 or t > b + 1:
        continue
    m = deserialize_message(data, C[top])
    if top == '/tf':
        for tr in m.transforms:
            p = (st(tr.header.stamp), tr.transform.translation.x, tr.transform.translation.y, yaw(tr.transform.rotation))
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                mo.append(p)
            elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_link':
                ob.append((t,) + p)
    elif t < a:
        if top == '/received_global_plan' and m.poses:
            planos.append((t, m))
        continue
    elif top == '/received_global_plan' and m.poses:
        planos.append((t, m))
    elif top == '/cmd_vel_ctrl':
        ctrl.append((t, m.twist.linear.x, m.twist.linear.y, m.twist.angular.z))
    elif top == '/odom':
        odom.append((t, m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z))
    elif top == '/evaluation':
        best = m.twists[m.best_index].traj.velocity if 0 <= m.best_index < len(m.twists) else None
        ev.append((t, len(m.twists), sum(1 for tw in m.twists if tw.total >= 0), best))
    elif top == '/behavior_tree_log':
        for e in m.event_log:
            if e.node_name in ('FollowPath', 'ComputePathToPose', 'SmoothPath', 'IsPathValid', 'PathExpiringTimer',
                               'CaminhoAtualServe', 'ManterCaminho', 'PlanejarESuavizar', 'NavigateRecovery',
                               'GoalNovo', 'DriveOnHeading', 'RecuperacaoAvaliada', 'GlobalUpdatedGoal'):
                bt.append((t, e.node_name, e.previous_status, e.current_status))
    elif top in ('/follow_path/_action/status', '/navigate_to_pose/_action/status'):
        fp.append((t, top.split('/')[1], [(bytes(g.goal_info.goal_id.uuid).hex()[:8], g.status) for g in m.status_list]))
mo.sort()
mo_t = [x[0] for x in mo]
def map_odom(stamp):
    """tf2: interpolacao linear entre as duas amostras; stamp 0 = a mais recente."""
    if stamp == 0:
        return mo[-1][1:]
    i = bisect.bisect_left(mo_t, stamp)
    if i <= 0: return mo[0][1:]
    if i >= len(mo): return mo[-1][1:]
    (t0, x0, y0, w0), (t1, x1, y1, w1) = mo[i-1], mo[i]
    f = (stamp - t0) / (t1 - t0) if t1 > t0 else 0
    return (x0 + f*(x1-x0), y0 + f*(y1-y0), w0 + f*ang(w1-w0))
def mo_ate(t):  # map->odom mais recente recebido ate o instante t do bag
    i = bisect.bisect_right(mo_t, t + 0.35) - 1   # carimbo vem ~0,30 s a frente (transform_timeout)
    return mo[max(i, 0)][1:]
def para_odom(px, py, pw, T):
    x, y, w = T
    dx, dy = px - x, py - y
    c, s = math.cos(w), math.sin(w)
    return (c*dx + s*dy, -s*dx + c*dy, ang(pw - w))
ob_t = [x[0] for x in ob]
def robo(t):
    i = min(max(bisect.bisect_right(ob_t, t) - 1, 0), len(ob) - 1)
    return ob[i][2:]
# caminho ativo do FollowPath = ultimo /received_global_plan publicado ate t (o DWB publica ao receber)
pl_t = [p[0] for p in planos]
eps = []
# info dos caminhos
with open(os.path.join(out, 'caminhos.csv'), 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['hora_recebido', 'n', 'header_stamp', 'stamp_pose_final', 'fim_map_x', 'fim_map_y', 'fim_map_yaw_graus'])
    vistos = set()
    for t, m in planos:
        if st(m.header.stamp) in vistos: continue
        vistos.add(st(m.header.stamp))
        e = m.poses[-1]
        w.writerow([H(t), len(m.poses), H(st(m.header.stamp)) if st(m.header.stamp) else 0,
                    H(st(e.header.stamp)) if st(e.header.stamp) else 0, round(e.pose.position.x, 3),
                    round(e.pose.position.y, 3), round(math.degrees(yaw(e.pose.orientation)), 1)])
ev_t = [e[0] for e in ev]; od_t = [o[0] for o in odom]
estado = {'A': True, 'B': True}   # check_xy_
ult_plano = None; linhas = []
for (t, vx, vy, wz) in ctrl:
    k = bisect.bisect_right(pl_t, t) - 1
    if k < 0: continue
    tp, m = planos[k]
    chave = st(m.header.stamp)   # o DWB republica o caminho podado a cada ciclo; caminho novo = carimbo novo
    if ult_plano != chave:
        estado = {'A': True, 'B': True}; ult_plano = chave; novo = True   # setPlannerPath -> reset()
    else:
        novo = False
    e = m.poses[-1]
    stamp = st(e.header.stamp)
    ex, ey, ew = e.pose.position.x, e.pose.position.y, yaw(e.pose.orientation)
    gA = para_odom(ex, ey, ew, map_odom(stamp))
    gB = para_odom(ex, ey, ew, mo_ate(t))
    rx, ry, rw = robo(t)
    res = {}
    for nome, g in (('A', gA), ('B', gB)):
        d = math.hypot(rx - g[0], ry - g[1])
        if estado[nome] and d <= XY:
            estado[nome] = False
        res[nome] = (d, ang(g[2] - rw), (not estado[nome]) and abs(ang(g[2] - rw)) <= YAW)
    j = bisect.bisect_right(ev_t, t) - 1
    evv = ev[j] if j >= 0 and t - ev[j][0] < 0.2 else None
    o = odom[max(bisect.bisect_right(od_t, t) - 1, 0)]
    linhas.append(dict(hora=H(t), caminho_novo=novo, robo_odom='%.3f,%.3f' % (rx, ry), robo_yaw=round(math.degrees(rw), 1),
                       alvoA_odom='%.3f,%.3f' % gA[:2], alvoA_yaw=round(math.degrees(gA[2]), 1),
                       distA=round(res['A'][0], 3), xy_registrado_A=not estado['A'], chegou_A=res['A'][2],
                       alvoB_odom='%.3f,%.3f' % gB[:2], distB=round(res['B'][0], 3), erro_yawB=round(math.degrees(res['B'][1]), 1),
                       xy_registrado_B=not estado['B'], chegou_B=res['B'][2],
                       wz_odom=round(o[3], 3), cmd_vx=round(vx, 3), cmd_vy=round(vy, 3), cmd_wz=round(wz, 3),
                       validas=(evv[2] if evv else ''), amostradas=(evv[1] if evv else '')))
with open(os.path.join(out, 'ciclos_controller.csv'), 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(linhas[0])); w.writeheader(); w.writerows(linhas)
with open(os.path.join(out, 'eventos_bt_acoes.csv'), 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['hora', 'fonte', 'no/acao', 'de', 'para'])
    for t, n, p, c in bt: w.writerow([H(t), 'bt', n, p, c])
    prev = {}
    for t, nome, lst in fp:
        for gid, s in lst:
            if prev.get((nome, gid)) != s:
                prev[(nome, gid)] = s; w.writerow([H(t), nome, gid, '', s])
# fixture numerica para o teste C++ com o codigo real do Nav2 (giro_travado/fixture_*.csv)
with open(os.path.join(out, 'fixture_map_odom.csv'), 'w') as f:
    for s_, x, y, yw in mo:
        if a - 40 <= s_ <= b + 1: f.write('%.6f %.5f %.5f %.6f\n' % (s_, x, y, yw))
with open(os.path.join(out, 'fixture_ciclos.csv'), 'w') as f:
    ult = None
    for (t, vx, vy, wz) in ctrl:
        k = bisect.bisect_right(pl_t, t) - 1
        if k < 0: continue
        m = planos[k][1]; e = m.poses[-1]
        rx, ry, rw = robo(t)
        f.write('%.6f %.5f %.5f %.6f %.6f %.5f %.5f %.6f\n' % (t, rx, ry, rw, st(e.header.stamp), e.pose.position.x,
                e.pose.position.y, yaw(e.pose.orientation)))
with open(os.path.join(out, 'map_odom.csv'), 'w', newline='') as f:
    w = csv.writer(f); w.writerow(['stamp', 'x', 'y', 'yaw_graus'])
    for s, x, y, yw in mo:
        if a - 20 <= s <= b + 1: w.writerow([H(s), round(x, 3), round(y, 3), round(math.degrees(yw), 2)])
print('ciclos:', len(linhas), '| caminhos na janela:', sum(1 for p in planos if p[0] >= a))
for l in linhas:
    if l['caminho_novo'] or l['hora'].endswith(('0', '5')) and l['hora'][-4] in '02468':
        pass
