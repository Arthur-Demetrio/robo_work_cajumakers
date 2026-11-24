#!/usr/bin/env python3
"""TABELA_LATERAL (Patch 03): episodios de deslize lateral (|vy|>0,02 no /cmd_vel_nav por >1 s).
Para cada um: fonte (DWB pelo /cmd_vel_ctrl, senao alinhador/destravar pelo log), vx ordenado (decomposto
no modelo do TravessiaLateral: ff=-k*vy, parede +-0,03, centro +-0,04), e deslocamento no frame do robo
pelo SLAM (map->base_link, casamento de scan) e pela odom (derivada do comando: NAO e prova independente).
Uso: lateral_vx.py <dados> <saida.csv> [k_ff]"""
import sys, os, csv, math, bisect
sys.path.insert(0, os.path.dirname(__file__))
from comum import ler, hora, Poses
pasta, saida = sys.argv[1], sys.argv[2]
K = float(sys.argv[3]) if len(sys.argv) > 3 else 0.2
cmd = ler(pasta, 'cmd_vel_nav.jsonl'); ctrl = ler(pasta, 'cmd_vel_ctrl.jsonl'); tc = [c['t'] for c in ctrl]
ros = [(d['t'], d['name'], d['msg']) for d in ler(pasta, 'rosout.jsonl') if d['name'] in ('alinhador_passagem', 'destravar')]
po = Poses(pasta)
eps, ini, ult = [], None, None
for d in cmd:
    if abs(d['vy']) > 0.02:
        if ini is None or d['t'] - ult > 0.6:
            if ini is not None and ult - ini > 1.0:
                eps.append((ini, ult))
            ini = d['t']
        ult = d['t']
if ini is not None and ult - ini > 1.0:
    eps.append((ini, ult))
def desloc(p0, p1, i_x, i_y, i_yaw):
    dx, dy = p1[i_x] - p0[i_x], p1[i_y] - p0[i_y]
    c, s = math.cos(p0[i_yaw]), math.sin(p0[i_yaw])
    return c*dx + s*dy, -s*dx + c*dy
linhas = []
for a, b in eps:
    am = [d for d in cmd if a <= d['t'] <= b and abs(d['vy']) > 0.02]
    k0, k1 = bisect.bisect_left(tc, a), bisect.bisect_right(tc, b)
    dwb = sum(1 for c in ctrl[k0:k1] if abs(c['vy']) > 0.02) > 3
    logs = [m for (t, n, m) in ros if a - 3 <= t <= b]
    if dwb:
        fonte = 'DWB'
    elif any('DESTRAVAR' in m for (t, n, m) in ros if a - 5 <= t <= b):
        fonte = 'destravar'
    else:
        fonte = 'alinhador'
    vy = sum(d['vy'] for d in am) / len(am)
    vxs = [d['vx'] for d in am]
    resto = [d['vx'] + K * d['vy'] for d in am]   # o que sobra tirando o feedforward
    n = len(am)
    cls = dict(so_ff=sum(1 for r in resto if abs(r) < 0.006),
               parede_ou_centro=sum(1 for r in resto if abs(r) >= 0.006))
    p0, p1 = po.em(a), po.em(b + 0.3)
    sx, sy = desloc(p0, p1, 0, 1, 2)          # SLAM (map)
    ox, oy = desloc(p0, p1, 3, 4, 5)          # odom
    pedido_x = sum(d['vx'] for d in am) * 0.1 if n else 0
    linhas.append(dict(
        hora=hora(a), duracao_s=round(b - a, 1), fonte=fonte, vy_medio=round(vy, 3),
        vx_medio=round(sum(vxs) / n, 3), vx_min=round(min(vxs), 3), vx_max=round(max(vxs), 3),
        frac_vx_negativo=round(sum(1 for v in vxs if v < -0.004) / n, 2),
        frac_so_feedforward=round(cls['so_ff'] / n, 2), frac_com_parede_ou_centro=round(cls['parede_ou_centro'] / n, 2),
        vx_ff_esperado=round(-K * vy, 3) if fonte == 'alinhador' else '',
        slam_dx_cm=round(100 * sx, 1), slam_dy_cm=round(100 * sy, 1), odom_dx_cm=round(100 * ox, 1),
        odom_dy_cm=round(100 * oy, 1),
        razao_slam_dx_dy=round(sx / sy, 2) if abs(sy) > 0.02 else '',
        log=' | '.join(m[:90] for m in logs if any(k in m for k in ('RESULTADO', 'DECISAO', 'Perto da parede', 'PARADA', 'saindo de lado', 'lateral')))[:300]))
with open(saida, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(linhas[0])); w.writeheader(); w.writerows(linhas)
for l in linhas:
    print(l['hora'], l['fonte'], 'dur', l['duracao_s'], 'vy', l['vy_medio'], 'vx med/min', l['vx_medio'], l['vx_min'],
          'neg', l['frac_vx_negativo'], 'so_ff', l['frac_so_feedforward'], '| SLAM dx,dy cm', l['slam_dx_cm'], l['slam_dy_cm'],
          '| odom', l['odom_dx_cm'], l['odom_dy_cm'])
