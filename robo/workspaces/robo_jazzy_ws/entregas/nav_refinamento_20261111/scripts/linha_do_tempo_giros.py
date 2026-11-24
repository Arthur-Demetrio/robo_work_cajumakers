#!/usr/bin/env python3
"""LINHA_DO_TEMPO e TABELA_GIROS (11/11) a partir dos JSONL extraidos (extrair_bag.py).
Uso: linha_do_tempo_giros.py <dados> <saida_prefixo> [min_giro_graus]
Atribuicao de cada giro:
  - DWB (Nav2): /cmd_vel_ctrl (saida do controller_server) com |wz|>0,03 durante o giro;
  - senao, o ultimo log do alinhador/destravar antes ou durante o giro (eles publicam direto no /cmd_vel_nav).
Angulo real: integral da IMU (/imu/yaw_deg, desenrolado). Preempcao: 'Novo goal' do alinhador com goal anterior
ainda ativo = troca intencional do operador (nao e falha)."""
import sys, os, re, csv, math, bisect
sys.path.insert(0, os.path.dirname(__file__))
from comum import ler, hora, ang, Poses

pasta, pref = sys.argv[1], sys.argv[2]
MIN = float(sys.argv[3]) if len(sys.argv) > 3 else 45.0
ros = ler(pasta, 'rosout.jsonl')
t0 = ros[0]['t'] if ros else 0
ev = []

def add(t, fonte, evento, detalhe, classe, prova):
    ev.append(dict(hora=hora(t), t_rel=round(t - t0, 2), fonte=fonte, evento=evento, detalhe=detalhe[:220],
                   classe=classe, prova=prova))

goal_ativo = False
goals = []  # (t, x, y, yaw)
for d in ros:
    m, n, t = d['msg'].replace('\n', ' '), d['name'], d['t']
    ref = 'rosout %s %s:%s' % (n, os.path.basename(d.get('file', '')), d.get('line', ''))
    if n == 'alinhador_passagem':
        g = re.match(r'Novo goal: x=([-\d.]+) y=([-\d.]+) yaw=([-\d.]+)', m)
        if g:
            x, y, yw = map(float, g.groups())
            goals.append((t, x, y, math.radians(yw)))
            add(t, 'operador', 'GOAL_NOVO' + ('_PREEMPCAO' if goal_ativo else ''), m,
                'operador (troca intencional)' if goal_ativo else 'operador', ref)
            goal_ativo = True
            continue
        for chave, evento, classe in (
                ('Goal substituido', 'GOAL_SUBSTITUIDO', 'operador (preempcao, nao e falha)'),
                ('virar para o caminho): o caminho sai', 'PRE_GIRO_DECIDIDO', 'autonomo'),
                ('TRAJETO CURTO', 'TRAJETO_CURTO', 'autonomo'),
                ('ANTI_CISCADO', 'ANTI_CISCADO', 'autonomo'),
                ('interrompido para alinhar', 'ALINHADOR_ASSUME_CORREDOR', 'autonomo (cancel interno)'),
                ('FALHOU', 'FALHA_ALINHADOR', 'falha autonoma'),
                ('Nao consegui', 'PEDE_AJUDA', 'falha autonoma'),
                ('SUCESSO', 'SUCESSO_ETAPA', 'autonomo'),
                ('PARADA DE SEGURANCA', 'PARADA_SEGURANCA', 'autonomo'),
                ('Collision Monitor segurando', 'CM_SEGURA_ALINHADOR', 'autonomo'),
                ('Planner:', 'ROTA_MAPA_DECISAO', 'autonomo'),
                ('GOAL FINAL: x=', 'GOAL_NAV2_ENVIADO', 'autonomo'),
                ('PREPARO', 'PREPARO', 'autonomo'), ('TRAVESSIA', 'TRAVESSIA', 'autonomo'),
                ('CURVA', 'CURVA', 'autonomo')):
            if chave in m:
                if evento == 'SUCESSO_ETAPA' and 'Medicao' in m:
                    continue
                add(t, 'alinhador', evento, m, classe, ref)
                if evento in ('PEDE_AJUDA',) or (evento == 'FALHA_ALINHADOR' and 'GOAL FINAL' in m):
                    goal_ativo = False
                break
    elif n == 'destravar' and any(k in m for k in ('DIAGNOSTICO', 'DECISAO', 'RESULTADO', 'FALHA', 'OK:', 'SEM CAMINHO')):
        add(t, 'destravar', 'DESTRAVAR', m, 'falha autonoma' if 'FALHA' in m else 'autonomo', ref)
    elif n == 'bt_navigator' and ('Begin navigating' in m or 'Goal' in m or 'cancel' in m.lower()):
        add(t, 'nav2', 'BT_NAVIGATOR', m, 'falha autonoma' if 'failed' in m.lower() else 'info', ref)
    elif n == 'controller_server' and ('legal trajectory' in m or 'progress' in m):
        add(t, 'nav2', 'DWB', m, 'autonomo', ref)
    elif n == 'planner_server' and 'failed to plan' in m:
        add(t, 'nav2', 'PLANNER_SEM_CAMINHO', m, 'autonomo', ref)
    elif n == 'collision_monitor' and 'stop' in m:
        add(t, 'cm', 'CM_PARA', m, 'protecao', ref)
# status da acao navigate_to_pose
vistos = {}
NOMES = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
for d in ler(pasta, 'navigate_to_pose___action__status.jsonl'):
    for gid, st, _ in d['s']:
        if st in NOMES and vistos.get(gid) != st:
            vistos[gid] = st
            add(d['t'], 'nav2', 'NAV2_GOAL_' + NOMES[st], 'goal %s' % gid,
                'falha autonoma' if st == 6 else ('cancel (alinhador/operador)' if st == 5 else 'autonomo'),
                '/navigate_to_pose/_action/status')
ev.sort(key=lambda e: e['t_rel'])
with open(pref + 'LINHA_DO_TEMPO.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(ev[0])); w.writeheader(); w.writerows(ev)

# ---------------------------------------------------------------- giros
imu = ler(pasta, 'imu__yaw_deg.jsonl')
ti = [d['t'] for d in imu]
yaw_u, ac, ant = [], 0.0, None
for d in imu:
    v = d['yaw_deg']
    if ant is not None:
        dv = (v - ant + 180) % 360 - 180
        ac += dv
    else:
        ac = v
    ant = v
    yaw_u.append(ac)
def yaw_imu(t):
    i = min(max(bisect.bisect_left(ti, t), 0), len(ti) - 1)
    return yaw_u[i]
cmd = ler(pasta, 'cmd_vel_nav.jsonl')
ctrl = ler(pasta, 'cmd_vel_ctrl.jsonl')
tc = [d['t'] for d in ctrl]
po = Poses(pasta)
# episodios: |wz| > 0,05 no /cmd_vel_nav, juntando buracos < 0,8 s e SEPARANDO quando muda a fonte
# (amostra com /cmd_vel_ctrl |wz|>0,03 nos 0,3 s anteriores = DWB; senao = publicador direto)
def fonte(t):
    k = bisect.bisect_right(tc, t) - 1
    while k >= 0 and t - tc[k] < 0.3:
        if abs(ctrl[k]['wz']) > 0.03:
            return 'dwb'
        k -= 1
    return 'direto'
eps, ini, ult, fat = [], None, None, None
for d in cmd:
    if abs(d['wz']) > 0.05:
        f = fonte(d['t'])
        if ini is None or d['t'] - ult > 0.8 or f != fat:
            if ini is not None:
                eps.append((ini, ult))
            ini, fat = d['t'], f
        ult = d['t']
if ini is not None:
    eps.append((ini, ult))
logs = [(d['t'], d['name'], d['msg'].replace('\n', ' ')) for d in ros if d['name'] in ('alinhador_passagem', 'destravar')]
tl = [l[0] for l in logs]
planos = ler(pasta, 'plan.jsonl'); tp = [p['t'] for p in planos]
cm_stops = [d['t'] for d in ros if d['name'] == 'collision_monitor' and 'stop' in d['msg']]

def origem(a, b):
    k0, k1 = bisect.bisect_left(tc, a), bisect.bisect_right(tc, b)
    n_ctrl = sum(1 for d in ctrl[k0:k1] if abs(d['wz']) > 0.03)
    if n_ctrl >= 3:
        return 'DWB (Nav2 controller)', ''
    j = bisect.bisect_right(tl, b) - 1
    while j >= 0 and logs[j][0] > a - 15:
        _, nome, m = logs[j]
        if nome == 'destravar':
            return 'destravar', m[:120]
        for chave, rot in (('virar para o caminho', 'alinhador VIRAR_CAMINHO (pre-giro)'),
                           ('TRAJETO CURTO', 'alinhador TRAJETO_CURTO (yaw final)'),
                           ('GIRO_FINAL', 'alinhador GIRO_FINAL'), ('giro final', 'alinhador GIRO_FINAL'),
                           ('PREPARO', 'alinhador PREPARO (corredor)'), ('TRAVESSIA', 'alinhador TRAVESSIA'),
                           ('CURVA', 'alinhador CURVA'), ('ANTI_CISCADO', 'alinhador ANTI_CISCADO')):
            if chave in m:
                return rot, m[:120]
        j -= 1
    return 'indeterminado', ''

giros = []
for a, b in eps:
    dy = yaw_imu(b + 0.15) - yaw_imu(a)
    if abs(dy) < MIN:
        continue
    rot, msg = origem(a, b)
    rb = po.em(a)
    gi = bisect.bisect_right([g[0] for g in goals], a) - 1
    g = goals[gi] if gi >= 0 else None
    goal_novo = g is not None and a - g[0] < 20.0
    k = bisect.bisect_right(tp, a) - 1
    head = None
    if k >= 0 and len(planos[k]['pts']) > 3:
        pts = planos[k]['pts']
        # rumo a ~0,5 m do inicio do plano
        s, j = 0.0, 1
        while j < len(pts) and s < 0.5:
            s += math.hypot(pts[j][0] - pts[j-1][0], pts[j][1] - pts[j-1][1]); j += 1
        head = math.degrees(math.atan2(pts[j-1][1] - pts[0][1], pts[j-1][0] - pts[0][0]))
    jm = [l for l in logs if a - 15 <= l[0] <= b + 3]
    obst = any('quina' in l[2] or 'Collision Monitor' in l[2] for l in jm) or any(a <= c <= b for c in cm_stops)
    res = 'SUCESSO' if any('SUCESSO' in l[2] for l in jm if l[0] >= a) else (
          'FALHOU' if any(('FALHOU' in l[2] or 'nao consegui' in l[2]) for l in jm if l[0] >= a) else '')
    giros.append(dict(hora=hora(a), t_rel=round(a - t0, 1), duracao_s=round(b - a, 1), delta_imu_graus=round(dy),
                      origem=rot, yaw_atual_mapa=round(math.degrees(rb[2])),
                      yaw_goal=(round(math.degrees(g[3])) if g else ''),
                      heading_plano_0_5m=(round(head) if head is not None else ''),
                      goal_novo_ate_20s=goal_novo, obstaculo_ou_quina=obst, resultado=res, log=msg))
with open(pref + 'TABELA_GIROS.csv', 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(giros[0]) if giros else ['hora']); w.writeheader(); w.writerows(giros)
from collections import Counter, defaultdict
c = Counter(g['origem'] for g in giros)
dur = defaultdict(float); ang_t = defaultdict(float)
for g in giros:
    dur[g['origem']] += g['duracao_s']; ang_t[g['origem']] += abs(g['delta_imu_graus'])
print('eventos na linha do tempo:', len(ev))
print('giros >= %.0f graus: %d' % (MIN, len(giros)))
for k, v in c.most_common():
    print('  %-40s %2d giros  %5.1f s  %4.0f graus' % (k, v, dur[k], ang_t[k]))
for g in giros:
    print(' ', g['hora'], g['delta_imu_graus'], g['duracao_s'], g['origem'], g['resultado'])
