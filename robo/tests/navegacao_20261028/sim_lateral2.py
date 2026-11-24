"""Simula a saida lateral (TravessiaLateral) em cruzamentos L, T e + com a fisica do robo real.

Robo chega pelo braco Oeste andando para +x e para com o corpo centrado no corredor novo.
Fisica copiada do harness (teste_no_ros_inercia.Mundo): smoother, atraso, ganhos 3.2/3.4,
heading hold da ponte, chute de yaw quando vy muda, inercia do giro, odom = comando x 5/3.4.
"""
import importlib.util, math, random, sys, os
D = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, D)
import sim as S
sp = importlib.util.spec_from_file_location('al', os.environ.get('AL', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'robo_navigation', 'scripts', 'alinhador_passagem.py')))
al = importlib.util.module_from_spec(sp); sp.loader.exec_module(al)

import os
PISO = os.environ.get('PISO', '1') == '1'   # modelo com PWM minimo (real)
DERIVA = os.environ.get('DERIVA', '1') == '1'  # curva4: deriva sistematica + rajadas, largura variavel
M = 0.41      # meia largura dos corredores
L = 1.6       # comprimento de cada braco a partir da borda do cruzamento


def segs_cruzamento(bracos, m=M, comp=L):
    """bracos: subconjunto de 'NSLO' (O sempre presente: o robo vem dele)."""
    segs = []
    lados = {'L': ((m, -m), (m, m)), 'O': ((-m, -m), (-m, m)),
             'N': ((-m, m), (m, m)), 'S': ((-m, -m), (m, -m))}
    for k, (a, b) in lados.items():
        if k not in bracos:
            segs.append((a, b))  # parede fechando o cruzamento
    for k in bracos:
        if k in 'LO':
            sg = 1 if k == 'L' else -1
            segs.append(((sg * m, m), (sg * (m + comp), m)))
            segs.append(((sg * m, -m), (sg * (m + comp), -m)))
        else:
            sg = 1 if k == 'N' else -1
            segs.append(((m, sg * m), (m, sg * (m + comp))))
            segs.append(((-m, sg * m), (-m, sg * (m + comp))))
    return segs


def rodar(bracos, lado, seed, verbose=False):
    rnd = random.Random(seed)
    m_ = rnd.uniform(0.395, 0.435) if DERIVA else M
    segs = segs_cruzamento(bracos, m=m_)
    vies = rnd.uniform(-0.20, 0.20) if DERIVA else 0.0   # deriva para frente/tras por metro de lado
    rajada_ate, rajada = -1.0, 0.0
    ex, ey, eyaw = rnd.uniform(-0.04, 0.04), rnd.uniform(-0.03, 0.03), math.radians(rnd.uniform(-3, 3))
    pose = [-0.08 + ex, ey, eyaw]
    odom = [0.0, 0.0, pose[2]]
    logs = []
    tl = al.TravessiaLateral(lado, log=logs.append)
    sm = [0.0, 0.0, 0.0]; fila = []; hh = pose[2]; gw = rnd.uniform(2.0, 5.0); ult_vy = 0.0
    wz_f = 0.0; t = 0.0; prox_scan = 0.0; dt = 0.01; minf = 9.0
    saida_b = (0.0, lado * (M + L))
    cmd = (0.0, 0.0, 0.0); prox_ctrl = 0.0
    while t < 90:
        if t >= prox_scan:
            prox_scan = t + 1 / 5.5
            x, y, th = pose
            lx, ly = x + 0.33 * math.cos(th), y + 0.33 * math.sin(th)
            pts = []
            for i in range(180):
                a = -math.pi + i * 2 * math.pi / 180
                r = S.raio(lx, ly, math.cos(th + a), math.sin(th + a), segs)
                if r is None or r > 8:
                    continue
                r += rnd.gauss(0, 0.012)
                bx, by = 0.33 + r * math.cos(a), r * math.sin(a)
                if (bx < 0.19 and abs(by) < 0.40) if not DERIVA else (-0.205 <= bx <= 0.365 and abs(by) <= 0.315):
                    continue
                pts.append((bx, by))
            tl.nova_medicao(t, al.medir_corredor_lateral(pts, tl.lado, tl.largura), tuple(odom))
            if hasattr(tl, 'nova_folga'):
                tl.nova_folga(t, *al.folgas_lateral(pts, tl.lado))
        if t >= prox_ctrl:  # controle a 20 Hz como no no
            prox_ctrl = t + 0.05
            restante = 0.40 - ((pose[0] - saida_b[0]) * 0 + (pose[1] - saida_b[1]) * lado)
            cmd = tl.passo(t, tuple(odom), restante)
            if tl.estado in ('fim', 'abortar'):
                break
        for k in range(3):
            d = cmd[k] - sm[k]
            lim = (0.2 if abs(cmd[k]) > abs(sm[k]) else 0.5) * dt
            sm[k] += max(-lim, min(lim, d))
        out = [v if abs(v) >= 0.015 else 0.0 for v in sm]
        fila.append(out)
        a_ = fila.pop(0) if len(fila) > 10 else [0, 0, 0]
        # PWM minimo da ponte: qualquer comando >= zona morta ja gira ~15 graus/s (medido 26/10)
        wz = (math.copysign(max(0.25, gw * abs(a_[2])), a_[2]) if a_[2] else 0.0) if PISO else gw * a_[2]
        andando = abs(a_[0]) + abs(a_[1]) > 0
        if a_[2] != 0 or not andando:
            hh = pose[2]
        else:
            e = math.atan2(math.sin(hh - pose[2]), math.cos(hh - pose[2]))
            if abs(e) > math.radians(0.5):
                wz += max(-0.15, min(0.15, 1.6 * e)) * 0.5
        if andando:
            wz += math.radians(rnd.gauss(0.6, 1.2))
        wz_f += (wz - wz_f) * dt / 0.2
        if abs(a_[1] - ult_vy) > 0.01:
            pose[2] += math.radians(rnd.gauss(1.5, 1.5))
        ult_vy = a_[1]
        th = pose[2]
        # mecanum de lado anda um pouco torto: deriva em x proporcional a vy
        # PWM minimo: para frente/tras qualquer comando anda ~0,15-0,2 m/s (medido 26-27/10)
        vx_cmd = (math.copysign(0.15 + (abs(a_[0]) - 0.015) * 2.0, a_[0]) if a_[0] else 0.0) if PISO else 3.2 * a_[0]
        vxr, vyr = vx_cmd + 0.15 * 3.4 * a_[1] * rnd.uniform(-1, 1), 3.4 * a_[1]
        if DERIVA and a_[1]:
            # curva4: de lado o robo real deriva 10-20% para frente/tras e, as vezes,
            # ate 50% por ~2 s (volta: 11 cm para tras em 2 s)
            if t > rajada_ate and rnd.random() < dt / 6.0:
                rajada_ate, rajada = t + 2.0, rnd.choice((-1, 1)) * 0.5
            vxr += abs(vyr) * (rajada if t < rajada_ate else vies)
        pose[0] += (vxr * math.cos(th) - vyr * math.sin(th)) * dt
        pose[1] += (vxr * math.sin(th) + vyr * math.cos(th)) * dt
        pose[2] += wz_f * dt
        odom[2] = pose[2]
        vxo, vyo = 5.0 * a_[0], 3.4 * a_[1]
        odom[0] += (vxo * math.cos(th) - vyo * math.sin(th)) * dt
        odom[1] += (vxo * math.sin(th) + vyo * math.cos(th)) * dt
        minf = min(minf, S.folga(pose, segs))
        t += dt
    return tl.estado, t, minf, logs, pose


if __name__ == '__main__':
    casos = [('ON', 1, 'L esquerda'), ('OS', -1, 'L direita'),
             ('ONS', 1, 'T (vem da haste) esquerda'), ('ONS', -1, 'T (vem da haste) direita'),
             ('OLN', 1, 'T (na barra) entra na haste esq'), ('OLS', -1, 'T (na barra) entra na haste dir'),
             ('OLNS', 1, '+ esquerda'), ('OLNS', -1, '+ direita')]
    tot = ok = 0
    for bracos, lado, nome in casos:
        res = []
        for seed in range(15):
            est, t, mf, logs, pose = rodar(bracos, lado, seed)
            bom = est == 'fim' and mf > 0.0
            tot += 1; ok += bom
            res.append((bom, est, t, mf))
            if not bom and len([r for r in res if not r[0]]) <= 1:
                print('   FALHA %s seed %d: %s %.1fs folga %.1fcm | %s' % (nome, seed, est, t, mf * 100, ' / '.join(logs[-6:])))
        tempos = [r[2] for r in res if r[0]]
        print('%-34s ok %2d/15  tempo medio %4.1f s  folga minima %4.1f cm' % (
            nome, sum(r[0] for r in res), sum(tempos) / max(1, len(tempos)), min(r[3] for r in res) * 100))
    print('TOTAL', ok, '/', tot)
