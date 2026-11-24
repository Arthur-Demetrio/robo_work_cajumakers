import os
"""Simulacao grosseira do robo real para testar a Travessia antes de mandar ao Marcos.

Modelo: ganhos reais medidos nos bags (vx ~3.2x, vy ~3.4x, wz 2-5x o comando),
velocity_smoother (acel 0.2, zona morta 0.015), atraso 0.1 s, chutes de yaw ao mudar vy,
heading hold da ponte, odom = comando integrado x (5.0, 3.4) + yaw do IMU, LiDAR 2 graus a 5.5 Hz.
"""
import math, random, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from nucleo_corredor import medir_corredor
from controle_corredor import Travessia

FP = [(0.365, 0.315), (0.365, -0.315), (-0.205, -0.315), (-0.205, 0.315)]


def segs_corredor(eixo, meia, x_ent, x_sai, cy):
    c, s = math.cos(eixo), math.sin(eixo)
    def P(u, v):
        return (u * c - v * s, cy + u * s + v * c)
    segs = []
    for sg in (1, -1):
        segs.append((P(x_ent, sg * meia), P(x_sai, sg * meia)))             # lado da bancada
        segs.append((P(x_ent, sg * meia), P(x_ent, sg * (meia + 0.6))))     # face de entrada
        segs.append((P(x_sai, sg * meia), P(x_sai, sg * (meia + 0.6))))     # face de saida
    return segs


def raio(px, py, ux, uy, segs):
    best = None
    for (a, b) in segs:
        ex, ey = b[0] - a[0], b[1] - a[1]
        den = ux * ey - uy * ex
        if abs(den) < 1e-12:
            continue
        t = ((a[0] - px) * ey - (a[1] - py) * ex) / den
        u = ((a[0] - px) * uy - (a[1] - py) * ux) / den
        if t > 0.08 and 0 <= u <= 1 and (best is None or t < best):
            best = t
    return best


def dist_seg(p, a, b):
    ex, ey = b[0] - a[0], b[1] - a[1]
    L = ex * ex + ey * ey
    u = max(0, min(1, ((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / L))
    return math.hypot(p[0] - a[0] - u * ex, p[1] - a[1] - u * ey)


def folga(pose, segs):
    """Menor distancia entre o contorno do robo e as paredes (negativa ~ colisao)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    cantos = [(x + u * c - v * s, y + u * s + v * c) for u, v in FP]
    m = 9
    for i in range(4):
        a, b = cantos[i], cantos[(i + 1) % 4]
        for (p, q) in segs:
            for pt in (p, q):
                m = min(m, dist_seg(pt, a, b))
            for pt in (a, b):
                m = min(m, dist_seg(pt, p, q))
    return m


def rodar(seed, yaw0_err, lat0, eixo_graus=-3.5, largura=0.81, verbose=False):
    rnd = random.Random(seed)
    eixo = math.radians(eixo_graus)
    meia = largura / 2
    x_ent, x_sai = 0.75, 1.40
    segs = segs_corredor(eixo, meia, x_ent, x_sai, -0.35)
    c, s = math.cos(eixo), math.sin(eixo)
    # robo 0.6 m antes da entrada, deslocado lat0 e com erro de yaw
    u0, v0 = x_ent - 0.6, lat0
    pose = [u0 * c - v0 * s, -0.35 + u0 * s + v0 * c, eixo + math.radians(yaw0_err)]
    odom = [0.0, 0.0, pose[2]]
    logs = []
    tr = Travessia(log=(lambda m: logs.append(m)))
    dt = 0.02
    sm = [0.0, 0.0, 0.0]
    fila = []
    hh_alvo = pose[2]
    ganho_w = rnd.uniform(2.0, 4.0)
    ult_vy = 0.0
    prox_scan = 0.0
    t = 0.0
    min_folga = 9
    cmd = (0, 0, 0)
    while t < 70:
        # LiDAR
        if t >= prox_scan:
            prox_scan = t + 1 / 5.5
            x, y, th = pose
            lx, ly = x + 0.33 * math.cos(th), y + 0.33 * math.sin(th)
            pts = []
            for i in range(180):
                a = -math.pi + i * 2 * math.pi / 180
                r = raio(lx, ly, math.cos(th + a), math.sin(th + a), segs)
                if r is None or r > 8:
                    continue
                r += rnd.gauss(0, 0.008)
                bx, by = 0.33 + r * math.cos(a), r * math.sin(a)
                if bx < 0.19 and abs(by) < 0.40:
                    continue  # corpo do robo tampa
                pts.append((bx, by))
            med = medir_corredor(pts, largura_conhecida=tr.largura)
            tr.nova_medicao(t, med, tuple(odom))
        # restante pelo "mapa" (pose real + ruido)
        uu = (pose[0]) * c + (pose[1] + 0.35) * s
        restante = (x_sai + 0.30) - uu
        cmd = tr.passo(t, tuple(odom), restante)
        if tr.estado in ('fim', 'abortar'):
            break
        # velocity smoother
        for k in range(3):
            d = cmd[k] - sm[k]
            lim = (0.2 if abs(cmd[k]) > abs(sm[k]) else 0.5) * dt
            sm[k] += max(-lim, min(lim, d))
        saida = [v if abs(v) >= 0.015 else 0.0 for v in sm]
        fila.append(saida)
        atual = fila.pop(0) if len(fila) > 5 else [0, 0, 0]
        # ponte: heading hold quando wz=0 e andando
        wz_real = ganho_w * atual[2]
        andando = abs(atual[0]) + abs(atual[1]) > 0
        if atual[2] != 0 or not andando:
            hh_alvo = pose[2]
        else:
            e = math.atan2(math.sin(hh_alvo - pose[2]), math.cos(hh_alvo - pose[2]))
            if abs(e) > math.radians(0.5):
                wz_real += max(-0.15, min(0.15, 0.8 * e)) * 0.5
        # perturbacoes: deriva e chute de yaw quando vy muda
        if andando:
            wz_real += math.radians(rnd.gauss(0.3, 0.8))
        if abs(atual[1] - ult_vy) > 0.01:
            pose[2] += math.radians(rnd.gauss(0.8, 1.0))
        ult_vy = atual[1]
        vxr, vyr = 3.2 * atual[0], 3.4 * atual[1]
        th = pose[2]
        pose[0] += (vxr * math.cos(th) - vyr * math.sin(th)) * dt
        pose[1] += (vxr * math.sin(th) + vyr * math.cos(th)) * dt
        pose[2] += wz_real * dt
        # odom: comando integrado x (5.0, 3.4), yaw do IMU
        odom[2] = pose[2]
        vxo, vyo = 5.0 * atual[0], 3.4 * atual[1]
        odom[0] += (vxo * math.cos(th) - vyo * math.sin(th)) * dt
        odom[1] += (vxo * math.sin(th) + vyo * math.cos(th)) * dt
        min_folga = min(min_folga, folga(pose, segs))
        t += dt
    return tr.estado, t, min_folga, logs, math.degrees(pose[2] - eixo)


if __name__ == '__main__':
    tot = 0; ok = 0; piores = []
    for seed in range(30):
        rnd = random.Random(1000 + seed)
        y0 = rnd.uniform(-6, 6); l0 = rnd.uniform(-0.05, 0.05)
        larg = rnd.choice([0.78, 0.81, 0.85])
        est, t, mf, logs, yf = rodar(seed, y0, l0, largura=larg)
        tot += 1; ok += (est == 'fim' and mf > 0.0)
        print('seed %2d yaw0 %+5.1f lat0 %+5.2f larg %.2f -> %-8s %5.1f s  folga min %5.1f cm  yaw final %+5.1f  medicoes %d' % (
            seed, y0, l0, larg, est, t, mf * 100, yf, sum('Medicao' in m for m in logs)))
        if est != 'fim' or mf < 0.01:
            print('    ', logs)
    print('ok', ok, '/', tot)
