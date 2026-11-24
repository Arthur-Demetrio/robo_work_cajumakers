import os
"""Simula o encosto numa mesa 80x50 contra a parede (mesmo modelo de robo dos testes da passagem)."""
import math, random, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import sim as S
from encosto import Encosto, medir_face

FP = S.FP
GANHO_VY = 8.0
GANHO_VX = 8.0


def cenario(fx=2.5, cy=4.6, parede=3.0):
    # face em x=fx (de y=cy-0.4 a cy+0.4), laterais ate a parede em x=fx+0.5, parede em x=parede
    segs = [((fx, cy - 0.4), (fx, cy + 0.4)),
            ((fx, cy - 0.4), (parede, cy - 0.4)), ((fx, cy + 0.4), (parede, cy + 0.4)),
            ((parede, cy - 3), (parede, cy - 0.4)), ((parede, cy + 0.4), (parede, cy + 3)),
            ((0.0, cy - 1.2), (fx, cy - 1.2))]  # um muro ao lado, para atrapalhar
    return segs


def folga_frente(pose, fx):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return min(fx - (x + u * c - v * s) for u, v in [(0.365, 0.315), (0.365, -0.315)])


def rodar(seed, dx0, dy0, dyaw0, ganho=(5.0, 8.0), verbose=False):
    rnd = random.Random(seed)
    fx, cy = 2.5, 4.6
    segs = cenario(fx, cy)
    pose = [fx - 0.60 + dx0, cy + dy0, math.radians(dyaw0)]
    odom = [0.0, 0.0, pose[2]]
    logs = []
    enc = Encosto(log=logs.append)
    dt = 0.01
    sm = [0, 0, 0]
    fila = []
    gw = rnd.uniform(*ganho)
    wz_f = 0.0
    prox = 0.0
    t = 0.0
    ult_vy = 0.0
    minf = 9
    vx_f = [0.0]
    vy_f = [0.0]
    while t < 70:
        if t >= prox:
            prox = t + 1 / 5.5
            x, y, th = pose
            lx, ly = x + 0.33 * math.cos(th), y + 0.33 * math.sin(th)
            pts = []
            for i in range(180):
                a = -math.pi + i * 2 * math.pi / 180
                r = S.raio(lx, ly, math.cos(th + a), math.sin(th + a), segs)
                if r is None or r > 8:
                    continue
                r += rnd.gauss(0, 0.012)
                pts.append((0.33 + r * math.cos(a), r * math.sin(a)))
            enc.nova_medicao(t, medir_face(pts), tuple(odom))
        cmd = enc.passo(t, tuple(odom))
        if enc.estado in ('fim', 'abortar'):
            break
        for k in range(3):
            d = cmd[k] - sm[k]
            lim = (0.2 if abs(cmd[k]) > abs(sm[k]) else 0.5) * dt
            sm[k] += max(-lim, min(lim, d))
        out = [v if abs(v) >= 0.015 else 0.0 for v in sm]
        # Collision Monitor: frente bloqueada a 6 cm
        if out[0] > 0 and folga_frente(pose, fx) < 0.06:
            out[0] = 0.0
        fila.append(out)
        a = fila.pop(0) if len(fila) > 10 else [0, 0, 0]
        wz = gw * a[2]
        if abs(a[0]) + abs(a[1]) > 0:
            wz += math.radians(rnd.gauss(0.6, 1.2))
        if abs(a[1] - ult_vy) > 0.01:
            pose[2] += math.radians(rnd.gauss(1.0, 1.0))
        ult_vy = a[1]
        wz_f += (wz - wz_f) * dt / 0.2
        th = pose[2]
        # comando pequeno anda mais do que o odom diz (medido: 0.025 -> ~20 cm/s)
        vx_alvo = GANHO_VX * a[0]
        vx_f[0] += (vx_alvo - vx_f[0]) * dt / 0.10
        vy_f[0] += (GANHO_VY * a[1] - vy_f[0]) * dt / 0.12
        vxr, vyr = vx_f[0], vy_f[0]
        pose[0] += (vxr * math.cos(th) - vyr * math.sin(th)) * dt
        pose[1] += (vxr * math.sin(th) + vyr * math.cos(th)) * dt
        pose[2] += wz_f * dt
        odom[2] = pose[2]
        odom[0] += (5.0 * a[0] * math.cos(th) - 3.4 * a[1] * math.sin(th)) * dt
        odom[1] += (5.0 * a[0] * math.sin(th) + 3.4 * a[1] * math.cos(th)) * dt
        minf = min(minf, folga_frente(pose, fx))
        t += dt
    x, y, th = pose
    return enc, logs, dict(folga=folga_frente(pose, fx), ang=math.degrees(th), centro=y - cy, t=t, minf=minf)


if __name__ == '__main__':
    ok = 0
    N = 30
    for seed in range(N):
        r = random.Random(seed)
        dx0, dy0, dyaw0 = r.uniform(-0.08, 0.08), r.uniform(-0.12, 0.12), r.uniform(-8, 8)
        enc, logs, res = rodar(seed, dx0, dy0, dyaw0)
        bom = enc.estado == 'fim' and 0.05 <= res['folga'] <= 0.11 and abs(res['ang']) < 3 and abs(res['centro']) < 0.04
        ok += bom
        print('seed %2d ini dx %+.2f dy %+.2f yaw %+5.1f -> %-7s folga %4.1f cm ang %+4.1f centro %+5.1f cm  %4.1f s  min folga %.1f cm %s' % (
            seed, dx0, dy0, dyaw0, enc.estado, res['folga'] * 100, res['ang'], res['centro'] * 100, res['t'], res['minf'] * 100, '' if bom else '<<<'))
        if not bom:
            print('     ', logs)
    print('ok', ok, '/', N)
