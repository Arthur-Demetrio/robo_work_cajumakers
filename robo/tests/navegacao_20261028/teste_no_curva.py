"""Teste do no (v3.11) com rclpy falso: travessia ate o cruzamento + saida lateral, em L/T/+."""
import math, random, sys
import os
D = os.path.dirname(os.path.abspath(__file__))
sys.argv = ['x']
sys.path.insert(0, D)
import teste_no_ros_inercia as T
import sim_lateral as SL
import sim as S


def rodar(bracos, lado, seed, erro_eixo=2.0):
    T.SUBS.clear(); T.PUBS.clear(); T.LOG.clear(); T.TF.clear(); T.NODE.clear()
    w = T.Mundo(seed, 0, 0, 0.82)
    r = random.Random(seed)
    w.segs = SL.segs_cruzamento(bracos)
    yaw0 = math.radians(r.uniform(-5, 5))
    w.pose = [-2.01 - 0.60 + r.uniform(-0.05, 0.05), r.uniform(-0.05, 0.05), yaw0]
    w.odom = [0.0, 0.0, yaw0]; w.hh = yaw0
    T.al.time = w
    T.al.main()
    no = T.NODE[0]
    w.sleep(0.5)
    info = dict(saida=(-0.41, 0.0), eixo_graus=r.uniform(-erro_eixo, erro_eixo))
    curva = dict(juncao=(r.uniform(-0.03, 0.03), r.uniform(-0.03, 0.03)), eixo_b=(0.0, float(lado)),
                 lado=float(lado), entrada_b=(0.0, lado * 0.41), saida_b=(0.0, lado * 2.01))
    a = math.radians(info['eixo_graus'])
    parada = (curva['juncao'][0] - 0.08 * math.cos(a), curva['juncao'][1] - 0.08 * math.sin(a))
    t0 = w.t
    r1 = no._travessia_lidar(info, 'TRAVESSIA 1', parada)
    pos_parada = tuple(w.pose)
    r2 = no._travessia_lateral(curva, 'CURVA 1') if r1 == 'ok' else 'nao tentou'
    return r1, r2, w, pos_parada, w.t - t0, list(T.LOG)


if __name__ == '__main__':
    casos = [('ON', 1, 'L esquerda'), ('OS', -1, 'L direita'), ('ONS', 1, 'T da haste, esq'),
             ('OLS', -1, 'T na barra -> haste dir'), ('OLNS', 1, '+ esquerda'), ('OLNS', -1, '+ direita')]
    tot = ok = 0
    for bracos, lado, nome in casos:
        for seed in range(6):
            r1, r2, w, pp, dur, log = rodar(bracos, lado, seed)
            fim_y = w.pose[1] * lado
            bom = r1 == 'ok' and r2 == 'ok' and w.min_folga > 0 and fim_y > 2.01
            tot += 1; ok += bom
            print('%-24s seed %d: travessia %s (parou x=%+.2f) | lateral %s | fim alem da saida %+.2f m | folga min %.1f cm | %.0f s%s' % (
                nome, seed, r1, pp[0], r2, fim_y - 2.01, w.min_folga * 100, dur, '' if bom else '  <-- FALHA'))
            if not bom:
                print('     ', ' / '.join(l for l in log[-8:]))
    print('TOTAL', ok, '/', tot)
