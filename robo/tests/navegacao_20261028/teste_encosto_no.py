import os
import sys, math, random
sys.argv = ['x']
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import teste_no_ros_inercia as T
import sim_encosto as SE
T.Mundo.__init__orig = T.Mundo.__init__
def rodar(seed, dx0, dy0, dyaw0, sair=False):
    T.SUBS.clear(); T.PUBS.clear(); T.LOG.clear(); T.TF.clear(); T.NODE.clear()
    w = T.Mundo(seed, 0, 0, 0.82)
    w.segs = SE.cenario(2.5, 4.6)
    w.pose = [2.5 - 0.60 + dx0, 4.6 + dy0, math.radians(dyaw0)]
    w.odom = [0.0, 0.0, w.pose[2]]; w.hh = w.pose[2]
    w.gw = random.Random(seed).uniform(5, 8)
    T.al.time = w
    T.al.main()
    no = T.NODE[0]
    w.sleep(0.5)
    no._encostar('WS_1')
    res = dict(folga=SE.folga_frente(w.pose, 2.5), ang=math.degrees(w.pose[2]), centro=w.pose[1] - 4.6, enc=no.encostado)
    if sair:
        no._recuar(0.25, w.odom[2], 'SAIR DA MESA')
        res['folga_depois'] = SE.folga_frente(w.pose, 2.5)
    return res, list(T.LOG), w
ok = 0
for seed in range(12):
    r = random.Random(seed)
    dx0, dy0, dyaw0 = r.uniform(-0.08, 0.08), r.uniform(-0.12, 0.12), r.uniform(-8, 8)
    res, log, w = rodar(seed, dx0, dy0, dyaw0, sair=(seed < 3))
    bom = res['enc'] and 0.05 <= res['folga'] <= 0.11 and abs(res['ang']) < 3 and abs(res['centro']) < 0.04 and w.min_folga > 0
    ok += bom
    print('seed %2d -> encostado %s folga %.1f cm ang %+.1f centro %+.1f cm min folga corpo %.1f cm %s %s' % (seed, res['enc'], res['folga']*100, res['ang'], res['centro']*100, w.min_folga*100, ('depois da re %.1f cm' % (res['folga_depois']*100)) if 'folga_depois' in res else '', '' if bom else '<<< ' + str([l for l in log if 'ENCOSTAR' in l][-3:])))
print('ok', ok, '/ 12')
print('\n'.join(l for l in rodar(5, 0.03, 0.08, 5)[1] if 'ENCOSTAR' in l))
