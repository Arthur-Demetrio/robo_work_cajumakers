import os
import sys, math, threading, time
D = os.path.dirname(os.path.abspath(__file__))
sys.argv = ['x']
sys.path.insert(0, D)
import teste_no_ros_inercia as T
al = T.al
sys.modules['nav2_msgs.action'].NavigateToPose = T.M(Goal=T.M)
def _ps():
    m = T.M(); m.pose.orientation.x = 0.0; m.pose.orientation.y = 0.0; return m
sys.modules['geometry_msgs.msg'].PoseStamped = _ps
M = T.M

# ---- yaw_chegada ----
cam = [(0.45 - 0.05 * i, -0.28 + 0.035 * i) for i in range(8)] + [(0.19, 0.09)]
y = al.yaw_chegada(cam, (0.19, 0.09))
print('yaw caminho: %.0f graus' % math.degrees(y))
assert 100 < math.degrees(y) < 160
print('yaw sem caminho: %.0f' % math.degrees(al.yaw_chegada(None, (1, 1), (0, 0, 0))))
assert abs(math.degrees(al.yaw_chegada(None, (1, 1), (0, 0, 0))) - 45) < 1
assert al.yaw_chegada(None, (0.1, 0), (0.0, 0.0, 0.7)) == 0.7
assert al.yaw_chegada([(0.19, 0.09)], (0.19, 0.09)) is None

# ---- _navegar com aceitar_perto: Nav2 nunca termina (giro final bloqueado) ----
class Fut:
    def __init__(self, val=None, done=False):
        self.val, self.done, self.cbs = val, done, []
    def add_done_callback(self, cb):
        if self.done: cb(self)
        else: self.cbs.append(cb)
    def result(self): return self.val
    def set(self, v):
        self.val, self.done = v, True
        for cb in self.cbs: cb(self)

class Nav:
    def __init__(self, termina=None):
        self.termina = termina; self.cancelado = False
    def wait_for_server(self, timeout_sec=0): return True
    def send_goal_async(self, req):
        res = Fut()
        nav = self
        def cancel():
            nav.cancelado = True; res.set(M(status=5)); return Fut(None, True)
        h = M(accepted=True, get_result_async=lambda: res, cancel_goal_async=cancel)
        if self.termina is not None:
            threading.Timer(self.termina, lambda: res.done or res.set(M(status=4))).start()
        return Fut(h, True)

T.SUBS.clear(); T.PUBS.clear(); T.LOG.clear(); T.TF.clear(); T.NODE.clear()
al.main()
no = T.NODE[0]
no.nav = Nav()
pos = [0.60, -0.10, 2.2]
T.TF[('map', 'base_link')] = tuple(pos)
def mover():
    for k in range(40):
        f = min(1.0, k / 15.0)
        T.TF[('map', 'base_link')] = (0.60 + (0.19 - 0.60) * f, -0.10 + (0.06 + 0.10) * f, 2.2)
        time.sleep(0.1)
threading.Thread(target=mover, daemon=True).start()
goal = no._pose(0.19, 0.09, math.radians(20))
t0 = time.monotonic()
r = no._navegar(goal, 'GOAL FINAL', aceitar_perto=True)
dt = time.monotonic() - t0
print('resultado', r, 'em %.1f s, cancelou Nav2: %s' % (dt, no.nav.cancelado))
print(T.LOG[-1])
assert r == 'ok' and no.nav.cancelado and 2.0 < dt < 4.0

# sem aceitar_perto: comportamento antigo (espera o Nav2)
no.nav = Nav(termina=1.0)
r = no._navegar(goal, 'GOAL FINAL')
print('sem aceitar_perto:', r, T.LOG[-1]); assert r == 'ok' and not no.nav.cancelado

# robo passando perto mas indo embora (nao deve aceitar)
no.nav = Nav(termina=4.0)
def passar():
    for k in range(60):
        x = 0.60 - 0.02 * k
        T.TF[('map', 'base_link')] = (x, 0.06, 2.2); time.sleep(0.1)
threading.Thread(target=passar, daemon=True).start()
r = no._navegar(goal, 'GOAL FINAL', aceitar_perto=True)
print('passando:', r, 'cancelou', no.nav.cancelado); assert not no.nav.cancelado

# ---- executar() de um ponto sem mesa: yaw trocado + aceita ----
no.sem_encosto = {'FINISH'}
no._caminho = lambda g: cam
no._analisar = lambda *a, **k: ([], 'nenhuma passagem')
enviados = []
orig = no._navegar
def nav_espiao(alvo, nome, vigia=None, aceitar_perto=False):
    enviados.append((math.degrees(al.yaw_de(alvo.pose.orientation)) if hasattr(al, 'yaw_de') else None, aceitar_perto))
    return 'ok'
no._navegar = nav_espiao
T.TF[('map', 'base_link')] = (0.45, -0.28, 2.2)
no.executar(no._pose(0.19, 0.09, math.radians(20)), 'FINISH')
print('executar FINISH:', enviados, [l for l in T.LOG if 'FINISH' in l][-2:])
assert enviados[0][1] is True
print('TUDO OK')
