#!/usr/bin/env python3
"""Bancada de encosto no Gazebo: roda o _encostar REAL do alinhador em malha fechada, varios cenarios,
e compara o que o alinhador DISSE com o que aconteceu (pose real do Gazebo).

Pilha (tudo com ROS_DOMAIN_ID=77, so localhost, GZ_PARTITION=bancada; nao fala com o robo):
  Gazebo (mesas, robo cinematico, gpu_lidar) <-> ros_gz_bridge
  ponte_sim.py (motor + Delta-2D + IMU + map->odom ideal)          [reinicia por cenario]
  command_imu_odometry REAL (localization.yaml)                    [reinicia por cenario]
  filtro_corpo_scan.py REAL  /scan_front -> /scan_nav              [reinicia por cenario]
  alinhador_bancada.py -> alinhador_passagem.py REAL               [reinicia por cenario]
  velocity_smoother + collision_monitor REAIS (nav2_params.yaml, overrides do navigation.launch.py)

Uso:
  python3 bancada.py                      # todos os cenarios de cenarios.yaml, 1 repeticao
  python3 bancada.py --so centrado lado_p10 --rep 3 --gui
"""
import argparse
import datetime
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import yaml

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
import mundo  # noqa: E402

WS = os.path.expanduser('~/robo_jazzy_ws')
SCRIPTS = WS + '/src/robo_navigation/scripts'
NAV2 = WS + '/src/robo_navigation/config/nav2_params.yaml'
LOC = WS + '/src/robo_localization/config/localization.yaml'
AREAS = os.path.expanduser('~/mapas/service_areas.yaml')
PARA_CHOQUE = 0.365

# criterio de encosto honesto (RELATORIO face_fragmentada: folga 8-9 cm, centro +-3 cm)
FOLGA_MIN, FOLGA_MAX = 0.06, 0.12
CENTRO_MAX = 0.03
ANG_MAX = math.radians(3.0)


def ambiente():
    cmd = 'source /opt/ros/jazzy/setup.bash && source %s/install/setup.bash && env -0' % WS
    out = subprocess.run(['bash', '-c', cmd], capture_output=True, check=True).stdout
    env = dict(kv.split('=', 1) for kv in out.decode().split('\0') if '=' in kv)
    env.update(ROS_DOMAIN_ID='77', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', GZ_PARTITION='bancada',
               PYTHONUNBUFFERED='1', RCUTILS_COLORIZED_OUTPUT='0')
    env.setdefault('DISPLAY', ':0')
    return env


ENV = ambiente()
os.environ.update(ENV)
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402


class Processos:
    def __init__(self, pasta):
        self.pasta = pasta
        self.p = {}

    def subir(self, nome, cmd, log=None, env_extra=None):
        f = open(os.path.join(self.pasta, (log or nome) + '.log'), 'w')
        env = dict(ENV, **(env_extra or {}))
        self.p[nome] = (subprocess.Popen(cmd, env=env, stdout=f, stderr=subprocess.STDOUT,
                                         start_new_session=True, cwd=AQUI), f)

    def parar(self, nomes=None):
        nomes = list(self.p) if nomes is None else [n for n in nomes if n in self.p]
        for n in nomes:
            pr, _ = self.p[n]
            if pr.poll() is None:
                try:
                    os.killpg(pr.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        fim = time.time() + 6
        for n in nomes:
            pr, f = self.p.pop(n)
            try:
                pr.wait(max(0.1, fim - time.time()))
            except subprocess.TimeoutExpired:
                os.killpg(pr.pid, signal.SIGKILL)
                pr.wait()
            f.close()

    def vivo(self, nome):
        return nome in self.p and self.p[nome][0].poll() is None


def gz_set_pose(nome, x, y, z, yaw):
    req = 'name: "%s" position {x: %.4f y: %.4f z: %.4f} orientation {z: %.6f w: %.6f}' % (
        nome, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2))
    r = subprocess.run(['gz', 'service', '-s', '/world/bancada/set_pose', '--reqtype', 'gz.msgs.Pose',
                        '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', req],
                       env=ENV, capture_output=True, text=True)
    return 'true' in r.stdout


def gz_parar_robo():
    subprocess.run(['gz', 'topic', '-t', '/sim/vel_real', '-m', 'gz.msgs.Twist', '-p', ''],
                   env=ENV, capture_output=True)


# ---------------- geometria (verdade) ----------------
def para_mundo(pose, p):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return (pose[0] + c * p[0] - s * p[1], pose[1] + s * p[0] + c * p[1])


def no_quadro(origem, p):
    """ponto do mundo -> quadro (x, y, yaw) de origem"""
    dx, dy = p[0] - origem[0], p[1] - origem[1]
    c, s = math.cos(origem[2]), math.sin(origem[2])
    return (c * dx + s * dy, -s * dx + c * dy)


def _seg_dist(p, a, b):
    ax, ay = b[0] - a[0], b[1] - a[1]
    L = ax * ax + ay * ay
    t = 0.0 if L == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / L))
    return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)


def _dentro(p, poly):
    sinal = None
    for a, b in zip(poly, poly[1:] + poly[:1]):
        cr = (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])
        if abs(cr) < 1e-12:
            continue
        if sinal is None:
            sinal = cr > 0
        elif (cr > 0) != sinal:
            return False
    return True


def dist_poligonos(A, B):
    if any(_dentro(p, B) for p in A) or any(_dentro(p, A) for p in B):
        return 0.0
    d = min(_seg_dist(p, a, b) for p in A for a, b in zip(B, B[1:] + B[:1]))
    return min(d, min(_seg_dist(p, a, b) for p in B for a, b in zip(A, A[1:] + A[:1])))


def retangulo(cx, cy, yaw, sx, sy):
    return [para_mundo((cx, cy, yaw), (ex * sx / 2, ey * sy / 2)) for ex, ey in ((1, 1), (-1, 1), (-1, -1), (1, -1))]


def footprint(pose):
    x0, x1, y0, y1 = mundo.FOOT
    return [para_mundo(pose, p) for p in ((x1, y1), (x0, y1), (x0, y0), (x1, y0))]


class Geometria:
    """Mesa real (as 2 metades com degrau) e obstaculos ativos, no frame do mundo."""
    def __init__(self, face, mesa, degrau):
        self.psi = face['yaw']
        self.fc = (face['x'], face['y'])
        self.larg = float(face.get('largura_medida', 0.8))
        prof = float(mesa.get('depth', 0.5))
        self.degrau = degrau
        self.quadro = (self.fc[0], self.fc[1], self.psi)
        self.polys = []
        for lado, fundo in ((-1, 0.0), (1, degrau)):
            c = para_mundo(self.quadro, (prof / 2 + fundo, lado * self.larg / 4))
            self.polys.append(retangulo(c[0], c[1], self.psi, prof, self.larg / 2))

    def pose_inicial(self, dist, lado, ang):
        p = para_mundo(self.quadro, (-dist, lado))
        return (p[0], p[1], self.psi + ang)

    def medir(self, pose, obst_polys=()):
        fp = footprint(pose)
        d_mesa = min(dist_poligonos(fp, P) for P in self.polys)
        d_obst = min([dist_poligonos(fp, P) for P in obst_polys] or [9.0])
        # para-choque (centro) no quadro da face: n para dentro da mesa, t para a esquerda
        pc = no_quadro(self.quadro, para_mundo(pose, (PARA_CHOQUE, 0.0)))
        ang = math.atan2(math.sin(pose[2] - self.psi), math.cos(pose[2] - self.psi))
        # folga ao longo do eixo do robo ate a face da metade que esta na frente do para-choque
        plano = self.degrau if pc[1] > 0 else 0.0
        folga = (plano - pc[0]) / max(math.cos(ang), 0.2)
        # centro na convencao do alinhador: centro da face (meio das 2 pontas) no frame do robo, y
        centro = no_quadro(pose, self.fc)[1]
        return dict(folga=folga, centro=centro, ang=ang, d_mesa=d_mesa, d_obst=d_obst)


# ---------------- no ROS da bancada ----------------
class NoBancada(Node):
    def __init__(self):
        super().__init__('bancada_encosto')
        self.verdade = None
        self.t_verdade = 0.0
        self.resultados = []
        self.odom_ok = 0.0
        self.pub = self.create_publisher(String, '/sim/encostar', 10)
        self.create_subscription(Odometry, '/sim/verdade', self.ao_verdade, 50)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom_ok', time.monotonic()), 10)
        self.create_subscription(String, '/ir_para_area/resultado', self.ao_resultado, 10)
        self.trilha = None

    def ao_verdade(self, m):
        p = m.pose.pose
        self.verdade = (p.position.x, p.position.y,
                        math.atan2(2 * p.orientation.w * p.orientation.z, 1 - 2 * p.orientation.z ** 2))
        self.t_verdade = time.monotonic()
        if self.trilha is not None:
            self.trilha.append((time.monotonic(), self.verdade))

    def ao_resultado(self, m):
        self.resultados.append((time.monotonic(), m.data))


def esperar(cond, timeout, passo=0.1):
    fim = time.monotonic() + timeout
    while time.monotonic() < fim:
        if cond():
            return True
        time.sleep(passo)
    return False


def classificar(c, disse_ok, med_final, d_min_mesa, d_min_obst):
    honesto = (FOLGA_MIN <= med_final['folga'] <= FOLGA_MAX and abs(med_final['centro']) <= CENTRO_MAX
               and abs(med_final['ang']) <= ANG_MAX)
    if d_min_mesa < 0.005 or d_min_obst < 0.005:
        return 'COLISAO'
    if disse_ok is None:
        return 'SEM_RESPOSTA'
    if disse_ok:
        return 'SUCESSO_OK' if honesto else 'SUCESSO_FALSO'
    return 'FALHA_EXPLICITA'


def avaliar(c, classe):
    """bom / aceitavel / ruim conforme o esperado do cenario"""
    if classe in ('COLISAO', 'SUCESSO_FALSO', 'SEM_RESPOSTA'):
        return 'ruim'
    esp = c.get('esperado', 'sucesso')
    if esp == 'sucesso':
        return 'bom' if classe == 'SUCESSO_OK' else 'nao_cumpriu'
    if esp == 'falha':
        return 'bom' if classe == 'FALHA_EXPLICITA' else 'aceitavel'
    return 'bom'   # 'seguro': qualquer desfecho sem colisao/sucesso falso


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--cenarios', default=os.path.join(AQUI, 'cenarios.yaml'))
    ap.add_argument('--so', nargs='*', help='nomes dos cenarios a rodar')
    ap.add_argument('--rep', type=int, default=1)
    ap.add_argument('--gui', action='store_true', help='abre a janela do Gazebo')
    ap.add_argument('--timeout', type=float, default=120.0)
    ap.add_argument('--saida', default=None)
    ap.add_argument('--versao', default=None,
                    help='pasta com alinhador_passagem.py/face_ws.py a testar (padrao: o src)')
    ap.add_argument('--degrau', type=float, default=None, help='sobrescreve o degrau da face (m)')
    a = ap.parse_args()
    # uma bancada por vez: duas usariam o mesmo ROS_DOMAIN_ID/GZ_PARTITION e misturariam os topicos
    import fcntl
    trava = open(os.path.join(AQUI, '.bancada.lock'), 'w')
    try:
        fcntl.flock(trava, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        sys.exit('Outra bancada.py ja esta rodando (trava %s). Espere ela terminar.' % trava.name)

    conf = yaml.safe_load(open(a.cenarios))
    cenarios = [c for c in conf['cenarios'] if not a.so or c['nome'] in a.so]
    areas = yaml.safe_load(open(AREAS))['service_areas']
    degrau = float(conf.get('degrau', 0.017) if a.degrau is None else a.degrau)
    pasta = a.saida or os.path.join(AQUI, 'resultados', datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(pasta, exist_ok=True)
    # areas da simulacao: copia do cadastro real (a mesa do Gazebo fica exatamente no cadastro;
    # erro de localizacao/cadastro entra pelo erro_mapa da ponte)
    areas_sim = os.path.join(pasta, 'service_areas_sim.yaml')
    with open(areas_sim, 'w') as fp:
        yaml.safe_dump({'frame': 'map', 'service_areas': areas}, fp)
    sdf = os.path.join(pasta, 'mundo.sdf')
    with open(sdf, 'w') as fp:
        fp.write(mundo.mundo_sdf(areas, degrau=degrau, n_obst=2))

    pr = Processos(pasta)
    rclpy.init()
    no = NoBancada()
    threading.Thread(target=rclpy.spin, args=(no,), daemon=True).start()
    registros = []
    try:
        pr.subir('gz', ['gz', 'sim', '-r', sdf] if a.gui else ['gz', 'sim', '-s', '-r', sdf])
        pr.subir('ponte_gz', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                              '/sim/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                              '/sim/verdade@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                              '/sim/vel_real@geometry_msgs/msg/Twist]gz.msgs.Twist'])
        if not esperar(lambda: no.verdade is not None, 40):
            raise RuntimeError('Gazebo nao publicou /sim/verdade em 40 s (veja gz.log)')
        pr.subir('smoother', ['ros2', 'run', 'nav2_velocity_smoother', 'velocity_smoother', '--ros-args',
                              '--params-file', NAV2, '-r', 'cmd_vel:=cmd_vel_nav'])
        pr.subir('cm', ['ros2', 'run', 'nav2_collision_monitor', 'collision_monitor', '--ros-args',
                        '--params-file', NAV2, '-p', 'stop_pub_timeout:=2.0',
                        '-p', 'SweptTurnStop.enabled:=false'])
        pr.subir('lifecycle', ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args',
                               '-r', '__node:=lifecycle_manager_bancada', '-p', 'autostart:=true',
                               '-p', 'bond_timeout:=0.0',
                               '-p', 'node_names:=[velocity_smoother, collision_monitor]'])
        for c in cenarios:
            for rep in range(a.rep):
                registros.append(rodar_cenario(c, rep, areas, degrau, areas_sim, pasta, pr, no, a.timeout,
                                               a.versao))
                salvar(pasta, registros)
    finally:
        pr.parar(['alinhador', 'filtro', 'odom', 'ponte'])
        gz_parar_robo()
        pr.parar()
        no.destroy_node()
        rclpy.try_shutdown()
    salvar(pasta, registros, final=True)
    print(open(os.path.join(pasta, 'relatorio.md')).read())


def rodar_cenario(c, rep, areas, degrau, areas_sim, pasta, pr, no, timeout, versao=None):
    nome = '%s_r%d' % (c['nome'], rep)
    ws = c.get('ws', 'WS_1')
    geo = Geometria(areas[ws]['face'], areas[ws].get('table', {}), degrau)
    print('=== %s (%s) ===' % (nome, c.get('descricao', '')), flush=True)
    pr.parar(['alinhador', 'filtro', 'odom', 'ponte'])
    gz_parar_robo()
    time.sleep(0.5)
    # obstaculos: (n, t) no quadro da face (n<0 = na frente da face), tamanho fixo 10x10 cm
    obst_polys = []
    for i in range(2):
        ob = (c.get('obstaculos') or [])[i] if i < len(c.get('obstaculos') or []) else None
        if ob:
            p = para_mundo(geo.quadro, (ob[0], ob[1]))
            gz_set_pose('obst_%d' % i, p[0], p[1], 0.10, geo.psi)
            obst_polys.append(retangulo(p[0], p[1], geo.psi, 0.10, 0.10))
        else:
            gz_set_pose('obst_%d' % i, mundo.ESTACIONAMENTO[0] - i, mundo.ESTACIONAMENTO[1], 0.10, 0.0)
    ini = geo.pose_inicial(c.get('dist', 0.60), c.get('lado', 0.0), math.radians(c.get('ang', 0.0)))
    gz_set_pose('robo', ini[0], ini[1], 0.01, ini[2])
    time.sleep(0.5)
    gz_parar_robo()
    modelo = dict(c.get('modelo') or {})
    arq_modelo = os.path.join(pasta, nome + '_modelo.json')
    with open(arq_modelo, 'w') as fp:
        json.dump(modelo, fp)
    pr.subir('ponte', ['python3', os.path.join(AQUI, 'ponte_sim.py'), '--ros-args',
                       '-p', 'modelo_arquivo:=%s' % arq_modelo, '-p', 'semente:=%d' % (rep + 1)],
             log=nome + '_ponte')
    pr.subir('odom', ['ros2', 'run', 'robo_localization', 'command_imu_odometry_node', '--ros-args',
                      '--params-file', LOC], log=nome + '_odom')
    pr.subir('filtro', ['python3', SCRIPTS + '/filtro_corpo_scan.py'], log=nome + '_filtro')
    pr.subir('alinhador', ['python3', os.path.join(AQUI, 'alinhador_bancada.py'), '--ros-args',
                           '-p', 'arquivo_areas:=%s' % areas_sim, '-p', 'modo_curva_t:=lateral'],
             log=nome + '_alinhador',
             env_extra={'BANCADA_SCRIPTS': os.path.abspath(versao)} if versao else None)
    log_al = os.path.join(pasta, nome + '_alinhador.log')
    pronto = esperar(lambda: 'gatilho /sim/encostar pronto' in open(log_al).read()
                     and time.monotonic() - no.odom_ok < 0.3, 30)
    time.sleep(3.0)   # laser_tf, IMU e map->odom estabilizam
    reg = dict(cenario=c['nome'], versao=versao or 'src', rep=rep, ws=ws, descricao=c.get('descricao', ''),
               esperado=c.get('esperado', 'sucesso'), inicio=dict(dist=c.get('dist', 0.60),
               lado=c.get('lado', 0.0), ang=c.get('ang', 0.0)), modelo=modelo)
    if not pronto:
        reg.update(classe='SEM_RESPOSTA', avaliacao='ruim', disse='pilha nao subiu (veja os logs)')
        return reg
    med0 = geo.medir(no.verdade, obst_polys)
    no.resultados.clear()
    no.trilha = []
    t0 = time.monotonic()
    no.pub.publish(String(data=ws))
    esperar(lambda: any(ws in r for _, r in no.resultados), timeout, 0.2)
    time.sleep(1.0)   # o robo terminar de parar
    trilha, no.trilha = no.trilha, None
    meds = [geo.medir(p, obst_polys) for _, p in trilha[::5]] or [med0]
    medf = geo.medir(no.verdade, obst_polys)
    disse = next((r for _, r in no.resultados if ws in r), None)
    disse_ok = None if disse is None else (' SUCESSO' in disse)
    d_mesa = min(m['d_mesa'] for m in meds + [medf])
    d_obst = min(m['d_obst'] for m in meds + [medf])
    classe = classificar(c, disse_ok, medf, d_mesa, d_obst)
    texto = open(log_al).read()
    reg.update(
        disse=disse, classe=classe, avaliacao=avaliar(c, classe), duracao=round(time.monotonic() - t0, 1),
        real_final=dict(folga_cm=round(medf['folga'] * 100, 1), centro_cm=round(medf['centro'] * 100, 1),
                        ang_graus=round(math.degrees(medf['ang']), 1)),
        real_inicio=dict(folga_cm=round(med0['folga'] * 100, 1), centro_cm=round(med0['centro'] * 100, 1),
                         ang_graus=round(math.degrees(med0['ang']), 1)),
        menor_dist_mesa_cm=round(d_mesa * 100, 1), menor_dist_obst_cm=round(d_obst * 100, 1),
        n_fragmentada=texto.count('partida pela zona cega'), n_parcial=texto.count('so distancia/angulo'),
        n_sem_face=texto.count('face ausente, ambigua'), n_mede_de_novo=texto.count('mede de novo') +
        texto.count('medir de novo'))
    print('  -> %s / %s | disse: %s | real: folga %.1f cm centro %+.1f cm ang %+.1f g | menor dist mesa %.1f cm'
          % (classe, reg['avaliacao'], disse, medf['folga'] * 100, medf['centro'] * 100,
             math.degrees(medf['ang']), d_mesa * 100), flush=True)
    return reg


def salvar(pasta, regs, final=False):
    with open(os.path.join(pasta, 'resultados.json'), 'w') as fp:
        json.dump(regs, fp, indent=1, ensure_ascii=False)
    if not final:
        return
    L = ['# Bancada de encosto (Gazebo) - %s' % datetime.datetime.now().strftime('%d/%m/%Y %H:%M'), '',
         'Criterio honesto: folga %d-%d cm, centro +-%d cm, angulo +-%d graus (pose real do Gazebo).' % (
             FOLGA_MIN * 100, FOLGA_MAX * 100, CENTRO_MAX * 100, math.degrees(ANG_MAX)), '',
         '| cenario | esperado | classe | avaliacao | o alinhador disse | real: folga / centro / ang | menor dist mesa | frag/parcial/sem face |',
         '|---|---|---|---|---|---|---|---|']
    for r in regs:
        rf = r.get('real_final', {})
        L.append('| %s r%d | %s | %s | %s | %s | %s / %+.1f / %+.1f | %s | %s/%s/%s |' % (
            r['cenario'], r['rep'], r['esperado'], r['classe'], r['avaliacao'],
            (r.get('disse') or '-').replace('|', '/'), rf.get('folga_cm', float('nan')),
            rf.get('centro_cm', float('nan')), rf.get('ang_graus', float('nan')),
            r.get('menor_dist_mesa_cm', '-'), r.get('n_fragmentada', '-'), r.get('n_parcial', '-'),
            r.get('n_sem_face', '-')))
    cont = {}
    for r in regs:
        cont[r['avaliacao']] = cont.get(r['avaliacao'], 0) + 1
    L += ['', 'Resumo: ' + ', '.join('%s %d' % kv for kv in sorted(cont.items()))]
    with open(os.path.join(pasta, 'relatorio.md'), 'w') as fp:
        fp.write('\n'.join(L) + '\n')


if __name__ == '__main__':
    main()
