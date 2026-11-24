import os
"""Teste do no ROS do alinhador v3 com um rclpy falso e o robo simulado (sim.py).

O tempo do modulo e trocado por um relogio simulado: cada time.sleep() avanca a fisica.
"""
import sys, types, math, random, importlib.util
D = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, D)
import sim as S


class M:
    """Mensagem generica: atributos criados sob demanda."""
    def __init__(self, **kw):
        self.__dict__.update(kw)

    def __getattr__(self, k):
        if k.startswith('__'):
            raise AttributeError(k)
        v = M()
        setattr(self, k, v)
        return v


def mod(nome, **attrs):
    m = types.ModuleType(nome)
    m.__dict__.update(attrs)
    sys.modules[nome] = m
    return m


SUBS, PUBS, LOG = {}, {}, []
TF = {}


class Node:
    def __init__(self, nome):
        self._p = {}

    def declare_parameter(self, n, v):
        self._p[n] = v

    def get_parameter(self, n):
        return M(value=self._p[n])

    def create_subscription(self, typ, topic, cb, qos, callback_group=None):
        SUBS[topic] = cb

    def create_publisher(self, typ, topic, qos):
        pub = M(topic=topic)
        pub.publish = lambda msg, t=topic: PUBS.setdefault(t, []).append(msg)
        return pub

    def get_logger(self):
        return M(info=lambda s: LOG.append(s), warn=lambda s: LOG.append('WARN ' + s),
                 error=lambda s: LOG.append('ERR ' + s))

    def get_clock(self):
        return M(now=lambda: M(to_msg=lambda: M()))

    def destroy_node(self):
        pass


class Buffer:
    def lookup_transform(self, a, b, t):
        if (a, b) not in TF:
            raise RuntimeError('sem tf')
        x, y, yaw = TF[(a, b)]
        return M(transform=M(translation=M(x=x, y=y, z=0.0),
                             rotation=M(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2))))


NODE = []
mod('rclpy', ok=lambda: True, init=lambda: None, try_shutdown=lambda: None)
mod('rclpy.action', ActionClient=lambda *a, **k: M())
mod('rclpy.callback_groups', ReentrantCallbackGroup=lambda: None)
mod('rclpy.executors', MultiThreadedExecutor=lambda num_threads=1: M(
    add_node=lambda n: NODE.append(n), spin=lambda: None, shutdown=lambda: None))
mod('rclpy.node', Node=Node)
mod('rclpy.qos', DurabilityPolicy=M(TRANSIENT_LOCAL=1), QoSProfile=lambda **k: None,
    ReliabilityPolicy=M(RELIABLE=1), qos_profile_sensor_data=None)
mod('rclpy.time', Time=lambda: None)
mod('action_msgs'); mod('action_msgs.msg', GoalStatus=M(STATUS_SUCCEEDED=4))
mod('geometry_msgs'); mod('geometry_msgs.msg', Pose=M, PoseArray=M, PoseStamped=M, TwistStamped=M)
mod('nav_msgs'); mod('nav_msgs.msg', OccupancyGrid=M, Odometry=M, Path=M)
mod('nav2_msgs'); mod('nav2_msgs.action', ComputePathToPose=M, NavigateToPose=M); mod('nav2_msgs.msg', CollisionMonitorState=M)
mod('sensor_msgs'); mod('sensor_msgs.msg', LaserScan=M)
mod('std_msgs'); mod('std_msgs.msg', String=M)
mod('tf2_ros', Buffer=Buffer, TransformListener=lambda b, n: None)

sp = importlib.util.spec_from_file_location('al', os.path.join(D, '..', 'robo_navigation', 'scripts', 'alinhador_passagem.py'))
al = importlib.util.module_from_spec(sp)
sp.loader.exec_module(al)


class Mundo:
    def __init__(self, seed, yaw0, lat0, largura, cm_trava=False):
        self.rnd = random.Random(seed)
        self.eixo = math.radians(-3.5)
        self.c, self.s = math.cos(self.eixo), math.sin(self.eixo)
        self.x_ent, self.x_sai = 0.75, 1.40
        self.segs = S.segs_corredor(self.eixo, largura / 2, self.x_ent, self.x_sai, -0.35)
        u0, v0 = self.x_ent - 0.6, lat0
        self.pose = [u0 * self.c - v0 * self.s, -0.35 + u0 * self.s + v0 * self.c,
                     self.eixo + math.radians(yaw0)]
        self.odom = [0.0, 0.0, self.pose[2]]
        self.t = 1000.0
        self.sm = [0.0, 0.0, 0.0]
        self.fila = []
        self.hh = self.pose[2]
        self.gw = self.rnd.uniform(2.0, 5.0)
        self.ult_vy = 0.0
        self.prox_scan = 0.0
        self.min_folga = 9
        self.cm_trava = cm_trava
        self.lido = 0

    def folga_dir(self):
        """Folga ate a parede mais proxima e direcao (no robo) ate ela."""
        x, y, th = self.pose
        c, s_ = math.cos(th), math.sin(th)
        cantos = [(x + u * c - v * s_, y + u * s_ + v * c) for u, v in S.FP]
        best = (9, 0, 0)
        for i in range(4):
            a, b = cantos[i], cantos[(i + 1) % 4]
            for (p, q) in self.segs:
                for pt in (p, q):
                    d = S.dist_seg(pt, a, b)
                    if d < best[0]:
                        dx, dy = pt[0] - x, pt[1] - y
                        best = (d, dx * c + dy * s_, -dx * s_ + dy * c)
                for pt in (a, b):
                    d = S.dist_seg(pt, p, q)
                    if d < best[0]:
                        dx, dy = pt[0] - x, pt[1] - y
                        best = (d, dx * c + dy * s_, -dx * s_ + dy * c)
        return best

    def monotonic(self):
        return self.t

    def sleep(self, dt):
        fim = self.t + dt
        while self.t < fim - 1e-9:
            self.passo(0.01)

    def cmd(self):
        msgs = PUBS.get('/cmd_vel_nav', [])
        if not msgs:
            return (0, 0, 0)
        tw = msgs[-1].twist
        return (tw.linear.x, tw.linear.y, tw.angular.z)

    def passo(self, dt):
        cmd = self.cmd()
        for k in range(3):
            d = cmd[k] - self.sm[k]
            lim = (0.2 if abs(cmd[k]) > abs(self.sm[k]) else 0.5) * dt
            self.sm[k] += max(-lim, min(lim, d))
        out = [v if abs(v) >= 0.015 else 0.0 for v in self.sm]
        if self.cm_trava and self.t > 1008:
            out = [0.0, 0.0, 0.0]
        # Collision Monitor simplificado: giro bloqueado com folga < 2 cm; translacao
        # bloqueada se for na direcao do ponto mais proximo com folga < 2 cm
        stop = False
        if any(out):
            f, dirx, diry = self.folga_dir()
            if f < 0.02:
                if out[2] != 0 or (out[0] * dirx + out[1] * diry) > 0:
                    stop = True
        if stop != getattr(self, '_stop', False):
            self._stop = stop
            if '/collision_monitor_state' in SUBS:
                SUBS['/collision_monitor_state'](M(action_type=1 if stop else 0))
        if stop:
            out = [0.0, 0.0, 0.0]
        else:
            SUBS['/cmd_vel'](M(twist=M(linear=M(x=out[0], y=out[1]), angular=M(z=out[2]))))
        self.fila.append(out)
        a = self.fila.pop(0) if len(self.fila) > 10 else [0, 0, 0]
        wz = self.gw * a[2]
        andando = abs(a[0]) + abs(a[1]) > 0
        if a[2] != 0 or not andando:
            self.hh = self.pose[2]
        else:
            e = math.atan2(math.sin(self.hh - self.pose[2]), math.cos(self.hh - self.pose[2]))
            if abs(e) > math.radians(0.5):
                wz += max(-0.15, min(0.15, 0.8 * e)) * 0.5
        if andando:
            wz += math.radians(self.rnd.gauss(0.6, 1.2))
        # inercia: o robo real continua girando ~0.2 s depois do comando
        self.wz_f = getattr(self, 'wz_f', 0.0) + (wz - getattr(self, 'wz_f', 0.0)) * dt / 0.2
        wz = self.wz_f
        if abs(a[1] - self.ult_vy) > 0.01:
            self.pose[2] += math.radians(self.rnd.gauss(1.5, 1.5))
        self.ult_vy = a[1]
        th = self.pose[2]
        vxr, vyr = 3.2 * a[0], 3.4 * a[1]
        self.pose[0] += (vxr * math.cos(th) - vyr * math.sin(th)) * dt
        self.pose[1] += (vxr * math.sin(th) + vyr * math.cos(th)) * dt
        self.pose[2] += wz * dt
        self.odom[2] = self.pose[2]
        vxo, vyo = 5.0 * a[0], 3.4 * a[1]
        self.odom[0] += (vxo * math.cos(th) - vyo * math.sin(th)) * dt
        self.odom[1] += (vxo * math.sin(th) + vyo * math.cos(th)) * dt
        self.t += dt
        # /odom a 50 Hz (aqui a 100 Hz, tanto faz)
        q = M(x=0.0, y=0.0, z=math.sin(self.odom[2] / 2), w=math.cos(self.odom[2] / 2))
        SUBS['/odom'](M(pose=M(pose=M(position=M(x=self.odom[0], y=self.odom[1]), orientation=q))))
        # TF map->base_link = pose real (SLAM perfeito) e base_link->laser
        TF[('map', 'base_link')] = tuple(self.pose)
        TF[('base_link', 'laser_raw')] = (0.33, 0.005, 0.0)
        if self.t >= self.prox_scan:
            self.prox_scan = self.t + 1 / 5.5
            x, y, th = self.pose
            lx, ly = x + 0.33 * math.cos(th), y + 0.33 * math.sin(th)
            ranges = []
            for i in range(180):
                ang = -math.pi + i * 2 * math.pi / 180
                r = S.raio(lx, ly, math.cos(th + ang), math.sin(th + ang), self.segs)
                if r is None or r > 8:
                    ranges.append(float('inf'))
                    continue
                r += self.rnd.gauss(0, 0.012)
                bx, by = 0.33 + r * math.cos(ang), r * math.sin(ang)
                ranges.append(float('inf') if (bx < 0.19 and abs(by) < 0.40) else r)
            SUBS['/scan_front'](M(header=M(frame_id='laser_raw'), angle_min=-math.pi,
                                  angle_increment=2 * math.pi / 180, range_min=0.08, range_max=12.0,
                                  ranges=ranges))
        self.min_folga = min(self.min_folga, S.folga(self.pose, self.segs))


def rodar(seed, yaw0, lat0, larg, cm_trava=False):
    SUBS.clear(); PUBS.clear(); LOG.clear(); TF.clear(); NODE.clear()
    w = Mundo(seed, yaw0, lat0, larg, cm_trava)
    al.time = w  # relogio simulado dentro do modulo
    al.main()    # cria o no (spin falso retorna na hora)
    no = NODE[0]
    w.sleep(0.5)
    sx, sy = w.x_sai * w.c, -0.35 + w.x_sai * w.s
    # eixo do mapa com 3 graus de erro, como aconteceu no teste real
    info = dict(saida=(sx, sy), eixo_graus=math.degrees(w.eixo) + 3.0)
    r = no._travessia_lidar(info, 'TRAVESSIA 1')
    return r, w, list(LOG)


def lote_grandes():
    ok = 0
    for seed in range(12):
        r_ = random.Random(500 + seed)
        y0 = r_.uniform(-6, 6); l0 = r_.choice([-1, 1]) * r_.uniform(0.10, 0.17)
        r, w, log = rodar(seed, y0, l0, 0.82)
        print('GRANDE seed %2d yaw0 %+5.1f lat0 %+5.2f -> %s %.1f s folga %.1f cm | %s' % (
            seed, y0, l0, r, w.t - 1000.5, w.min_folga * 100, ' / '.join(m.split(': ', 1)[-1] for m in log if 'Medic' in m or 'abort' in m or 'Collision' in m)[:300]))
        ok += r == 'ok' and w.min_folga > 0
    print('grandes ok', ok, '/ 12')


if __name__ == '__main__' and len(sys.argv) > 1:
    lote_grandes()
elif __name__ == '__main__':
    ok = 0
    casos = [(s, random.Random(s).uniform(-6, 6), random.Random(s + 1).uniform(-0.05, 0.05),
              random.Random(s + 2).choice([0.78, 0.81, 0.85])) for s in range(12)]
    for seed, y0, l0, lg in casos:
        r, w, log = rodar(seed, y0, l0, lg)
        u = w.pose[0] * w.c + (w.pose[1] + 0.35) * w.s
        print('seed %2d yaw0 %+5.1f lat0 %+5.2f larg %.2f -> %s  %.1f s  folga min %.1f cm  centro passou da saida %+.2f m' % (
            seed, y0, l0, lg, r, w.t - 1000.5, w.min_folga * 100, u - w.x_sai))
        ok += r == 'ok' and w.min_folga > 0
    print('ok', ok, '/', len(casos))
    print('--- log de um caso:')
    print('\n'.join(rodar(3, 4.0, 0.03, 0.81)[2]))
    print('--- Collision Monitor travando:')
    r, w, log = rodar(5, 1.0, 0.0, 0.81, cm_trava=True)
    print(r, log[-3:])
