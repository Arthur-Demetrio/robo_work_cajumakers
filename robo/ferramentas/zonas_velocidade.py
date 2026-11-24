#!/usr/bin/env python3
"""Zonas de velocidade para o Nav2 (v1, 26/10): rapido em reta longa e livre, normal no resto.

Nao usa mapa desenhado (nada de zona marcada a mao): decide a cada 0,2 s olhando
  - o caminho do planner (/plan): quantos metros a frente ele segue reto;
  - quanto falta para o fim do caminho (perto do goal = normal);
  - se o robo esta virado para onde o caminho vai (girando = normal);
  - o LiDAR (/scan_front): espaco livre na frente e dos lados do robo.
Publica nav2_msgs/SpeedLimit em /speed_limit (em % da velocidade maxima do DWB):
  RAPIDO = 100 %  (max_vel_x do DWB, ex. 0.13)
  NORMAL = v_normal / v_rapida (ex. 0.08 / 0.13 = 61.5 %), igual ao robo de hoje.
Desce para NORMAL na hora; so sobe para RAPIDO depois de tempo_subir s com tudo livre.
Sem caminho novo, sem scan ou sem TF: NORMAL. Ao fechar (Ctrl+C) publica NORMAL.

A travessia de passagem e o encosto na mesa sao feitos pelo alinhador (comando direto),
entao nao mudam: continuam lentos como hoje.

IMPORTANTE: este no so funciona junto com o nav2_params.yaml ajustado para v_rapida
(max_vel_x/max_speed_xy do DWB, velocity_smoother e Collision Monitor). Sem o ajuste ele
nao faz nada (100 % de 0.08 = 0.08). Com o ajuste e SEM este no rodando, o robo anda
rapido em todo lugar: rode sempre os dois juntos.

Uso:
    python3 zonas_velocidade.py
    python3 zonas_velocidade.py --ros-args -p v_rapida:=0.13 -p v_normal:=0.08
Ver o que ele decide:  ros2 topic echo /zona_velocidade
"""
import math
import threading
import time

FRENTE_X = 0.365     # para-choque em base_link
TRASEIRA_X = -0.205
MEIA_LARGURA = 0.315


# ---------------------------------------------------------------------------
# Nucleo (sem ROS)
# ---------------------------------------------------------------------------
def indice_mais_perto(caminho, x, y):
    melhor, im = 1e9, 0
    for i, (px, py) in enumerate(caminho):
        d = (px - x) ** 2 + (py - y) ** 2
        if d < melhor:
            melhor, im = d, i
    return im, math.sqrt(melhor)


def reta_a_frente(caminho, i0, tol=0.05, maximo=4.0):
    """Metros do caminho, a partir de i0, que cabem numa faixa reta de +-tol."""
    if i0 >= len(caminho) - 1:
        return 0.0
    x0, y0 = caminho[i0]
    acum = 0.0
    ant = caminho[i0]
    ok_ate = 0.0
    for k in range(i0 + 1, len(caminho)):
        px, py = caminho[k]
        acum += math.hypot(px - ant[0], py - ant[1])
        ant = (px, py)
        L = math.hypot(px - x0, py - y0)
        if L < 1e-6:
            continue
        ux, uy = (px - x0) / L, (py - y0) / L
        desvio = max(abs((qx - x0) * uy - (qy - y0) * ux) for qx, qy in caminho[i0:k + 1])
        if desvio > tol:
            break
        ok_ate = acum
        if acum >= maximo:
            break
    return ok_ate


def resto_do_caminho(caminho, i0):
    return sum(math.hypot(caminho[k + 1][0] - caminho[k][0], caminho[k + 1][1] - caminho[k][1])
               for k in range(i0, len(caminho) - 1))


def rumo_do_caminho(caminho, i0, adiante=0.40):
    """Direcao do caminho de i0 ate ~adiante m a frente (None se curto demais)."""
    acum = 0.0
    x0, y0 = caminho[i0]
    for k in range(i0 + 1, len(caminho)):
        acum += math.hypot(caminho[k][0] - caminho[k - 1][0], caminho[k][1] - caminho[k - 1][1])
        if acum >= adiante:
            return math.atan2(caminho[k][1] - y0, caminho[k][0] - x0)
    if len(caminho) - 1 > i0:
        return math.atan2(caminho[-1][1] - y0, caminho[-1][0] - x0)
    return None


def folgas(pts, alcance_frente=1.0, alcance_lado=0.40):
    """(livre na frente do para-choque, livre do lado da lateral) em m, pelo 2o menor ponto
    (um ponto sozinho, como o retorno fantasma, nao derruba a decisao)."""
    frente, lado = [alcance_frente], [alcance_lado]
    for x, y in pts:
        if x >= FRENTE_X and abs(y) <= MEIA_LARGURA + 0.05:
            frente.append(min(alcance_frente, x - FRENTE_X))
        elif TRASEIRA_X <= x <= FRENTE_X + 0.40 and MEIA_LARGURA <= abs(y) <= MEIA_LARGURA + alcance_lado:
            lado.append(abs(y) - MEIA_LARGURA)
    frente.sort()
    lado.sort()
    return (frente[1] if len(frente) > 1 else frente[0]), (lado[1] if len(lado) > 1 else lado[0])


def avaliar(caminho, pose, pts, cfg):
    """Devolve (rapido: bool, motivo: str, medidas: dict). pose = (x, y, yaw) no mapa."""
    if not caminho or len(caminho) < 2:
        return False, 'sem caminho', {}
    i0, dist = indice_mais_perto(caminho, pose[0], pose[1])
    med = dict(fora=dist)
    if dist > cfg['fora_max']:
        return False, 'longe do caminho (%.2f m)' % dist, med
    med['reta'] = reta_a_frente(caminho, i0, cfg['tol_reta'])
    med['resto'] = resto_do_caminho(caminho, i0)
    rumo = rumo_do_caminho(caminho, i0)
    med['rumo'] = 0.0 if rumo is None else math.degrees(
        abs(math.atan2(math.sin(rumo - pose[2]), math.cos(rumo - pose[2]))))
    med['frente'], med['lado'] = folgas(pts) if pts is not None else (0.0, 0.0)
    if pts is None:
        return False, 'sem scan', med
    if med['resto'] < cfg['fim_min']:
        return False, 'perto do fim (%.2f m)' % med['resto'], med
    if med['reta'] < cfg['reta_min']:
        return False, 'curva (reta %.2f m)' % med['reta'], med
    if med['rumo'] > cfg['rumo_max']:
        return False, 'virando (%.0f graus)' % med['rumo'], med
    if med['frente'] < cfg['frente_min']:
        return False, 'obstaculo na frente (%.2f m)' % med['frente'], med
    if med['lado'] < cfg['lado_min']:
        return False, 'apertado dos lados (%.2f m)' % med['lado'], med
    return True, 'reta livre (%.1f m)' % med['reta'], med


class Histerese:
    """Desce na hora; sobe so depois de tempo_subir s seguidos de 'rapido'."""

    def __init__(self, tempo_subir=0.6):
        self.tempo_subir = tempo_subir
        self.rapido = False
        self.desde = None

    def passo(self, t, quer_rapido):
        if not quer_rapido:
            self.rapido, self.desde = False, None
        elif not self.rapido:
            self.desde = self.desde if self.desde is not None else t
            if t - self.desde >= self.tempo_subir:
                self.rapido = True
        return self.rapido


CFG_PADRAO = dict(reta_min=1.0, tol_reta=0.05, fim_min=0.80, rumo_max=15.0,
                  frente_min=0.60, lado_min=0.15, fora_max=0.25)


# ---------------------------------------------------------------------------
# No ROS
# ---------------------------------------------------------------------------
def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import Path
    from nav2_msgs.msg import SpeedLimit
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    def yaw_de(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    class Zonas(Node):
        def __init__(self):
            super().__init__('zonas_velocidade')
            self.declare_parameter('v_rapida', 0.13)   # = max_vel_x do DWB
            self.declare_parameter('v_normal', 0.08)   # velocidade de hoje
            self.declare_parameter('tempo_subir', 0.6)
            self.declare_parameter('topico_scan', '/scan_front')
            for k, v in CFG_PADRAO.items():
                self.declare_parameter(k, v)
            self.lock = threading.Lock()
            self.caminho, self.caminho_t = None, 0.0
            self.pts, self.pts_t = None, 0.0
            self.laser_tf = None
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.create_subscription(Path, '/plan', self.ao_plano, 5)
            self.create_subscription(LaserScan, self.get_parameter('topico_scan').value,
                                     self.ao_scan, qos_profile_sensor_data)
            self.pub = self.create_publisher(SpeedLimit, '/speed_limit', 10)
            self.pub_info = self.create_publisher(String, '/zona_velocidade', 10)
            self.hist = Histerese(self.get_parameter('tempo_subir').value)
            self.estado = None
            self.ult_pub = 0.0
            self.create_timer(0.2, self.ciclo)
            self.get_logger().info('Zonas de velocidade: RAPIDO=%.3f NORMAL=%.3f (%.1f %%)' % (
                self.p('v_rapida'), self.p('v_normal'), self.pct_normal()))

        def p(self, n):
            return self.get_parameter(n).value

        def pct_normal(self):
            return max(1.0, min(100.0, 100.0 * self.p('v_normal') / self.p('v_rapida')))

        def ao_plano(self, msg):
            pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
            with self.lock:
                self.caminho, self.caminho_t = pts, time.monotonic()

        def ao_scan(self, msg):
            if self.laser_tf is None:
                try:
                    tr = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
                    self.laser_tf = (tr.transform.translation.x, tr.transform.translation.y,
                                     yaw_de(tr.transform.rotation))
                except Exception:
                    return
            lx, ly, lth = self.laser_tf
            pts = []
            a = msg.angle_min
            for r in msg.ranges:
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    pts.append((lx + r * math.cos(a + lth), ly + r * math.sin(a + lth)))
                a += msg.angle_increment
            with self.lock:
                self.pts, self.pts_t = pts, time.monotonic()

        def publicar(self, rapido, motivo, forcar=False):
            pct = 100.0 if rapido else self.pct_normal()
            agora = time.monotonic()
            novo = 'RAPIDO' if rapido else 'NORMAL'
            if novo != self.estado:
                self.get_logger().info('%s (%.0f %% = %.3f): %s' % (
                    novo, pct, pct / 100.0 * self.p('v_rapida'), motivo))
            if forcar or novo != self.estado or agora - self.ult_pub > 1.0:
                m = SpeedLimit()
                m.header.stamp = self.get_clock().now().to_msg()
                m.header.frame_id = 'base_link'
                m.percentage = True
                m.speed_limit = float(pct)
                self.pub.publish(m)
                self.ult_pub = agora
            self.estado = novo

        def ciclo(self):
            agora = time.monotonic()
            with self.lock:
                caminho = self.caminho if agora - self.caminho_t < 2.5 else None
                pts = self.pts if agora - self.pts_t < 0.6 else None
            try:
                tr = self.tf_buffer.lookup_transform('map', 'base_link', Time())
                pose = (tr.transform.translation.x, tr.transform.translation.y,
                        yaw_de(tr.transform.rotation))
            except Exception:
                pose = None
            if pose is None:
                quer, motivo, med = False, 'sem TF map->base_link', {}
            else:
                cfg = {k: self.p(k) for k in CFG_PADRAO}
                quer, motivo, med = avaliar(caminho, pose, pts, cfg)
            rapido = self.hist.passo(agora, quer)
            if quer and not rapido:
                motivo = 'esperando %.1f s de reta livre' % self.hist.tempo_subir
            self.publicar(rapido, motivo)
            info = String()
            info.data = '%s %s | %s' % ('RAPIDO' if rapido else 'NORMAL', motivo,
                                        ' '.join('%s=%.2f' % kv for kv in sorted(med.items())))
            self.pub_info.publish(info)

    rclpy.init()
    no = Zonas()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            no.publicar(False, 'no encerrado', forcar=True)
            time.sleep(0.2)
        except Exception:
            pass
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
