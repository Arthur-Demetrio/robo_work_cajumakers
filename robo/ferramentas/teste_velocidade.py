#!/usr/bin/env python3
"""Mede a velocidade REAL do robo para cada comando (pelo LiDAR e pela IMU).

Por que: na prova1 o robo andou praticamente na mesma velocidade real (~0,12-0,21 m/s)
com comandos de 0,03 a 0,08 m/s, e girou ~15-18 graus/s com qualquer comando de 0,03 a
0,14 rad/s. Ou seja: aumentar o limite do Nav2 talvez nao deixe o robo mais rapido; quem
manda e a conversao comando -> PWM da base. Este teste mostra a curva de verdade.

Como montar:
  - Nav2 ligado mas SEM goal (o velocity_smoother parado); Collision Monitor ligado.
  - Robo em area aberta, DE FRENTE para uma parede lisa a 1,5-2,5 m, nada no caminho,
    e pelo menos 1 m livre atras (ele vai e volta).
  - Para o giro: 0,6 m livres em volta do robo.
Uso:
    python3 teste_velocidade.py                 # linear: 0.03 0.05 0.08 0.11 0.14
    python3 teste_velocidade.py --giro          # giro: 0.05 0.10 0.14 0.20 0.28
    python3 teste_velocidade.py 0.04 0.10 0.16  # escolher os comandos
Ctrl+C para o robo na hora.

Os comandos vao para /cmd_vel_smoothed (entrada do Collision Monitor): passam pela
protecao do Collision Monitor, mas nao pelo limite de 0.08 do velocity_smoother.
"""
import math
import statistics
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from nav2_msgs.msg import CollisionMonitorState
from sensor_msgs.msg import LaserScan
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

TOPICO = '/cmd_vel_smoothed'
T_CMD = 1.6       # s com o comando ligado
T_PARADO = 1.5    # s parado entre comandos
FRENTE_MIN = 0.55  # para se a parede ficar mais perto que isso (m, do LiDAR)


class Teste(Node):
    def __init__(self):
        super().__init__('teste_velocidade')
        self.pub = self.create_publisher(TwistStamped, TOPICO, 10)
        self.create_subscription(LaserScan, '/scan_front', self.ao_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.ao_odom, 20)
        self.create_subscription(CollisionMonitorState, '/collision_monitor_state', self.ao_cm, 10)
        self.frente = []   # (t, distancia a frente no LiDAR)
        self.yaws = []     # (t, yaw da odom/IMU)
        self.cm_stop = False
        self.laser_yaw = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def ao_cm(self, msg):
        self.cm_stop = msg.action_type == 1

    def ao_scan(self, msg):
        if self.laser_yaw is None:
            # angulo do LiDAR em relacao a frente do robo (base_link)
            try:
                q = self.tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time()).transform.rotation
                self.laser_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            except Exception:
                return
        ds = []
        a = msg.angle_min + self.laser_yaw
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max and abs(math.atan2(math.sin(a), math.cos(a))) < math.radians(6):
                ds.append(r)
            a += msg.angle_increment
        if len(ds) >= 2:
            self.frente.append((time.monotonic(), statistics.median(ds)))

    def ao_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.yaws.append((time.monotonic(), yaw))

    def cmd(self, vx=0.0, wz=0.0):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        m.twist.linear.x, m.twist.angular.z = float(vx), float(wz)
        self.pub.publish(m)

    def rodar(self, segundos, vx=0.0, wz=0.0):
        """Publica o comando a 20 Hz. Devolve False se teve de parar por seguranca."""
        fim = time.monotonic() + segundos
        bloqueou = False
        while time.monotonic() < fim:
            rclpy.spin_once(self, timeout_sec=0.05)
            if vx > 0 and self.frente and self.frente[-1][1] < FRENTE_MIN:
                self.get_logger().warn('Parede a %.2f m: parando.' % self.frente[-1][1])
                self.parar()
                return False
            bloqueou = bloqueou or (self.cm_stop and (vx != 0 or wz != 0))
            self.cmd(vx, wz)
        if bloqueou:
            self.get_logger().warn('Collision Monitor deu STOP durante o comando: medida invalida.')
        return not bloqueou

    def parar(self, segundos=0.0):
        fim = time.monotonic() + max(segundos, 0.3)
        while time.monotonic() < fim:
            self.cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)

    def esperar_dados(self):
        fim = time.monotonic() + 5.0
        while time.monotonic() < fim and (len(self.frente) < 3 or len(self.yaws) < 3):
            rclpy.spin_once(self, timeout_sec=0.1)
        return len(self.frente) >= 3 and len(self.yaws) >= 3


def inclinacao(serie, t0, t1):
    """Velocidade media (derivada por minimos quadrados) da serie entre t0 e t1."""
    pts = [(t, v) for t, v in serie if t0 <= t <= t1]
    if len(pts) < 3:
        return None
    mt = sum(p[0] for p in pts) / len(pts)
    mv = sum(p[1] for p in pts) / len(pts)
    den = sum((p[0] - mt) ** 2 for p in pts)
    return None if den == 0 else sum((p[0] - mt) * (p[1] - mv) for p in pts) / den


def desenrolar(yaws):
    out, soma, ant = [], 0.0, None
    for t, y in yaws:
        if ant is not None:
            soma += math.atan2(math.sin(y - ant), math.cos(y - ant))
        out.append((t, soma))
        ant = y
    return out


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    giro = '--giro' in sys.argv
    valores = [float(a) for a in args] or ([0.05, 0.10, 0.14, 0.20, 0.28] if giro else
                                           [0.03, 0.05, 0.08, 0.11, 0.14])
    # Ctrl+C sem derrubar o ROS antes de mandar o comando zero
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    no = Teste()
    log = no.get_logger()
    linhas = []
    try:
        if not no.esperar_dados():
            log.error('Sem /scan_front ou /odom. O robo esta ligado?')
            return 1
        if not giro:
            d0 = no.frente[-1][1]
            log.info('Parede a frente a %.2f m.' % d0)
            if d0 < 1.3:
                log.error('Preciso de pelo menos 1,3 m livres na frente (achei %.2f m).' % d0)
                return 1
        for v in valores:
            if giro:
                for sinal in (+1, -1):
                    t0 = time.monotonic()
                    ok = no.rodar(T_CMD, wz=sinal * v)
                    t1 = time.monotonic()
                    no.parar(T_PARADO)
                    serie = desenrolar(no.yaws)
                    w = inclinacao(serie, t0 + 0.6, t1)
                    total = [s for t, s in serie if t <= t1 + T_PARADO]
                    ini = [s for t, s in serie if t <= t0]
                    giro_tot = abs(total[-1] - ini[-1]) if total and ini else float('nan')
                    linhas.append(('giro %+.2f rad/s' % (sinal * v),
                                   'real %.2f rad/s (%.0f graus/s)' % (abs(w or 0), math.degrees(abs(w or 0))),
                                   'giro total %.0f graus' % math.degrees(giro_tot), ok))
                    log.info(' | '.join(str(x) for x in linhas[-1]))
            else:
                for sinal in (+1, -1):
                    d_ini = no.frente[-1][1]
                    t0 = time.monotonic()
                    ok = no.rodar(T_CMD, vx=sinal * v)
                    t1 = time.monotonic()
                    no.parar(T_PARADO)
                    vr = inclinacao(no.frente, t0 + 0.6, t1)
                    d_fim = no.frente[-1][1]
                    linhas.append(('linear %+.2f m/s' % (sinal * v),
                                   'real %.3f m/s' % abs(vr or 0),
                                   'andou %.2f m em %.1f s (com o embalo)' % (abs(d_ini - d_fim), T_CMD), ok))
                    log.info(' | '.join(str(x) for x in linhas[-1]))
                    if not ok:
                        break
                if linhas and not linhas[-1][3]:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            no.parar(0.5)
        except Exception as exc:
            print('ATENCAO: nao consegui mandar o comando zero (%s). Confira se o robo parou.' % exc)
        print('\n===== RESULTADO (copie e me mande) =====')
        for a, b, c, ok in linhas:
            print('%-20s %-28s %-36s %s' % (a, b, c, '' if ok else 'INVALIDO'))
        no.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
