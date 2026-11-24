#!/usr/bin/env python3
"""Ponte simulada: faz o papel do Arduino + Delta-2D + slam_toolbox na bancada de encosto.

  /cmd_vel (saida do Collision Monitor) -> MODELO DO MOTOR -> /sim/vel_real (Gazebo, VelocityControl)
  /sim/scan (gpu_lidar ideal)           -> MODELO DO DELTA-2D -> /scan_front (frame laser_raw)
  /sim/verdade (pose real no mundo)     -> /imu/data (yaw real, 25 Hz)
                                         -> TF map->odom = verdade * odom^-1 (localizacao ideal, a cada 0,2 s)
O /odom vem do no REAL command_imu_odometry (tabela + yaw da IMU): o erro de odometria aparece sozinho.

Modelo do motor (numeros medidos, CLAUDE.md "Fisica do robo"):
  - frente: velocidade real = tabela speed_table_x (comando 0,016 -> ~0,10 m/s: minimo do PWM 70);
  - lado: tabela speed_table_y * fator_lado (medido: ~0,05-0,06 m/s real com comando 0,08-0,16);
  - lado vira avanco: vx += k_avanco * |vy| (06/11: k 0,31-0,37 a 0,08);
  - giro parado: |wz| < 0,15 -> 0,15 rad/s (minimo do PWM 50); zona morta 0,02 rad/s;
  - deriva de rumo andando de lado (missao23: -0,35 graus/s);
  - atraso de 1a ordem (tau) e timeout de comando 0,25 s (como a ponte).
Tudo ajustavel pelo parametro 'modelo' (JSON) para os cenarios.
"""
import json
import math
import os
import random
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist, TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

LOC_YAML = os.path.expanduser('~/robo_jazzy_ws/src/robo_localization/config/localization.yaml')

MODELO_PADRAO = dict(
    fator_frente=1.0,
    fator_lado=0.60,
    k_avanco=0.33,            # fracao de |vy| que vira vx
    sinal_avanco=1.0,         # +1 = lado vira avanco (para a mesa), -1 = vira re
    giro_min=0.15,
    giro_zona_morta=0.02,
    deriva_lado=math.radians(-0.35),
    tau=0.10,
    timeout_cmd=0.25,
    zona_morta_cmd=0.005,
    # LiDAR Delta-2D
    r_cego=0.146,
    r_max=6.0,
    ruido=0.004,
    perda_raio=0.03,          # prob. de um raio isolado voltar inf
    perda_rajada=0.10,        # prob. por scan de uma rajada de 3-8 raios seguidos inf
    # IMU
    imu_hz=25.0,
    imu_falha_andando=0.0,    # prob. por segundo (andando) de um buraco de 0,6 s na IMU
    # localizacao
    periodo_mapa=0.2,
    erro_mapa=[0.0, 0.0, 0.0],  # erro fixo do map->odom (dx, dy, dyaw): SLAM desalinhado
)


def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            f = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + f * (ys[i] - ys[i - 1])
    return ys[-1]


def yaw_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compor(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inversa(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


class PonteSim(Node):
    def __init__(self):
        super().__init__('ponte_sim')
        self.declare_parameter('modelo_arquivo', '')
        self.declare_parameter('semente', 1)
        self.m = dict(MODELO_PADRAO)
        arq = self.get_parameter('modelo_arquivo').value
        if arq:
            self.m.update(json.load(open(arq)))
        self.rng = random.Random(int(self.get_parameter('semente').value))
        p = yaml.safe_load(open(LOC_YAML))['command_imu_odometry']['ros__parameters']
        self.t_cmd, self.t_x, self.t_y = p['speed_table_cmd'], p['speed_table_x'], p['speed_table_y']

        self.cmd = (0.0, 0.0, 0.0)
        self.t_ultimo_cmd = 0.0
        self.vel = [0.0, 0.0, 0.0]
        self.verdade = None
        self.odom = None
        self.mapa_odom = None
        self.t_mapa = 0.0
        self.erro_T = None
        self.imu_buraco_ate = 0.0

        self.pub_vel = self.create_publisher(Twist, '/sim/vel_real', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan_front', qos_profile_sensor_data)
        self.pub_imu = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self.pub_real = self.create_publisher(TwistStamped, '/sim/vel_real_eco', 10)
        self.tf = TransformBroadcaster(self)
        self.tf_est = StaticTransformBroadcaster(self)
        self.tf_est.sendTransform([self._tf('base_link', 'laser_raw', (0.33, 0.005, 0.0), 0.12),
                                   self._tf('base_link', 'imu_link', (0.0, 0.0, 0.0), 0.05)])

        self.create_subscription(TwistStamped, '/cmd_vel', self.ao_cmd, 20)
        self.create_subscription(LaserScan, '/sim/scan', self.ao_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/sim/verdade', self.ao_verdade, 50)
        self.create_subscription(Odometry, '/odom', self.ao_odom, 50)
        self.create_timer(0.02, self.passo_motor)
        self.create_timer(1.0 / self.m['imu_hz'], self.passo_imu)
        self.create_timer(0.05, self.passo_tf)
        self.get_logger().info('ponte_sim: modelo %s' % json.dumps(self.m))

    def _tf(self, pai, filho, xyz, z=0.0, yaw=0.0):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = pai, filho
        t.transform.translation.x, t.transform.translation.y = xyz[0], xyz[1]
        t.transform.translation.z = z
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return t

    # ---------------- motor ----------------
    def ao_cmd(self, msg):
        tw = msg.twist
        self.cmd = (tw.linear.x, tw.linear.y, tw.angular.z)
        self.t_ultimo_cmd = time.monotonic()

    def velocidade_real(self, cmd):
        m = self.m
        cx, cy, cw = cmd
        zm = m['zona_morta_cmd']
        vx = math.copysign(interp(abs(cx), self.t_cmd, self.t_x) * m['fator_frente'], cx) if abs(cx) > zm else 0.0
        vy = math.copysign(interp(abs(cy), self.t_cmd, self.t_y) * m['fator_lado'], cy) if abs(cy) > zm else 0.0
        andando = vx != 0.0 or vy != 0.0
        if abs(cw) < m['giro_zona_morta']:
            wz = 0.0
        elif andando:
            wz = cw
        else:
            wz = math.copysign(max(abs(cw), m['giro_min']), cw)
        if vy != 0.0:
            vx += m['sinal_avanco'] * m['k_avanco'] * abs(vy)
            wz += m['deriva_lado']
        return vx, vy, wz

    def passo_motor(self):
        cmd = self.cmd if time.monotonic() - self.t_ultimo_cmd <= self.m['timeout_cmd'] else (0.0, 0.0, 0.0)
        alvo = self.velocidade_real(cmd)
        a = 1.0 - math.exp(-0.02 / max(self.m['tau'], 1e-3))
        for i in range(3):
            self.vel[i] += a * (alvo[i] - self.vel[i])
            if alvo[i] == 0.0 and abs(self.vel[i]) < 1e-3:
                self.vel[i] = 0.0
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = self.vel
        self.pub_vel.publish(t)
        e = TwistStamped()
        e.header.stamp = self.get_clock().now().to_msg()
        e.twist = t
        self.pub_real.publish(e)

    # ---------------- LiDAR ----------------
    def ao_scan(self, msg):
        m = self.m
        n = len(msg.ranges)
        perdidos = set()
        if self.rng.random() < m['perda_rajada']:   # por scan: 3-8 raios seguidos inf
            i0, k = self.rng.randrange(n), self.rng.randint(3, 8)
            perdidos.update((i0 + j) % n for j in range(k))
        out = LaserScan()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'laser_raw'
        out.angle_min, out.angle_max = msg.angle_min, msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = 0.0
        out.scan_time = 1.0 / 5.9
        out.range_min, out.range_max = 0.08, m['r_max']   # como o /scan_front real
        rs = []
        for i, r in enumerate(msg.ranges):
            if (not math.isfinite(r) or r < m['r_cego'] or r > m['r_max'] or i in perdidos
                    or self.rng.random() < m['perda_raio']):
                rs.append(float('inf'))
            else:
                rs.append(r + self.rng.gauss(0.0, m['ruido']))
        out.ranges = rs
        self.pub_scan.publish(out)

    # ---------------- IMU e localizacao ----------------
    def ao_verdade(self, msg):
        p = msg.pose.pose
        self.verdade = (p.position.x, p.position.y, yaw_q(p.orientation), msg.twist.twist.angular.z)

    def ao_odom(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y, yaw_q(p.orientation))

    def passo_imu(self):
        if self.verdade is None:
            return
        agora = time.monotonic()
        andando = any(abs(v) > 0.01 for v in self.vel)
        if (andando and self.m['imu_falha_andando'] > 0 and agora > self.imu_buraco_ate
                and self.rng.random() < self.m['imu_falha_andando'] / self.m['imu_hz']):
            self.imu_buraco_ate = agora + 0.6
        if agora < self.imu_buraco_ate:
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'imu_link'
        yaw = self.verdade[2]
        m.orientation.z, m.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        m.angular_velocity.z = self.verdade[3]
        m.orientation_covariance[0] = m.orientation_covariance[4] = m.orientation_covariance[8] = 0.01
        self.pub_imu.publish(m)

    def passo_tf(self):
        if self.verdade is None or self.odom is None:
            return
        agora = time.monotonic()
        if self.mapa_odom is None or agora - self.t_mapa >= self.m['periodo_mapa']:
            mo = compor(self.verdade[:3], inversa(self.odom))
            if self.erro_T is None:
                # erro fixo medido NA POSE INICIAL do robo: ele se acha em (x+dx, y+dy, yaw+dyaw)
                e = self.m['erro_mapa']
                v = self.verdade
                self.erro_T = compor((v[0] + e[0], v[1] + e[1], v[2] + e[2]), inversa(v[:3]))
            self.mapa_odom = compor(self.erro_T, mo)
            self.t_mapa = agora
        x, y, yaw = self.mapa_odom
        self.tf.sendTransform(self._tf('map', 'odom', (x, y), 0.0, yaw))


def main():
    rclpy.init()
    no = PonteSim()
    try:
        rclpy.spin(no)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            no.pub_vel.publish(Twist())
        except Exception:
            pass
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
