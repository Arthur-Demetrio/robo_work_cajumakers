#!/usr/bin/env python3

import math
import threading
import time

import rclpy
from rclpy.action import (
    ActionServer,
    GoalResponse,
    CancelResponse,
)
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import DriveOnHeading


class MecanumEscapeNode(Node):

    def __init__(self):
        super().__init__('mecanum_escape_node')

        # ----------------------------------------------------
        # Geometria / comportamento
        # ----------------------------------------------------

        # Robô:
        # y físico aproximadamente +/-0.315 m.
        self.robot_half_width = 0.315

        # Margem adicional para escolher uma lateral.
        self.side_margin = 0.03

        # Setor analisado de cada lado.
        # Esquerda: +60° a +120°
        # Direita:  -120° a -60°
        self.side_sector_min_deg = 60.0
        self.side_sector_max_deg = 120.0

        # Scan mais velho que isso não será usado.
        self.scan_timeout = 0.50

        # Se o Collision Monitor bloquear a lateral durante
        # muito tempo, desistimos em vez de ficar insistindo.
        self.no_progress_timeout = 1.50

        # ----------------------------------------------------
        # Estado compartilhado
        # ----------------------------------------------------

        self.lock = threading.Lock()

        self.latest_scan = None
        self.latest_scan_receive_time = 0.0

        self.last_final_cmd_y = 0.0

        self.callback_group = ReentrantCallbackGroup()

        # ----------------------------------------------------
        # Entradas
        # ----------------------------------------------------

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_front',
            self.scan_callback,
            qos_profile_sensor_data,
            callback_group=self.callback_group,
        )

        # Saída REAL depois do Collision Monitor.
        # Usamos isso para saber se a lateral realmente
        # está sendo liberada.
        self.final_cmd_sub = self.create_subscription(
            TwistStamped,
            '/cmd_vel',
            self.final_cmd_callback,
            20,
            callback_group=self.callback_group,
        )

        # ----------------------------------------------------
        # Saída para pipeline Nav2
        # ----------------------------------------------------

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel_nav',
            10,
        )

        # ----------------------------------------------------
        # Action server
        # ----------------------------------------------------

        self.action_server = ActionServer(
            self,
            DriveOnHeading,
            'mecanum_escape',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info(
            'Mecanum Escape pronto. '
            'Aguardando action /mecanum_escape.'
        )

    # ========================================================
    # Callbacks
    # ========================================================

    def scan_callback(self, msg):
        with self.lock:
            self.latest_scan = msg
            self.latest_scan_receive_time = time.monotonic()

    def final_cmd_callback(self, msg):
        with self.lock:
            self.last_final_cmd_y = msg.twist.linear.y

    def goal_callback(self, goal_request):
        self.get_logger().info(
            'Solicitacao de escape lateral recebida.'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().warn(
            'Cancelamento do escape lateral solicitado.'
        )
        return CancelResponse.ACCEPT

    # ========================================================
    # Laser
    # ========================================================

    def robust_side_clearance(
        self,
        scan,
        positive_side,
    ):
        """
        Retorna uma estimativa robusta da folga lateral.

        Em vez de usar somente o menor ponto (muito sensível
        a ruído), usamos aproximadamente o percentil 20 dos
        retornos do setor lateral.

        positive_side=True:
            esquerda (+Y)

        positive_side=False:
            direita (-Y)
        """

        values = []

        angle = scan.angle_min

        for r in scan.ranges:

            deg = math.degrees(angle)

            if positive_side:
                in_sector = (
                    self.side_sector_min_deg
                    <= deg
                    <= self.side_sector_max_deg
                )
            else:
                in_sector = (
                    -self.side_sector_max_deg
                    <= deg
                    <= -self.side_sector_min_deg
                )

            if in_sector:

                if math.isfinite(r):
                    if scan.range_min <= r <= scan.range_max:
                        values.append(r)

                elif math.isinf(r):
                    # "Sem retorno" dentro do alcance é tratado
                    # como espaço aberto até range_max.
                    if math.isfinite(scan.range_max):
                        values.append(scan.range_max)

            angle += scan.angle_increment

        if len(values) < 3:
            return None

        values.sort()

        # Percentil ~20%.
        index = int(0.20 * (len(values) - 1))

        return values[index]

    # ========================================================
    # Comando
    # ========================================================

    def publish_side_command(self, vy):
        msg = TwistStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = float(vy)
        msg.twist.linear.z = 0.0

        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0

        self.cmd_pub.publish(msg)

    def stop(self):
        # Publicamos algumas vezes para garantir que o smoother
        # receba explicitamente o comando de parada.
        for _ in range(3):
            self.publish_side_command(0.0)
            time.sleep(0.02)

    # ========================================================
    # Action
    # ========================================================

    async def execute_callback(self, goal_handle):

        start_time = time.monotonic()

        request = goal_handle.request

        # Usaremos target.x apenas como "distância desejada",
        # apesar do movimento real ser lateral.
        distance = abs(float(request.target.x))

        if distance < 0.01:
            distance = 0.10

        speed = abs(float(request.speed))

        if speed < 0.005:
            speed = 0.03

        timeout = (
            float(request.time_allowance.sec)
            + float(request.time_allowance.nanosec) / 1e9
        )

        if timeout <= 0.0:
            timeout = 6.0

        # Espaço necessário do centro do LiDAR até a região
        # lateral que o robô ocupará.
        required_clearance = (
            self.robot_half_width
            + distance
            + self.side_margin
        )

        # ----------------------------------------------------
        # Conferir scan
        # ----------------------------------------------------

        with self.lock:
            scan = self.latest_scan
            scan_age = (
                time.monotonic()
                - self.latest_scan_receive_time
            )

        result = DriveOnHeading.Result()

        if scan is None or scan_age > self.scan_timeout:

            self.stop()

            result.total_elapsed_time = Duration(
                seconds=time.monotonic() - start_time
            ).to_msg()

            result.error_code = 1
            result.error_msg = 'LaserScan ausente ou antigo'

            self.get_logger().error(
                'Escape cancelado: /scan_front invalido.'
            )

            goal_handle.abort()
            return result

        left = self.robust_side_clearance(
            scan,
            positive_side=True,
        )

        right = self.robust_side_clearance(
            scan,
            positive_side=False,
        )

        if left is None or right is None:

            self.stop()

            result.total_elapsed_time = Duration(
                seconds=time.monotonic() - start_time
            ).to_msg()

            result.error_code = 1
            result.error_msg = (
                'Pontos laterais insuficientes no LaserScan'
            )

            goal_handle.abort()
            return result

        self.get_logger().info(
            f'Folga lateral: '
            f'esquerda={left:.3f} m, '
            f'direita={right:.3f} m, '
            f'necessaria={required_clearance:.3f} m'
        )

        # ----------------------------------------------------
        # Escolher direção
        # ----------------------------------------------------

        left_ok = left >= required_clearance
        right_ok = right >= required_clearance

        if not left_ok and not right_ok:

            self.stop()

            result.total_elapsed_time = Duration(
                seconds=time.monotonic() - start_time
            ).to_msg()

            result.error_code = 1
            result.error_msg = (
                'Nenhuma lateral possui folga suficiente'
            )

            self.get_logger().warn(
                'Escape lateral recusado: '
                'nenhum lado e seguro.'
            )

            goal_handle.abort()
            return result

        if left_ok and right_ok:
            direction = +1.0 if left >= right else -1.0

        elif left_ok:
            direction = +1.0

        else:
            direction = -1.0

        if direction > 0.0:
            direction_name = 'ESQUERDA'
        else:
            direction_name = 'DIREITA'

        vy = direction * speed

        self.get_logger().warn(
            f'ESCAPE MECANUM: {direction_name} '
            f'| vy={vy:.3f} m/s '
            f'| alvo={distance:.3f} m'
        )

        # ----------------------------------------------------
        # Executar
        # ----------------------------------------------------

        feedback = DriveOnHeading.Feedback()

        traveled = 0.0
        previous_time = time.monotonic()
        last_progress_time = previous_time

        while rclpy.ok():

            now = time.monotonic()

            if goal_handle.is_cancel_requested:

                self.stop()

                result.total_elapsed_time = Duration(
                    seconds=now - start_time
                ).to_msg()

                result.error_code = 1
                result.error_msg = 'Cancelado'

                goal_handle.canceled()
                return result

            elapsed = now - start_time

            if elapsed > timeout:

                self.stop()

                result.total_elapsed_time = Duration(
                    seconds=elapsed
                ).to_msg()

                result.error_code = 1
                result.error_msg = (
                    'Timeout no escape lateral'
                )

                self.get_logger().warn(
                    'Escape lateral excedeu timeout.'
                )

                goal_handle.abort()
                return result

            # Scan precisa continuar vivo durante a manobra.
            with self.lock:
                current_scan_age = (
                    now - self.latest_scan_receive_time
                )

                final_y = self.last_final_cmd_y

            if current_scan_age > self.scan_timeout:

                self.stop()

                result.total_elapsed_time = Duration(
                    seconds=elapsed
                ).to_msg()

                result.error_code = 1
                result.error_msg = (
                    'LaserScan ficou antigo durante escape'
                )

                self.get_logger().error(
                    'Escape interrompido: LiDAR perdeu dados.'
                )

                goal_handle.abort()
                return result

            # Continua solicitando movimento.
            self.publish_side_command(vy)

            dt = now - previous_time
            previous_time = now

            # Integra SOMENTE o comando que realmente passou
            # pelo Collision Monitor.
            useful_final_y = final_y * direction

            if useful_final_y > 0.005:
                traveled += useful_final_y * dt
                last_progress_time = now

            # Se o Collision Monitor estiver bloqueando,
            # desistimos em vez de empurrar indefinidamente.
            if (
                elapsed > 0.50
                and
                now - last_progress_time
                > self.no_progress_timeout
            ):

                self.stop()

                result.total_elapsed_time = Duration(
                    seconds=elapsed
                ).to_msg()

                result.error_code = 1
                result.error_msg = (
                    'Movimento lateral bloqueado '
                    'pelo sistema de seguranca'
                )

                self.get_logger().warn(
                    'Escape lateral bloqueado. '
                    'Passando para proxima recovery.'
                )

                goal_handle.abort()
                return result

            feedback.distance_traveled = float(traveled)
            goal_handle.publish_feedback(feedback)

            if traveled >= distance:

                self.stop()

                elapsed = (
                    time.monotonic() - start_time
                )

                result.total_elapsed_time = Duration(
                    seconds=elapsed
                ).to_msg()

                result.error_code = 0
                result.error_msg = ''

                self.get_logger().info(
                    f'Escape lateral concluido: '
                    f'{direction_name}, '
                    f'{traveled:.3f} m.'
                )

                goal_handle.succeed()
                return result

            time.sleep(0.05)

        self.stop()

        result.total_elapsed_time = Duration(
            seconds=time.monotonic() - start_time
        ).to_msg()

        result.error_code = 1
        result.error_msg = 'ROS encerrado'

        goal_handle.abort()

        return result


def main(args=None):

    rclpy.init(args=args)

    node = MecanumEscapeNode()

    executor = MultiThreadedExecutor(
        num_threads=3
    )

    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
