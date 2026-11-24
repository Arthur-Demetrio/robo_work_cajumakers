#!/usr/bin/env python3
"""Atravessa uma passagem estreita entrando alinhado.

1) No RViz, ferramenta "Publish Point": clique no CENTRO DA ENTRADA (lado do robô).
2) Clique no CENTRO DA SAÍDA (outro lado).
O robô vai a um ponto de preparo antes da entrada, alinha e atravessa.

Uso: python3 ~/passagem_alinhada.py [frente|lado] [dist_preparo_m] [dist_saida_m]
"""
import math
import sys

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

STATUS = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCESSO',
    GoalStatus.STATUS_CANCELED: 'CANCELADO',
    GoalStatus.STATUS_ABORTED: 'ABORTADO',
}


class Passagem(Node):
    def __init__(self, modo, d_prep, d_saida):
        super().__init__('passagem_alinhada')
        self.modo, self.d_prep, self.d_saida = modo, d_prep, d_saida
        self.pontos = []
        self.goal_handle = None
        self.create_subscription(PointStamped, '/clicked_point', self.clique, 10)
        self.cliente = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def clique(self, msg):
        if len(self.pontos) >= 2:
            return
        if msg.header.frame_id != 'map':
            self.get_logger().warn(
                f'Ponto veio no frame "{msg.header.frame_id}". Use Fixed Frame = map no RViz.')
            return
        self.pontos.append((msg.point.x, msg.point.y))
        nome = 'ENTRADA' if len(self.pontos) == 1 else 'SAÍDA'
        self.get_logger().info(f'{nome}: x={msg.point.x:.2f} y={msg.point.y:.2f}')
        if len(self.pontos) == 1:
            self.get_logger().info('Agora clique no centro da SAÍDA (outro lado da passagem).')

    def pose(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        p.pose.orientation.z = math.sin(yaw / 2.0)
        p.pose.orientation.w = math.cos(yaw / 2.0)
        return p

    def navegar(self, pose, nome):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.get_logger().info(
            f'{nome}: x={pose.pose.position.x:.2f} y={pose.pose.position.y:.2f}')
        envio = self.cliente.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, envio)
        self.goal_handle = envio.result()
        if not self.goal_handle.accepted:
            self.get_logger().error(f'{nome}: goal rejeitado.')
            return False
        resultado = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, resultado)
        status = resultado.result().status
        self.goal_handle = None
        self.get_logger().info(f'{nome}: {STATUS.get(status, status)}')
        return status == GoalStatus.STATUS_SUCCEEDED

    def executar(self):
        self.get_logger().info(
            'No RViz, use "Publish Point" e clique no centro da ENTRADA da passagem.')
        while rclpy.ok() and len(self.pontos) < 2:
            rclpy.spin_once(self, timeout_sec=0.1)
        (x1, y1), (x2, y2) = self.pontos
        comprimento = math.hypot(x2 - x1, y2 - y1)
        if comprimento < 0.05:
            self.get_logger().error('Entrada e saída coincidem. Clique em pontos distintos.')
            return
        eixo = math.atan2(y2 - y1, x2 - x1)
        ux, uy = math.cos(eixo), math.sin(eixo)
        # frente: +x do robô ao longo do eixo | lado: +y do robô (esquerda) ao longo do eixo
        yaw = eixo if self.modo == 'frente' else eixo - math.pi / 2.0
        self.get_logger().info(
            f'Passagem: {comprimento:.2f} m | eixo {math.degrees(eixo):.1f} graus | '
            f'modo {self.modo} | orientação do robô {math.degrees(yaw):.1f} graus')
        preparo = self.pose(x1 - self.d_prep * ux, y1 - self.d_prep * uy, yaw)
        saida = self.pose(x2 + self.d_saida * ux, y2 + self.d_saida * uy, yaw)
        if not self.cliente.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('navigate_to_pose indisponível. O Nav2 está ativo?')
            return
        if self.navegar(preparo, '1/2 PREPARO (alinhar fora da passagem)'):
            self.navegar(saida, '2/2 TRAVESSIA')
        else:
            self.get_logger().warn('Preparo não concluído: travessia não enviada.')

    def cancelar(self):
        if self.goal_handle is not None:
            try:
                fut = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
                self.get_logger().info('Cancelamento enviado.')
            except Exception:
                pass


def main():
    modo = sys.argv[1] if len(sys.argv) > 1 else 'frente'
    if modo not in ('frente', 'lado'):
        print('Modo deve ser "frente" ou "lado".')
        return
    d_prep = float(sys.argv[2]) if len(sys.argv) > 2 else 0.60
    d_saida = float(sys.argv[3]) if len(sys.argv) > 3 else 0.50
    rclpy.init()
    no = Passagem(modo, d_prep, d_saida)
    try:
        no.executar()
    except KeyboardInterrupt:
        no.cancelar()
    finally:
        no.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
