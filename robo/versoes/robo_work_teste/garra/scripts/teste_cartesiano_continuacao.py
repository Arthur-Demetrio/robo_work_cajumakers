import rclpy

from rclpy.node import Node

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.srv import GetPositionFK

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState


class TesteCartesianoContinuacao(Node):

    def __init__(self):
        super().__init__('teste_cartesiano_continuacao')

        self.client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )

        self.fk_client = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )

        self.referencial = 'base_link'
        self.grupo = 'garra'
        self.link = 'tool0'

        # Posição inicial
        self.x_inicial = 0.202859503
        self.y = 0.005295
        self.z = 0.184070

        # Percurso de 5 mm
        self.incremento = 0.00010014
        self.quantidade_waypoints = 22

        # Orientação
        self.orientacao = [
            -0.358600796847,
            0.629698444383,
            0.557539534392,
            0.405012351950
        ]

        self.juntas = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

        self.posicoes_iniciais = [
            -0.0846821556137546,
            -0.1257160714712795,
            -0.000005708591217942365
        ]

    def calcular_fk(self, nomes_juntas, posicoes):

        if not self.fk_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'Serviço FK não encontrado!'
            )
            return None

        request = GetPositionFK.Request()

        request.header.frame_id = self.referencial
        request.header.stamp = self.get_clock().now().to_msg()

        request.fk_link_names = [self.link]

        request.robot_state = RobotState()
        request.robot_state.is_diff = True

        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = list(nomes_juntas)
        request.robot_state.joint_state.position = list(posicoes)

        future = self.fk_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(
                'Falha ao executar FK!'
            )
            return None

        resposta = future.result()

        if resposta.error_code.val != 1:
            self.get_logger().error(
                f'Erro na FK: {resposta.error_code.val}'
            )
            return None

        if not resposta.pose_stamped:
            self.get_logger().error(
                'FK não retornou nenhuma posição!'
            )
            return None

        pose = resposta.pose_stamped[0].pose

        self.get_logger().info(
            '===== FK DO ÚLTIMO PONTO ====='
        )

        self.get_logger().info(
            f'X: {pose.position.x:.9f} m'
        )

        self.get_logger().info(
            f'Y: {pose.position.y:.9f} m'
        )

        self.get_logger().info(
            f'Z: {pose.position.z:.9f} m'
        )

        return pose

    def executar(self):

        self.get_logger().info(
            'Aguardando serviço cartesiano...'
        )

        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                'Serviço cartesiano não encontrado!'
            )
            return

        request = GetCartesianPath.Request()

        request.header.frame_id = self.referencial
        request.header.stamp = self.get_clock().now().to_msg()

        # Estado inicial explícito
        request.start_state.is_diff = True

        request.start_state.joint_state.name = self.juntas
        request.start_state.joint_state.position = (
            self.posicoes_iniciais
        )

        request.group_name = self.grupo
        request.link_name = self.link

        # Geração dos waypoints
        waypoints = []

        for i in range(1, self.quantidade_waypoints + 1):

            pose = Pose()

            pose.position.x = (
                self.x_inicial + i * self.incremento
            )

            pose.position.y = self.y
            pose.position.z = self.z

            pose.orientation.x = self.orientacao[0]
            pose.orientation.y = self.orientacao[1]
            pose.orientation.z = self.orientacao[2]
            pose.orientation.w = self.orientacao[3]

            waypoints.append(pose)

        request.waypoints = waypoints

        # Parâmetros do planejamento
        request.max_step = 0.0001
        request.jump_threshold = 0.0

        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0

        request.avoid_collisions = False

        request.max_velocity_scaling_factor = 0.1
        request.max_acceleration_scaling_factor = 0.1

        # Informações do teste
        x_destino = (
            self.x_inicial
            + self.quantidade_waypoints * self.incremento
        )

        self.get_logger().info(
            f'X inicial: {self.x_inicial:.6f} m'
        )

        self.get_logger().info(
            f'X destino: {x_destino:.6f} m'
        )

        self.get_logger().info(
            f'Waypoints: {len(waypoints)}'
        )

        self.get_logger().info(
            f'Max step: {request.max_step:.6f} m'
        )

        self.get_logger().info(
            'Calculando caminho cartesiano...'
        )

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        if future.result() is None:
            self.get_logger().error(
                'Falha ao chamar o serviço!'
            )
            return

        response = future.result()

        trajectory = response.solution.joint_trajectory

        self.get_logger().info(
            f'Fração: {response.fraction * 100:.2f}%'
        )

        self.get_logger().info(
            f'Código: {response.error_code.val}'
        )

        self.get_logger().info(
            f'Quantidade de pontos: {len(trajectory.points)}'
        )

        self.get_logger().info(
            f'Juntas: {trajectory.joint_names}'
        )

        if not trajectory.points:
            self.get_logger().warning(
                'Nenhum ponto foi retornado.'
            )
            return

        primeiro = trajectory.points[0]
        ultimo = trajectory.points[-1]

        self.get_logger().info(
            f'Primeiras posições: {primeiro.positions}'
        )

        self.get_logger().info(
            f'Últimas posições: {ultimo.positions}'
        )

        self.get_logger().info(
            f'Duração: '
            f'{ultimo.time_from_start.sec}s '
            f'{ultimo.time_from_start.nanosec}ns'
        )

        # Últimos 5 estados
        self.get_logger().info(
            'Últimos estados calculados:'
        )

        for ponto in trajectory.points[-5:]:

            self.get_logger().info(
                str(ponto.positions)
            )

        # FK do último ponto
        pose_final = self.calcular_fk(
            trajectory.joint_names,
            ultimo.positions
        )

        if pose_final is not None:

            x_alcancado = pose_final.position.x
            y_alcancado = pose_final.position.y
            z_alcancado = pose_final.position.z

            erro_x = (x_destino - x_alcancado) * 1000.0
            erro_y = (self.y - y_alcancado) * 1000.0
            erro_z = (self.z - z_alcancado) * 1000.0

            self.get_logger().info(
                '===== COMPARAÇÃO CARTESIANA ====='
            )

            self.get_logger().info(
                f'X solicitado: {x_destino:.9f} m'
            )

            self.get_logger().info(
                f'X alcançado: {x_alcancado:.9f} m'
            )

            self.get_logger().info(
                f'Y solicitado: {self.y:.9f} m'
            )

            self.get_logger().info(
                f'Y alcançado: {y_alcancado:.9f} m'
            )

            self.get_logger().info(
                f'Z solicitado: {self.z:.9f} m'
            )

            self.get_logger().info(
                f'Z alcançado: {z_alcancado:.9f} m'
            )

            self.get_logger().info(
                f'Erro X: {erro_x:.4f} mm'
            )

            self.get_logger().info(
                f'Erro Y: {erro_y:.4f} mm'
            )

            self.get_logger().info(
                f'Erro Z: {erro_z:.4f} mm'
            )

        if response.fraction >= 0.999:
            self.get_logger().info(
                'Caminho cartesiano completo!'
            )
        else:
            self.get_logger().warning(
                'Caminho cartesiano interrompido.'
            )


def main():

    rclpy.init()

    node = TesteCartesianoContinuacao()

    try:
        node.executar()

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()