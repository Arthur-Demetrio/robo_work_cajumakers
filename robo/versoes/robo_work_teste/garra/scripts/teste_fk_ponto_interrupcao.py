
import rclpy

from rclpy.node import Node

from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class TesteFKPontoInterrupcao(Node):

    def __init__(self):
        super().__init__('teste_fk_ponto_interrupcao')

        self.cliente_fk = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )

        self.juntas = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

        self.posicoes = [
            -0.0846821556137546,
            -0.1257160714712795,
            -0.000005708591217942365
        ]

    def executar(self):

        self.get_logger().info(
            'Aguardando serviço FK...'
        )

        if not self.cliente_fk.wait_for_service(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                'Serviço /compute_fk indisponível.'
            )
            return

        request = GetPositionFK.Request()

        request.header.frame_id = 'base_link'
        request.fk_link_names = ['tool0']

        estado = RobotState()
        estado.joint_state = JointState()

        estado.joint_state.name = self.juntas
        estado.joint_state.position = self.posicoes

        estado.is_diff = True

        request.robot_state = estado

        self.get_logger().info(
            '===== TESTE FK PONTO DE INTERRUPÇÃO ====='
        )

        self.get_logger().info(
            f'Juntas utilizadas: {self.posicoes}'
        )

        future = self.cliente_fk.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0
        )

        if not future.done():
            self.get_logger().error(
                'Tempo limite aguardando resposta FK.'
            )
            return

        resposta = future.result()

        if resposta is None:
            self.get_logger().error(
                'Resposta FK inválida.'
            )
            return

        self.get_logger().info(
            f'Código de retorno: {resposta.error_code.val}'
        )

        if resposta.error_code.val != 1:
            self.get_logger().error(
                'Não foi possível calcular o FK.'
            )
            return

        pose = resposta.pose_stamped[0].pose

        self.get_logger().info(
            '===== POSIÇÃO CALCULADA ====='
        )

        self.get_logger().info(
            f'X: {pose.position.x:.12f} m'
        )

        self.get_logger().info(
            f'Y: {pose.position.y:.12f} m'
        )

        self.get_logger().info(
            f'Z: {pose.position.z:.12f} m'
        )

        self.get_logger().info(
            '===== ORIENTAÇÃO CALCULADA ====='
        )

        self.get_logger().info(
            f'X: {pose.orientation.x:.12f}'
        )

        self.get_logger().info(
            f'Y: {pose.orientation.y:.12f}'
        )

        self.get_logger().info(
            f'Z: {pose.orientation.z:.12f}'
        )

        self.get_logger().info(
            f'W: {pose.orientation.w:.12f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = TesteFKPontoInterrupcao()

    try:
        node.executar()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
