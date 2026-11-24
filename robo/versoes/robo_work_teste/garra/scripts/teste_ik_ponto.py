
import rclpy

from rclpy.node import Node
from rclpy.duration import Duration

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

from geometry_msgs.msg import PoseStamped


class TesteIKPontoInterrupcao(Node):

    def __init__(self):
        super().__init__('teste_ik_ponto_interrupcao')

        self.cliente_ik = self.create_client(
            GetPositionIK,
            '/compute_ik'
        )

        self.juntas = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

        self.seed_inicial = [
            -0.043724192961706994,
            -0.06428771595118771,
            -0.000005718891711
        ]

        self.posicao = [
            0.20066,
            0.005295,
            0.184070
        ]

        self.orientacao = [
            -0.352875592,
             0.633810774,
             0.561180612,
             0.398546158
        ]

    def executar(self):

        self.get_logger().info(
            'Aguardando serviço IK...'
        )

        if not self.cliente_ik.wait_for_service(
            timeout_sec=10.0
        ):
            self.get_logger().error(
                'Serviço /compute_ik indisponível.'
            )
            return

        request = GetPositionIK.Request()

        request.ik_request.group_name = 'garra'
        request.ik_request.ik_link_name = 'tool0'

        request.ik_request.avoid_collisions = False

        request.ik_request.timeout = Duration(
            seconds=2
        ).to_msg()

        # Pose desejada
        pose = PoseStamped()

        pose.header.frame_id = 'base_link'

        pose.pose.position.x = self.posicao[0]
        pose.pose.position.y = self.posicao[1]
        pose.pose.position.z = self.posicao[2]

        pose.pose.orientation.x = self.orientacao[0]
        pose.pose.orientation.y = self.orientacao[1]
        pose.pose.orientation.z = self.orientacao[2]
        pose.pose.orientation.w = self.orientacao[3]

        request.ik_request.pose_stamped = pose

        # Seed inicial
        estado = RobotState()

        estado.joint_state = JointState()

        estado.joint_state.name = self.juntas
        estado.joint_state.position = self.seed_inicial

        request.ik_request.robot_state = estado
        request.ik_request.robot_state.is_diff = True

        self.get_logger().info(
            '===== TESTE IK PONTO DE INTERRUPÇÃO ====='
        )

        self.get_logger().info(
            f'Posição: X={self.posicao[0]:.6f}, '
            f'Y={self.posicao[1]:.6f}, '
            f'Z={self.posicao[2]:.6f}'
        )

        self.get_logger().info(
            'Calculando solução IK...'
        )

        future = self.cliente_ik.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=5.0
        )

        if not future.done():
            self.get_logger().error(
                'Tempo limite aguardando resposta IK.'
            )
            return

        resposta = future.result()

        if resposta is None:
            self.get_logger().error(
                'O serviço não retornou uma resposta válida.'
            )
            return

        codigo = resposta.error_code.val

        self.get_logger().info(
            f'Código de retorno: {codigo}'
        )

        if codigo == 1:

            self.get_logger().info(
                'SOLUÇÃO IK ENCONTRADA!'
            )

            estado_final = resposta.solution.joint_state

            for nome, posicao in zip(
                estado_final.name,
                estado_final.position
            ):
                if nome in self.juntas:
                    self.get_logger().info(
                        f'{nome}: {posicao:.12f} rad'
                    )

        else:
            self.get_logger().warn(
                'NÃO FOI ENCONTRADA UMA SOLUÇÃO IK.'
            )

        self.get_logger().info(
            '===== FIM DO TESTE ====='
        )


def main(args=None):

    rclpy.init(args=args)

    node = TesteIKPontoInterrupcao()

    try:
        node.executar()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
