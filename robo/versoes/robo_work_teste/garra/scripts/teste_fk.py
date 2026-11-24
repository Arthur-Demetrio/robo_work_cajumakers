
import rclpy

from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class TesteFK(Node):

    def __init__(self):
        super().__init__('teste_fk_configuracao_inicial')

        self.client = self.create_client(
            GetPositionFK,
            '/compute_fk'
        )

        self.juntas = [
            'Revolucionar_46',
            'Revolucionar_49',
            'Revolucionar_52'
        ]

        # Configuração inicial do segundo segmento cartesiano
        self.posicoes = [
            -0.043724192961706994,
            -0.06428771595118771,
            -0.000005718891711
        ]

    def calcular_fk(self):

        request = GetPositionFK.Request()

        request.header.frame_id = 'base_link'
        request.fk_link_names = ['tool0']

        estado = RobotState()
        estado.is_diff = True

        estado.joint_state = JointState()
        estado.joint_state.name = self.juntas
        estado.joint_state.position = self.posicoes

        request.robot_state = estado

        future = self.client.call_async(request)

        rclpy.spin_until_future_complete(
            self,
            future
        )

        if future.result() is None:
            self.get_logger().error(
                'Falha na chamada do serviço FK.'
            )
            return

        response = future.result()

        if response.error_code.val != 1:
            self.get_logger().error(
                f'FK falhou: código {response.error_code.val}'
            )
            return

        pose = response.pose_stamped[0].pose

        self.get_logger().info('===== RESULTADO FK =====')

        self.get_logger().info(
            f'X: {pose.position.x:.9f} m'
        )

        self.get_logger().info(
            f'Y: {pose.position.y:.9f} m'
        )

        self.get_logger().info(
            f'Z: {pose.position.z:.9f} m'
        )

        self.get_logger().info(
            'Orientação (quaternion):'
        )

        self.get_logger().info(
            f'X: {pose.orientation.x:.9f}'
        )

        self.get_logger().info(
            f'Y: {pose.orientation.y:.9f}'
        )

        self.get_logger().info(
            f'Z: {pose.orientation.z:.9f}'
        )

        self.get_logger().info(
            f'W: {pose.orientation.w:.9f}'
        )


def main():

    rclpy.init()

    node = TesteFK()

    try:
        if node.client.wait_for_service(timeout_sec=10.0):
            node.calcular_fk()
        else:
            node.get_logger().error(
                'Serviço /compute_fk não encontrado.'
            )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

