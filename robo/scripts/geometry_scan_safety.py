#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

# Geometria no frame base_link
LIDAR_X = 0.330
LIDAR_Y = 0.005

FRENTE = 0.365
TRASEIRA = -0.205
MEIA_LARGURA = 0.315

MARGEM_INTERIOR = 0.020

class SafetyScan(Node):
    def __init__(self):
        super().__init__('geometry_scan_safety')

        self.pub = self.create_publisher(
            LaserScan,
            '/scan_safety',
            qos_profile_sensor_data
        )

        self.sub = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.receive,
            qos_profile_sensor_data
        )

        self.contador = 0

        self.get_logger().info(
            'SCAN SAFETY: /scan_raw -> /scan_safety'
        )

    def receive(self, source):
        if source.header.frame_id != 'laser_raw':
            self.get_logger().error(
                'Frame inesperado: ' + source.header.frame_id
            )
            return

        values = list(source.ranges)

        retirados = 0
        traseiros = []

        for i, r in enumerate(values):
            if not (
                math.isfinite(r)
                and source.range_min <= r <= source.range_max
            ):
                continue

            a = source.angle_min + i * source.angle_increment

            x = LIDAR_X + r * math.cos(a)
            y = LIDAR_Y + r * math.sin(a)

            interno = (
                TRASEIRA + MARGEM_INTERIOR < x < FRENTE - MARGEM_INTERIOR
                and abs(y) < MEIA_LARGURA - MARGEM_INTERIOR
            )

            if interno:
                values[i] = math.inf
                retirados += 1

            elif (
                x < TRASEIRA
                and abs(y) <= MEIA_LARGURA + 0.05
            ):
                traseiros.append((
                    TRASEIRA - x,
                    x,
                    y,
                    math.degrees(a)
                ))

        msg = LaserScan()
        msg.header = source.header
        msg.angle_min = source.angle_min
        msg.angle_max = source.angle_max
        msg.angle_increment = source.angle_increment
        msg.time_increment = source.time_increment
        msg.scan_time = source.scan_time
        msg.range_min = source.range_min
        msg.range_max = source.range_max
        msg.ranges = values
        msg.intensities = list(source.intensities)

        self.pub.publish(msg)

        self.contador += 1

        if self.contador % 30 == 0:
            minimo = min(traseiros) if traseiros else None

            if minimo:
                detalhe = (
                    f'gap_x={minimo[0]:.3f}m, '
                    f'x={minimo[1]:.3f}m, '
                    f'y={minimo[2]:.3f}m, '
                    f'ang={minimo[3]:.1f}deg'
                )
            else:
                detalhe = (
                    'Sem retorno traseiro externo; '
                    'isso NAO significa area livre'
                )

            self.get_logger().info(
                f'auto-excluidos={retirados}, '
                f'pontos_traseiros={len(traseiros)}, '
                + detalhe
            )

def main():
    rclpy.init()
    node = SafetyScan()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
